#include <stdlib.h>
#include <math.h>
#include <embree4/rtcore.h>
#include "thread_pool.hpp"
#include "random.hpp"

#include <iostream>

#define CHECK_ERR(dev) do {\
  RTCError err = rtcGetDeviceError(dev);\
  if (err != RTC_ERROR_NONE) {\
    printf("RTCError @%d", __LINE__);\
    abort();\
  }\
} while (0)

// feature set
#define ENABLE_FEATURE_CAMERA_DISTORTION

typedef struct { float e[16]; } Mat4x4;
typedef struct { float u, v; } Float2;
typedef struct { float x, y, z; } Float3;

struct SceneInfo {
  RTCDevice device;
  RTCScene scene;
};

/// @brief thread pool singleton
static std::unique_ptr<ThreadPool> pool;
static std::unique_ptr<SceneInfo> sinfo;

void gen_ray(
  int u, int v, 
  int width, int height, 
  RTCRay *ray,
  // distortion parameters
  float cx = 0.0, float cy = 0.0, float fx = 1.0, float fy = 1.0,
  float k1 = 0.0, float k2 = 0.0, float p1 = 0.0, float p2 = 0.0,
  float k3 = 0.0, float k4 = 0.0, float k5 = 0.0, float k6 = 0.0
) {
  /**
   * OpenCV Rectification Transform
   * 
   * given: u,v pixel coordinates
   * 
   * x = (u - c_x) / f_x
   * y = (v - c_y) / f_y
   * 
   * [XYW]^T = R^-1 * [xy1]^T
   * 
   * x' = X/W
   * y' = Y/W
   * 
   * x'' = x'(1 + k1*r^2 + k2*r^4 + k3*r^6) + 2*p1*x'*y' + p2*(r^2 + 2*x'^2)
   * y'' = y'(1 + k1*r^2 + k2*r^4 + k3*r^6) + p1*(r^2 + 2*y'^2) + 2*p2*x'*y'
   * 
   * map_x(u, v) = x''*f_x + c_x
   * map_y(u, v) = y''*f_y + c_y
   */

  float x = (u - cx) / fx;
  float y = (v - cy) / fy;

#ifdef ENABLE_FEATURE_CAMERA_DISTORTION
  float r2 = x*x + y*y;
  float r4 = r2*r2;
  float r6 = r2*r4;

  /// TODO: fisheye distortion
  float radial_distortion = 1 + k1*r2 + k2*r4 + k3*r6;
  float x_rad = x * radial_distortion;
  float y_rad = y * radial_distortion;
  float x_tan = 2*p1*x*y + p2*(r2 + 2*x*x);
  float y_tan = p1*(r2 + 2*y*y) + 2*p2*x*y;

  x = x_tan + x_rad;
  y = y_tan + y_rad;
#endif

  // create ray, accounting for radial/tangential distortion if necessary
  Float3 dir = { x, y, -1.0f };

  // origin (camera)
  ray->org_x = ray->org_y = ray->org_z = 0.0;

  float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z /* z^2 = 1 always */);
  ray->dir_x = dir.x / len;
  ray->dir_y = dir.y / len;
  ray->dir_z = dir.z / len;

  ray->tnear = 0.001f;
  ray->tfar = 100.0;  // normalize direction

  ray->flags = 0;
  ray->mask = -1;
}

extern "C" {
  /// @brief struct for holding camera (in/ex)trinsics
  struct Camera {
    /// @brief camera resolution
    int resolution[2];
    /// @brief X/Y focal lengths, X/Y centers
    float sparse_intrinsics[4];
    /// @brief k1, k2, p1, p2, k3, ...
    float distortion_coeff[8];
  };

  void init_workers() {
    if (!pool) {
      pool = std::make_unique<ThreadPool>();
    }

    if (!sinfo) {
      sinfo = std::make_unique<SceneInfo>();
    }

    sinfo->device = rtcNewDevice(NULL);
    sinfo->scene = rtcNewScene(sinfo->device);
    CHECK_ERR(sinfo->device);

    RTCGeometry geom = rtcNewGeometry(sinfo->device, RTC_GEOMETRY_TYPE_TRIANGLE);
    CHECK_ERR(sinfo->device);
    
    Float3 *vertices = (Float3 *) rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
      sizeof(Float3), 4
    );
    CHECK_ERR(sinfo->device);

    float rad = 2.5f;
    float d = -12.0f;
    
    vertices[0] = (Float3) { -rad, -rad, d };
    vertices[1] = (Float3) { +rad, -rad, d };
    vertices[2] = (Float3) { +rad, +rad, d };
    vertices[3] = (Float3) { -rad, +rad, d };

    rtcSetGeometryVertexAttributeCount(geom, 1);
    Float2 *uv = (Float2 *) rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0,
        RTC_FORMAT_FLOAT2, sizeof(Float2), 4);
    CHECK_ERR(sinfo->device);

    uv[0] = (Float2) { 0, 0 };
    uv[1] = (Float2) { 1, 0 };
    uv[2] = (Float2) { 1, 1 };
    uv[3] = (Float2) { 0, 1 };

    unsigned int* indices = (unsigned int*) rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      sizeof(unsigned int) * 3, 2
    );
    CHECK_ERR(sinfo->device);
    
    // Triangle 0: vertices 0, 1, 2
    indices[0] = 0;
    indices[1] = 1;
    indices[2] = 2;

    // Triangle 1: vertices 2, 3, 0
    indices[3] = 2;
    indices[4] = 3;
    indices[5] = 0;    

    rtcCommitGeometry(geom);
    CHECK_ERR(sinfo->device);
    int id = rtcAttachGeometry(sinfo->scene, geom);
    CHECK_ERR(sinfo->device);
    rtcReleaseGeometry(geom);
    CHECK_ERR(sinfo->device);
    rtcCommitScene(sinfo->scene);
    CHECK_ERR(sinfo->device);

    // rtcReleaseScene(scene);
    // rtcReleaseDevice(device);
  }

  /// @brief allocate an image buffer
  /// @param width width of image buffer in pixels
  /// @param height height of image buffer in pixels
  /// @param channels number of channels per pixel (e.g. 3)
  /// @return pointer to the image buffer
  unsigned char *allocate_image_buffer(int width, int height, int channels) {
    unsigned char *buffer = (unsigned char *) malloc(width * height * channels);
    return buffer;
  }

  /// @brief free a previously allocated image buffer
  /// @param buffer pointer to the image bufer
  void free_image_buffer(unsigned char *buffer) {
    free(buffer);
  }

  void render_frame(unsigned char *buffer, struct Camera *cam, float time) {
    // unpack distortion coefficients
    float cx = cam->sparse_intrinsics[0];
    float cy = cam->sparse_intrinsics[1];
    float fx = cam->sparse_intrinsics[2];
    float fy = cam->sparse_intrinsics[3];
    float k1 = cam->distortion_coeff[0];
    float k2 = cam->distortion_coeff[1];
    float p1 = cam->distortion_coeff[2];
    float p2 = cam->distortion_coeff[3];
    float k3 = cam->distortion_coeff[4];

    int width = cam->resolution[0];
    int height = cam->resolution[1];
    int tile_size = 230;
    int num_tiles = ((width + tile_size - 1) / tile_size) * ((height + tile_size - 1) / tile_size);
    auto barrier = std::make_shared<Barrier>(num_tiles);

    // printf("GEOM: %p", rtcGetGeometry());

    for (int t = 0; t < height; t += tile_size) {
      for (int s = 0; s < width; s += tile_size) {
        int v_lim = std::min(height, t + tile_size);
        int u_lim = std::min(width, s + tile_size);

        pool->enq([=]() {
          // interpolation targets
          float uv[2];

          // tile loop
          for (int v = t; v < v_lim; v++) {
            for (int u = s; u < u_lim; u++) {
              if (u == 0 || v == 0 || u == u_lim - 1 || v == v_lim - 1) {
                buffer[(v * width + u) * 3] = 255;
                buffer[(v * width + u) * 3 + 1] = 0.5*(1+std::sin(time)) * 255;;
                buffer[(v * width + u) * 3 + 2] = 0;
              }

              RTCRayHit rayhit;
              
              rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
              gen_ray(
                u, v, width, height, 
                &rayhit.ray, 
                cx, cy, fx, fy, 
                k1, k2, p1, p2, 
                k3, 0, 0, 0);

              rtcIntersect1(sinfo->scene, &rayhit);
              auto geom_id = rayhit.hit.geomID;

              if (geom_id != RTC_INVALID_GEOMETRY_ID) {
                // printf("HIT\n"); 

                int index = (v * width + u) * 3;

                rtcInterpolate0(
                  rtcGetGeometry(sinfo->scene, geom_id), 
                  rayhit.hit.primID, 
                  rayhit.hit.u, rayhit.hit.v,
                  RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, 
                  uv, 2);

                buffer[index+0] = uv[0] * 255;
                buffer[index+1] = uv[1] * 255;
                buffer[index+2] = 0.5*(1+std::sin(time)) * 255;
              }
            }
          }

          barrier->mark_complete();
        });
      }
    }

    // sync with workers
    barrier->wait();
  }
}