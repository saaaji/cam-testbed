#include <stdlib.h>
#include <math.h>
#include <embree3/rtcore.h>

typedef struct { float x, y, z; } float3;

extern "C" {
  unsigned char *allocate_image_buffer(int width, int height, int channels) {
    unsigned char *buffer = (unsigned char *) malloc(width * height * channels);

    RTCDevice device = rtcNewDevice(NULL);
    RTCScene scene = rtcNewScene(device);

    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    float3 *vertices = (float3 *) rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
      sizeof(float3), 4
    );

    float rad = 2.5f;
    
    vertices[0] = (float3) { -rad, -rad, -3.0f };
    vertices[1] = (float3) { +rad, -rad, -3.0f };
    vertices[2] = (float3) { +rad, +rad, -3.0f };
    vertices[3] = (float3) { -rad, +rad, -3.0f };

    unsigned int* indices = (unsigned int*) rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
        sizeof(unsigned int) * 3, 2);
    
    // Triangle 0: vertices 0, 1, 2
    indices[0] = 0;
    indices[1] = 1;
    indices[2] = 2;

    // Triangle 1: vertices 2, 3, 0
    indices[3] = 2;
    indices[4] = 3;
    indices[5] = 0;    

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    /*
    left_dist_coeffs:
    - - -0.33398974592013153
      - 0.1259186788280236
      - 0.00013446131942386718
      - -0.0005642474752944876
      - -0.022701140495366815
    left_camera_matrix:
    - - 1031.3842253259802
      - 0.0
      - 967.1790093804033
    - - 0.0
      - 1031.7878887270722
      - 521.231695361321
    - - 0.0
      - 0.0
      - 1.0
    */

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        float ndc_x = (2.0f * x / width - 1.0f);
        float ndc_y = (1.0f - 2.0f * y / height);
        // float3 dir = { ndc_x, ndc_y, -1.0f };

        float u = (float) x / width;
        float v = (float) y / width;

        float f = 0.2;

        u = (u - 0.5) / f;
        v = (v - 0.5) / f;

        float r2 = u*v+v*v;
        float r4 = r2*r2;
        float r6 = r4*r2;

        float k1 = -0.33;
        float k2 = 0.12;
        float p1 = 0.0001;
        float p2 = -0.0005;
        float k3 = -0.0227;

        // float k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;

        float radial = 1 + k1*r2 + k2*r4 + k3*r6;
        float x_dist = u * radial + 2*p1*u*v + p2*(r2 + 2*u*u);
        float y_dist = v * radial + p1*(r2+2*v*v) + 2*p2*u*v;

        // float x_dist = u;
        // float y_dist = v;

        float3 dir = { x_dist, y_dist, -1.0f };

        float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
        dir.x /= len; dir.y /= len; dir.z /= len;

        RTCRayHit rayhit = { 0 };
        rayhit.ray.org_x = 0.0f;
        rayhit.ray.org_y = 0.0f;
        rayhit.ray.org_z = 0.0f;
        rayhit.ray.dir_x = dir.x;
        rayhit.ray.dir_y = dir.y;
        rayhit.ray.dir_z = dir.z;
        rayhit.ray.tnear = 0.001f;
        rayhit.ray.tfar = 1000.0f;
        rayhit.ray.mask = -1;
        rayhit.ray.flags = 0;
        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);
        rtcIntersect1(scene, &context, &rayhit);

        int index = (y * width + x) * 3;
        if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
          float scale = 5.0;
          
          float hit_x = rayhit.ray.org_x + rayhit.ray.dir_x * rayhit.ray.tfar;
          float hit_y = rayhit.ray.org_y + rayhit.ray.dir_y * rayhit.ray.tfar;

          int x = (int) floor(hit_x * scale);
          int y = (int) floor(hit_y * scale);
          
          if ((x+y)%2==0) {
            buffer[index] = 255;
            buffer[index+1] = 255;
            buffer[index+2] = 255;
          } else {
            buffer[index] = 0;
            buffer[index+1] = 0;
            buffer[index+2] = 0;
          }
        } else {
          buffer[index + 0] = 0;
          buffer[index + 1] = 0;
          buffer[index + 2] = 0;
        }
      }
    }

    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return buffer;
  }

  void free_image_buffer(unsigned char *buffer) {
    free(buffer);
  }
}