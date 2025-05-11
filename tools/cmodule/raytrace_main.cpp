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
      sizeof(float3), 3
    );

    vertices[0] = (float3) { -1.0f, -1.0f, -3.0f };
    vertices[1] = (float3) { +1.0f, -1.0f, -3.0f };
    vertices[2] = (float3) { +0.0f, +1.0f, -3.0f };

    unsigned int* indices = (unsigned int*)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
        sizeof(unsigned int) * 3, 1);
    
    indices[0] = 0; 
    indices[1] = 1; 
    indices[2] = 2;

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        float ndc_x = (2.0f * x / width - 1.0f);
        float ndc_y = (1.0f - 2.0f * y / height);
        float3 dir = { ndc_x, ndc_y, -1.0f };
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
          buffer[index + 0] = 255;
          buffer[index + 1] = 255;
          buffer[index + 2] = 255;
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