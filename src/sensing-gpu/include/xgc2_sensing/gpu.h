#ifndef XGC2_SENSING_GPU_MODULE_H
#define XGC2_SENSING_GPU_MODULE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XGC2_SENSING_GPU_ABI 1u

typedef struct xgc2_sensing_gpu_projection {
  double range_m, min_range_m, h_fov_deg, v_fov_deg;
  int32_t h_res, v_res;
  double point_cover_spacing_m, publish_rate_hz;
  uint32_t penetrating;
  uint32_t heading_crop;
  double heading_cos_min, vertical_slab_tan;
} xgc2_sensing_gpu_projection;

/* Resize caller-owned storage to bytes and return its writable address. A null
 * result for a nonzero size signals failure. Records are four native floats:
 * x, y, z, intensity (16 bytes), in world coordinates. No ownership crosses ABI. */
typedef void *(*xgc2_sensing_gpu_grow)(void *context, size_t bytes);

typedef struct xgc2_sensing_gpu_api {
  uint32_t abi, size;
  /* xyz contains count packed XYZ float records; upload once on the owner. */
  void *(*create)(const float *xyz, size_t count,
                  const xgc2_sensing_gpu_projection *first,
                  char *error, size_t error_size);
  /* position and orientation are doubles; quaternion order is w,x,y,z.
   * Return zero on success. All functions run on the same context owner thread.
   * Failures write a terminated error string; exceptions never cross the ABI. */
  int (*scan)(void *renderer, const double position[3],
              const double orientation_wxyz[4], double stamp_s,
              const xgc2_sensing_gpu_projection *projection,
              xgc2_sensing_gpu_grow grow, void *context,
              char *error, size_t error_size);
  void (*destroy)(void *renderer);
} xgc2_sensing_gpu_api;

/* Return null for an unsupported ABI. The table lives for the module lifetime. */
const xgc2_sensing_gpu_api *xgc2_sensing_gpu_entry(uint32_t abi);

#ifdef __cplusplus
}
#endif
#endif
