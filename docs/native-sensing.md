# Native sensing boundary

Penetration and compute selection are independent. `raycast` observes the nearest
surface; `penetrating` retains scene samples hidden behind other surfaces. A
sensor's `cpu`, `gpu`, or `auto` backend does not select its observation model.
Auto attempts the native GPU implementation and keeps the requested CPU sensor
configuration when that implementation is unavailable. Explicit GPU errors are
reported rather than represented as CPU execution.

The CPU engine is the shared Math scene library. `libxgc2-sensing-gpu` owns the
MARSIM renderer and GPU point filtering, and `libxgc2-sensing-dev` owns the C ABI
and loader headers. Neither package requires an XSim process. The scene adapter
and common scene library consume this ABI instead of embedding another renderer.
No compatibility package or old module entry point is retained.

One GPU owner thread creates, scans and destroys its contexts. Geometry is
uploaded once per sampling policy, shared across robots; the owner reuses the
spatial index, candidate storage, projection buffers and readback storage.
Opaque projection keeps source order for depth ties. Penetrating point sets
skip distance and source-index sorting. Offscreen scans do not swap an X11
window. Context/framebuffer switches do not change the original projection math.
CPU workers remain bounded, and unchanged noiseless observations reuse payloads
with the current publication timestamp.

The ABI returns world-frame native float32 XYZ/intensity records. The scene
adapter converts them into its existing XYZ wire contract. It does not relabel
simulated imagery as physical replay. GPU static-scene sensing is distinct from
CPU depth, ray labels, noise and dynamic-body features; Auto retains CPU execution
for capabilities not implemented by the GPU consumer.

Acceptance compares occlusion semantics, not identical point sets between
simulators. Gazebo's native CPU/GPU ray plugins and XSim's geometry have different
sampling precision. MARSIM's own reference comparison does require matching
projection results. Hardware probes are `sensing_modes_test`,
`xsim_sensing_backend_test`, and the scene adapter's `gpu_integration_test`.
They are separate from hardware-independent CTest gates.
