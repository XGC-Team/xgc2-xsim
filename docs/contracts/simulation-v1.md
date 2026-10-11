# Simulation service v1

Service name: `xgc2.simulation`. API version: `v1`. Transport: XRPC `http.v1`.
Describe returns its identity under the `service_ref` field. This explicitly
declared discovery route accepts an unbound request; business routes do not.
Its `describe_path` is `/v1/describe`. The explicit process owner supplies the
nonempty `target_id`, socket allocation and configuration/resource grants;
there is no provider default target or constant instance identity.
The endpoint identifies one live world. It is created and closed by the world
owner: an embedded plugin or the simulator itself. A process supervisor starts
the world explicitly and exposes its ServiceRef; invoking an operation does
not start a simulator. Gazebo, MAVROS, SITL and other necessary independent
processes retain explicit workflow start/stop nodes. XRPC exposes domain
capabilities after a process exists; it is not a process-launch supervisor.

## Common meaning and engine ownership

The service owns entity realization, sensors, engine thread dispatch, resource
ownership and completion. Clients do not call an engine's ROS services, poll
its graph, inspect its processes, or infer completion from telemetry. Engine
adapters use native engine APIs. A device or user algorithm may still use ROS.
Telemetry and sensor samples remain on their selected data channels; this API
configures and describes them, not a mandatory sample transport.
The Gazebo adapter is a thin world-lifetime plugin around directly testable
native domain functions and data models. It starts with the native application/
world and closes with it, like other native integrated services. A separate ROS
translation sidecar or Core readiness assembler is not this ownership model.

Core depends on these semantics and declared capabilities, never an engine
name. `engine` is diagnostic information. A capability means a tested behavior,
not merely that a method exists. Missing capabilities produce `unsupported`;
there is no silent fallback, approximation or success stub. Physical fidelity,
contact models, noise and hardware timing are not promised equal by this API.

## Identity, state and units

- All requests use XRPC instance binding. `instance_id` changes when the world
  is recreated. It never changes for an entity reset or pause.
- An entity has an opaque string `id`, immutable `role` (`robot`, `obstacle`,
  `object`, `sensor`), and server-assigned integer `generation`. Recreating an
  ID changes its generation; a stale reference cannot affect its replacement.
  A client can supply a desired ID on creation or let the service allocate it.
  Engine model names, prim paths and numeric handles are private mappings.
- `EntityRef` is `{id, generation}` within the bound world instance. Mutating
  an existing entity requires its generation. This fences replacement; no
  generic revision is required on every read or every harmless operation.
- Pose uses right-handed world coordinates, metres and an `xyzw` unit
  quaternion: `{position:[x,y,z], orientation:[x,y,z,w]}`. The world's axes,
  gravity and frame ID are described. Linear and angular velocity use world
  coordinates in m/s and rad/s. Sensor configuration `pose` is relative to
  its immutable parent link.
- Simulation time is `{epoch, nanoseconds}`. Nanoseconds is an integer decimal
  string, avoiding JSON double truncation. Rewinding time creates a new epoch;
  resetting entity state alone does not rewind world time.
- Entity availability means the engine realization and required resources
  have been committed. Algorithm readiness, actuator arming and a delivered
  camera frame are separate facts and are not implied by entity creation.

## Assets and extensibility

An entity specification is `{id?, role, asset, pose, parameters?, sensors?}`.
`asset` is `{id, revision?, realization}`. A realization is a provider-owned
typed artifact `{media_type, uri}` or `{media_type, content}` (exactly one);
the provider parses it. Platform stores and forwards it without decoding
engine internals. Asset tooling selects an available realization before
invocation; workflow kernel code does not translate SDF, USD or model physics.
Inline content is bounded by the service's advertised request limit. URIs are
resolved by the service using its configured resource roots and credentials,
not by the caller's machine. Asset ID/revision identify intent, not a filesystem
path or a claim that two different models have identical dynamics.

`parameters` follows the asset's advertised JSON schema. Domain extensions
have namespaced IDs and a schema/version in `capabilities`; they cannot change
the meaning of the common methods. The core API never becomes a universal
`Execute(engine, command, arbitrary_args)` interface. New common semantics are
added to this contract when they serve a real product need across backends.

## Routes

All paths start with `/v1`. JSON is used except for explicitly typed resources.

| Method and path | Meaning | Capability |
| --- | --- | --- |
| GET `/describe` | ServiceRef identity, readiness (`ready`, `facts`, optional `wait_ready_ms`), engine diagnostics, capabilities, limits, world frame, supported artifact types and asset schemas | mandatory |
| GET `/health` | Service lifecycle (`starting`, `ready`, `failed`, `stopping`), revision and required native component states; does not probe other providers | mandatory |
| POST `/health/observe` | Hold until native health revision differs from `after_revision`, or the caller deadline ends | `health.observe` |
| GET `/world` | Current world mode, simulation time, step size and entity summary | mandatory |
| GET `/entities` | Current entities and references; this is a user query, not a completion polling API | mandatory |
| GET `/entities/{id}` | Entity specification, reference, lifecycle and state snapshot | mandatory |
| POST `/entities` | Create the supplied `entity`; completion returns its EntityRef and committed state | `entities.create` |
| DELETE `/entities/{id}` | Remove the specified `generation` and its owned sensors; completion means no longer addressable and owned resources released | `entities.remove` |
| POST `/entities/{id}/state` | Apply explicit pose/twist/enabled state at an engine boundary; not a controller stream | `entities.set_state` |
| POST `/entities/{id}/reset` | Restore initial entity state or an explicit supplied state; scope includes owned sensors and controller reset hooks declared by the provider | `entities.reset` |
| POST `/world/pause` | Pause physics time advancement; management operations remain responsive | `world.pause` |
| POST `/world/resume` | Resume continuous advancement | `world.resume` |
| POST `/world/step` | While paused, advance exactly `steps` positive steps; complete after those steps and return resulting time | `world.step` |
| POST `/world/reset` | Reset the explicit scope described below; return resulting epoch and surviving entity references | `world.reset` |
| GET `/sensors` | Describe managed sensors, immutable parents, desired and actual applied configuration | `sensors.describe` |
| GET `/sensors/{id}` | Describe one managed sensor and its SensorRef | `sensors.describe` |
| POST `/sensors` | Create a typed `sensor` on its declared parent link; return actual initialization and SensorRef | `sensors.create` |
| PATCH `/sensors/{id}/config` | Apply advertised live `config` fields with sensor generation and revision fencing | `sensors.configure` |
| DELETE `/sensors/{id}` | Release that sensor and its native resources using its generation | `sensors.remove` |
| GET `/operations/{operation_id}` | Inspect an operation by explicit user/recovery request | mandatory |
| POST `/operations/{operation_id}/wait` | Hold one bounded request until terminal, driven by completion notification; no periodic client receipt polling | mandatory |
| POST `/operations/{operation_id}/cancel` | Request cancellation; return actual state, never claim rollback of applied effects | mandatory |
| POST `/call/{service}/{Method}` | Invoke one method of a capability the world serves, with its JSON request as the body (Capability calls, below) | per capability |

Capabilities may be granular (for example reset modes or sensor kinds).
Describe includes exact support and limits. A v1 engine may omit unsupported
sensor kinds; it must not reinterpret one kind as another. `unsupported` is
distinct from transient unavailability and does not trigger retries.

World reset body is `{scope, entities?, reset_time, state?}`. `scope` is `entities` (reset
the selected EntityRefs in `entities`) or `all_entities` (all currently owned
entities). It preserves membership and initial resource definitions. Creating
an empty world or replacing a scene is an explicit set of entity operations,
not an ambiguous reset side effect. `reset_time:true` is a distinct capability;
it increments the time epoch and states its new time. Reset does not restart
external algorithm processes. A backend unable to provide a requested reset
scope rejects it before applying any portion.

## Concrete HTTP documents

The examples below use one bound world; all calls also carry the XRPC headers
`X-Xrpc-Instance-ID`, `X-Xrpc-Timeout-Ms` and `X-Request-ID`. No binding is
duplicated inside domain JSON. Integers must have integer JSON syntax; a float,
boolean, negative generation or duplicate object field is invalid. All request
fields are closed except an explicitly advertised asset `parameters` schema.
Requests for unsupported state/configuration fields fail before effects.

Discovery response:

```json
{"service":"xgc2.simulation","api_version":"v1","instance_id":"fresh-world-identity","ready":true,"facts":{"world_generation":1,"frames_flowing":true,"health":"ready","capabilities":[{"name":"xgc2.chassis.hold","entities":["robot-1"]}]},"describe_path":"/v1/describe","service_ref":{"target_id":"lab-world","service":"xgc2.simulation","api_version":"v1","instance_id":"fresh-world-identity","profile":"http.v1","endpoint":{"kind":"unix","address":"/granted/runtime/simulation.sock"}},"capabilities":["entities.create","entities.remove","entities.set_state","entities.reset","world.pause","world.resume","world.step","world.reset.entities","world.reset.all_entities"],"world_frame":{"id":"world","axes":"right-handed z-up"},"artifact_types":["application/sdf+xml"],"limits":{"entities":256,"pending_commands":32,"operation_receipts":128,"operation_waiters":32,"operation_timeout_ms":60000,"receipt_retention_ms":300000,"artifact_bytes":65536}}
```

POST `/v1/entities`:

```json
{"operation_timeout_ms":30000,"entity":{"id":"robot-1","role":"robot","asset":{"id":"robot-definition","revision":"immutable-revision","realization":{"media_type":"application/sdf+xml","uri":"models/robot/model.sdf"}},"pose":{"position":[1,2,0],"orientation":[0,0,0,1]}}}
```

The alternative realization uses `content` instead of `uri`. A URI denotes a
provider-granted resource reference; it is never an arbitrary caller pathname.
Asset resolution, manifest validation, geometry/random parameters and working
artifact preparation belong to the provider's explicit prepare CLI or host.
Prepare consumes a granted asset reference, versioned configuration and output
grant, and returns the artifact reference and provenance. It does not start a
world. Core only authorizes/forwards generic content references and grants.
Science settings, FCU/timing/noise/lidar defaults stay in the provider's asset
schema and applied configuration; these examples do not replace them.

POST `/v1/entities/robot-1/state`:

```json
{"generation":4,"operation_timeout_ms":10000,"state":{"pose":{"position":[3,2,0],"orientation":[0,0,0,1]},"twist":{"linear":[0,0,0],"angular":[0,0,0]},"enabled":true}}
```

POST `/v1/entities/robot-1/reset` has `generation`, `operation_timeout_ms` and
optional `state` of the same shape. Omitting state restores the provider's
initial state. DELETE `/v1/entities/robot-1` has exactly `generation` and
`operation_timeout_ms`. POST `/v1/world/pause` and `/resume` have only
`operation_timeout_ms`; `/step` additionally has a positive bounded `steps`.

POST `/v1/world/reset`:

```json
{"scope":"entities","entities":[{"id":"robot-1","generation":4}],"reset_time":false,"operation_timeout_ms":10000}
```

`all_entities` omits `entities`; selected entity IDs are unique and all
generations are validated before any effects. `reset_time` is always explicit.
The common reset state extension is accepted only when separately advertised;
a provider must not parse it and silently ignore it.

Admission returns 202:

```json
{"id":"create-robot-1","kind":"/v1/entities","state":"accepted"}
```

POST `/v1/operations/create-robot-1/wait` takes an empty object, using a separate
observation request ID. Native completion returns 200:

```json
{"id":"create-robot-1","kind":"/v1/entities","state":"succeeded","result":{"time":{"epoch":0,"nanoseconds":"120000000"},"paused":true,"step_size_seconds":0.001,"entities":[{"ref":{"id":"robot-1","generation":4},"role":"robot","lifecycle":"ready","state":{"pose":{"position":[1,2,0],"orientation":[0,0,0,1]},"twist":{"linear":[0,0,0],"angular":[0,0,0]},"enabled":true}}]},"effects":{"applied":true}}
```

Queries use the same result/time/state shapes. GET `/v1/entities/{id}` includes
its EntityRef, lifecycle, applied state and immutable specification; querying
state does not prove a previous operation completed. A failed operation returns
`error:{code,message}` and explicit effects where application began. A stale
generation is `conflict`; no capability is expressed by successful empty data.

Sensor creation body is `{operation_timeout_ms,sensor}`. Sensor is
`{id,parent:{id,generation,link},realization:{media_type,content}}`; the provider
parses the typed artifact and reports its common `kind` (`camera`, `imu`,
`lidar`). Native type names such as Gazebo `ray` are diagnostics. The parent
generation is validated before creation. There are no nested sensor route aliases.

PATCH config body is `{generation,expected_revision,persist,config,
operation_timeout_ms}`. `generation` belongs to the sensor; its parent is
immutable. Native Gazebo advertises `active`, `update_rate_hz` and relative
`pose` as live fields. It returns desired/applied revisions and configurations
separately, including the native engine's actual rate readback. Its ephemeral
provider rejects `persist:true` and returns `persisted:null`. Camera calibration
metadata application belongs to the camera source provider's config capability,
which does not change rendered optical truth. Sensor removal takes exactly
`generation` and `operation_timeout_ms`.
Sensor data endpoints and source demand/capture are separate lifecycle facts;
the parent world owns attachment and destruction, while the source owns a
capture's immutable metadata and bytes.

SensorRef is `{id, generation}`; its description also supplies immutable
`parent:{id,generation,link}`. Attachments are owned by the parent and inherit
its lifetime. Raw asset sensors retain authored science settings and gain
native initialization/destruction acknowledgements without renaming their data
sources. Initialization means the native sensor's Init completed, and removal
means native resource destruction was acknowledged. An observation deadline
cannot turn an in-flight irreversible create/remove into false completion.
A data endpoint has `kind`, `address`, format
and frame metadata; transport-specific details stay in the owning data
adapter. Configuration completion and receipt of the first sample differ.
Explicit capture is provided by the camera/source service, with metadata and
bytes from one capture; consumers do not toggle a source and then race a read.

Native health is ready only after world initialization and every explicitly
required component's owned initialization succeeds. Clock samples, ROS graph
membership, TCP probes and a discovered endpoint are not readiness authorities.
Optional native extensions aggregate into the world's existing SDK listener and
management executor; their declarations state actual consumers, schemas and
completion, without requiring every library to expose RPC.

## Operations and completion

A mutation has a request ID and an operation budget `operation_timeout_ms`.
The operation budget is domain lifetime; the XRPC deadline bounds this caller's
wait. They are not interchangeable. Services advertise a finite maximum
operation budget and retained terminal-result limits.

After admission, the response is an Operation:
`{id, kind, state, target?, result?, error?, effects?}`. `state` is `accepted`,
`running`, `succeeded`, `failed` or `cancelled`. These words have one meaning:
accepted means validation and bounded admission succeeded; running means the
service has begun executing; succeeded means the domain postcondition is met.
HTTP 202 carries a nonterminal operation; HTTP 200/201 carries a terminal
success. Failed admission is an HTTP/domain error and has no operation.
The operation ID equals the admitted mutation's `X-Request-ID` (1–128 URL-safe
ASCII letters, digits, `.`, `_`, `:`, `-`). Observation calls use their own
request IDs; they do not change the ID of the operation being observed.

Requests may wait for domain completion within their XRPC deadline. Long
operations return their reference and are observed through `/wait`, which
registers a bounded completion waiter. The service sends the terminal result
when the engine signals it; the caller does not repeatedly query `/entities`,
ROS state, or `/operations`. A waiter deadline only ends observation; it does
not cancel the operation. Event streams may be added without changing these
states or introducing a second completion authority.

The service owns request deduplication: one instance and request ID identify
one admitted mutation. Reuse with different method/target/payload conflicts.
Deduplication has a described finite retention window, not an exactly-once
promise across restarts. SDKs never auto-replay mutations. A transport failure
after possible delivery is outcome-unknown; the caller can explicitly inspect
the retained operation identified by the request ID. Expiry means unknown,
not that nothing happened. Admitted operations retain their ID independently
of any socket and are bounded in number.

Cancellation before irreversible application yields cancelled with no effects.
Once effects begin, cancellation is cooperative: the service returns what
was actually applied, may finish successfully, or fail with explicit effects.
It never promises that closing a connection undoes engine changes. When an
engine has an irreversible async factory, expiration after submission remains
nonterminal until reconciled; the provider observes its own engine event and
finishes cleanup/result recording. It cannot mark failure and allow a late
entity to appear unowned. Operation result retention starts at terminal state.

Multi-entity operations are not implicitly transactional. A supported batch
declares atomicity and per-item results; callers must not infer rollback.
Data-plane telemetry is evidence of state, not another operation authority.

## Readiness

`GET /v1/describe` is also the readiness report of the process (the readiness contract for Run-owned services
of the platform architecture). Next to the fields above it carries `service`, `api_version`, `instance_id` (the
instance of the ServiceRef), `ready` and `facts`:

- `ready` is true when the world is loaded, its native components are initialized (health `ready`) and the
  domain dependencies are valid. A service whose dependency is replaced either exits or reports `ready:false`;
  it never serves a stale binding.
- `facts` carries the dependencies that make the service valid: `world_generation` (integer, the generation of
  the loaded world; it changes, or the service exits, when the world is replaced), `frames_flowing` (the world
  has delivered frames to its outputs), `health` (the lifecycle state of `/v1/health`), `capabilities` (the
  capabilities the world serves, see Capability calls) and, when the host publishes on ROS, `ros_master_uri`.
  Clients ignore facts they do not know. xsim loads one world when it starts, so its `world_generation` is 1;
  another world is another process with another `instance_id`.
- The optional query `wait_ready_ms=<0..30000>` (a canonical decimal integer; anything else is
  `invalid_argument`) holds the reply, without polling, until `ready` is true, the wait elapses or the XRPC
  deadline of the call passes, and then answers with the current describe. A service that is ready answers at
  once, and one that starts to stop answers waiting calls at once. The caller waits with one outstanding call
  instead of repeating describe.
- The XRPC discovery exemption matches the declared route exactly, so a describe call with a query is bound to
  the instance like a business call: read `instance_id` from an unbound `GET /v1/describe` first.

## Capability calls

A world host can serve domain capabilities next to the simulation operations. A capability is a service with a
name, for example `xgc2.chassis.hold`, and JSON methods; a method is named `<service>/<Method>` on every XRPC
profile. On this service it is invoked with

    POST /v1/call/<service>/<Method>        body: the JSON request of the method (an empty body is {})

The call is bound to the instance like a business call. The reply body is the JSON result with status 200. A
failure is the XRPC error envelope `{"error":{"code","message",...}}` with the status of its code; the
capability may add fields to the error object (for example `details`): `invalid_argument` 400, `not_found` 404
(also an unknown service or method), `internal` 500. The host only routes the call: validation, semantics and
result belong to the capability. A call is not an operation and has no receipt.

The describe facts list the capabilities the world serves, with the entities each one acts on:
`"capabilities":[{"name":"xgc2.chassis.hold","entities":["<id>",...]}]`. The list follows the entities of the
world. A caller resolves "entity X, capability C" to a world by finding X among the entities of C, without
knowing which engine runs the world.

## Chassis HOLD

HOLD keeps one simulated chassis from executing motion commands until it is explicitly released. It is not a
world pause, an entity disable or a reset: the other entities, the simulation clock and the sensors and
publication of the held entity keep running. The capability `xgc2.chassis.hold` serves the methods below; the
domain, the JSON requests and replies and their semantics are those of the chassis-hold contract of
xgc2-chassis-hold and are passed through unchanged. This section is the binding to the simulation service.

| Method | Request | Reply |
| --- | --- | --- |
| `Describe` | `{}` | instance, roster, feedback of each robot, supported operations |
| `State` | `{"robot_ids":[...]}`, or `{}` for every robot | state of each robot |
| `Engage` | `{"robot_ids":[...]}` or `{"all":true}` | outcome and state of each robot |
| `Release` | `{"expected_instance","changes":[{"robot_id","expected_revision"}]}` | outcome and state of each robot |

The `instance` of the HOLD replies is the `instance_id` of this service, and Release is fenced by it and by the
revision of each robot. A `State` that names an unknown robot is `not_found`, and the `details.robot_ids` of its
error lists the unknown IDs.

- **Roster.** The ground robots of the world, keyed by their entity ID, which is the robot ID of the
  Experiment: the Scout and Mecanum entities of xsim. They are the entities listed for the capability. An
  unmanned aircraft is not a chassis and is `unknown_robot`. An entity joins the roster when it is created and
  leaves it when it is removed; the HOLD state of an ID is kept while the ID is not in the roster, so an entity
  created again under the same ID starts held.
- **Gate.** Engage closes the gate for command admission before it replies. A velocity command is refused where
  it would reach the model when its entity is held or when it was received before the last release of the
  entity, so a command that waited in a queue is never replayed. Receipt is stamped when the host's input path
  takes the command in, on the monotonic clock of the HOLD domain (not simulation time). The entity stays
  enabled: its sensors and publication keep running.
- **Zero.** At every world boundary the world commands zero to the model of each held entity and drops the
  entity's pending velocity controls. Engage replies when the zero has been written, or after at most 60 ms with
  stage `gated`. `zero_written` says that the model was commanded, not that the entity stands still.
- **Rest.** The entities report their own twist as feedback. After the zero, a twist of at most 0.02 m/s and
  0.05 rad/s held for 300 ms moves the stage to `stopped`.
- **World pause.** HOLD is independent of the pause. The state changes at once; the zero is written at the next
  world boundary, which runs while the world is paused. A paused world does not move, so `stopped` is reached
  only after the world runs.
- **Restart.** HOLD state is not persisted. A restarted world is a new instance with every robot released, and
  a Release carrying the old instance is a `conflict` for every robot.

## Versioning and conformance

API major versions identify semantic incompatibility; additions are optional
declared capabilities or new fields with defined defaults. Unknown required
inputs are rejected; clients tolerate unknown response fields. No aliases or
dual legacy routes are required for the internal development migration.

Each engine runs the same externally observable conformance cases for its
declared capabilities: paused CRUD, replacement identity, completed versus
accepted, world/individual reset scope, exact step count, cancellation at the
application boundary, shutdown with outstanding operations and unsupported
capabilities; readiness (`ready`, `facts`, `wait_ready_ms`); capability calls
and the listing of capabilities in the facts; chassis HOLD (engage, state and
release with revisions and instances, zero output and rest, refused commands
that are not replayed, engage and release while paused, removal and re-creation
of a held entity, restart). Backend tests additionally
prove actual native state and resource release. A fake backend or successful HTTP
response alone is insufficient.
