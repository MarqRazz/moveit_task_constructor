<!--
Copyright (c) 2026 Kuka
All rights reserved.
-->
# MoveIt Task Constructor: an authoring guide

This document exists to make an agent who has never seen MoveIt Task Constructor
(MTC) competent on the first attempt at building or modifying a task, and to stop
it from inventing structure the framework already provides. It is a reference, not
a tutorial: read the section you need, act on it, and verify against the source
before relying on any behavioural claim.

**Two kinds of statement live in this document, and they age differently.** The
decisions and the model are stable. The behavioural claims (a line number, "this
returns one candidate", "the default cost is X") can go stale under us. Every such
claim below carries a `file:line` citation into this repository so you can
re-check it. Where the code and this document disagree, the code wins and this
document is the defect.

## Source of truth

All citations are relative to the root of **this repository** — headers in
`core/include/moveit/task_constructor/`, implementations in `core/src/`, the
canonical worked example in `demo/src/pick_place_task.cpp`. A citation written as
`stage.h:360` means `core/include/moveit/task_constructor/stage.h`; one written
`compute_ik.cpp:246` means `core/src/stages/compute_ik.cpp`. Full paths are given
where the short form would be ambiguous.

Every `file:line` citation in this document was verified against commit
`781c0de0` (branch `ros2`, 2026-08-14). If you are far ahead of that commit,
re-verify before quoting a line number; `docs/verify_citations.py` in this
repository re-checks them all in one pass.

The framework author's own account of the intended model is the dissertation
"Modular Models for Multi-Phase Robotic Manipulation Tasks" (M. Görner, 2025),
which names the concept *Task Construction*; it is cited below by page. Those page
citations have **not** been machine-verified — treat them as pointers, not proof.

---

## 1. The execution model

### What a Task is

A `Task` is the root of a tree of stages. It is literally a wrapper around a
single container, by default a `SerialContainer` named "task pipeline"
(`core/include/moveit/task_constructor/task.h:76,83-84`). You add stages to it,
call `init()` to validate the tree, then `plan()` to search for solutions
(`task.h:130-133`). Planning does not return one trajectory; it enumerates
*solutions*, ranked by cost, and you pick one (usually the front of
`task.solutions()`, `task.h:147`) to execute.

MTC is an offline, sampling-based *task and motion* planner. It does not run a
state machine at execution time. It searches a graph of partial motions and
stitches them into complete trajectories. Understanding that search is the thing
an agent most needs and most often gets wrong.

### What a Stage is

A stage is a black-box computable motion phase (dissertation p34). It owns a
method that computes robot trajectories plus the world-model changes they imply,
a set of typed properties that parameterize it, and an *interface* that declares
which side(s) it reads from and writes to. Stages never talk to each other during
computation; they communicate only by passing completed partial solutions
(dissertation p35). This is the single design decision that everything else
follows from: a stage is understandable in isolation given the states handed to it.

### InterfaceState and Solution: the two currencies

An **`InterfaceState`** is a potential start or goal for planning. It is
essentially a planning scene (the robot state plus all collision objects) plus a
bag of typed properties (`core/include/moveit/task_constructor/storage.h:77-171`).
Interface states mark the endpoints of every trajectory a stage generates.

A **`SolutionBase`** connects two interface states with a cost
(`storage.h:269-349`). Its concrete forms are `SubTrajectory` (one motion,
`storage.h:353`), `SolutionSequence` (a chain, `storage.h:386`), and
`WrappedSolution` (a lifted child solution, `storage.h:420`). A solution is a
**failure** when its cost is not finite; `isFailure()` is literally
`!std::isfinite(cost_)` (`storage.h:303-306`). Failures are stored and shown for
introspection but are never extended into a larger plan.

### The three (really four) interface types

The whole model turns on `enum InterfaceFlag`
(`core/include/moveit/task_constructor/stage.h:75-86`). Four combinations exist:

| Interface | Flags | Reads | Writes | Base class |
|---|---|---|---|---|
| **Generator** | `WRITES_PREV_END \| WRITES_NEXT_START` | nothing | both sides | `Generator` (`stage.h:360`) |
| **Forward propagator** | `READS_START \| WRITES_NEXT_START` | its start | its end (forward in time) | `PropagatingForward` (`stage.h:334`) |
| **Backward propagator** | `READS_END \| WRITES_PREV_END` | its end | its start (backward in time) | `PropagatingBackward` (`stage.h:347`) |
| **Connector** | `READS_START \| READS_END` | both sides | nothing outward | `Connecting` (`stage.h:405`) |

Read "start" as the side facing the previous stage and "end" as the side facing
the next stage. Directions describe the *direction of inference*, not always the
direction of execution.

- A **Generator** needs no input. It seeds the search by spawning states out of
  both sides (`Generator::spawn`, `stage.h:368-374`). `CurrentState`,
  `FixedState`, and every pose generator is a generator. The Task's own interface
  is defined to be a generator interface (dissertation p41), which is why a
  well-formed task begins with a generator.

- A **Forward propagator** reads a start state and computes a motion to a new end
  state, forward in time. "Move up 10 cm from here" is forward.

- A **Backward propagator** reads an *end* state (a known goal) and infers the
  motion and start state that reach it, backward in time. This is the one that is
  unintuitive. Approach motions toward a grasp are naturally backward: the grasp
  pose is known, and the short Cartesian approach is computed from it, yielding a
  pre-grasp start state that a transit motion can then target (dissertation
  p38-39). `MoveRelative` and `MoveTo` are `PropagatingEitherWay`
  (`stage.h:290`): they auto-derive their direction from context, or you pin it
  with `restrictDirection()` (`stage.h:302`).

- A **Connector** reads a state on each side and plans a trajectory between them.
  It writes nothing outward; it only bridges. `Connect` is the transit-motion
  workhorse. Because it collects states arriving from both the forward and the
  backward frontier, it is where a generator-seeded chain from the front meets a
  goal-seeded chain from the back (dissertation p39). It is also the main
  combinatorial bottleneck: it may face M x N state pairs.

### How solutions flow, and "the interface is not satisfied"

At `init()`, each container composes its children's interfaces and checks they fit
together. The container's own interface is derived from its first and last child
(dissertation p46-47); it is *not* something you set. If the composition is
inconsistent, for example a `Connect` with nothing producing states on one of its
sides, `init()` throws `InitStageException` (`stage.h:117-139`). That thrown error
is the literal meaning of "the interface is not satisfied": the tree cannot
possibly produce states where a stage needs to read them.

At plan time the failure is quieter. A forward propagator whose start side never
receives a state simply never computes. It is not an error; the stage just sits
idle and the task reports no solutions. The two most common causes are (a) no
generator seeds that part of the chain, or (b) a monitoring generator was never
pointed at a monitored stage (see below), so it never fires.

### Monitoring generators (the non-local channel)

A plain generator only knows what was configured on it. But a pose generator
usually needs the *scene from a solution computed elsewhere in the task* (the
grasp must be sampled against the state in which the object exists). A
`MonitoringGenerator` (`stage.h:388-402`) hooks into another stage's
`onNewSolution` and fires once per solution that stage produces. `GeneratePose`
and all its subclasses are monitoring generators
(`core/include/moveit/task_constructor/stages/generate_pose.h:49`), and
`GeneratePose::canCompute()` returns true only when it has received at least one
monitored solution (`core/src/stages/generate_pose.cpp:67-70`). **If you forget
`setMonitoredStage(...)`, the generator never computes and the task silently finds
nothing.** This is the channel that carries planning commitments (which end
effector, which grasp) forward past a `Connect`, which otherwise passes no new
information (dissertation p40-41).

### Cost, pruning, and search order

Every interface state carries a `Priority` of `(status, depth, cost)`
(`storage.h:96-119`). The search prefers deeper (closer to a complete solution)
and, within equal depth, cheaper paths (dissertation p56). States on branches
that can no longer reach a complete solution are **pruned**: an interface state's
`Status` is `ENABLED`, `ARMED`, or `PRUNED` (`storage.h:84-89`), and a state built
with non-finite cost is born `PRUNED` (`storage.h:100`). Because a serial chain is
solved from both ends, a failure on one end prunes the matching states on the
other end so the planner stops expanding them (dissertation p58). You can disable
this with `setPruning(false)` on a container (`container.h:55-56`), which you may
want when a stage legitimately fails often but downstream stages should still be
explored.

---

## 2. The stage vocabulary

Every stage under `core/include/moveit/task_constructor/stages/`. "Interface"
tells you how it wires in.

| Stage | Header | Interface | What it does / reach for it when | Key required properties |
|---|---|---|---|---|
| `CurrentState` | `current_state.h:49` | Generator | Fetch the live planning scene as the start state. First stage of an online task. | none |
| `FixedState` | `fixed_state.h:48` | Generator | Seed a hand-built scene (offline planning, tests). | a `PlanningScene` |
| `GeneratePose` | `generate_pose.h:49` | Monitoring generator | Base class; emit a single fixed `pose` as a `target_pose`, stamped in any frame. | `pose`; a monitored stage |
| `GenerateGraspPose` | `generate_grasp_pose.h:47` | Monitoring generator | Sample grasp candidates by rotating about an axis of the object frame. Reach for it to pick. | `eef`, `object`, `angle_delta`, `pregrasp`; monitored stage |
| `GeneratePlacePose` | `generate_place_pose.h:51` | Monitoring generator | Given a target pose for the *held object*, emit a `target_pose` for the ik frame. Reach for it to place. | `object`, `pose`; monitored stage |
| `GenerateRandomPose` | `generate_random_pose.h:49` | Monitoring generator | Perturb a base pose with per-dimension samplers. | `pose`, samplers |
| `FixedCartesianPoses` | `fixed_cartesian_poses.h:49` | Monitoring generator | Emit a fixed list of Cartesian poses as targets. | poses; monitored stage |
| `ComputeIK` | `compute_ik.h:68` | Wrapper (over a generator) | Turn Cartesian `target_pose`s from its child into joint states via IK. It is a *wrapper*, not a stage you feed poses to directly. See frames section. | wraps a pose generator; `group`/`eef`, `ik_frame` |
| `MoveTo` | `move_to.h:56` | Propagator (either way) | Move a group to a joint-named pose, a joint map, a `PoseStamped`, or a `PointStamped`. Reach for it for "go to home", "open hand". | `group`, `goal`, a planner |
| `MoveRelative` | `move_relative.h:57` | Propagator (either way) | Cartesian relative motion along a `Vector3Stamped`/`TwistStamped`, or a joint delta. Reach for it for approach, lift, retreat. | `group`, `direction`, a planner |
| `Connect` | `connect.h:64` | Connector | Plan a free-space transit between two states, per-group planners, then try to merge. Reach for it between a start state and a generated goal. | `GroupPlannerVector` |
| `ModifyPlanningScene` | `modify_planning_scene.h:65` | Propagator (either way) | Attach/detach objects, add/remove objects, edit the allowed-collision matrix. No robot motion. Reach for it around a grasp. | the edits you schedule |
| `FixCollisionObjects` | `fix_collision_objects.h:50` | Propagator (either way) | Nudge objects out of shallow collision in the input scene. Reach for it when perceived objects overlap geometry. | `max_penetration` |
| `PredicateFilter` | `predicate_filter.h:56` | Wrapper | Pass or reject a child's solutions by a `bool(solution, comment)` predicate; rejects become annotated failures. Reach for it for applicability checks. | wraps a child; `predicate` |
| `PassThrough` | `passthrough.h:49` | Wrapper | Forward a child's solution unchanged, but let you re-cost it via `setCostTerm` without losing the original. | wraps a child |
| `LimitSolutions` | `limit_solutions.h:47` | Wrapper | Forward only the best N of a child's solutions. Reach for it to cap a prolific generator. | wraps a child; `max_solutions` |
| `NoOp` | `noop.h:50` | Propagator (either way) | Do nothing to the scene; carries custom properties for post-planning execution steering. | none |
| `Pick` / `Place` | `pick.h:92,109` | Serial container | Prebuilt approach + grasp posture + attach + lift (and the inverse for place), wrapping a grasp stage. A convenience assembly, not a primitive. | `eef`, `object`, motions |

The ones that surprise people:

- **`Connect` writes nothing outward.** It does not create a goal; it bridges two
  states that other stages already produced. If your `Connect` finds nothing,
  check that both its neighbours actually produce states on the facing sides.

- **`ComputeIK` is a wrapper, not an IK stage you call.** You give it a *child*
  pose generator; it reads that child's `target_pose` from the interface and runs
  IK (`compute_ik.cpp:246`). You almost never set `target_pose` on it directly;
  the header says as much (`compute_ik.h:95-99`).

- **`ModifyPlanningScene` is a full stage**, not a side effect. Attaching an
  object, permitting a collision pair, and forbidding it again are three separate
  stages placed at the right points in the chain (see `pick_place_task.cpp:245-306`).
  Because it is a propagator, its backward form *inverts* the edit, which is what
  lets a place sequence be inferred backward from the placed state.

- **`PassThrough` and `PredicateFilter` exist so you never subclass a stage just
  to re-cost or filter it.** Reach for them before writing a custom wrapper.

### Planners (the "how", decoupled from the "where")

Stages say *where* to move; a `planner` object says *how* (dissertation p44).
Construct these once and share them (`solvers.h`):

- `solvers::JointInterpolationPlanner` (`solvers/joint_interpolation.h:54`):
  joint-space interpolation, cheap, ignores the environment.
- `solvers::CartesianPath` (`solvers/cartesian_path.h:51`): straight-line
  Cartesian path, for approach/lift/retreat.
- `solvers::PipelinePlanner` (`solvers/pipeline_planner.h:59`): a full MoveIt
  planning pipeline (OMPL, Pilz, etc.), for free-space transit.
- `solvers::MultiPlanner` (`solvers/multi_planner.h`): try several planners in
  sequence.

---

## 3. Containers and control flow

A container is itself a stage with children (`container.h:48`). Its interface is
derived from its children, never set directly. That is the trap to internalize:
**you shape a container's behaviour by choosing its children and their order, not
by configuring the container.**

- **`SerialContainer`** (`container.h:95`): chain children so each child's end
  state is the next child's start state (dissertation p46). A single new child
  solution can spawn several container solutions if it joins several existing
  partial paths. Group a semantically meaningful phase ("pick object") here.

- **`Alternatives`** (`container.h:148`): run all children on the same problem and
  report every child's solutions, sorted by cost (dissertation p48). Reach for it
  to **explore several strategies in parallel with no preference**: two arms that
  could both grasp, or two grasp families you want ranked against each other.

- **`Fallbacks`** (`container.h:166`): try children in order; a later child is
  attempted only where the earlier one failed (dissertation p49). Reach for it for
  **cheap-first, expensive-second**: direct joint interpolation, then fall back to
  sampling-based planning; or a direct grasp, then a tool-assisted one. Note the
  subtlety at `container.h:183-184`: `Fallbacks` overrides its scheduling in
  `FallbacksPrivate`, so its own `canCompute`/`compute` are inert. It is the only
  container that gates *whether* a child runs at all, and it can re-attempt through
  a child's *failures*, not only its successes (dissertation p49).

- **`Merger`** (`container.h:189`): plan children for *disjoint joint groups* in
  parallel and merge their trajectories into one for simultaneous execution
  (dissertation p50). Only valid when the groups do not interfere. Reach for it to
  move an arm and a separate mechanism at once, not to pick between options.

Choosing between the two parallel forms: **`Alternatives` when you want every
strategy's solutions ranked together; `Fallbacks` when there is a clear preference
order and you would rather not pay for the expensive branch unless the cheap one
fails.**

---

## 4. Frames (read this section twice)

Frame confusion is the most expensive class of authoring error, and the usual
place a task accidentally welds itself to a single grasp orientation. Three
distinct frame ideas routinely get conflated.

### 4.1 The IK frame is the frame you are solving *for*

`ComputeIK`, `MoveRelative`, and `MoveTo` each take an `ik_frame`
(`compute_ik.h:84-93`, `move_relative.h:67-75`, `move_to.h:66-72`). The IK frame is
the frame that gets placed at the target. The solver finds joint values such that
*this frame* reaches the goal. The link that IK actually solves is the rigidly
connected parent link of the ik frame, and the target is rewritten so the ik frame
lands on the goal (`compute_ik.cpp:318-321`):

```
target_pose = target_pose * ik_pose.inverse() * frameTransform(link)
```

The consequence is concrete and decisive:

- **Set the IK frame to the tool (gripper) frame, and IK solves for where the
  gripper goes.** The object, if attached, follows wherever the gripper's
  kinematics put it.
- **Set the IK frame to an attached object frame, and IK solves for where the
  object goes.** The gripper is then wherever it must be to place the object
  there. This is exactly how placing works: `GeneratePlacePose` emits a target for
  the held object, and the ik frame is set so IK solves the object onto the target
  (`generate_place_pose.cpp:106-130`, and the demo at `pick_place_task.cpp:376-380`).

In the demo, `setIKFrame(grasp_frame_transform, hand_frame)`
(`pick_place_task.cpp:233,378`) offsets the ik frame from the hand link by a fixed
transform. That transform, not the pose generator, is what fixes the grasp
orientation relative to the object.

### 4.2 A stage's target-pose frame is where the goal is expressed

`target_pose` is a `PoseStamped`; its `header.frame_id` names the frame the goal
is expressed in. `GenerateGraspPose` stamps its sampled `target_pose` in the
*object* frame (`generate_grasp_pose.cpp:166`) and samples by rotating about
`rotation_axis`, default the object's z (`generate_grasp_pose.cpp:59,170-177`).
`ComputeIK` resolves an empty frame id to the planning frame and otherwise
transforms through the scene (`compute_ik.cpp:280-291`). So the grasp *family*
(what the candidate poses are) comes from the generator and its axis; the grasp
*orientation* (top-down, side-on) comes from the ik-frame offset in 4.1. These are
independent knobs, and mixing them up is a common error.

### 4.3 A MoveRelative direction's frame decides whether the motion is tool-relative

`MoveRelative::setDirection` takes a `Vector3Stamped` or `TwistStamped`
(`move_relative.h:90-94`). The direction vector is rotated into the world by the
orientation of its own `header.frame_id`
(`move_relative.cpp:229,261,288`):

```
linear = frameTransform(direction.header.frame_id).linear() * linear
```

This is the load-bearing distinction:

- **Stamp the direction in a world or container frame** (for example
  `world` with `z = 1`) and the motion is world-vertical *regardless of how the
  tool is oriented*. Every grasp that uses it is thereby constrained to be
  compatible with a straight-up lift, which in practice means top-down. The demo
  lifts and lowers in `world` z on purpose (`pick_place_task.cpp:291-295,349-353`).
- **Stamp the direction in the tool frame** (for example `hand_frame` with
  `z = 1`) and the motion follows the tool's own orientation. The demo approaches
  along `hand_frame` z (`pick_place_task.cpp:207-211`) and retreats along
  `hand_frame` z (`pick_place_task.cpp:422-425`), so those motions stay correct for
  any grasp orientation.

The general lesson: **a direction stamped in a fixed world or object frame makes
the motion independent of tool orientation but couples every grasp to that fixed
direction; a direction stamped in the tool frame follows the tool and imposes no
such coupling.** If a task only ever produces top-down grasps, look first for a
world-vertical `MoveRelative` direction that is silently demanding it. Where a
task encodes that assumption across several stages — lift, place, and retreat all
stamped in world z — moving to angled grasps is not a one-line edit: every such
direction, and the collision and reach assumptions built on top of it, has to move
together.

---

## 5. Properties

Properties are the typed parameter store on every stage
(`core/include/moveit/task_constructor/properties.h`). A stage *declares* the
properties it understands (`PropertyMap::declare`, `properties.h:267-277`); setting
an undeclared property on a stateful stage will not be read.

### Inheritance and forwarding

A property can be initialized from one of several sources
(`stage.h:158-164`):

```
DEFAULT = 0, MANUAL = 1, PARENT = 2, INTERFACE = 4  // INTERFACE beats PARENT
```

- **From the parent container**: `stage->properties().configureInitFrom(Stage::PARENT, {"group", "eef"})`
  (`properties.h:301`) tells the stage to take those values from its parent if not
  set locally. First you must expose the values onto the parent, usually
  `task.properties().exposeTo(child->properties(), {...})` (`properties.h:280-283`),
  which the demo does at `pick_place_task.cpp:193-194`. Values are searched upward
  through nested containers (dissertation p47), so a property set on the `Task`
  reaches a deeply nested stage as long as each level is marked to inherit it.

- **From the interface**: `configureInitFrom(Stage::INTERFACE, {"target_pose"})`
  makes a stage read that property off the incoming interface state, which is how
  `ComputeIK` receives `target_pose` from its child's solution
  (`pick_place_task.cpp:235-236`, `compute_ik.cpp:246`). `INTERFACE` outranks
  `PARENT`.

- **Forwarding across a stage**: `setForwardedProperties({...})` /
  `forwardProperties(src, dst)` (`stage.h:214-219`) copies named properties from an
  incoming interface state to the outgoing one, so a value set upstream survives
  through a stage that does not itself consume it.

The practical recipe, from the demo: set shared values once on the `Task`
(`t.setProperty("group", ...)`, `pick_place_task.cpp:128-132`), `exposeTo` a
container, and `configureInitFrom(Stage::PARENT, ...)` on each child. Do not repeat
the same value on every stage by hand.

---

## 6. Cost and solution selection

Cost is what ranks solutions and what steers the search toward promising branches
(dissertation p55). Cost terms live in
`core/include/moveit/task_constructor/cost_terms.h`.

- A bare `SolutionBase` has cost `0` by default (`storage.h:331`). Motion stages
  set their own default: both `MoveRelative` and `MoveTo` default to
  `cost::PathLength` (`core/src/stages/move_relative.cpp:54`,
  `core/src/stages/move_to.cpp:58`). `Connect` also defaults to `PathLength`
  (`core/src/stages/connect.cpp:64`). `ComputeIK` costs each solution as joint
  distance from a comparison pose (`compute_ik.cpp:436`).
- A `SerialContainer`'s cost is the sum of its children's costs unless overridden
  (dissertation p56).
- Override any stage's cost with `setCostTerm` (`stage.h:230-235`), which accepts a
  `CostTerm` or a lambda of signature `double(const SubTrajectory&, std::string&)`
  (`cost_terms.h:85-109`).

Prebuilt terms (`cost_terms.h`): `Constant` (114), `PathLength` (127),
`DistanceToReference` (144), `TrajectoryDuration` (161), `LinkMotion` (169),
`LinkRotation` (181), `Clearance` (200, inverse distance to collision).

To influence *which* solution wins: choose the cost term that matches the
objective (shortest execution -> `TrajectoryDuration`; most clearance ->
`Clearance`; least tool travel for a tool-use task -> `LinkMotion`). To *change* a
solution's cost without hiding its original, wrap it in `PassThrough` and set a
cost term there (`passthrough.h:44-48`). To bound the search, cap a prolific stage
with `LimitSolutions` (`limit_solutions.h:47`) or `ComputeIK::setMaxIKSolutions`
(`compute_ik.h:109`); enumerating more task solutions lowers the best observed cost
but costs planning time (dissertation p69-70).

---

## 7. Debugging a task

### Read the tree

`task.printState(os)` prints per-stage counts of solutions and propagated states
(`task.h:141`), and `task.explainFailure(os)` prints an explanation for a planning
failure (`task.h:144`). Enable introspection (`task.h:106-107`) and MTC publishes
the full tree to the RViz Motion Planning Tasks panel, which shows, per stage, the
success and failure counts and lets you click any solution (including failures) to
visualize it. Stages attach geometric markers to solutions (target frames, the ik
frame, approach arrows) precisely so failures are inspectable
(`compute_ik.cpp:330-333`, `move_relative.cpp:110-165`).

### Tell the failure modes apart

- **"No solution" and every stage shows 0/0 (no successes, no failures):** a stage
  upstream never produced states, so nothing downstream ever ran. Usual cause: a
  monitoring generator with no `setMonitoredStage`, or no generator seeding the
  chain at all. This is "the interface is never satisfied" at run time.
- **`InitStageException` at `init()`:** the tree is structurally impossible; the
  composed interfaces do not fit. Fix the stage ordering or the missing
  generator/connector, not the parameters.
- **A stage shows many failures and 0 successes:** the stage ran but every attempt
  was rejected. Click a failure in the panel and read its comment.
  - `ComputeIK` failures read "no IK found", "Collision between ...", or
    "Constraints violated" (`compute_ik.cpp:438-469`). "eef in collision" means the
    end-effector geometry collided at the target before IK was even attempted
    (`compute_ik.cpp:351`).
  - `MoveRelative` failures read "min_distance not reached" or "failed to move full
    distance" (`move_relative.cpp:326,332`); the red segment of the arrow marker is
    the part it could not achieve.
  - `Connect` producing nothing usually means intrinsically infeasible pairs,
    often an orientation path constraint that forbids the needed rotation
    (dissertation p39-40).
- **Solutions exist but the wrong one wins:** a cost-term problem, not a structure
  problem. Section 6.

The heuristic: 0/0 means a wiring or seeding fault (structure); many-failures/0
means a geometry or reachability fault (parameters or scene); solutions-but-wrong
means a cost fault.

---

## 8. A worked skeleton

The minimal correct shape of a pick-and-place-like task. Each stage is present for
a reason; the order is the point. Full example: `demo/src/pick_place_task.cpp`;
the concise stage tree is dissertation Figure 3.8 (p67).

```
Task ("pipeline")                     // SerialContainer root
├─ CurrentState                       // generator: seed with the live scene
│   └─ (wrapped in PredicateFilter)   // reject if the object is already attached
├─ MoveTo "open hand"                 // forward propagator: prepare the gripper
├─ Connect "move to pick"             // connector: transit to the grasp (bridges
│                                     //   the open-hand state and the grasp state)
├─ SerialContainer "pick object"      // one semantic phase
│   ├─ MoveRelative "approach"        // backward: Cartesian approach, tool-frame dir
│   ├─ ComputeIK "grasp IK"           // wrapper over ...
│   │   └─ GenerateGraspPose          //   monitoring generator (monitors CurrentState)
│   ├─ ModifyPlanningScene "allow (hand,object)"   // permit contact before closing
│   ├─ MoveTo "close hand"            // forward: grasp posture
│   ├─ ModifyPlanningScene "attach object"         // object joins the robot model
│   └─ MoveRelative "lift"            // forward: lift, world-frame dir (see 4.3)
├─ Connect "move to place"            // connector: transport transit
├─ SerialContainer "place object"     // inverse of pick
│   ├─ MoveRelative "lower"           // world-frame dir
│   ├─ ComputeIK "place IK"           // wrapper over ...
│   │   └─ GeneratePlacePose          //   monitors the pick container's solutions
│   ├─ MoveTo "open hand"
│   ├─ ModifyPlanningScene "detach object"
│   └─ MoveRelative "retreat"         // tool-frame dir
└─ MoveTo "move home"                 // forward: return
```

Why each is there: the generator seeds; the connectors carry transit and are where
forward and backward frontiers meet; the pose generators are *monitoring* so the
grasp/place commitment reaches them past the connectors; `ComputeIK` turns their
Cartesian targets into joint states; the `ModifyPlanningScene` stages toggle the
model and collisions at the exact moments the physics changes; and the approach /
lift / retreat directions are stamped in tool or world frames per section 4.3.

---

## 9. Pitfalls, grounded in real code

Framed as general lessons. Where a file in this repository illustrates one, it is
cited as an instance, not a critique of that file.

1. **A world-vertical `MoveRelative` direction constrains every grasp to be
   top-down.** Because the direction is rotated only by its stamped frame's
   orientation (`move_relative.cpp:261`), a lift stamped in `world` z demands that
   the grasp be compatible with a straight-up motion. A task whose lift, place, and
   retreat are all stamped that way has made "tool approach axis is near
   world-vertical" a precondition without ever saying so. The fix for angled grasps
   is to stamp such directions in the tool frame — but only where the task's
   collision and reach assumptions still hold, which is why it is a design change
   rather than a substitution.

2. **Forgetting `setMonitoredStage` makes a pose generator silently dead.**
   `GeneratePose::canCompute` is false with no monitored solution
   (`generate_pose.cpp:67-70`). The task finds nothing and shows 0/0; there is no
   error. Always wire a monitoring generator to the stage whose scene it needs
   (`pick_place_task.cpp:226,373`).

3. **Confusing the IK frame with the target-pose frame.** The ik frame is what you
   solve *for* (section 4.1); the target-pose frame is where the goal is *expressed*
   (section 4.2). Setting the ik frame to the object when you meant the tool
   silently solves the object onto the target and puts the gripper somewhere
   unexpected.

4. **Setting `target_pose` directly on `ComputeIK`.** It is a wrapper; it reads
   `target_pose` from its child's interface (`compute_ik.h:95-99`,
   `compute_ik.cpp:246`). Feeding it a pose directly instead of giving it a pose
   generator child is fighting the design.

5. **Treating scene edits as free side effects.** Attaching an object, allowing a
   collision, and forbidding it again are separate `ModifyPlanningScene` stages
   placed at exact points (`pick_place_task.cpp:245-306`). Miss one and IK either
   rejects a legitimate grasp as a collision or accepts a motion that is physically
   a collision.

6. **Expecting a `Connect` to create a goal.** It only bridges states other stages
   produce (`connect.h:56-63`). If it is empty, the fault is a neighbour that
   produces no facing state, not the `Connect`.

7. **Leaving pruning on when a stage is meant to fail often.** Pruning removes
   branches that cannot complete (dissertation p58), which is usually right, but if
   an early stage legitimately fails on most inputs and you still want later stages
   explored, `setPruning(false)` on the container (`container.h:55`).

8. **Re-declaring shared properties on every stage.** Use
   `exposeTo` + `configureInitFrom(Stage::PARENT, ...)` (section 5). Hand-copying a
   value onto ten stages is how two of them drift apart.

---

## 10. What MTC does not provide (say it plainly)

- **No execution-time state machine or reactive control.** MTC plans complete
  trajectories offline; failure handling, replanning, and reactive policies are out
  of scope (dissertation p63). If you need runtime branching on sensor input, that
  lives above MTC, not inside a stage.
- **No built-in grasp synthesis beyond sampling.** `GenerateGraspPose` rotates a
  single prototype grasp about one axis (`generate_grasp_pose.cpp:170-177`). Real
  grasp planning (learned or geometric) is an external generator you plug in, not a
  stage MTC ships.
- **No automatic frame or grasp-orientation inference.** Nothing derives "grasp
  this from the side"; orientation is the ik-frame offset you supply (section 4.1).
- **No dynamics across phase boundaries.** Phases return to rest between segments
  unless you add the path-reparameterization wrapper (dissertation p51-52), which
  is a post-process, not automatic.
- **No cost term that is minimized inside a planner.** Cost ranks completed
  solutions and steers the search; it does not inform a stage's internal planner,
  which is a black box (dissertation p55). You cannot express "find the
  minimum-cost IK" through a cost term alone; use `max_ik_solutions` and rank.
- **No merge of trajectories that share joints.** `Merger` is only valid for
  disjoint joint groups (dissertation p50). Two arms in the same workspace are not
  automatically deconflicted.

---

## Appendix: how the pieces map to the source

- Execution model and interfaces: `stage.h`, `container.h`, `storage.h`;
  dissertation Ch. 3.2.
- Properties: `properties.h`; dissertation 3.2.5, 3.3.2, 3.4.2.
- Cost: `cost_terms.h`; dissertation 3.5.
- Solvers: `solvers.h` and `solvers/`; dissertation 3.2.6.
- Worked task: `demo/src/pick_place_task.cpp`; dissertation 3.9.1 and Figure 3.8.
- Driving MTC from a behavior tree instead of C++: the `MoveIt.BT` repository and
  its own `docs/authoring_guide.md`.
