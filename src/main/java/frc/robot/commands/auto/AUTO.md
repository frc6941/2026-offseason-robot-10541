# Autonomous — dashboard options reference

This is the field manual for the **Auto** dropdowns on SmartDashboard/Elastic. It lists
every chooser, every option it can take, what the option *means*, and what the robot
actually *does* as a result.

All 14 choosers are always visible, but **most of them are ignored by most routines.**
Start with `Auto/Routine`, then set only the knobs that routine reads (see the matrix
below). Everything else is left at its default and has no effect.

---

## Which knobs each routine reads

| `Auto/Routine` | Knobs that matter | Everything else |
|----------------|-------------------|-----------------|
| **Do Nothing** | *(none)* | ignored |
| **Drive Forward** | *(none)* | ignored |
| **Depot Collect** | `Depot Axis` | ignored |
| **Mid Step Only** | `First Mid Mode`, `First Mid Direction`, `First Mid Kind` | ignored |
| **Mid Sweep** ⭐ (Mid Two Cycle) | **all** First/Second Mid + Shoot Position + `Depot Axis` + `Depot Round` + `Outpost Round` | — |
| **Go To Target** | `Target` | ignored |
| **Trench Clear** | `Side` | ignored |
| **Bump Cross** | `Side` | ignored |
| **Depot Through** | `Side` | ignored |

> The `Second Mid *` and `Shoot Position` choosers **only** affect **Mid Sweep**. The
> single-sweep **Mid Step Only** routine reuses just the *First* set.

---

## `Auto/Routine` — the top-level pick

*Default: **Mid Sweep** (Mid Two Cycle).*

| Option | What it does | Uses pathfinding? |
|--------|--------------|-------------------|
| **Do Nothing** | Robot sits still the entire auto period. | no |
| **Drive Forward** | Resets pose to the alliance-flipped center start, drives straight forward at 1 m/s for 2 s, stops. This is the **safe fallback** — it still works if PathPlanner can't generate a path. | no |
| **Depot Collect** | Drives to the depot and intakes a load of balls (approach set by `Depot Axis`). | yes |
| **Mid Step Only** | Runs **one** neutral-zone sweep (using the *First Mid* knobs) with the intake on, then ends. No shooting. | yes |
| **Mid Sweep** ⭐ | The full two-cycle routine: **sweep → shoot → sweep → shoot**, with optional depot/outpost detours. The whole thing is capped at `autoDurationSeconds` (default 20 s). | yes |
| **Go To Target** | Pathfinds to a single named location (set by `Target`) and stops. | yes |
| **Trench Clear** | Drives from the trench start to the trench-clear point on one side (`Side`). | yes |
| **Bump Cross** | Crosses the bump from inner to outer on one side (`Side`). | yes |
| **Depot Through** | Drives *through* the depot on one side (`Side`) with the intake running. | yes |

---

## The Mid-Sweep knobs

**Mid Sweep** runs two cycles. Each cycle = *sweep the neutral zone to collect balls* →
*go to a launch spot and score*. You configure the two cycles independently (First / Second).

### `Auto/First Mid Mode` and `Auto/Second Mid Mode` — the sweep shape

*Default: both **Full Pool Sweep** (SALESMAN).*

This is the **most important** choice: it sets the path the robot drives through the
neutral zone, which decides *which* balls it can reach and *how far downfield / how wide*
it commits. Enum name → dashboard label → behavior:

| Dashboard label (enum) | Waypoints | Reach & shape | Use when |
|------------------------|-----------|---------------|----------|
| **Full Pool Sweep** (`SALESMAN`) | edge → center → center → edge (4) | Full width, edge-to-edge across the fuel pool. Most coverage, most time. | You want maximum pickup and have the time. |
| **Full Pool Turn-In Sweep** (`SALESMAN_TURN`) | turn-in → center → center → turn-in (4) | Like Full Pool but starts offset/inset so the robot turns into the pool. | Full coverage with a cleaner entry angle. |
| **Safe Inner Sweep** (`CONSERVATIVE`) | near-corners (2), inset 0.35 m | Short sweep close to your own side. Low risk. | Time-limited or avoiding traffic. |
| **Center Line Sweep** (`NEUTRAL`) | left-center → right-center (2) | Straight across the middle of the pool. | Quick center grab. |
| **Near Tower Sweep** (`FLIGHTLESS`) | near-tower points (2) | Hugs the alliance/neutral boundary near the tower — minimal incursion. | Defensive / stay close to home. |
| **Near Tower Wide Sweep** (`FLIGHTLESS_WIDE`) | near-tower x, out to edge y (2) | Same nearness as Near Tower, but stretched wall-to-wall. | Wide pickup while staying shallow. |
| **Near Tower Wave Sweep** (`FLIGHTLESS_WAVE`) | near-tower points, **weaving** (amplitude 0.35 m) | Sinusoidal weave near the tower — covers more lateral area than a straight near-tower line. | Sweep up scattered balls near home. |
| **Far Edge Sweep** (`DAVIS`) | far edges (2), ~0.75 m past field center | Aggressive: reaches into the far half at the edges. | Contest far balls, accept the risk. |
| **Far Edge + Center Sweep** (`DAVIS_FRIENDSHIP`) | far-edge → center → center → far-edge (4) | Far reach **plus** the centers — big, greedy path. | Grab far and central balls in one pass. |
| **Back Center Sweep** (`CORIOLIS`) | ~0.75 m *before* center, center y (2) | Pulls from just short of the center line. | Safe-ish central pickup. |
| **Mid-Back Center Sweep** (`CENTER_FORWARD`) | ~0.35 m before center, center y (2) | Slightly further forward than Back Center. | A touch more reach than Back Center. |
| **Center Wave Sweep** (`WAVE`) | center points, **weaving** (amplitude 0.65 m) | Big sinusoidal weave across the center — max lateral coverage of the mid pool. | Sweep up a spread-out center pool. |

Rules of thumb: rows higher in the table = **more coverage / more time**; "Near Tower"
rows = **shallow & safe**; "Far Edge / Davis" rows = **deep & aggressive**; "Wave" rows =
**weave to catch scattered balls**.

### `Auto/First Mid Direction` and `Auto/Second Mid Direction`

*Default: First = **Left To Right**, Second = **Right To Left**.*

| Option | What it does |
|--------|--------------|
| **Left To Right** | Sweeps starting from the left; the robot faces **−90°** (intake leading toward the right). |
| **Right To Left** | Sweeps starting from the right; the robot faces **+90°** (intake leading toward the left). |

The default has the two cycles go **opposite directions**, so cycle 2 naturally sweeps
back across what cycle 1 didn't finish.

### `Auto/First Mid Kind` and `Auto/Second Mid Kind`

*Default: both **Full**.*

| Option | What it does |
|--------|--------------|
| **Full** | Drives the entire waypoint list for the chosen mode. |
| **Half** | Sweeps only to the midpoint of the path, then (for non-"Near Tower" modes) drives a short leg back toward the home tower. Use to grab **half** the zone and return sooner. |

### `Auto/First Shoot Position` and `Auto/Second Shoot Position`

*Default: First = **Right**, Second = **Left**.*

After each sweep the robot goes to a **bump launch spot** to score. This picks which one:

| Option | What it does |
|--------|--------------|
| **Left** | Score from the **left** bump launch pose. |
| **Right** | Score from the **right** bump launch pose. |

*(This chooser is ignored on any cycle that instead detours to the depot or outpost — see below.)*

### `Auto/Depot Round` — insert a depot pickup

*Default: **No Depot**.* Picks **when**, if ever, Mid Sweep detours to grab a depot load.

| Option | Result |
|--------|--------|
| **No Depot** | Never visit the depot. |
| **Depot Start** | Collect from the depot **before** the first sweep. |
| **Depot Round 1** | End of cycle **1**: go to the bump launch, then shoot-on-move toward the depot and intake through it (instead of a plain shoot). |
| **Depot Round 2** | Same, but at the end of cycle **2**. |

### `Auto/Outpost Round` — insert an outpost score

*Default: **No Outpost**.* Picks **when**, if ever, a cycle scores at the outpost instead
of the bump.

| Option | Result |
|--------|--------|
| **No Outpost** | Never detour to the outpost. |
| **Outpost Round 1** | End of cycle **1**: approach the outpost and shoot-on-move there. |
| **Outpost Round 2** | Same, at the end of cycle **2**. |

> ⚠️ **Interaction rules** (enforced in `AutoSelector`): the "Outpost Start" option does
> not exist, and if the **outpost round equals the depot round**, the outpost is
> **suppressed** (the depot detour wins that cycle). So a single cycle can end in exactly
> one of: normal shoot, depot detour, or outpost detour.

---

## `Auto/Depot Axis` — how to enter the depot

*Default: **X**.* Read by **Depot Collect** and by any depot detour in **Mid Sweep**.

| Option | What it does |
|--------|--------------|
| **X** | Approach along the **X** axis, robot facing 180°. Start pose is offset +1 m in x from the depot; intake while driving in to the depot end. |
| **Y** | Approach along the **Y** axis, robot facing −90°. Enter the depot from the side and intake to the depot center. |

Pick whichever geometry matches where the robot is coming from.

---

## `Auto/Side` — left/right variant

*Default: **Left**.* Read by **Trench Clear**, **Bump Cross**, and **Depot Through**.

| Option | Trench Clear | Bump Cross | Depot Through |
|--------|--------------|------------|---------------|
| **Left** | Left trench start → left clear | Left bump inner → outer | Drive through the left depot, intaking |
| **Right** | Right trench start → right clear | Right bump inner → outer | Drive through the right depot, intaking |

---

## `Auto/Target` — the Go-To-Target destination

*Default: **Outpost**.* Read only by **Go To Target**. Each option pathfinds to a named
pose and stops (heading settled on arrival).

| Option | Destination |
|--------|-------------|
| **Outpost** | The outpost scoring pose. |
| **Hub Center Start** | The center starting spot in front of the hub (facing the hub). |
| **Left Bump Launch** / **Right Bump Launch** | The left/right bump launch pose, pre-aimed at the hub. |
| **Left Trench Launch** / **Right Trench Launch** | The left/right trench launch pose, pre-aimed at the hub. |
| **Left Climb** / **Right Climb** | The left/right climb pose at the tower. |
| **Left Tower Through** / **Right Tower Through** | A pass-through point beside the left/right tower upright. |

---

## Worked examples

**"Simplest safe auto that always works"** → `Routine = Drive Forward`. Nothing else matters.

**"Grab one quick load and stop"** → `Routine = Mid Step Only`, `First Mid Mode = Center
Line Sweep`, `First Mid Kind = Full`, `First Mid Direction = Left To Right`.

**"Two full cycles, score both at the bumps, no detours"** (the default) →
`Routine = Mid Sweep`, leave everything at default (`Full Pool Sweep` ×2, opposite
directions, shoot Right then Left, `No Depot`, `No Outpost`).

**"Two cycles, but reload from the depot after cycle 1"** → `Routine = Mid Sweep`,
`Depot Round = Depot Round 1`, `Depot Axis = X`. Cycle 1 ends with a depot pickup; cycle 2
ends with a normal bump shot.

**"Aggressive far-reach first, safe near-tower second, score cycle 2 at the outpost"** →
`Routine = Mid Sweep`, `First Mid Mode = Far Edge Sweep`, `Second Mid Mode = Near Tower
Sweep`, `Outpost Round = Outpost Round 2`.

---

## Where each choice lands in code (if you need to dig)

- Options and their friendly labels: **`AutoSelector.java`** (`configureChoosers`,
  `midModeLabel`, the `enum`s at the bottom).
- Dashboard keys (`Auto/…`): `AutoSelector.publishChoosers`.
- What each option actually *does*: **`AutoCommands.java`** (e.g. `midTwoCycle`,
  `handlePostSweepAction`, `neutralZoneSweep`, `neutralSweepPoints`).
- The field coordinates every option aims at: **`AutoPoints.java`**.
- The speeds/timeouts/tolerances behind the motion: **`AutoParams.java`** (live-tunable
  over NetworkTables under `Params/AutoCommands` and `Params/AutoBuilder`).
