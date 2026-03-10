# Joint Slider Architecture — Why It's Built This Way

This document records every freeze root cause encountered during development and the
fix that resolved it. Each section is a distinct bug, not a symptom of the same bug.

---

## Bug 1 — `updateNSView` never called across NavigationSplitView columns

**Symptom:** Arm never moved at all after slider drag.

**Cause:** SwiftUI's `updateNSView` is only called during a **default run-loop mode**
re-render. AppKit enters `NSEventTrackingRunLoopMode` for the entire duration of a
mouse drag. SwiftUI's render scheduler is suspended during event-tracking, so
`updateNSView` is never invoked while the slider is held down.

**Fix:** A 120 Hz `Timer` added to `RunLoop.main` with `.common` mode. `.common`
includes both default and event-tracking modes, so the timer fires continuously
regardless of what the run loop is doing.

---

## Bug 2 — `SCNTransaction` implicit 0.25 s animation fighting joint writes

**Symptom:** Arm moved but lagged 250 ms behind the slider, then appeared frozen
during fast drags.

**Cause:** SceneKit maintains its own animation system (`SCNTransaction`) separate from
CoreAnimation (`CATransaction`). The default implicit transaction duration is 0.25 s.
Every `simdTransform` write during a drag started a new 0.25 s animation from the
current *presentation* position to the new *model* position. During fast drags the
presentation tree never converged.

**Fix:** All `simdTransform` writes moved inside `renderer(_:updateAtTime:)` on the
CVDisplayLink thread. Apple guarantees writes inside this callback go directly to the
presentation layer, bypassing the implicit animation pipeline entirely — no
`SCNTransaction.begin/commit` needed or wanted.

---

## Bug 3 — Deadlock: `buildRobotNodes` on main thread vs CVDisplayLink render lock

**Symptom:** App hung (spinning beachball) when loading a URDF while the scene was
rendering.

**Cause:** SceneKit holds an internal render lock on the CVDisplayLink thread during
each frame. `scene.rootNode.addChildNode(...)` on the main thread tried to acquire
the same lock → deadlock.

**Fix:** `buildRobotNodes` moved into `renderer(_:updateAtTime:)` (CVDisplayLink
thread). Scene graph mutations now always happen while SceneKit already holds its own
lock, so no contention is possible.

---

## Bug 4 — SwiftUI Binding reset fighting AppKit mouse-tracking

**Symptom:** Joints froze after ~2–15 seconds of sustained drag. Values in the
renderer log stopped changing even while the mouse was held down.

**Cause:** `@Published var jointAngles` fired `objectWillChange` on every slider write
(up to 120×/s). SwiftUI re-rendered `JointSliderPanel`, calling `updateNSView` on
every `ContinuousNSSlider`. The `updateNSView` implementation wrote
`nsView.doubleValue = value` when the stored value differed from the slider's current
position by more than `1e-9`. During a drag the slider had advanced ahead of the
last-committed `appState.jointAngles` value, so **every re-render reset the slider
backward**. The slider's `valueChanged` action wrote the reset value back, making
`jointAngles` appear frozen.

**Fix (partial):** Removed the `nsView.doubleValue = value` write from `updateNSView`
entirely. The slider position is owned by AppKit; `makeNSView` sets the correct
initial value once.

---

## Bug 5 — Collision checker triggering 120 Hz SwiftUI re-renders

**Symptom:** Joints froze again after adding collision detection. The renderer log
showed values freeze mid-drag accompanied by occasional catch-up jumps.

**Cause:** The 120 Hz timer computed a collision result and wrote
`state.collisionResult = collision` every tick. `collisionResult` was `@Published`,
so `objectWillChange` fired 120×/s. SwiftUI re-rendered `JointSliderPanel` 120×/s,
calling `updateNSView` on every slider. Even without writing `doubleValue`, the sheer
volume of `updateNSView` calls starved AppKit's mouse-tracking event loop, causing
periodic input drops.

**Fix:** Two changes:
1. `CollisionResult` publishing throttled to ≤10 Hz in the timer.
2. `@Published` removed from `jointAngles` — plain `var`, no `objectWillChange` on
   write. The 3-D view polls it directly via the timer; nothing needs SwiftUI
   reactivity for per-frame angle updates.

---

## Bug 6 — NSSlider.target weak reference going nil during NSTableView cell reuse

**Symptom:** Joints froze one-by-one, progressively, during a session. Each new drag
had a chance of permanently freezing that joint. Eventually all joints were frozen.

**Cause:** `JointSliderPanel` used a SwiftUI `List` (backed by `NSTableView`). Inside
each row, `JointRow` held a `@State private var displayAngle` that was written on
every slider tick via an `onChange` callback. This caused `JointRow.body` to be
re-evaluated at up to 120 Hz per row being dragged. SwiftUI's `List` diffing
occasionally determined that the underlying `NSTableViewCell` needed to be
reconfigured, triggering `makeNSView` on the `ContinuousNSSlider` inside that row.
`makeNSView` created a new `NSSlider` and a new `Coordinator`. The old `Coordinator`
was released (ARC). `NSControl.target` is a `weak` reference, so the old `NSSlider`'s
target silently became nil. AppKit was still tracking mouse events on the old slider,
but actions were dropped with no error. That joint was frozen for the session.

**Root fix:** The entire slider panel was rewritten as pure AppKit with no SwiftUI
state:

- `List` / `NSTableView` replaced with `ScrollView + VStack` (no cell reuse).
- `JointRow` + `ContinuousNSSlider` (`NSViewRepresentable`) replaced with
  `JointRowControl` (`NSViewRepresentable`) wrapping a `JointRowView` (pure `NSView`).
- Each `JointRowView` is created **once** and never torn down.
- No `@Binding`, no `@State`, no SwiftUI state mutations during drag.
- `Coordinator.sliderMoved` writes directly to `appState.jointAngles[name]` (plain
  var) and updates the value label via `NSTextField.stringValue` — pure AppKit, no
  path back to SwiftUI's render or diff cycle.

---

## Final architecture summary

| Layer | Responsibility | Thread |
|---|---|---|
| `JointRowView` (NSView) | NSSlider + labels, created once per joint | Main |
| `Coordinator.sliderMoved` | Writes `appState.jointAngles[name]` directly | Main |
| 120 Hz `.common` timer | Snapshots `jointAngles` + obstacles + model under NSLock | Main |
| `renderer(_:updateAtTime:)` | Reads snapshot, calls `updatePose` + `updateCollisionTint` | CVDisplayLink BG |
| Collision publish (≤10 Hz) | Writes `appState.collisionResult` → SwiftUI re-renders sidebar | Main |

**Key invariants:**
- `jointAngles` is never `@Published` — no `objectWillChange` on write.
- No `simdTransform` write ever happens on the main thread.
- No SwiftUI state mutation ever happens inside an active slider drag.
- `NSSlider.target` (weak) always points to a live `Coordinator` because `JointRowView`
  owns the coordinator lifetime directly.
