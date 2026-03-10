# Frozen Robot Joints — Root-Cause Analysis

## Observed Symptoms (chronological)

| # | Attempt | Observed behaviour | Conclusion |
|---|---|---|---|
| 1 | `@EnvironmentObject` only via `updateNSView` | Arm never moved at all | `updateNSView` not called across `NavigationSplitView` columns |
| 2 | Stored `@State` properties forcing `updateNSView` | Moved briefly, then froze immediately on first sustained drag | AppKit event-tracking run-loop mode pauses SwiftUI re-renders |
| 3 | Combine `@Published` `.sink` subscription | Froze on first drag | `willSet` fires before sibling properties are consistent; also misses event-tracking mode |
| 4 | `Timer` on `RunLoop.main` + `SCNTransaction(0)` | Lasted longer, eventually froze | `SCNTransaction` nested inside AppKit's implicit outer 0.25 s transaction; the outer transaction's duration won |
| 5 | `SCNSceneRendererDelegate` + snapshot properties | Froze | Data race: delegate fired on CVDisplayLink BG thread while main thread wrote `AppState` |
| 6 | Delegate + `CATransaction.setDisableActions(true)` | Still froze; `shoulder_lift_joint` froze first | `CATransaction` controls CoreAnimation layers, NOT SceneKit's `SCNTransaction` — wrong API |
| 7 | Flat-hierarchy FK approach + delegate | Angles flowing (`lift` 0→-0.73→-1.26 in logs) but arm frozen | FK world transforms were correct, but `simdTransform` not taking visible effect from BG thread |
| 8 | Parent-child hierarchy (no FK) + delegate + NSLock snapshot + 120 Hz timer | "Some joints frozen" | Angles reach renderer correctly (`joints=7`), but `simdTransform` inside `renderer(_:updateAtTime:)` on BG thread is subject to SceneKit's implicit `SCNTransaction` animation — each write starts a new 0.25 s animation, visual presentation lags or freezes |

---

## Deep Analysis

### A — The Run-Loop Stall (root cause of freezes 1–4)

SwiftUI's `updateNSView` is only called:
- After a SwiftUI `@State`/`@Published` change propagates through the view tree
- During the **default** run-loop mode (`NSDefaultRunLoopMode`)

AppKit enters **`NSEventTrackingRunLoopMode`** for the entire duration of a sustained slider drag (mouse-down → mouse-up). During this mode, SwiftUI's render loop is **suspended**. The effect:

- `@Published` changes fire immediately on the slider's thread
- `ObservableObject` notifies subscribers
- SwiftUI's scheduler queues a re-render — but the re-render never fires because the run loop is in event-tracking mode
- `updateNSView` is never called

A `Timer` added with `.common` mode IS fired during event-tracking, which is why attempt 4 lasted longer. However, attempt 4 still froze because of the nested `SCNTransaction` problem (see §B).

### B — SCNTransaction vs CATransaction (root cause of freeze 6)

SceneKit maintains its OWN animation system (`SCNTransaction`) that is **independent of CoreAnimation** (`CATransaction`). They share a similar API but are completely separate:

| | `CATransaction` | `SCNTransaction` |
|---|---|---|
| Controls | UIKit/AppKit layer animations | SceneKit node property animations |
| Affect `SCNNode.simdTransform`? | **No** | **Yes** |
| Default animation duration | 0 (explicit) | 0.25 s (implicit) |

Wrapping `simdTransform` writes in `CATransaction.setDisableActions(true)` has **zero effect** on SceneKit animations. The 0.25 s `SCNTransaction` implicit animation continues unopposed. Every 16 ms a new 0.25 s animation is started from the current animated position to the new target — the presentation tree perpetually lags the model tree.

### C — The Presentation Tree vs Model Tree (root cause of visual freeze in latest attempt)

SceneKit maintains **two copies** of the scene graph:

1. **Model tree** — the `SCNNode` instances you code against. `node.simdTransform = X` writes here.
2. **Presentation tree** — what Metal actually renders. Updated by the `SCNTransaction` animation system.**What you observe:** `node.simdTransform` updated correctly in the model tree. Logs confirm angles flow. But the **presentation tree** is animated from last-frame value to new value over 0.25 s. During rapid slider movement, each frame starts a fresh 0.25 s animation. The presentation tree never converges. When movement stops, it snaps. This looks like "frozen" or "laggy" visual movement.

**Fix:** `SCNTransaction.begin(); SCNTransaction.animationDuration = 0; …; SCNTransaction.commit()`. NOT `CATransaction`.

### D — BG Thread Scene Graph Mutations (data race)

`renderer(_:updateAtTime:)` fires on the **CVDisplayLink thread** (background), confirmed by logs showing `thread=BG`. While Apple doc says modifications within this callback are safe (SceneKit holds an internal lock during `updateAtTime:`), there are edge cases:

- `buildRobotNodes` calls `scene.rootNode.addChildNode(container)` — structural mutations from BG thread
- `updatePose` writes `jNode.simdTransform` for all joints from BG thread — safe per Apple but subject to SCNTransaction implicit animations (§C)
- The NSLock snapshot protects `AppState` reads, but `jointNodes` dict itself is unprotected

### E — The `.common` Timer + Snapshot Stale Problem

The 120 Hz timer correctly fires during event-tracking. But the timer only refreshes `snapshotAngles` — the renderer still runs on BG thread and reads the snapshot. If there is any drift in timing (timer fires at 120 Hz, renderer reads at 60 Hz), a stale snapshot can be read. More critically: the snapshot is refreshed on main thread, but `buildRobotNodes` checks `revision != lastBuiltRevision` on the BG thread — this comparison is unprotected.

---

## Ordered List of Likely Causes (most → least likely)

1. **SCNTransaction implicit animation (0.25 s default) on `simdTransform` writes** — Every `jNode.simdTransform = X` inside `renderer(_:updateAtTime:)` is silently wrapped in SceneKit's implicit transaction with 0.25 s duration. The presentation tree always lags 0.25 s behind the model tree. During rapid updates, the presentation tree never fully converges → joints appear frozen mid-animation. **Fix: wrap all `simdTransform` writes in `SCNTransaction(animationDuration: 0)`.**

2. **`renderer(_:updateAtTime:)` on CVDisplayLink BG thread — wrong thread for scene graph operations** — While Apple permits minor property writes in this callback, structural mutations (`addChildNode`, etc.) and bulk property updates reliably cause visual glitches on macOS with Metal rendering. Moving all scene graph work to the main thread removes ALL thread-related uncertainty. **Fix: do ALL scene graph writes from the main-thread 120 Hz timer, making the delegate a no-op.**

3. **`simdTransform` in a parent-child hierarchy interacts with SceneKit's transform propagation during the same render pass** — When multiple joints in a chain all have their transforms set in the same `updateAtTime:` callback, SceneKit may not yet have propagated parent transforms before computing child world transforms. Setting transforms in BFS order does NOT guarantee SceneKit resolves the hierarchy in that order during the same pass. **Fix: moved to main thread (point 2) avoids this by letting SceneKit's display link handle propagation between frames.**

4. **The NSLock snapshot is refreshed at 120 Hz but can be up to 8 ms stale** — At 60 fps the render latency is 16.7 ms; the snapshot could theoretically lag by up to 8 ms + 16.7 ms = 24.7 ms. During maximal slider velocity this could appear as 1–2° lag. Not "frozen" per se but contributes to visual unresponsiveness. **Fix: reading live AppState directly on main thread eliminates this lag entirely.**

5. **`buildRobotNodes` scene-graph structural mutations from BG thread** — Adding/removing child nodes while Metal is preparing the previous frame for rendering is documented as safe only within the `updateAtTime:` callback; exiting the callback mid-mutation is undefined. **Fix: dispatch structural mutations to main thread.**

6. **SwiftUI `NavigationSplitView` detail-column isolation breaking `updateNSView` propagation** — SwiftUI may not propagate EnvironmentObject changes to `NSViewRepresentable.updateNSView` across `NavigationSplitView` column boundaries in all versions of macOS Tahoe. **Fix: use direct `appState` reference held by coordinator instead of relying on `updateNSView`.**

---

## Definitive Fix Architecture

> Move **all** scene graph work (build + pose update) to the **main thread** via the 120 Hz `.common`-mode timer. Wrap writes in `SCNTransaction(animationDuration: 0)`. Remove `SCNSceneRendererDelegate` pose-update logic. Keep `rendersContinuously = true`.

```
SwiftUI slider drag
  └─ @Published jointAngles changes (main thread)
       └─ 120 Hz Timer fires (.common mode, main thread, works during event-tracking)
            └─ read appState directly (no lock needed — same thread)
                 └─ SCNTransaction.begin(); animationDuration = 0
                      ├─ updatePose(jointNodes)       ← main thread, safe
                      └─ updateObstacles(obstacles)   ← main thread, safe
                           └─ SCNTransaction.commit()

CVDisplayLink (60 Hz BG thread)
  └─ renderer(_:updateAtTime:) → no-op (empty)
  └─ SceneKit renders presentation tree ← sees model tree changes from main
```

This eliminates: run-loop stall, SCNTransaction animation lag, BG thread race, NSLock snapshot staleness, and structural-mutation thread safety concerns — all in one change.
