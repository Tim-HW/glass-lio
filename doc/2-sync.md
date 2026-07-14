# [0] Sync — pairing a scan with the IMU that spans it

Assembles a `MeasureGroup`: one LiDAR scan, plus the IMU samples that cover it.

Code: `syncMeasure()` in [`glasslio_node.cpp`](../src/glasslio_node.cpp).

Trivial-looking, and it is where two of the nastier bugs in this pipeline can hide —
both of which produce a deskew that *silently does nothing* rather than an error.

---

## 1. The requirement: bracket the scan on both sides

Deskew needs `R(t)` at the acquisition time of **every point** in the scan. `GyrInt`
interpolates between gyro knots and **clamps** outside their range — it does not
extrapolate. So any point falling outside the IMU coverage gets the endpoint
rotation, i.e. **no correction**, and no complaint.

Therefore a scan is released only once the IMU buffer covers it on *both* sides:

```
   IMU:   ●────●────●────●────●────●────●────●
              ↑                          ↑
          before t₀                  past t₁
   scan:      ├──────────────────────────┤
             t₀                          t₁
```

- **An IMU sample before `t₀`.** Needed to interpolate ω exactly *at* the anchor
  instant. Without it the integration starts at the first sample *inside* the scan,
  and the leading points are under-corrected.
- **IMU coverage past `t₁`.** Without it, every point in the tail of the scan clamps
  to the last knot, and the tail — where the correction is *largest* relative to the
  scan-end reference — is quietly under-corrected.

If a scan arrives with no preceding IMU at all, it is **dropped** with a warning.
There is nothing to integrate from.

---

## 2. The `scan_guard_sec` trick

Here is the catch: the release decision has to be made *before* the cloud is parsed,
and **`t₁` is only knowable by parsing the cloud** (it is the max over per-point
timestamps — see [deskew.md §3](3-deskew.md)).

So we can't test "IMU covers `t₁`" directly. Instead we wait for coverage past a
conservative proxy:

```
scan_end ≈ header.stamp + scan_guard_sec        (0.12 s)
```

`scan_guard_sec` must **exceed the scan period**. At 10 Hz a scan spans ~0.100 s, so
0.12 s is the scan plus 20 ms of margin.

Set it too small and you are back to clamping the tail of every scan — the failure is
invisible in the logs and shows up only as a subtly warped cloud. Set it far too
large and you just add latency, waiting for IMU you don't need.

---

## 3. Consumed IMU is not eagerly dropped

The obvious implementation — "copy the IMU covering this scan, then delete it" — is
wrong, and it breaks the *next* scan.

Scans are contiguous: scan `k` ends where scan `k+1` begins. The IMU samples sitting
just inside the end of scan `k`'s window are exactly the ones needed to **bracket the
start** of scan `k+1` (§1). Delete them and every subsequent scan loses its leading
bracket.

So the prune rule is deliberately conservative — drop only what is strictly older
than the *current* scan's start:

```cpp
while (imu_buffer_.size() > 1 && stamp_sec(imu_buffer_[1]) <= scan_t) {
  imu_buffer_.pop_front();
}
```

Keeping `size() > 1` guarantees we never empty the buffer of the one sample that
brackets the next scan's start.

---

## 4. Time jumps: three cases, three responses

A timestamp older than the last one seen means one of three different things, and
conflating them causes damage:

| Observation | Meaning | Response |
|---|---|---|
| Jump back **> 1 s** (`kRestartJumpSec`) | The **source restarted** — a bag loop, a replay | **Reset the whole estimator.** The map and pose describe a world we are no longer in. |
| Jump back by a **hair** | One out-of-order message (or two publishers racing) | **Drop that message only.** Clearing the buffer here would throw away good data. |
| Neither | Normal | Proceed. |

The middle case is worth dwelling on: the naive "time went backwards → clear the
buffers" reflex turns a single stray packet into a pipeline stall. And the warning
for it names the usual cause — a stray second `ros2 bag play` still running.

The restart case is *not* handled inline. The callback that detects it only **requests**
a reset; the worker thread applies it. Freeing the map from a callback while the
worker is mid-scan is a use-after-free — see
[pipeline.md](pipeline.md#threading).

---

## 5. Where sync runs

`syncMeasure()` runs on the **callback** side, under `buf_mutex_`, and does no heavy
work — it moves shared pointers, nothing more. Every ready scan is pushed onto the
bounded worker queue and the callback returns immediately.

This is the whole reason the callbacks stay fast: sync is cheap, and everything
expensive (deskew, ICP, map insert) happens on the far side of the queue.
