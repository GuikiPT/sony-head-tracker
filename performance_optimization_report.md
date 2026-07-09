# Performance optimization report — sony-head-tracker (local only)

Date: 2026-07-09 · Machine: Windows 11 Home 10.0.26200 · Toolchain: MSVC `/O2 /std:c++latest` (same flags as the release build)

All work is local to this folder. Nothing was pushed, no remotes were touched,
no PR was opened. Functionality, supported devices, settings, UI meaning,
output format, OpenTrack compatibility, and the compatibility list are
unchanged. All changes are uncommitted working-tree edits.

## Summary

Seven small, individually validated optimizations to the per-packet hot path
(HID parse → orientation filter → UDP/JSON output) and the GUI's per-packet /
per-frame work. Wire output is **byte-identical** (enforced by a randomized
20,000-sample byte-equivalence check built into the new benchmark harness),
decoded sensor values are **bit-identical**, and all 41 unit tests pass before
and after every change.

Headline numbers (simulated hot path, no headset — see *Methodology*):

- **Packet parsing reduced from 0.000836 ms to 0.000178 ms, a 0.000659 ms
  reduction, or 78.8% lower latency.** The optimized parse path is **4.71x**
  faster.
- **JSON telemetry serialisation reduced from 0.002561 ms to 0.000821 ms, a
  0.001740 ms reduction, or 68.0% lower latency.** The optimized serializer is
  **3.12x** faster.
- **Total measured end-to-end hot-path latency (parse → filter → UDP send)
  reduced from 0.02388 ms to 0.02016 ms per packet, a 0.00372 ms reduction, or
  15.6% lower latency** (1.18x). The remaining ~0.017 ms is the two `sendto`
  syscalls the wire format requires (one OpenTrack datagram + one JSON
  datagram); excluding that fixed syscall floor, the processing around it
  dropped from ≈0.0072 ms to ≈0.0035 ms (≈52%, estimate).
- **GUI paint cost per frame reduced from 0.566 ms to 0.222 ms in the
  replicated back-buffer benchmark, a 0.344 ms reduction, or 60.9% lower**
  (2.55x) — the ~3.8 MB back-buffer bitmap is no longer allocated and freed on
  every ~25 fps graph repaint, and only the invalidated region is blitted.
- **~0.0049 ms of GUI-thread text formatting per packet removed** (derived):
  telemetry/raw readout strings are now built at the display refresh rate
  (10 Hz / 5 Hz) from the newest sample instead of on every packet, with
  identical displayed text.

## Methodology

- No headset was connected during this work (`probe` exits 2), so the packet
  source is **SIMULATED**: buffers shaped like the WH-1000XM5's Android Head
  Tracker report (3×16-bit rotation + 3×16-bit gyro, unit exponent 10⁻⁶) feed
  the **real library code** — `decodePackedDescriptorValues`,
  `OrientationFilter::process`, `toJson`, `UdpOutput::send` (real loopback
  socket). GUI text formatting and the paint back-buffer pattern are
  **REPLICATED** from `gui.cpp` because the window class is not callable
  headlessly.
- Harness: [bench/bench_main.cpp](bench/bench_main.cpp), built and run with
  [build-bench.cmd](build-bench.cmd) at `/O2`. Each metric runs 2,000–10,000,000
  iterations and takes the best of 5 repeats; the A/B below additionally takes
  the best of 2 full harness runs per side, measured **back-to-back in the same
  session** (baseline sources restored via `git stash`, then re-applied) to
  minimise thermal/background noise. Observed run-to-run noise on this laptop is
  ±10–15%; only deltas well above that are claimed.
- Correctness gates built into the harness: a verbatim copy of the **baseline**
  decoder and the **baseline** `toJson` are compared against the library on
  every run — 20,000 randomized fields/buffers (bit-identical doubles required)
  and 20,000 randomized samples (identical bytes required).

## Before vs after (final interleaved A/B, best of 2 runs per side)

| Benchmark | Baseline | Optimized | Reduction | % lower | Speedup |
| --- | --- | --- | --- | --- | --- |
| Packet parse, 2 vector fields | 0.000836 ms | 0.000178 ms | 0.000659 ms | 78.8% | 4.71x |
| Orientation filter (not modified) | 0.000373 ms | 0.000336 ms | — (noise) | — | — |
| toJson, gyro present | 0.002561 ms | 0.000821 ms | 0.001740 ms | 68.0% | 3.12x |
| toJson, gyro null | 0.002084 ms | 0.000700 ms | 0.001384 ms | 66.4% | 2.98x |
| toOpenTrackPose | 0.0000014 ms | 0.0000015 ms | unchanged | — | — |
| udp.send (both datagrams, syscall-bound) | 0.019406 ms | 0.019760 ms | within noise | — | — |
| **End-to-end: parse → filter → udp.send** | **0.023880 ms** | **0.020156 ms** | **0.003724 ms** | **15.6%** | **1.18x** |
| Paint pattern: old (fresh buffer/frame) vs new (cached + clip blit) | 0.566220 ms | 0.221720 ms | 0.344500 ms | 60.9% | 2.55x |

The filter row is untouched code and shows the harness's noise band. The
`udp.send` row is ~95% `sendto` syscall time, so the serialisation gains inside
it are not separately resolvable there; they are resolved by the `toJson` rows
and by the end-to-end row.

## Per-optimization results

### O1 — JSON serialisation without `std::format` runtime parsing
- **File / area:** [src/protocol.cpp](src/protocol.cpp) `toJson` (+ new `toJsonTo` in [protocol.hpp](include/sony_head_tracker/protocol.hpp))
- **What was slow:** one ~330-character format string parsed at runtime per
  packet, two intermediate `std::string` allocations (gyro/accel), and the
  gyroscope vector formatted twice (once for its deprecated `angularVelocity`
  alias).
- **Change:** append-based serialisation via `std::to_chars`; the gyro vector is
  formatted once into a stack buffer and appended twice; `toJsonTo` writes into
  a caller-owned buffer and `toJson` remains as a thin wrapper.
- **Why identical:** the standard defines `{:.9g}` / `{:.3f}` formatting in
  terms of `std::to_chars` ([format.string.std]), so output bytes are identical;
  verified by the harness's 20,000-sample byte-equivalence check and the
  protocol unit tests.
- **Measured:** 0.002561 → 0.000821 ms (−0.001740 ms, **68.0% lower**, 3.12x).

### O2 — UDP output: cached JSON destination + reused serialisation buffer
- **File / area:** [src/output_udp.cpp](src/output_udp.cpp), [output_udp.hpp](include/sony_head_tracker/output_udp.hpp) `UdpOutput::send`
- **What was slow:** per packet, the JSON destination `sockaddr` was rebuilt with
  an `ntohs`/`htons` round-trip, and a fresh JSON `std::string` was allocated.
- **Change:** `open()` precomputes the port+1 address once; `send()` serialises
  into a member buffer via `toJsonTo` (single allocation for the process
  lifetime). Same wrap-around behaviour for port 65535 as before.
- **Why identical:** same two datagrams, same bytes, same destination; only
  redundant recomputation and allocation removed.
- **Measured:** at its step, udp.send 0.01809 → 0.01755 ms (−3%); in the final
  A/B the difference is **within syscall noise** — kept because it is strictly
  less work per packet with zero behavioural surface.

### O3 — Word-wise packed-field decoding (was per-bit loop)
- **File / area:** [src/hid_descriptor.cpp](src/hid_descriptor.cpp) `decodePackedDescriptorValues`
- **What was slow:** 16 iterations per value × 3 values per field × 2 fields per
  packet of single-bit extract/branch/OR.
- **Change:** assemble up to 9 bytes covering the bit range with `memcpy`
  (little-endian Windows targets), shift and mask once; identical
  truncated-buffer semantics (missing bytes read as zero).
- **Why identical:** pinned by the existing descriptor unit tests (including
  truncation, sign extension, degenerate bit sizes) and the harness's
  randomized bit-identical equivalence check (bit sizes 0–65, counts 0–4,
  buffers 0–19 bytes).
- **Measured (step):** parse 0.000842 → 0.000283 ms (−66.4%).

### O4 — Cached powers of ten in `descriptorScale`
- **File / area:** [src/hid_descriptor.cpp](src/hid_descriptor.cpp) `descriptorScale`
- **What was slow:** `std::pow(10.0, exponent)` on every decoded value (6 per
  packet).
- **Change:** 16-entry table for the HID unit-exponent range [−8, 7], filled by
  the same `std::pow` call previously used (out-of-range exponents still call
  `std::pow`), so values are bit-identical.
- **Measured (step):** parse 0.000283 → 0.000204 ms (further −28%). Combined
  O3+O4 in the final A/B: **0.000836 → 0.000178 ms (−78.8%, 4.71x)**.

### O5 — GUI: format readout text at display rate, not packet rate
- **File / area:** [src/gui.cpp](src/gui.cpp) `onSample`, `flushTelemetryUi`, `flushRawUi`, raw-packet handler
- **What was slow:** every packet built three `std::wstring`s via `std::format`
  (~0.0019 ms) plus a hex dump through `wostringstream` (~0.0047 ms) on the GUI
  thread, while the readouts repaint at most at 10 Hz (raw line: 5 Hz) — and the
  hex dump was built even in Simple Mode, where the control is hidden.
- **Change:** the newest sample / raw bytes are stored (fixed-size copy, no
  formatting) and the strings are materialised inside the flush functions right
  before they are pushed to the controls. `setStatsNow`/`setRawNow` materialise
  pending state first, so the last-writer-wins ordering of status messages vs
  sample text is exactly as before.
- **Why identical:** at every flush instant the text is formatted from the
  newest sample — the same string the old code would have displayed; all UI
  strings, refresh rates, and modes unchanged. Verified by GUI smoke test
  (launch, run, clean close) and code-path review.
- **Measured (derived from per-call costs at 25 pps):** GUI-thread work drops
  from 0.00657 ms per packet (0.00187 text + 0.00470 hex) to the same cost at
  ≤10 Hz/≤5 Hz — an average saving of **≈0.0049 ms per packet** (≈75% of the
  per-packet GUI formatting cost), growing with packet rate (some supported
  devices report at up to 100 Hz).

### O6 — GUI: cached paint back buffer + clip-limited blit
- **File / area:** [src/gui.cpp](src/gui.cpp) `paint()`
- **What was slow:** every WM_PAINT (once per sample, ~25 fps) created and
  destroyed a window-sized compatible bitmap (~3.8 MB at the default window
  size) and blitted the entire window even when only the graph band was
  invalidated.
- **Change:** the back buffer is cached across frames (recreated on client-size
  change) and only `ps.rcPaint` is blitted. The full frame is still rendered
  into the buffer, so pixels are identical; the screen regions outside the blit
  already show exactly those pixels.
- **Measured (replicated pattern):** 0.566 → 0.222 ms per frame (**−60.9%**,
  2.55x), ≈8.6 ms/s less GUI-thread + GDI work at 25 fps.

### O7 — Allocation-free HID reader parse loop
- **Files / area:** [src/hid_backend.cpp](src/hid_backend.cpp) `usageArray` + reader loop, [src/hid_descriptor.cpp](src/hid_descriptor.cpp) (new `decodePackedDescriptorValuesInto`)
- **What was slow:** per vector field per packet, a fresh packed-bytes vector
  and a fresh result vector were heap-allocated (≈4–6 allocations per packet on
  the reader thread).
- **Change:** the reader `Context` owns two scratch buffers reused for every
  packet; `decodePackedDescriptorValues` is now a wrapper over the `Into`
  variant, preserving the public function and its semantics.
- **Measured (simulated micro-benchmark, decode of one 3×16-bit field):**
  111.9 → 69.1 ns (−38.2%, 1.62x) per field; ×2 fields per packet plus the
  packed-buffer reuse inside `usageArray`.

## Files changed

| File | Change |
| --- | --- |
| [src/protocol.cpp](src/protocol.cpp) | to_chars-based serializer, `toJsonTo` |
| [include/sony_head_tracker/protocol.hpp](include/sony_head_tracker/protocol.hpp) | additive `toJsonTo` declaration |
| [src/output_udp.cpp](src/output_udp.cpp) | cached JSON destination, reused buffer |
| [include/sony_head_tracker/output_udp.hpp](include/sony_head_tracker/output_udp.hpp) | two new private members |
| [src/hid_descriptor.cpp](src/hid_descriptor.cpp) | word-wise decode, pow10 table, `Into` variant |
| [include/sony_head_tracker/hid_descriptor.hpp](include/sony_head_tracker/hid_descriptor.hpp) | additive `decodePackedDescriptorValuesInto` declaration |
| [src/hid_backend.cpp](src/hid_backend.cpp) | scratch-buffer `usageArray`, reader context scratch |
| [src/gui.cpp](src/gui.cpp) | deferred readout formatting; cached back buffer + clip blit |
| [bench/bench_main.cpp](bench/bench_main.cpp) | **new** — benchmark harness + equivalence gates |
| [build-bench.cmd](build-bench.cmd) | **new** — builds/runs the harness at `/O2` |

No public API was removed or changed; the two new functions are additive.
CLI commands, console output, ports, wire formats, UI text, settings,
supported-device logic, repair/probe behaviour, and the compatibility list are
untouched.

## Validation

- `build.cmd` — release build, warning-clean at `/W4`, after every change.
- `build-tests.cmd` — **41/41 unit tests pass** after every change.
- `build-bench.cmd` — decode equivalence **bit-identical** and toJson
  **byte-identical** vs verbatim baseline reference implementations, every run.
- `sony-head-tracker.exe version` / `probe` / `bridge --seconds 2` — same
  output and exit codes as baseline (no headset present: probe 2, bridge 3).
- GUI smoke test — window created ("Sony Head Tracker 2.0.0"), ran with timer
  and paints for several seconds, closed cleanly via WM_CLOSE.

## Risks and tradeoffs

- `toJson` no longer uses `std::format`; equivalence rests on the standard's
  to_chars-based definition of `g`/`f` formatting **plus** the randomized
  byte-equivalence gate in the harness, which would catch any divergence.
- The word-wise decoder assumes little-endian byte order (true for every
  Windows target this app builds for; noted in a comment).
- The cached back buffer holds one window-sized bitmap for the app's lifetime
  (~4 MB at default size) instead of allocating per frame — a deliberate
  memory-for-time trade.
- GUI readout strings are now built at flush time; if a future change reads
  `latestStatsText` and friends outside the flush/materialize paths it must
  call the materialize helpers first (they are cheap no-ops when nothing is
  pending).
- Benchmarks are simulated/replicated as labeled; absolute real-device numbers
  (25 pps Bluetooth cadence, HidP_* call costs) could not be measured without
  the headset connected.

## Investigated but not changed

- **Bridge CLI per-packet console line** (`\rYPR …` + flush in `main.cpp`):
  it is user-visible output written after the UDP send, so it does not delay
  the current packet; changing its cadence would change CLI behaviour.
- **Sensor API fallback 5 ms poll loop** (`sensor_api_backend.cpp`): an
  event-driven `ISensorEvents` sink would remove up to ~5 ms of polling latency
  on the fallback path, but cannot be verified without hardware that actually
  uses this backend.
- **One connected socket per destination port** (send() instead of sendto()):
  would shave some of the ~0.017 ms syscall floor, but splits the two streams
  across two source ports — an observable wire-level change, rejected.
- **Graph history ring buffer**: the per-sample `erase(begin())` moves ~1.4 KB;
  ~100 ns, not worth the indexing risk in `drawGraph`.
- **Per-sample `PostMessage` heap copies** (reader → GUI thread): ~2 small
  allocations per packet, sub-microsecond; a lock-free slot would complicate
  shutdown ordering for negligible gain.
- **Orientation filter / quaternion math**: outputs must stay bit-identical;
  any refactor (e.g., skipping redundant normalize calls) would change
  floating-point results. Left untouched (it is only ~0.0004 ms).
- **`enumerate()` / startup**: startup-only, not latency-relevant.

## Recommended future optimizations (require hardware)

1. With the WH-1000XM5 connected, measure true end-to-end latency (device
   timestamp → UDP send) via `bridge` and confirm the parse-loop gains at the
   real 25 pps cadence (`run-bench.exe` covers the CPU side only).
2. Event-driven Sensor API backend (`SetEventSink`) to remove the 5 ms polling
   quantum on the fallback path; verify with a device that uses that backend.
3. Profile `HidP_GetUsageValueArray`/`HidP_GetScaledUsageValue` per packet on
   real reports; if they dominate, a one-time capability-derived direct bit
   extraction (already available via `decodePackedDescriptorValuesInto`) could
   bypass them, but must be validated against real descriptors.
4. If a future device reports at 10 ms intervals (100 pps), re-run the GUI
   benchmarks — the deferred-formatting and cached-paint gains scale linearly
   with packet rate.
