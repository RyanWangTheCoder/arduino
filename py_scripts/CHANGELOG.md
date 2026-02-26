# Changelog (host scripts)

## Unreleased

### btCentral.py

- **Cooperative scheduling in `consumer_task` (asyncio fairness)**
  - **Change:** Periodically yields to the event loop (`await asyncio.sleep(0)`) after processing a number of frames and also inside long EMG/IMU per-sample loops.
  - **Rationale:** Prevents `consumer_task` from monopolizing the event loop during heavy parsing/processing so `writer_task` (CSV flush) and `health_task` stay responsive; helps reduce queue buildup/drops.

- **UDP send failure backoff (best-effort + CPU protection)**
  - **Change:** `udp_send()` remains best-effort (exceptions swallowed) but now applies exponential backoff when sends fail, skipping attempts until the backoff window expires.
  - **Rationale:** If the plotter/receiver isn't running or UDP sends error, repeatedly calling `sendto()` at high rates wastes CPU. Backoff reduces needless work while keeping recording non-blocking.

- **CSV rotation filename collision avoidance**
  - **Change:** `TimeRotatingCsv.open()` now includes milliseconds in the file `open_id`.
  - **Rationale:** Avoids rare filename collisions/overwrites when a rotation happens within the same second.

- **Operational robustness / diagnostics**
  - **BLE queue overflow warnings:** When the BLE queue is full, dropped-chunk warnings are emitted periodically (drop policy unchanged: drop newest chunk).
  - **Reconnect hygiene:** On reconnect, resets the stateful `FrameParser` and `DeviceClockExtender` and clears frame sequence tracking.
  - **Rationale:** Makes data loss visible, avoids stale buffered bytes after reconnect, and prevents incorrect extended device timestamps if firmware counters reset.

