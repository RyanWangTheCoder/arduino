# MyoWare 2.0 & Wireless Shield — Hardware Reference

---

## 0. REF Electrode Placement (Vendor Guidance + Actual Practice)

### Official Recommendation

From the **SparkFun MyoWare 2.0 Advanced Guide**
(https://cdn.sparkfun.com/assets/learn_tutorials/1/9/5/6/MyoWare_v2_AdvancedGuide-Updated.pdf):

> *"The reference electrode should be placed on a separate section of the body,
> such as the **bony portion of the elbow** or a **nonadjacent muscle**."*

### Vendor-Listed REF Sites for Forearm/Wrist Region

The Advanced Guide body diagram lists these electrically neutral reference
positions relevant to forearm recordings:

| Landmark | Location | Suitability |
|---|---|---|
| Epicondylus lateralis/medialis | Lateral/medial epicondyle at elbow | ✅ Bony, away from flexors |
| Olecranon | Bony point of elbow | ✅ Bony, electrically neutral |
| Processus styloideus ulnae | Ulnar styloid — wrist, pinky side | ✅ Bony, but far from sensor |
| Processus styloideus radii | Radial styloid — wrist, thumb side | ✅ Bony, but far from sensor |

### Actual Placement Used in This Study

**REF electrode: lateral epicondyle region, proximal end of the extensor
muscle group, closer to the elbow — on the opposite side of the forearm
from the flexor electrodes (MID and END).**

```
  Elbow end
  ─────────────────────────────────────────────
  [posterior/lateral forearm — extensor side]
       ●  REF  ← placed here, near epicondyle
  ─────────────────────────────────────────────
        forearm cross-section
  ─────────────────────────────────────────────
  [anterior/medial forearm — flexor side]
       ●  MID  ← over flexor muscle belly
       ●  END  ← 1-3 cm distal toward wrist
  ─────────────────────────────────────────────
  Wrist end
```

**Rationale:**
- Lateral epicondyle is a bony prominence with minimal EMG signal from
  the forearm flexors — consistent with vendor guidance
- Placing REF on the extensor side keeps it away from the target flexor
  muscle signal, maximising the differential signal quality
- Near the elbow minimises the sensor board cable length needed to reach
  the REF snap socket — practical for a board-mounted sensor

**Potential crosstalk caveat:**
The lateral epicondyle region has extensor muscle nearby. If the REF snap
lands on extensor muscle belly (not the bony epicondyle itself), a small
amount of extensor EMG may enter the REF signal, slightly degrading the
common-mode rejection ratio (CMRR) of the differential amplifier. To
minimise this, place REF on the palpable bony prominence (the epicondyle
itself), not on the soft tissue adjacent to it.

**For the paper:** cite as *"REF electrode placed over the lateral
epicondyle region, proximal to the extensor muscle group, consistent with
vendor-recommended electrically neutral bony prominence placement
[SparkFun MyoWare 2.0 Advanced Guide, 2023]."*

### Why NOT Ulnar Styloid (Wrist) for This Setup

The ulnar styloid is a valid REF site (listed in vendor guide) but is
less practical here because:
- It is at the far end of the forearm from the sensor board
- Requires a longer external reference cable (adds noise risk)
- The on-board REF snap socket is at the elbow-side of the board
  when the sensor is mounted over the forearm flexor belly

### SENIAM Note

SENIAM guidelines (Hermens et al. 2000) — the academic standard — **explicitly
exclude forearm muscles for wrist movements**. The MyoWare vendor guide directly
fills this gap with specific anatomical landmarks. Our placement follows vendor
guidance and is consistent with the general SENIAM principle of placing REF on
electrically neutral tissue.

### Cable and Noise Implications

The Advanced Guide states: *"MyoWare 2.0 Cable Shield is recommended"*
when using the optional external reference cable. This confirms unshielded
longer cables pick up EM interference. Our setup uses the on-board REF snap
socket — rigid metal snap, ~3 cm unamplified path, no cable sway or
triboelectric noise.

---

## 0b. EMG Sensor Comparison: MyoWare 2.0 vs SEED/Grove EMG

### Why RAW signals from both sensors look similar

Both sensors output a **bandpass-filtered, amplified EMG signal** centered around a DC bias — not truly "raw" ADC from bare electrodes. The oscillating waveform (fast bursts during contraction, near-flat at rest) is the same underlying physiology captured by the same class of analog front-end.

| Feature | MyoWare 2.0 | SEED/Grove EMG |
|---|---|---|
| Instrumentation amplifier | ✅ Medical-grade INA | ✅ Consumer-grade |
| Hardware bandpass | **20–500 Hz** | ~20–500 Hz |
| Hardware envelope output | ✅ **ENV pin** (rectified + smoothed) | ❌ None |
| DC bias | ~1843 counts (stable) | Less stable |
| Electrode connectors | Snap (standard Ag/AgCl) | Integrated, less flexible |
| SNR | Higher | Lower |

**MyoWare "RAW"** = output of the hardware bandpass filter, *before* the hardware envelope detector — not raw ADC from bare electrodes.

**SEED EMG** = same class of filtered signal, but no hardware envelope output.

### Which is better for fatigue detection (MDF-based)?

**MyoWare 2.0 is the better choice.** Reasons:

1. **Hardware ENV pin** is unique to MyoWare 2.0 — SEED has no hardware envelope output.
   Both sensors share a similar ~20 Hz hardware low cutoff, but MyoWare's ENV pin provides
   a free signal quality / validity indicator that SEED cannot match.

2. **Higher SNR** from the medical-grade instrumentation amplifier gives cleaner FFT
   spectra, which is critical for reliable MDF shift detection across a tennis session.
   The `sensordata_plotter.py` ENV comparison subplot uses the ENV pin to detect saturation
   and electrode contact loss in real time. SEED requires software-only envelope.

3. **Software 50 Hz high-pass** in `preprocess_emg()` further rejects the 20–50 Hz band
   (residual motion artifact + low-frequency EMG components less useful for MDF analysis).
   This is a software choice, not a hardware difference — both sensors have a ~20 Hz
   hardware low cutoff.

4. **Existing codebase is built around MyoWare** — `btCentral.py`, `FatigueMonitor`,
   the Android app all assume dual-output (RAW + ENV). Switching to SEED would require
   significant rework with no benefit.

**Bottom line:** MyoWare's hardware bandpass + ENV pin + higher SNR make it the right
choice for MDF-based fatigue detection. The SEED sensor is adequate for simple
amplitude-based detection but not for frequency-domain fatigue analysis.

---

## 1. Background (RAW Signal Wiring Fix)
The Fatigue Monitoring system relies on capturing raw electromyography (EMG) signals from a SparkFun MyoWare 2.0 Muscle Sensor connected to a Wireless Shield (ESP32). To track muscle fatigue, the offline analysis script (`emg_util.py`) computes the Median Frequency (MDF) and Mean Frequency (MNF) of the muscle's electrical firing using Fast Fourier Transforms (FFT). 

For frequency analysis to work, **a clean, high-frequency, alternating-current (AC) `RAW` signal is strictly required**. The rectified and integrated Envelope (`ENV`) signal provided by the sensor cannot be used for frequency-based fatigue tracking because the hardware low-pass filter destroys the necessary high-frequency data.

## 2. The Problem: Data Log Analysis
During testing, the captured EMG CSV logs showed clear signs of hardware disconnect on the RAW channel. 

**Log Batch 1:**
```csv
dev_us,emg_seq,env,raw,ref,frame_seq,host_rx_ms
230836391,0,1751,65,0,0,1770606129424
230836891,1,1746,128,0,0,1770606129424
230837391,2,1730,173,0,0,1770606129424
```
* **Observation:** The `raw` ADC values are extremely low (65 - 173). On an ESP32 12-bit ADC (range 0-4095), an AC-coupled resting RAW signal from the MyoWare sensor should hover near the midpoint (roughly `2048`, representing ~1.65V). 

**Log Batch 2 (During Exertion):**
```csv
dev_us,emg_seq,env,raw,ref,frame_seq,host_rx_ms
28487653,42,3824,1011,0,4,1770609737530
28488153,43,3824,1009,0,4,1770609737530
28488653,44,3824,1005,0,4,1770609737530
```
* **Observation:** The `env` (Envelope) is registering massive exertion, pegged near the ADC maximum at `3824`. However, the `raw` signal is completely dead, flatlining at exactly `1005` to `1011` (a variance of only ±4 units).
* **Conclusion:** It is physically impossible for the sensor to output a massive Envelope voltage without a correspondingly massive, wildly oscillating RAW AC waveform. The static `raw` column proves that the ESP32's analog pin is completely disconnected from the sensor's RAW output and is simply floating, picking up static background capacitance.

## 3. Root Cause (Pogo Pin Failure)
The SparkFun MyoWare 2.0 ecosystem uses snap connectors to stack boards. However, **the snap connectors themselves only route three signals**: `VIN`, `GND`, and `ENV`.

To pass the `RAW` and `REF` signals to the Wireless Shield, SparkFun designed the shield with two tiny, spring-loaded **pogo pins** on its underside. These pogo pins are meant to physically press against the `RAW` and `REF` plated through-hole (PTH) pads on the Muscle Sensor board below it.

If your CSV data shows a floating `RAW` input (as seen in the logs above), it means **the RAW signal path has failed**. A pogo pin, the RAW/A4 solder jumper, a board trace, or a mechanical alignment problem can cause this. Common reasons include:
1. **Misalignment:** The boards are not snapped together perfectly flat, preventing the pogo pin from touching the pad.
2. **Damaged Pin:** A pogo pin is stuck in the compressed position, bent, or missing.
3. **Dirty Contact:** Flux residue, oxidation, or skin oils on the Muscle Sensor's `RAW` PTH pad are preventing electrical conductivity.

Because `emg_util.py` ignores the hardware `ENV` column and relies entirely on the `RAW` column, the offline fatigue pipeline will fail completely when this pogo pin connection is compromised.

## 4. The Fix

### Step 1: Inspect and Clean (Non-Destructive)
1. Unsnap the Wireless Shield from the Muscle Sensor.
2. Look at the bottom of the Wireless Shield. Verify the two gold pogo pins are present and springy.
3. Clean the `RAW` PTH pad on the top of the Muscle Sensor with rubbing alcohol.
4. Reseat the boards, ensuring they are perfectly parallel and pressed tightly together.
5. Capture a new log to see if the `raw` column now hovers near `2048` at rest.

### Step 2: The Hardwire Bypass (If Pogo Pins Fail)
If the pogo pins are damaged or unreliable during active sports/movement, you must bypass them with a hardwired connection:
1. Locate the **RAW** PTH pad on the MyoWare Muscle Sensor board.
2. Locate the pad labeled **`32`** on the underside of the Wireless Shield.
3. **Solder a flexible jumper wire** directly between these two pads.

### The `RAW`, `A4`, and `32` labels are different things
The board uses several labels for different parts of the signal path:

| Label or part | Meaning |
|---|---|
| `RAW` on the MyoWare sensor | High-frequency, midpoint-biased EMG output needed for spectral analysis |
| RAW pogo pin | Spring-loaded contact from the Wireless Shield to the sensor's RAW plated-through hole |
| `RAW/A4` area on the Wireless Shield | A solder jumper: the sensor-side RAW net and the adjacent A4-side board connection |
| `A4` hole | A physical board/header location labeled with the Arduino-style analog name |
| `32` hole/pad | Physical ESP32 GPIO32 breakout used by this firmware |
| `ADC_CHANNEL_4` | ESP32 ADC peripheral channel identifier; on the ESP32-WROOM-32E, ADC1 channel 4 is GPIO32 |

The intended signal path is therefore:

```text
MyoWare RAW pad
  -> RAW pogo pin
  -> Wireless Shield RAW side of the solder jumper
  -> A4-side PCB route
  -> ESP32 GPIO32 / ADC1 channel 4
```

The SparkFun [MyoWare 2.0 Wireless Shield product page](https://www.sparkfun.com/myoware-2-0-wireless-shield.html) confirms that the shield has low-profile pogo pins and RAW/REF/ENV solder jumpers. The product photo shows `RAW` next to `A4`, while the GPIO `32` breakout is a separate physical location. Different physical locations can still be the same electrical net, but that must be confirmed with a continuity measurement; the labels alone do not prove it.

The firmware does not read a symbol named `A4`. It configures ADC1 channel 4 and then classifies channel-4 DMA records as RAW:

```cpp
static constexpr adc_channel_t ADC_CH_RAW = ADC_CHANNEL_4;
patterns[1].channel = ADC_CH_RAW;
patterns[1].unit = ADC_UNIT_1;
```

Thus, soldering directly from the sensor's RAW pad to the physical shield pad `32` is a valid bypass: it skips the pogo contact, the RAW/A4 solder jumper, and any uncertain intermediate PCB route.

### End-to-end connection map
The labels describe different layers of the same signal path:

```text
MyoWare electrodes
  -> MyoWare amplifier
  -> EMG-board RAW plated-through hole
  -> RAW pogo contact on the Wireless Shield
  -> RAW-side solder jumper
  -> A4-side PCB route
  -> ESP32 GPIO32
  -> ADC1 channel 4
  -> emgDrainTask s.raw
  -> BLE frame and host CSV raw column
```

The other outputs follow parallel paths. The EMG board's `RECT` output is the shield's envelope/`ENV` path and normally travels through the stacked-board connection to GPIO39 / ADC1 channel 3. The `REF` output uses the second pogo contact and the `REF/A5` jumper, but this firmware sets `REF_CONNECTED = false` and does not include REF in the two-channel ADC DMA pattern.

The large snap connections provide the ordinary stacked-board power, ground, and envelope connections. The two small pogo pins provide the extra RAW and REF connections. This is why a RAW pogo failure can leave ENV working: the two signals use different physical routes.

The repair wire creates this shorter route:

```text
EMG-board RAW pad -- flexible solder wire --> Wireless Shield pad 32 / GPIO32
```

It bypasses the spring contact and the intermediate solder-jumper route. The 2026-10-25 capture confirms that this direct route reached the firmware's GPIO32 ADC input. It does not prove that the original pogo contact or the complete RAW-to-A4-to-32 route will remain reliable during movement. Use strain relief on the repair wire and keep it away from neighboring power, ground, A3, A4, and A5 pads.

### How to confirm the physical path
Perform these tests with USB and battery power disconnected and the boards separated:

1. Measure each shield pogo pin to the shield's RAW and REF jumper sides. The RAW pogo pin should have near-zero resistance to the RAW side.
2. Measure across the RAW-to-A4 solder jumper. An open or cracked jumper breaks the path even if the pogo pin touches the sensor.
3. Measure the A4-side connection to the physical pad `32`. Near-zero resistance confirms the board route; an open reading means the two physical points are not connected on that board or the probe points are misidentified.
4. With the boards stacked, measure from the MyoWare RAW plated-through hole to physical pad `32` while gently pressing and moving the shield. Intermittent continuity indicates a pogo or mechanical-contact problem.

Do not use the old Uno convention that `A4` must mean I2C SDA/GPIO21 as evidence about this board. The current evidence establishes that the firmware reads GPIO32 and that the pad-32 bypass worked; it does not establish RAW/A4-to-32 continuity without the continuity test above.

### The "Ghost" Signal & ADC Crosstalk
During the period where the RAW input was floating, the CSV logs showed the `raw` column maintaining a constant ~37% to ~44% ratio relative to the `env` column, with a correlation of 0.998.

**Why does RAW mirror ENV if it is disconnected?**
This is caused by **ADC Multiplexer Charge Sharing (Crosstalk)**. The ESP32 ADC Continuous (DMA) mode switches channels at 40 kHz. The internal Sample-and-Hold (S/H) capacitor charges to the actively driven `ENV` voltage (GPIO 39). When the multiplexer switches to the floating `RAW` pin (GPIO 32), the pin has no driving source. The charge is distributed between the S/H capacitor and the parasitic capacitance of the floating pin/trace, resulting in a predictable fractional voltage drop (the ~40% ghost signal). 

Because this ghost signal is just a slow DC-like trace, when the Python script passes it through `preprocess_emg` (which applies a high-pass filter to block DC noise), the ghost signal gets completely wiped out, resulting in a flatline.

### Final Verification
After moving the solder wire to the correct pad (`32`), the `raw` signal finally showed true, independent, high-frequency AC oscillation. 

To visualize this properly in the real-time Python plotter, a `--raw-zoom` flag was added to `sensordata_plotter.py`. The MyoWare sensor centers the resting RAW signal at ~1.65V (1800-2048 counts). The `--raw-zoom` flag dynamically zooms the Y-axis to perfectly track the baseline noise, allowing the user to clearly see the high-frequency bursts of muscle noise bulging outward exactly in sync with the hardware envelope.

Further down, the firmware configures the hardware Direct Memory Access (DMA) to capture high-speed data from `ADC1 channel 4` (which maps to Pin 32):
```cpp
// PIN_ENV=39 -> ADC1 channel 3, PIN_RAW=32 -> ADC1 channel 4
static constexpr adc_channel_t ADC_CH_ENV  = ADC_CHANNEL_3;
static constexpr adc_channel_t ADC_CH_RAW  = ADC_CHANNEL_4;
```
Finally, inside the `emgDrainTask`, the data from channel 4 is saved into the `s.raw` variable and transmitted via BLE. Because the ESP32 is listening to **GPIO32** for the RAW signal, the repair wire was placed on the shield pad labeled `32`. The nearby `A4` location is part of the board's RAW jumper labeling; it should not be treated as interchangeable with `32` unless continuity is confirmed.

### What about the REF pogo pin?
While the MyoWare Advanced Guide notes that monitoring the Reference (`REF`) electrode signal can be useful for advanced diagnostics (such as measuring common-mode noise on the user's skin), **you do not need to solder a bypass wire for the REF pin** with your current setup.

The proof is in the `myoware_imu_v2.ino` firmware. The code explicitly disables the REF pin:
```cpp
static constexpr bool REF_CONNECTED = false;
```
Furthermore, the high-speed Direct Memory Access (DMA) for the ADC is hardcoded to only capture **two channels** (`ADC_CH_ENV` and `ADC_CH_RAW`):
```cpp
adc_continuous_config_t adcCfg = {
  .pattern_num    = 2,       // Only capturing 2 channels!
```
Because the firmware is incapable of reading a third analog channel in its current state, and the Python offline analysis does not use REF data, repairing a broken `REF` pogo pin connection will have zero effect on your data. The `RAW` connection is the only critical fix.

### Visual Proof (The "Ghost" Signal in Plotter)
If you run the captured logs through the offline plotter (`python sensordata_plotter.py --mode offline`), the visual output perfectly proves the disconnect. 

You will observe:
1. **EMG RMS Flatline:** The calculated `EMG RMS` trace (blue) sits almost dead flat near zero, even while the hardware `ENV` trace (orange) shows massive spikes. If the RAW signal were connected, the RMS would trace the exact same mountain-peak shapes as the ENV line.
2. **Preprocessed EMG Flatline:** The `Preprocessed EMG` trace (blue) is a pure flatline.
3. **The Ghost RAW Trace:** The `Raw EMG` trace (black) shows very tiny, smoothed-out bumps that perfectly mimic the shape of the orange ENV line below it.

**Why does RAW mirror ENV if it is disconnected?**
This is caused by **capacitive coupling**. A floating pin on a compact PCB acts like a tiny antenna. The copper trace for the `ENV` signal can run physically near the floating `RAW` net. When the muscle flexes, the strong, slow-moving DC voltage of the `ENV` signal induces a small, faint "ghost" voltage onto the nearby floating RAW trace.

Because this ghost signal is just a slow DC-like trace, when the Python script passes it through `preprocess_emg` (which applies a high-pass filter to block DC noise and keep only AC muscle frequencies), the ghost signal gets completely wiped out, resulting in the flatline seen in the preprocessed plot. Because the preprocessed signal is dead, all subsequent calculations (RMS, MDF, MNF) fail.

## 5. Verification
After applying the jumper wire, capture a new log. You should observe the following:
* **In the CSV:** At rest, the `raw` column should hover around `2048`. During a muscle flex, the `raw` column should oscillate rapidly and wildly up and down (e.g., swinging quickly between `500` and `3500`).
* **In the Plotter:** The `Raw EMG` and `Preprocessed EMG` traces will show dense, high-frequency "fuzz" (AC waveform) during flexes, and the `EMG RMS` line will spike up in perfect sync with the `ENV` line.

### 5.1 Measured confirmation after the pad-32 solder repair

A new recording was captured on 2026-10-25 after soldering the MyoWare `RAW` pad to Wireless Shield pad `32`. The detailed report is [log_10252026.md](../py_scripts/log_10252026.md).

This recording confirms that the soldered path is electrically carrying a usable RAW signal:

- The firmware was listening to `ADC1 channel 4`, which is the ESP32-WROOM-32E's GPIO32 input. The repair used the physical shield pad labeled `32`; the separate `RAW/A4` jumper route was not assumed to be continuous without measurement.
- The EMG log contained 314,156 samples at the configured 2,000 Hz rate over approximately 157.5 seconds.
- RAW values ranged from 1,622 to 2,076 counts, with a median of 1,859. The signal was near the MyoWare ADC midpoint rather than stuck near zero or a rail.
- RAW was not flat: 81.6% of adjacent samples changed, and the longest exact constant run was only 12 samples.
- During a low-activity interval, high-pass RAW RMS was approximately 3.35 counts. During an active interval it rose to approximately 31.6 counts, with a RAW range of approximately 454 counts.
- The correct amplitude comparison is not signed RAW versus ENV sample correlation, because RAW oscillates around its DC midpoint while ENV is rectified and smoothed. One-second high-pass RAW RMS versus hardware ENV mean had Pearson `r = 0.988` and Spearman `rho = 0.991`.
- Active RAW energy in the practical 20–500 Hz EMG band was approximately 10.4 times the 0–20 Hz energy, compared with 4.3 times during a low-activity interval.

An independent IMU cross-check also supported the result. The canonical `emg_util` analysis found a positive EMG-versus-gyro relationship (`r = 0.377` at zero lag; approximately `r = 0.434` with the IMU about 0.5 seconds later). This is supporting evidence that the RAW bursts are related to real movement, not proof by itself: isometric muscle activation can occur without much motion, and motion can occur without EMG activation.

There were 928 missing EMG samples out of 315,084 expected samples, approximately 0.3%, across four BLE transport gaps. That is a capture-continuity issue, not evidence that the GPIO32 solder connection failed.

**Conclusion:** the pad-32 solder repair is confirmed electrically for this recording. It does not yet prove that the jumper will remain mechanically reliable during every sports movement; a controlled contraction and movement test is still recommended.

### 5.2 Narrowband lines in RAW from on-board I2C activity (2026-11-01)

A good RAW connection is not enough on its own for clean spectra. In `log0305` the RAW channel carries narrowband lines,
and their strength tracks the firmware's IMU I2C polling:
- **200 and 400 Hz** are about 10 dB stronger while the IMU is polled every 5 ms than during IMU outages.
- **333 Hz** appears only while the IMU is polled about every 3 ms.

60 Hz mains has no 200 Hz harmonic. The coupling path from the I2C bus into the ADC inputs is a hypothesis; a bench
test is still pending.

For the evidence table, the firmware mechanism and the planned test, see `HLD_FW.md` →
*As-Built Review* → *KI-2* and *Pending next steps* item 3. Treat these line bins as interference in any MDF analysis.
