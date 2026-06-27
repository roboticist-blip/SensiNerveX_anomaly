# Vibration Anomaly Detection with Multi-Baseline Commissioning

**Target:** WeAct STM32F405RGT6 @ 168 MHz
**Sensor:** MPU-6050 (I2C1)
**Storage:** TF/SD card (SDIO 4-bit, PC8–12 + PD2, DMA2, FatFS)
**Debug:** UART2 @ 115200 baud (PA2 TX / PA3 RX)
**ML Core:** Sparse Autoencoder — bare-metal C, no framework, on-device backpropagation

---

## Overview

SensiNerveX detects abnormal vibration on industrial machinery using a
small autoencoder trained directly on the microcontroller — no external ML
framework, no host PC, no cloud inference. The system learns what a
machine's normal vibration looks like, then flags any window whose
reconstruction error exceeds a calibrated threshold as anomalous.

Many real machines don't have one "normal" state — a lathe, for example,
produces a different vibration signature when idle, spinning up, cutting
steadily, or shutting down. Calibrating against only one of these states
causes every other legitimate state to be flagged as anomalous. To solve
this, SensiNerveX supports **Recorded-Cycle Commissioning**: a one-time
guided recording of a full machine cycle, automatic detection of the
distinct stable operating states within it, and one calibrated baseline
per state.

The autoencoder itself is not duplicated per state — one shared model is
trained sequentially across every detected state, so it learns a
representation expected to reconstruct all of them well. Only the
**threshold** and **feature centroid** are state-specific. At runtime, the
system picks the nearest known state before judging whether a window is
anomalous *for that state*.

This means the system is calibrated in accordance with the specific
machine it's deployed on — however many operating states it actually has
— without manual per-deployment configuration or assumptions about
machine type baked into the firmware.

A simpler single-state calibration path is also available for bench
testing or machines with only one operating condition.

---

## Architecture Overview

```
MPU-6050 (200 Hz)
     |
     v IMU_Sample (ax,ay,az,gx,gy,gz)
+------------------------------------------------------------------------+
|                          SUPERVISOR FSM                                |
|                                                                          |
|  IDLE --+--> RECORD --> SEGMENT --------------------+                  |
|         |    (capture    (detect plateaus,           |                  |
|         |     full cycle) calibrate each)             v                  |
|         +--> COLLECT --> CALIBRATE ------------> MONITOR --> TRAIN      |
|               (single-     (single-baseline         |              |     |
|                state)       calibration)     (nearest-baseline  (train  |
|                                                anomaly check)  on normal)|
+------------------------------------------------------------------------+
     |                                                  |
     v                                                  v
  SD Card                                          UART2 Debug
  RAWIMU.CSV   CYCLE.BIN     BASELINE.BIN          [MONITOR] state=1 normal mse=0.0008
  ANOMLOG.CSV  EVAL.CSV      WEIGHTS.BIN            [SEGMENT] Found 3 plateau(s)
  RUNSTAT.CSV  LATENT.CSV    SNXCFG.BIN             [ALERT]   Sustained anomaly! state=2
  TRAIN.CSV    CALIB.CSV     PROFILE.CSV
```

### Autoencoder Architecture

```
Input x (10) -> Enc_H1 (8) [ReLU] -> Latent z (4) [ReLU]
                                          |
Output x^(10) <- Dec_H1 (8) [ReLU] <-----+  [Linear output]

Anomaly Score = MSE(x, x^)
Anomaly if score > active_baseline.threshold x 1.10

Total parameters: 254 (AE_WEIGHT_FLAT_SIZE)
One shared model trained sequentially across all commissioned states --
NOT one model per state. Only the threshold and feature centroid are
per-state (see SNX_Baseline).
```

### Feature Vector (10 dimensions)

| Index | Feature        | Description                          |
|-------|----------------|---------------------------------------|
| 0     | ax_mean        | Mean acceleration X                   |
| 1     | ax_std         | Std deviation X                       |
| 2     | ax_rms         | RMS acceleration X                    |
| 3     | ay_mean        | Mean acceleration Y                   |
| 4     | ay_std         | Std deviation Y                       |
| 5     | ay_rms         | RMS acceleration Y                    |
| 6     | az_mean        | Mean acceleration Z                   |
| 7     | az_std         | Std deviation Z                       |
| 8     | az_rms         | RMS acceleration Z                    |
| **9** | **resultant_peak** | Max sqrt(ax^2+ay^2+az^2) in window — **used as the stability/energy proxy during SEGMENT plateau detection** |

Window: 50 samples x 200 Hz = 250 ms per window, 50% overlap
(`SNX_WINDOW_SIZE=50`, `SNX_WINDOW_STRIDE=25`).

---

## Hardware Wiring

### MPU-6050 -> STM32F405RGT6

| MPU-6050 Pin | STM32 Pin | Notes                    |
|--------------|-----------|--------------------------|
| VCC          | 3.3V      |                          |
| GND          | GND       |                          |
| SCL          | PB8       | I2C1_SCL (AF4)           |
| SDA          | PB9       | I2C1_SDA (AF4)           |
| AD0          | GND       | I2C address = 0x68       |
| INT          | --        | Not used (polling mode)  |

Pull-up resistors: 4.7 kOhm on SCL and SDA to 3.3V.

### TF/SD Card -> STM32F405RGT6 (SDIO 4-bit)

| SD Pin   | STM32 Pin | Notes                    |
|----------|-----------|----------------------------|
| CMD      | PD2       | SDIO_CMD (AF12)             |
| CLK      | PC12      | SDIO_CK  (AF12)              |
| D0       | PC8       | SDIO_D0  (AF12)               |
| D1       | PC9       | SDIO_D1  (AF12)               |
| D2       | PC10      | SDIO_D2  (AF12)                 |
| D3       | PC11      | SDIO_D3  (AF12)                  |
| VCC      | 3.3V      |                                |
| GND      | GND       |                                |

SDIO is used with DMA2 (Stream3/Stream6, Channel4). **CCM SRAM is not
DMA-accessible** -- `g_ae`, `g_norm`, and `g_sv` live in CCM, but all FatFS
I/O (including the `CYCLE.BIN` commissioning recording) reads/writes
through ordinary stack/static buffers in normal SRAM, never CCM directly.

### UART2 Debug (PA2 TX / PA3 RX)

Connect a USB-UART adapter:
- PA2 -> adapter RX
- PA3 -> adapter TX (required -- commands are sent this way)
- GND -> adapter GND
- Baud: 115200, 8N1

---

## FSM State Descriptions

| State       | Entry Condition                            | Exit Condition                                                   |
|-------------|---------------------------------------------|----------------------------------------------------------------------|
| `IDLE`      | Boot, or `'R'` reset                        | `'G'` (recommended) or `'C'` (single-state)                           |
| `RECORD`    | `'G'` command                               | `'D'` command, `SNX_CYCLE_AUTO_TIMEOUT_MS` (10 min), or `SNX_CYCLE_MAX_SAMPLES` reached |
| `SEGMENT`   | `RECORD` exits                              | Plateau detection + per-plateau calibration complete -> `MONITOR`; or no stable plateau found -> `FALLBACK` |
| `COLLECT`   | `'C'` command (single-state)                | >= `SNX_MIN_NORMAL_SAMPLES` (64) windows                                |
| `CALIBRATE` | 64 normal windows collected (single-state)  | 100 calib windows OR 60 s timeout                                         |
| `MONITOR`   | Commissioning complete (either path)        | Train batch full (16 normal windows)                                       |
| `TRAIN`     | 16 normal windows buffered in `MONITOR`     | Training pass complete -> back to `MONITOR`                                 |
| `FALLBACK`  | SD error, IMU fault, or failed commissioning | `'R'` command -> `IDLE`                                                      |

**Threshold formula:**
`threshold = median(MSE) + 10 x MAD(MSE)`
computed per detected operating state over up to `SNX_CALIBRATION_WINDOWS`
(100) windows of that state. Median + MAD is used rather than mean + std
because it resists being skewed by the small fraction of
transient/outlier windows that inevitably leak into any real calibration
window.

---

## Recommended Commissioning Procedure

This is the procedure to give clients. It applies to any machine with more
than one operating condition.

1. Mount the device on the target machine.
2. Power on. Wait for the boot banner and `[INIT]` lines.
3. Send **`G`** over UART. The system begins streaming raw IMU data to
   `CYCLE.BIN` and prints progress every 2 seconds:
   `[RECORD] samples=1200 (6.0s)`
4. **Run the machine through one full representative operating cycle** --
   power-on/spin-up, steady operation at typical load, any tool-change or
   mode-switch the machine normally does, and shutdown/idle if relevant.
   Longer steady periods between transitions give the plateau detector
   more to work with -- aim for at least several seconds of continuous
   steady operation per state.
5. Send **`D`** when the cycle is complete. The system closes the
   recording and enters `SEGMENT`:
   ```
   [SEGMENT] Replaying 18400 samples -> 735 feature windows
   [SEGMENT] Found 3 plateau(s) (max_states=16, eps=0.0500, hold=20)
   [SEGMENT] Calibrating plateau 1/3 (start_window=12, n=100)
   [SEGMENT] Plateau 0 (n=100): median=0.000412 MAD=0.000098 threshold=0.001392
   [SEGMENT] Calibrating plateau 2/3 (start_window=210, n=100)
   ...
   [SEGMENT] Commissioning complete: 3 baseline(s) ready. -> MONITOR
   ```
6. The system is now in `MONITOR`, judging each incoming window against
   the nearest matching baseline's threshold. Send `S` at any time to see
   which baseline is currently active and its threshold.
7. To verify: deliberately introduce an abnormal vibration (impact, added
   imbalance, etc.) and confirm `[ALERT] Sustained anomaly!` appears,
   while normal transitions between the machine's known states do **not**
   trigger alerts.

To re-commission (e.g. after a mechanical change to the machine), simply
send `'G'` again -- this overwrites `CYCLE.BIN` and re-runs the full
process, replacing all previously stored baselines.

### If commissioning fails

```
[SEGMENT] ERROR: no stable plateau found. Increase SNX_STABILITY_EPS,
lower SNX_STABILITY_HOLD_WINDOWS, or re-record with longer steady periods.
```

This means no region of the recording held steady for long enough to
qualify as a plateau (`SNX_STABILITY_HOLD_WINDOWS`, default 20 windows ~
2.5s). Either the recording didn't include enough continuous steady
operation, or `SNX_STABILITY_EPS` (default `0.05f`, in raw
`resultant_peak` units) is too tight for this machine's vibration
amplitude range. **Both constants are placeholders** -- tune them against
a real recorded cycle from the actual target machine before relying on
this in the field; see `snx_config.h` for where they live.

---

## Single-State Calibration (bench/testing only)

For quick testing with a single operating condition, or boards that don't
yet need multi-state support.

1. Mount device on machine in **one normal operating condition**.
2. Send **`C`** via UART.
3. Wait for: `[COLLECT] 64 normal windows buffered. Starting calibration.`
4. System enters `CALIBRATE` -- trains the AE and measures the MSE
   distribution for 100 windows.
5. Wait for: `[CALIB] Done. median=X MAD=Y threshold=Z`
6. System auto-transitions to `MONITOR`, with exactly one baseline
   (`baselines[0]`) populated from this calibration.

This path is not recommended for machines with multiple legitimate
operating states -- use `'G'` (commissioning) instead.

---

## UART Commands

Send single ASCII characters via serial terminal (115200 baud). Commands
marked **numeric entry** expect a follow-up line of digits terminated by
Enter (`\r` or `\n`) -- typing this is buffered internally and does not
echo per-digit on the debug console.

| Command | Action                                                                  |
|---------|---------------------------------------------------------------------------|
| `G`     | **(recommended)** Begin commissioning recording -> `RECORD`                  |
| `D`     | **(recommended)** End commissioning recording -> `SEGMENT` -> `MONITOR`        |
| `C`     | Restart single-baseline calibration (clears model)                             |
| `T`     | Force one training pass now                                                       |
| `S`     | Print full status report (active baseline, threshold, counts, etc.)                |
| `R`     | Xavier re-initialise weights                                                         |
| `W`     | Save weights + threshold + all baselines to SD                                       |
| `L`     | Load weights + threshold + all baselines from SD                                       |
| `H`     | **Numeric entry.** Manually override the **currently active baseline's** threshold     |
| `M`     | **Numeric entry.** Set `max_states` (1-`SNX_MAX_STATES_CEILING`, default ceiling 16) for this machine, persisted to SD |
| `N`     | Toggle raw IMU CSV logging ON/OFF                                                          |
| `0`-`5` | Set ground-truth fault label (`NORMAL`/`IMPACT`/`IMBALANCE`/`LOOSE_MOUNT`/`MISALIGNMENT`/`BEARING_FAULT`) for research logging |

`H` applies to whichever baseline `MONITOR` currently considers active --
useful for field-tuning a specific operating state's sensitivity without
affecting the others. If no baseline has been commissioned yet, it falls
back to a single global threshold.

---

## SD Card Files

| File           | Format | Contents                                                                |
|----------------|--------|----------------------------------------------------------------------------|
| `RAWIMU.CSV`   | CSV    | timestamp_ms, ax, ay, az, gx, gy, gz (toggleable via `N`)                    |
| `ANOMLOG.CSV`  | CSV    | timestamp_ms, mse, threshold, f0..f9 -- written on each detected anomaly      |
| `RUNSTAT.CSV`  | CSV    | timestamp_ms, windows, anomalies, threshold, last_mse                          |
| `EVAL.CSV`     | CSV    | window_id, timestamp_ms, mse, ground_truth_label, prediction (research)          |
| `LATENT.CSV`   | CSV    | timestamp_ms, z1..z4, ground_truth_label (research, latent space)                  |
| `TRAIN.CSV`    | CSV    | session_id, epoch, loss (research, per-epoch training loss)                          |
| `CALIB.CSV`    | CSV    | window_id, mse -- written during both single-state `CALIBRATE` and `SEGMENT`'s per-plateau calibration (window_id resets to 1 at the start of each plateau) |
| `PROFILE.CSV`  | CSV    | window_id, cycles, time_us -- `AE_Forward()` timing per `MONITOR` window               |
| `WEIGHTS.BIN`  | Binary | Magic + CRC + AE weights + norm stats + legacy single threshold (see layout below)     |
| `CYCLE.BIN`    | Binary | Raw `IMU_Sample` stream captured during `RECORD`. Overwritten on each new `'G'`. |
| `BASELINE.BIN` | Binary | Magic + count + array of `SNX_Baseline` (centroid + threshold), one per detected operating state. |
| `SNXCFG.BIN`   | Binary | Single byte: operator-configured `max_states` for this machine. |

### WEIGHTS.BIN Binary Layout

```
Offset  Size   Field
0x00    4      Magic: 0x534E5833 ('SNX3')
0x04    4      Version: (MAJOR<<16)|MINOR
0x08    4      n_weights: 254
0x0C    4      CRC-16/CCITT over weight bytes
0x10    1016   AE weights (254 x float32)
0x408   40     NormStats.mean (10 x float32)
0x430   40     NormStats.var  (10 x float32)
0x458   4      NormStats.n_updates (uint32)
0x45C   4      NormStats.is_initialised (uint32)
0x460   4      legacy threshold (float32) -- unused once BASELINE.BIN exists
------------------------------------------------
Total: 0x464 = 1124 bytes
```

### BASELINE.BIN Binary Layout

```
Offset  Size                          Field
0x00    4                             Magic: 0x534E4258 ('SNXB')
0x04    1                             count (uint8, number of valid baselines)
0x05    count x sizeof(SNX_Baseline)  Array of SNX_Baseline structs

SNX_Baseline (per entry):
  float   centroid[10]    Raw (pre-normalised) feature centroid
  float   threshold       median + 10xMAD for this state
  uint8_t valid
```

### CYCLE.BIN

Raw, headerless stream of `IMU_Sample` structs, one per captured sample
during `RECORD`. No magic/version header -- `Storage_GetCycleSampleCount()`
derives sample count from file size. Capped at `SNX_CYCLE_MAX_SAMPLES`
(120,000 samples = 10 minutes at 200 Hz).

---

## Build Instructions

### Prerequisites

```bash
sudo apt install gcc-arm-none-eabi cmake ninja-build openocd
```

### Build

```bash
mkdir build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Debug
ninja
```

For a different node ID:

```bash
cmake .. -DCMAKE_C_FLAGS="-DSNX_NODE_ID=0x02"
```

### Flash

```bash
ninja flash
# or manually:
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program SensiNerveX_Anomaly.bin verify reset exit 0x08000000"
```

---

## CubeMX Configuration Checklist

Generate these peripherals in CubeMX for STM32F405RGT6:

- [x] **RCC**: HSE Crystal/Resonator -> PLL -> 168 MHz SYSCLK (PLLM=8, PLLN=336, PLLP=/2)
- [x] **I2C1**: Fast Mode (400 kHz), PB6/PB7, No DMA
- [x] **SDIO**: 4-bit wide bus, PC8-PC12 + PD2, with DMA2 (Stream3/Stream6, Channel4)
- [x] **USART2**: Async, 115200, PA2/PA3, No DMA
- [x] **TIM6**: Configured for 200 Hz periodic interrupt, NVIC enabled
- [x] **FatFS**: Middleware -> FatFS -> user-defined (SDIO), `FF_FS_EXFAT=0`, `FF_USE_FASTSEEK=1`, `_VOLUMES=1`
- [x] **SYS**: Debug Serial Wire, Timebase Source = SysTick

> Do NOT place `g_ae`, `g_norm`, or `g_sv` anywhere DMA needs to reach --
> CCM SRAM (0x10000000) is CPU-bus-only. All SDIO/FatFS transfers
> (including the `CYCLE.BIN` commissioning path) go through ordinary
> stack/static buffers in normal SRAM (0x20000000), never CCM directly.

---

## Threshold Tuning

The formula is `median + 10xMAD`, computed independently per commissioned
operating state. If a specific state is too sensitive or not sensitive
enough in the field:

```
Option 1 -- re-commission entirely:
  Send 'G', run the machine through its full cycle again, send 'D'.
  This regenerates ALL baselines from scratch.

Option 2 -- adjust one state's threshold without re-commissioning:
  Make sure MONITOR's active baseline is the one you want to adjust
  (check with 'S'), then send 'H', followed by the new threshold value
  and Enter (e.g. "0.0042" then Enter).

Option 3 -- change the plateau-detection sensitivity before commissioning:
  In snx_config.h:
    SNX_STABILITY_EPS            -- lower = stricter "is this stable" test
    SNX_STABILITY_HOLD_WINDOWS   -- higher = longer steady period required
  Both are placeholders that need tuning against a real recorded cycle
  from the actual target machine.
```

Use `CALIB.CSV` (per-plateau, resets per state) or `EVAL.CSV` (continuous,
tagged by ground-truth label) to plot the MSE distribution in Python:

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('EVAL.CSV')
plt.hist(df['mse'], bins=50)
plt.xlabel('MSE'); plt.ylabel('Count')
plt.savefig('mse_dist.png')
```

---

## Memory Map Summary

| Region | Address    | Size   | Contents                                                            |
|--------|------------|--------|--------------------------------------------------------------------------|
| FLASH  | 0x08000000 | 1 MB   | Code, const data, linker copy table                                      |
| CCM    | 0x10000000 | 64 KB  | `g_ae`, `g_norm`, `g_sv` (incl. `baselines[16]`, via `.ccmram`)             |
| SRAM   | 0x20000000 | 112 KB | Stack, heap, HAL buffers, FatFS work area, **SEGMENT's `energy[]`/`is_stable[]` scratch (~24 KB, only during commissioning)** |

CCM usage estimate (verify against your actual `.map` file -- these are
estimates, not measured values):
- `AE_Model` (weights + velocities + gradients + activations): ~6.8 KB
- `NormStats`: ~0.1 KB
- `SNX_Supervisor` (window_buf + train_buf + state + **16 x `SNX_Baseline`
  ~ 16 x 45 bytes ~ 0.7 KB**): ~4.7 KB
- **Total CCM: ~11.6 KB of 64 KB**

`SEGMENT`'s scratch arrays (`energy[]`, `is_stable[]`) live in normal
SRAM, not CCM, and only exist transiently during the one-time
commissioning pass -- they do not count against the CCM budget above, but
**do** need headroom in normal SRAM alongside FatFS buffers and stacks.
Confirm against your linker `.map` output before deployment.

---

## File List

```
SensiNerveX_anomaly/
├── CMakeLists.txt
├── README.md
├── Linker/
│   └── STM32F405RGT6_FLASH.ld
└── Core/
    ├── Inc/
    │   ├── snx_config.h       <- All tunable parameters
    │   ├── ae_model.h         <- Autoencoder API
    │   ├── features.h         <- Feature extraction API
    │   ├── supervisor.h       <- FSM API + global handles + SNX_Baseline
    │   ├── storage.h          <- SD card API + commissioning/baseline I/O
    │   ├── imu.h              <- MPU-6050 API
    │   ├── debug_uart.h       <- UART2 debug API (single-byte, no line buffering)
    │   ├── main.h
    │   ├── sdio.h / sdio_diskio.h
    │   └── stm32f4xx_it.h
    └── Src/
        ├── main.c             <- Application entry, TIM6 ISR, RX-echo gating
        ├── ae_model.c         <- Forward, backward, SGD update
        ├── features.c         <- Feature extraction + Z-score norm
        ├── supervisor.c       <- FSM incl. RECORD/SEGMENT, multi-baseline MONITOR
        ├── storage.c          <- FatFS: CSV logs + binary weights + CYCLE/BASELINE/CFG
        ├── imu.c              <- MPU-6050 I2C driver
        ├── debug_uart.c       <- UART2 printf + single-byte RX poll
        ├── sdio.c / sdio_diskio.c
        └── stm32f4xx_it.c     <- TIM6 IRQHandler, SysTick
```

---
<<<<<<< HEAD

## Design Notes -- Why One Shared Autoencoder, Not One Per State

A natural-seeming alternative to this design is training a separate small
autoencoder per detected operating state. This was deliberately rejected:

1. **CCM SRAM budget multiplies by N states.** A second/third/fourth full
   `AE_Model` (weights + velocities + gradients + activations) is far more
   expensive than a second/third/fourth `SNX_Baseline` (just a centroid +
   threshold, ~45 bytes).
2. **A shared AE that reconstructs multiple known-normal states well is a
   stronger fault detector**, not a weaker one. Real faults (bearing wear,
   looseness, imbalance) tend to produce vibration signatures unfamiliar
   to the model in *every* state's feature space, not just one -- so a
   single model that has learned to compress all legitimate states well
   is more likely to flag a genuine fault regardless of which state it
   occurs in, compared to N independently-trained smaller models that
   each only ever saw one state's data.
3. **Per-state thresholds capture exactly the part that legitimately
   varies** (how noisy reconstruction is within a given state's normal
   range) without requiring the reconstruction *mechanism* itself to be
   duplicated.

---

## Known Placeholders Requiring Real-Machine Tuning

Two constants in `snx_config.h` are explicitly marked as placeholders and
must be tuned against a real recorded cycle from the actual target
machine before trusting automatic plateau detection in the field:

- `SNX_STABILITY_EPS` (default `0.05f`) -- stability epsilon in raw
  `resultant_peak` units. Depends on sensor mount and the machine's
  vibration amplitude range.
- `SNX_STABILITY_HOLD_WINDOWS` (default `20`, ~2.5s) -- how long a region
  must remain stable before being accepted as a plateau.

Recommended tuning approach: record one real commissioning cycle, then
temporarily log the `energy[]` trace from `handle_segment()`'s Pass 1 (via
`DBG_Printf` or by dumping to SD), inspect it, and pick values that
cleanly separate the machine's known states -- the same approach used to
tune `SNX_LEARNING_RATE`/`SNX_MOMENTUM` by observing real training loss
curves.

A normal-SRAM budget check (`SEGMENT`'s ~24 KB scratch arrays against your
actual linker `.map` output) is also recommended before field deployment.
