# SensiNerveX v3.0 — Vibration Anomaly Detection

**Target:** WeAct STM32F405RGT6 @ 168 MHz  
**Sensor:** MPU-6050 (I2C1)  
**Storage:** TF/SD card (SPI1, FatFS)  
**Debug:** UART2 @ 115200 baud (PA2 TX / PA3 RX)  
**ML Core:** Sparse Autoencoder — bare-metal C, no framework

---

## Architecture Overview

```
MPU-6050 (200 Hz)
     │
     ▼ IMU_Sample (ax,ay,az,gx,gy,gz)
┌─────────────────────────────────────────────────────────┐
│                   SUPERVISOR FSM                        │
│                                                         │
│  IDLE → COLLECT → CALIBRATE → MONITOR → TRAIN           │
│                     │              │                    │
│              (learn threshold)  (detect anomalies)      │
└─────────────────────────────────────────────────────────┘
     │                                       │
     ▼                                       ▼
  SD Card                              UART2 Debug
  raw_imu.csv                          [MONITOR] normal mse=0.003
  anomaly_log.csv                      [ALERT]   ANOMALY streak=5
  ae_weights.bin                       [STATS]   windows=1200
  run_stats.csv
```

### Autoencoder Architecture

```
Input x (10) → Enc_H1 (8) [ReLU] → Latent z (4) [ReLU]
                                          │
Output x̂(10) ← Dec_H1 (8) [ReLU] ←──────┘  [Linear output]

Anomaly Score = MSE(x, x̂)
Anomaly if score > threshold

Total parameters: 254 weights + 30 biases = 284 scalars
Memory footprint (weights + velocities + gradients): ~3.4 KB in CCM SRAM
```

### Feature Vector (10 dimensions)

| Index | Feature       | Description                        |
|-------|---------------|------------------------------------|
| 0     | ax_mean       | Mean acceleration X                |
| 1     | ax_std        | Std deviation X                    |
| 2     | ax_rms        | RMS acceleration X                 |
| 3     | ay_mean       | Mean acceleration Y                |
| 4     | ay_std        | Std deviation Y                    |
| 5     | ay_rms        | RMS acceleration Y                 |
| 6     | az_mean       | Mean acceleration Z                |
| 7     | az_std        | Std deviation Z                    |
| 8     | az_rms        | RMS acceleration Z                 |
| 9     | resultant_peak| Max √(ax²+ay²+az²) in window       |

Window: 50 samples × 200 Hz = 250 ms per window, 50% overlap.

---

## Hardware Wiring

### MPU-6050 → STM32F405RGT6

| MPU-6050 Pin | STM32 Pin | Notes                    |
|--------------|-----------|--------------------------|
| VCC          | 3.3V      |                          |
| GND          | GND       |                          |
| SCL          | PB8       | I2C1_SCL                 |
| SDA          | PB9       | I2C1_SDA                 |
| AD0          | GND       | I2C address = 0x68       |
| INT          | —         | Not used (polling mode)  |

Pull-up resistors: 4.7 kΩ on SCL and SDA to 3.3V.

### TF/SD Card → STM32F405RGT6 (SPI1)

| SD Pin  | STM32 Pin | Notes               |
|---------|-----------|---------------------|
| CS      | PA4       | SPI1_NSS / GPIO_Out |
| SCK     | PA5       | SPI1_SCK (AF5)      |
| MOSI    | PA7       | SPI1_MOSI (AF5)     |
| MISO    | PA6       | SPI1_MISO (AF5)     |
| VCC     | 3.3V      |                     |
| GND     | GND       |                     |

As confirmed from WeAct board schematic (TF interface section, D row).

### UART2 Debug (PA2 TX / PA3 RX)

Connect USB-UART adapter:
- PA2 → adapter RX
- PA3 → adapter TX (optional, for commands)
- GND → adapter GND
- Baud: 115200, 8N1

---

## FSM State Descriptions

| State      | Entry Condition                        | Exit Condition                              |
|------------|----------------------------------------|---------------------------------------------|
| `IDLE`     | Boot or 'R' reset command              | 'C' command or auto-start                   |
| `COLLECT`  | Auto after init                        | ≥ `SNX_MIN_NORMAL_SAMPLES` (64) windows     |
| `CALIBRATE`| 64 normal windows collected            | 100 calib windows OR 60 s timeout           |
| `MONITOR`  | Calibration complete                   | Train batch full (16 normal windows)        |
| `TRAIN`    | 16 normal windows buffered in MONITOR  | Training pass complete → back to MONITOR    |
| `FALLBACK` | SD error or IMU fault flag             | 'R' command (IDLE)                          |

**Threshold formula:**  
`threshold = mean_MSE + 3.0 × std_MSE`  
(computed over 100 normal calibration windows)

---

## SD Card Files

| File              | Format  | Contents                                 |
|-------------------|---------|------------------------------------------|
| `raw_imu.csv`     | CSV     | timestamp_ms, ax, ay, az, gx, gy, gz    |
| `anomaly_log.csv` | CSV     | timestamp_ms, mse, threshold, f0..f9    |
| `run_stats.csv`   | CSV     | timestamp_ms, windows, anomalies, thr, mse |
| `ae_weights.bin`  | Binary  | Magic + CRC + weights + norm + threshold |

### ae_weights.bin Binary Layout

```
Offset  Size   Field
0x00    4      Magic: 0x534E5833 ('SNX3')
0x04    4      Version: (MAJOR<<16)|MINOR
0x08    4      n_weights: 254
0x0C    4      CRC-16/CCITT over weight bytes
0x10    1016   AE weights (254 × float32)
0x408   40     NormStats.mean (10 × float32)
0x430   40     NormStats.var  (10 × float32)
0x458   4      NormStats.n_updates (uint32)
0x45C   4      NormStats.is_initialised (uint32)
0x460   4      anomaly_threshold (float32)
─────────────────────────────────────────────
Total: 0x464 = 1124 bytes
```

---

## UART Commands

Send single ASCII characters via serial terminal (115200 baud):

| Command | Action                                      |
|---------|---------------------------------------------|
| `C`     | Restart full calibration (clears model)     |
| `T`     | Force one training pass now                 |
| `S`     | Print full status report                    |
| `R`     | Xavier re-initialise weights                |
| `W`     | Save weights + threshold to SD              |
| `L`     | Load weights + threshold from SD            |
| `H`     | Manually set anomaly threshold (prompts)    |
| `N`     | Toggle raw IMU CSV logging ON/OFF           |

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

- [x] **RCC**: HSE Crystal / Ceramic Resonator → PLL → 168 MHz SYSCLK
- [x] **I2C1**: Fast Mode (400 kHz), PB6/PB7, No DMA
- [x] **SPI1**: Full-Duplex Master, PA5/PA6/PA7, Prescaler /4 = 21 MHz
- [x] **USART2**: Async, 115200, PA2/PA3, No DMA
- [x] **TIM6**: Prescaler=839, Period=999 → 200 Hz, NVIC enabled (Priority 1)
- [x] **FatFS**: Middleware → FatFS → User-defined (SPI)
- [x] **GPIO**: PA4 as Output (SD CS), PC13 as Input (user key)
- [x] **SYS**: Debug Serial Wire, Timebase Source = SysTick

> ⚠️ Do NOT enable DMA on I2C1 or SPI1 — CCM SRAM is CPU-only.

---

## Calibration Procedure

1. Mount device on machine in **normal operating condition** (no load, idle, or known-good)
2. Power on → system auto-enters `COLLECT` state
3. Wait for: `[COLLECT] 64 normal windows buffered. Starting calibration.`
4. System enters `CALIBRATE` — trains AE + measures MSE distribution for 100 windows
5. Wait for: `[CALIB] Done. MSE mean=X std=Y threshold=Z`
6. System auto-transitions to `MONITOR`
7. Introduce abnormal vibration → watch for `[ALERT] Sustained anomaly detected!`

To re-calibrate: send `C` via UART.

---

## Threshold Tuning

The default formula is `mean + 3σ`. If you get too many false positives:

```
# Option 1: increase sigma multiplier in snx_config.h
#define SNX_THRESHOLD_SIGMA_MULT    4.0f   // was 3.0f

# Option 2: manually set via UART
# Send 'H', then type threshold value (e.g., "0.050")
```

Use `anomaly_log.csv` to plot MSE distribution in Python:

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('anomaly_log.csv')
plt.hist(df['mse'], bins=50)
plt.axvline(df['threshold'].iloc[0], color='r', label='threshold')
plt.xlabel('MSE'); plt.ylabel('Count'); plt.legend()
plt.savefig('mse_dist.png')
```

---

## Memory Map Summary

| Region  | Address    | Size  | Contents                                   |
|---------|------------|-------|--------------------------------------------|
| FLASH   | 0x08000000 | 1 MB  | Code, const data, linker copy table        |
| CCM     | 0x10000000 | 64 KB | `g_ae`, `g_norm`, `g_sv` (via `.ccmram`)   |
| SRAM    | 0x20000000 | 128 KB| Stack, heap, HAL buffers, FatFS work area  |

CCM usage estimate:
- `AE_Model` (weights + velocities + gradients + activations): ~6.8 KB
- `NormStats`: ~0.1 KB
- `SNX_Supervisor` (window_buf + train_buf + state): ~4.0 KB
- **Total CCM: ~11 KB of 64 KB**

---

## File List

```
SensiNerveX_Anomaly/
├── CMakeLists.txt
├── Linker/
│   └── STM32F405RGT6_FLASH.ld
└── Core/
    ├── Inc/
    │   ├── snx_config.h       ← All tunable parameters
    │   ├── ae_model.h         ← Autoencoder API
    │   ├── features.h         ← Feature extraction API
    │   ├── supervisor.h       ← FSM API + global handles
    │   ├── storage.h          ← SD card API
    │   ├── imu.h              ← MPU-6050 API
    │   ├── debug_uart.h       ← UART2 debug API
    │   ├── main.h
    │   ├── tim.h
    │   └── stm32f4xx_it.h
    └── Src/
        ├── main.c             ← Application entry, TIM6 ISR
        ├── ae_model.c         ← Forward, backward, SGD update
        ├── features.c         ← Feature extraction + Z-score norm
        ├── supervisor.c       ← FSM: COLLECT/CALIBRATE/MONITOR/TRAIN
        ├── storage.c          ← FatFS: CSV logs + binary weights
        ├── imu.c              ← MPU-6050 I2C driver
        ├── debug_uart.c       ← UART2 printf + RX poll
        ├── tim.c              ← TIM6 @ 200 Hz
        └── stm32f4xx_it.c     ← TIM6_DAC_IRQHandler, SysTick
```

---
