# H@N 433MHz LoRa Module Scanner

A 433 MHz LoRa listener, scanner and band logger for the Elecrow CrowPanel ESP32 2.4"
"ESP32 Solo Miner LCD" board (SKU `DHM04624D`) with an Ai-Thinker **Ra-01 (SX1278)**
radio wired to it.

> Working name - the repo can be renamed later if this grows past a scanner.

**Flash it from your browser:** <https://hackatnight.com/HAN-433-LoRa-Scanner/>

## Status

Two screens, switched with the **METER** / **SWEEP** tabs at the bottom (a short press
of **BOOT** also switches):

- **Meter** - one channel, 433.000 MHz: big dBm readout, level bar with peak hold, the
  estimated noise floor and a scrolling history graph. Tap to reset the peak. Each
  reading is the loudest sample in its 100 ms frame, so short bursts aren't missed.
- **Sweep** - 430.00 to 439.95 MHz in 50 kHz steps: a live spectrum with a max-hold
  trace over a scrolling waterfall. Tap to clear the max hold.

Colors show how far a signal sits above the noise floor: blue is noise, then green,
yellow and red. Readings are also printed over USB serial at `115200` baud.

**Touch calibration** runs on the first boot: tap the four crosses. It's saved to the
SD card (`/han433_touch.cal`) and to the ESP32's flash. Hold **BOOT** for 2 seconds to
restart into calibration.

Next step: timestamped detection logging to the SD card.

## Screen and touch pins (built in)

| Part | Pins |
|---|---|
| ILI9341V screen | SCLK `14`, MOSI `13`, CS `15`, DC `2`, backlight `27` (no SDO pin) |
| XPT2046 touch | SCLK `14`, DIN `13`, DOUT `4` (not 12), CS `33`, PENIRQ `36`, 600 kHz |

The screen and touch share HSPI. The radio has its own bus (VSPI). The microSD card
(CLK `18`, MISO `19`, MOSI `23`, CS `5`) also uses VSPI on its own pins, so it is only
read and written at startup, before the radio starts.

## The radio wiring (two cables)

| Radio pin | Board connector | ESP32 GPIO |
|---|---|---|
| SCK  | GPIO_D (yellow) | 25 |
| MOSI | GPIO_D (white)  | 32 |
| MISO | I2C (yellow)    | 21 |
| NSS  | I2C (white)     | 22 |
| 3V3  | GPIO_D (red)    | - |
| GND  | GPIO_D (black)  | - |

`RST` and `DIO0` are left unconnected for now.

## How the firmware is built

No local toolchain needed. GitHub Actions (`.github/workflows/build.yml`) builds the
firmware, merges it into one flashable image, and commits it to
`docs/firmware/HAN-433-Scanner.bin`, which the web flasher serves. Push a change to
`src/` and the flasher updates a minute or two later.

The toolchain is the same as H@N-CYD-GBC: **arduino-cli** with the ESP32 core `2.0.17`,
LovyanGFX `1.1.16` and RadioLib `7.8.1`. `tools/build.sh` does the compile and merge,
in CI and locally:

```sh
tools/build.sh              # build docs/firmware/HAN-433-Scanner.bin
tools/build.sh flash COM4   # build, then write it to a board
```

On Windows, `tools\dev.ps1 -Flash -Log 60` builds, flashes and shows 60 seconds of
filtered serial output in one step.
