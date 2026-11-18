# H@N 433MHz LoRa Module Scanner

A 433 MHz LoRa listener, scanner and band logger for the Elecrow CrowPanel ESP32 2.4"
"ESP32 Solo Miner LCD" board (SKU `DHM04624D`) with an Ai-Thinker **Ra-01 (SX1278)**
radio wired to it.

> Working name - the repo can be renamed later if this grows past a scanner.

**Flash it from your browser:** <https://hackatnight.com/HAN-433-LoRa-Scanner/>

## Status

Single-channel RSSI meter on 433.000 MHz. The screen shows the live signal level in dBm,
a level bar with a peak-hold marker, the estimated noise floor, and a scrolling history
graph. Colors show how far the signal sits above the noise floor: blue is noise, then
green, yellow and red. Tap the screen to reset the peak. The level is also printed over
USB serial at `115200` baud.

Each on-screen reading is the loudest of the samples taken in its 100 ms frame (one
sample every 5 ms), so short bursts aren't missed. It reads a few dB above a single
idle sample for the same reason.

Next steps: a full-band sweep (the "what's in my air" waterfall), and timestamped
detection logging to the SD card.

## Screen and touch pins (built in)

| Part | Pins |
|---|---|
| ILI9341V screen | SCLK `14`, MOSI `13`, CS `15`, DC `2`, backlight `27` (no SDO pin) |
| XPT2046 touch | SCLK `14`, DIN `13`, DOUT `4` (not 12), CS `33`, PENIRQ `36`, 600 kHz |

The screen and touch share HSPI. The radio has its own bus (VSPI), so the two never
contend.

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
firmware with PlatformIO, merges it into one flashable image, and commits it to
`docs/firmware/HAN-433-Scanner.bin`, which the web flasher serves. Push a change to
`src/` and the flasher updates a minute or two later.

To build locally instead: `pio run -e crowpanel-miner`.
