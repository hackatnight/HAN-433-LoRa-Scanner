# H@N 433MHz LoRa Module Scanner

A 433 MHz LoRa listener, scanner and band logger for the Elecrow CrowPanel ESP32 2.4"
"ESP32 Solo Miner LCD" board (SKU `DHM04624D`) with an Ai-Thinker **Ra-01 (SX1278)**
radio wired to it.

> **FOR RESEARCH PURPOSES ONLY.** This is an experimental receiver and test transmitter.
> Only transmit on frequencies and at power levels you are licensed or permitted to use,
> and follow your local radio regulations. You are responsible for how you use it.

> Working name - the repo can be renamed later if this grows past a scanner.

Two builds from the same source: **portrait** (Meter / Sweep / Scan screens, RX/TX tabs) and
**landscape** (`LANDSCAPE=1 tools/build.sh`; one screen with spectrum, waterfall and
scanner together - "scan what you see" - and the TX sound board).

**Flash it from your browser:** <https://hackatnight.com/HAN-433-LoRa-Scanner/>

## Status

Two tabs at the bottom, **RX** and **TX**, with the menu look of H@N-CYD-GBC.

**RX** has two screens, switched with the header button (a short press of **BOOT**
also switches):

- **Meter** - one channel, 433.000 MHz: big dBm readout, level bar with peak hold, the
  estimated noise floor and a scrolling history graph. Tap to reset the peak. Each
  reading is the loudest sample in its 100 ms frame, so short bursts aren't missed.
- **Sweep** - 430.00 to 439.95 MHz in 50 kHz steps: a live spectrum with a max-hold
  trace over a scrolling waterfall. Tap to clear the max hold.

**TX** is a test transmitter. Tap a row to change it, then **TRANSMIT**:

- Frequency: 430.000 to 439.995 MHz, picked with step buttons (5 kHz to 1 MHz)
- Power: 2, 5, 10, 14 or 17 dBm (starts at 2)
- Signal: a plain carrier, or a 1 kHz tone an FM receiver can hear
- Length: 0.5, 1, 2 or 5 s. A tap or BOOT stops it early.

The SX1278 has no audio input, so the tone is made in software: the radio sends a
carrier and the firmware rewrites its frequency registers along a 16-step sine,
16 times per cycle (+-3 kHz deviation at 1 kHz). The settings are remembered.
Transmitting on 70 cm needs an amateur license (or stay within your local ISM rules).

Colors show how far a signal sits above the noise floor: blue is noise, then green,
yellow and red. Readings are also printed over USB serial at `115200` baud.

**Touch calibration** runs on the first boot: tap the four crosses. It's saved to the
SD card (`/han433_touch.cal`) and to the ESP32's flash. Hold **BOOT** for 2 seconds to
restart into calibration.

**Detection log.** While on SWEEP, signals that rise above the noise are appended to
`/han433_det.csv` on the SD card. The header shows how many have been logged so far.

```
boot,uptime_ms,utc,freq_mhz,peak_dbm,snr_db,dur_ms,sweeps
5,84211,2026-11-18T08:21:03.112Z,433.500,-21,70,1454,29
```

- A detection has to show up in two sweeps in a row, at least 6 dB above the loudest
  5% of that sweep's bins. One that lasts more than a minute is written once a minute.
- The board's own spurs are learned and ignored: the 40 MHz crystal x 11 at 440 MHz,
  plus a few weaker ones around 432, 436.7 and 438.3 MHz. The learned map is saved to
  `/han433_spur.bin`, so only the first ~2 minutes on a new card log spur rows. A
  steady local carrier is learned the same way after a minute or two.
- A transmitter right next to the board overloads the receiver. Keep test radios 2+ m
  away on low power. Close up, RF can also lock up the USB-serial chip; replugging
  the cable fixes that.
- Rows are kept in RAM and written in batches (every 16 rows or 30 s), so the last few
  are lost if power is cut.
- **Time:** `boot` counts power-ups (it restarts after flashing) and `uptime_ms` is
  time since boot. `utc` is filled in once a PC has sent the time over serial as
  `T<unix ms>`. `tools\dev.ps1 -Log` does this automatically. Without a PC the column
  stays empty.

## Screen and touch pins (built in)

| Part | Pins |
|---|---|
| ILI9341V screen | SCLK `14`, MOSI `13`, CS `15`, DC `2`, backlight `27` (no SDO pin) |
| XPT2046 touch | SCLK `14`, DIN `13`, DOUT `4` (not 12), CS `33`, PENIRQ `36`, 600 kHz |

The screen and touch share HSPI. The radio has its own bus (VSPI). The microSD card
(CLK `18`, MISO `19`, MOSI `23`, CS `5`) also uses VSPI on its own pins. The two share
the one peripheral through the GPIO matrix: the card's pins are only connected during a
log write (~20 ms, between sweeps). Leaving them connected let the radio's SPI clock
radiate from the SD traces into the 433 band.

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

**Timing.** Send `timing on` over serial (or from the BLE remote) and every 2 seconds the
board prints where its time goes: the radio part of each sweep, the drawing done inside
the sweep and how often it made a step overrun its settle wait, the drawing and BLE work
after each sweep, late listen batches and samples, FIFO overruns, and late TX ticks.
`timing off` stops it. This is to measure what moving drawing onto the second core would
gain before doing it.

## Reusing the UI

`src/han_ui.h` is the H@N-CYD-GBC menu look in one file: themes, the gradient header
with twinkling stars and a shine along its bronze edge, the glowing selected row,
rounded rows and buttons, and a tab bar. To use it in another LovyanGFX project,
declare `lcd`, `SCR_W` and `SCR_H`, include the file, call `applyTheme(0)` once and
`uiTick()` from the main loop.
