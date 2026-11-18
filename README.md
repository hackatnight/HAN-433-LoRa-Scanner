# H@N 433MHz LoRa Module Scanner

A 433 MHz LoRa listener, scanner and band logger for the Elecrow CrowPanel ESP32 2.4"
"ESP32 Solo Miner LCD" board (SKU `DHM04624D`) with an Ai-Thinker **Ra-01 (SX1278)**
radio wired to it.

> Working name - the repo can be renamed later if this grows past a scanner.

**Flash it from your browser:** <https://hackatnight.com/HAN-433-LoRa-Scanner/>

## Status

Bring-up. The current build only confirms the radio answers over SPI and prints the live
channel RSSI over USB serial at `115200` baud. No display yet. Next steps: on-screen
signal meter, a full-band sweep (the "what's in my air" waterfall), and timestamped
detection logging to the SD card.

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
