// H@N 433MHz LoRa Module Scanner - bring-up test.
//
// This first build only proves the Ra-01 (SX1278) answers over SPI and prints
// the live channel signal level over USB serial. No display yet - this exists to
// confirm the module and the two cables are wired correctly before anything is
// built on top of it.
//
// Board: Elecrow CrowPanel 2.4" "ESP32 Solo Miner LCD" (SKU DHM04624D).
// Radio wiring (two-cable hookup):
//   SCK = IO25, MISO = IO21, MOSI = IO32, NSS = IO22
//   RST and DIO0 are left unconnected (marked NC below).

#include <RadioLib.h>

#define PIN_SCK   25
#define PIN_MISO  21
#define PIN_MOSI  32
#define PIN_NSS   22

SPIClass radioSPI(VSPI);

// Module(cs, dio0, rst, dio1) - dio0/rst/dio1 are unconnected on this wiring.
SX1278 radio = new Module(PIN_NSS, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC,
                          radioSPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nH@N 433MHz LoRa Module Scanner - bring-up");

  radioSPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  // begin() resets (skipped here - RST is NC), reads the chip version, and sets
  // default LoRa parameters. Start at 433.0 MHz, inside this module's band.
  int st = radio.begin(433.0);
  if (st == RADIOLIB_ERR_NONE) {
    Serial.println("Radio found - SX1278 responding. Listening on 433.0 MHz.");
  } else {
    Serial.print("Radio NOT found. Error code: ");
    Serial.println(st);
    Serial.println("Check NSS and MISO first - those are the usual suspects.");
    while (true) delay(1000);
  }

  radio.startReceive();   // continuous receive, so the channel can be read
}

void loop() {
  // Instantaneous channel RSSI (not a packet) - the noise floor, which jumps up
  // when something transmits nearby.
  float rssi = radio.getRSSI(false);
  Serial.print("RSSI: ");
  Serial.print(rssi, 1);
  Serial.println(" dBm");
  delay(500);
}
