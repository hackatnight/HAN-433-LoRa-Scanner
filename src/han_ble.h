// H@N 433 - Bluetooth LE link (NimBLE), for the Web Bluetooth page on the flasher site.
//
// One service, Nordic UART style, so any BLE terminal app also works:
//   6E400001-...  the service
//   6E400002-...  RX  (write)   command lines in - the same commands as serial
//   6E400003-...  TX  (notify)  text out - everything the board prints to serial
//   6E400004-...  SPEC (notify) one spectrum row per sweep, binary:
//                 u32 start kHz, u16 kHz per point, u8 first point, u8 total points,
//                 then one byte per point = dBm + 160 (little-endian; split over several
//                 notifies when the MTU is small)
//   6E400005-...  AUDIO (write) speech for TX: u32 byte offset, then 4-bit samples
// Everything the main loop prints goes through `con` (serial + BLE). The NimBLE task only
// queues incoming bytes; all sending happens on the main loop.
#pragma once
#include <NimBLEDevice.h>

namespace ble {

#define HAN_UUID(n) "6E40000" #n "-B5A3-F393-E0A9-E50E24DCCA9E"
NimBLEServer *srv;
NimBLECharacteristic *txC, *specC;
volatile uint16_t conn = BLE_HS_CONN_HANDLE_NONE;

// commands in: a ring the NimBLE task fills and the main loop drains
char inq[512];
volatile uint16_t inHead, inTail;
bool pending() { return inHead != inTail; }
int read() {
  if (inHead == inTail) return -1;
  char c = inq[inTail];
  inTail = (inTail + 1) % sizeof inq;
  return c;
}

struct RxCb : NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    for (char ch : v) {
      uint16_t h = (inHead + 1) % sizeof inq;
      if (h == inTail) break;                     // full: drop the rest
      inq[inHead] = ch; inHead = h;
    }
    if (v.size() && v.back() != '\n') {           // a write is one line, with or without \n
      uint16_t h = (inHead + 1) % sizeof inq;
      if (h != inTail) { inq[inHead] = '\n'; inHead = h; }
    }
  }
};
// speech in: 4-bit samples (two per byte, low nibble first) written by the page to AUDIO
// as u32 byte offset + data; "say N" then sends N samples (tx::send(S_SPEECH))
const uint32_t SPEECH_MAX = 32768;                // bytes = 65536 samples = 8.2 s at 8 kHz
uint8_t speech[SPEECH_MAX];
struct AudioCb : NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.size() < 5) return;
    uint32_t at;
    memcpy(&at, v.data(), 4);
    if (at < SPEECH_MAX) memcpy(speech + at, v.data() + 4, min((size_t)(SPEECH_MAX - at), v.size() - 4));
  }
};
struct SrvCb : NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, ble_gap_conn_desc *d) override { conn = d->conn_handle; }
  void onDisconnect(NimBLEServer *) override { conn = BLE_HS_CONN_HANDLE_NONE; }   // advertising restarts itself
};

bool subscribed(NimBLECharacteristic *c) { return conn != BLE_HS_CONN_HANDLE_NONE && c && c->getSubscribedCount(); }
int room() { return srv->getPeerMTU(conn) - 3; }

// One notify; when `wait`, retries for up to 300 ms while the stack is out of buffers.
bool notify(NimBLECharacteristic *c, const uint8_t *d, int n, bool wait) {
  for (uint32_t t = millis();;) {
    os_mbuf *om = ble_hs_mbuf_from_flat(d, n);
    int rc = om ? ble_gatts_notify_custom(conn, c->getHandle(), om) : BLE_HS_ENOMEM;   // om is consumed
    if (!rc) return true;
    if (!wait || rc != BLE_HS_ENOMEM || millis() - t > 300 || conn == BLE_HS_CONN_HANDLE_NONE) return false;
    delay(2);
  }
}

// text out, a line at a time (or a full buffer)
uint8_t ob[160];
int olen;
void flushLine() {
  if (subscribed(txC))
    for (int i = 0, r = max(20, room()); i < olen; i += r) notify(txC, ob + i, min(r, olen - i), true);
  olen = 0;
}
void put(uint8_t c) {
  if (conn == BLE_HS_CONN_HANDLE_NONE) { olen = 0; return; }
  ob[olen++] = c;
  if (c == '\n' || olen == sizeof ob) flushLine();
}

// a spectrum row; dropped (not waited for) when the link is busy
void spectrum(uint32_t startKHz, uint16_t stepKHz, const int16_t *dbm, int n) {
  if (!subscribed(specC) || n > 255) return;
  uint8_t p[247];
  int per = min((int)sizeof p, room()) - 8;
  if (per < 1) return;
  for (int first = 0; first < n; first += per) {
    int k = min(per, n - first);
    memcpy(p, &startKHz, 4); memcpy(p + 4, &stepKHz, 2);
    p[6] = first; p[7] = n;
    for (int i = 0; i < k; i++) p[8 + i] = constrain(dbm[first + i] + 160, 0, 255);
    if (!notify(specC, p, 8 + k, false)) return;
  }
}

void begin() {
  NimBLEDevice::init("H@N 433");
  NimBLEDevice::setMTU(247);
  srv = NimBLEDevice::createServer();
  srv->setCallbacks(new SrvCb);
  NimBLEService *s = srv->createService(HAN_UUID(1));
  s->createCharacteristic(HAN_UUID(2), NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)->setCallbacks(new RxCb);
  txC = s->createCharacteristic(HAN_UUID(3), NIMBLE_PROPERTY::NOTIFY);
  specC = s->createCharacteristic(HAN_UUID(4), NIMBLE_PROPERTY::NOTIFY);
  s->createCharacteristic(HAN_UUID(5), NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)->setCallbacks(new AudioCb);
  s->start();
  NimBLEAdvertising *a = NimBLEDevice::getAdvertising();
  a->addServiceUUID(HAN_UUID(1));
  a->setScanResponse(true);                       // the name goes in the scan response
  a->start();
}

}  // namespace ble

// Serial + BLE: use con.printf / con.println where the code used Serial's.
struct Con : Print {
  size_t write(uint8_t c) override { Serial.write(c); ble::put(c); return 1; }
  size_t write(const uint8_t *b, size_t n) override {
    Serial.write(b, n);
    for (size_t i = 0; i < n; i++) ble::put(b[i]);
    return n;
  }
} con;
