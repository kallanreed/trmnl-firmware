#include "ble_firmware.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>

#include "config.h"
#include "display.h"
#include "filesystem.h"
#include "pins.h"
#include "trmnl_log.h"

#ifdef BOARD_TRMNL_X
#include "iqs323_task.h"
#endif

// Defined in display.cpp; selects pDevice from device_list by DEVICE_MODEL.
void hw_config_init(void);

// ---------------------------------------------------------------- UUIDs ----
// 7a0b0001-5ec4-4d6b-9b1e-2f3a7c1d0001  service
// 7a0b0002-...                          control (write + notify)
// 7a0b0003-...                          data    (write, no response)
#define SVC_UUID  "7a0b0001-5ec4-4d6b-9b1e-2f3a7c1d0001"
#define CTRL_UUID "7a0b0002-5ec4-4d6b-9b1e-2f3a7c1d0002"
#define DATA_UUID "7a0b0003-5ec4-4d6b-9b1e-2f3a7c1d0003"

// Control opcodes (client -> device)
enum : uint8_t {
  OP_BEGIN  = 0x01, // [0x01][u32 total_len][u32 crc32]  little-endian
  OP_COMMIT = 0x02, // [0x02]
  OP_ABORT  = 0x03, // [0x03]
};

// Notification codes (device -> client): [code][u32 received]
enum : uint8_t {
  EV_READY     = 0x10, // BEGIN accepted, send data
  EV_PROGRESS  = 0x11, // periodic during transfer
  EV_RENDERING = 0x12, // CRC good, panel refreshing
  EV_DONE      = 0x13, // image on screen
  EV_ERROR     = 0x1F, // see err byte
};

// Error codes, sent as the byte after EV_ERROR
enum : uint8_t {
  ERR_NONE = 0, ERR_TOO_BIG, ERR_NO_MEM, ERR_SEQ, ERR_OVERFLOW,
  ERR_LEN_MISMATCH, ERR_CRC, ERR_NOT_BEGUN,
};

// ------------------------------------------------------------- transfer ----
static uint8_t  *g_buf          = nullptr;  // PSRAM reassembly buffer
static uint32_t  g_expected_len = 0;
static uint32_t  g_expected_crc = 0;
static uint32_t  g_received     = 0;
static uint16_t  g_next_seq     = 0;
static bool      g_in_transfer  = false;

// Set by the BLE callback, consumed by ble_fw_process(). The panel refresh
// takes seconds, so it must not run on the NimBLE host task.
static volatile bool g_render_pending = false;

static NimBLECharacteristic *g_ctrl = nullptr;

// ---------------------------------------------------------------- CRC32 ----
// IEEE 802.3, reflected, init/final 0xFFFFFFFF. Computed incrementally so we
// never make a second pass over a 750KB buffer.
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *data++;
    for (int k = 0; k < 8; k++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
    }
  }
  return ~crc;
}
static uint32_t g_running_crc = 0;

// ----------------------------------------------------------------- utils ----
static void notify(uint8_t code, uint8_t err = ERR_NONE) {
  if (!g_ctrl) return;
  uint8_t pkt[6];
  pkt[0] = code;
  pkt[1] = err;
  memcpy(&pkt[2], &g_received, sizeof(uint32_t));
  g_ctrl->setValue(pkt, sizeof(pkt));
  g_ctrl->notify();
}

static void transfer_reset(void) {
  if (g_buf) {
    free(g_buf);
    g_buf = nullptr;
  }
  g_expected_len = g_expected_crc = g_received = g_running_crc = 0;
  g_next_seq = 0;
  g_in_transfer = false;
}

static void fail(uint8_t err) {
  Log_info("BLE transfer failed, err=%u", err);
  transfer_reset();
  notify(EV_ERROR, err);
}

// ------------------------------------------------------------- callbacks ----
class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &) override {
    NimBLEAttValue v = chr->getValue();
    if (v.length() < 1) return;
    const uint8_t *p = v.data();

    switch (p[0]) {
      case OP_BEGIN: {
        if (v.length() < 9) { fail(ERR_LEN_MISMATCH); return; }
        transfer_reset();
        memcpy(&g_expected_len, &p[1], sizeof(uint32_t));
        memcpy(&g_expected_crc, &p[5], sizeof(uint32_t));

        if (g_expected_len == 0 || g_expected_len > MAX_IMAGE_SIZE) {
          fail(ERR_TOO_BIG);
          return;
        }
        // X-class boards have 8MB PSRAM; keep the big buffer out of DRAM.
        g_buf = (uint8_t *)ps_malloc(g_expected_len);
        if (!g_buf) g_buf = (uint8_t *)malloc(g_expected_len);
        if (!g_buf) { fail(ERR_NO_MEM); return; }

        g_in_transfer = true;
        Log_info("BLE BEGIN len=%u crc=%08x", g_expected_len, g_expected_crc);
        notify(EV_READY);
        break;
      }

      case OP_COMMIT: {
        if (!g_in_transfer) { fail(ERR_NOT_BEGUN); return; }
        if (g_received != g_expected_len) { fail(ERR_LEN_MISMATCH); return; }
        if (g_running_crc != g_expected_crc) {
          Log_info("BLE CRC mismatch: got %08x want %08x", g_running_crc, g_expected_crc);
          fail(ERR_CRC);
          return;
        }
        Log_info("BLE COMMIT ok, %u bytes", g_received);
        notify(EV_RENDERING);
        g_render_pending = true; // ble_fw_process() draws it
        break;
      }

      case OP_ABORT:
        Log_info("BLE ABORT");
        transfer_reset();
        notify(EV_DONE);
        break;

      default:
        break;
    }
  }
};

class DataCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &) override {
    if (!g_in_transfer || !g_buf) return;

    NimBLEAttValue v = chr->getValue();
    if (v.length() < 3) return; // u16 seq + at least one payload byte

    const uint8_t *p = v.data();
    uint16_t seq;
    memcpy(&seq, p, sizeof(uint16_t));
    const uint8_t *payload = p + 2;
    size_t n = v.length() - 2;

    if (seq != g_next_seq) {
      Log_info("BLE seq gap: got %u want %u", seq, g_next_seq);
      fail(ERR_SEQ);
      return;
    }
    if (g_received + n > g_expected_len) { fail(ERR_OVERFLOW); return; }

    memcpy(g_buf + g_received, payload, n);
    g_running_crc = crc32_update(g_running_crc, payload, n);
    g_received += n;
    g_next_seq++;

    // Keep notification traffic light; roughly every 32 chunks.
    if ((g_next_seq & 0x1F) == 0) notify(EV_PROGRESS);
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
    Log_info("BLE client connected");
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
    Log_info("BLE client disconnected (reason %d)", reason);
    transfer_reset();
    NimBLEDevice::startAdvertising();
  }
};

// ------------------------------------------------------------------ init ----
void ble_fw_init(void) {
  Serial.begin(115200);
  Log_info("TRMNL X BLE firmware starting");

  // Same hardware bring-up order bl_init() uses for BOARD_TRMNL_X.
  hw_config_init();
  pins_init();
#ifdef BOARD_TRMNL_X
  iqs323_task_i2c_lock();
  display_init();
  otg_turn_off();
  iqs323_task_i2c_unlock();
#else
  display_init();
#endif
  filesystem_init();
  Wire.setClock(100000);

  Log_info("Display %ux%u ready", display_width(), display_height());

  NimBLEDevice::init("TRMNL-X");
  NimBLEDevice::setMTU(517);

  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService *svc = server->createService(SVC_UUID);

  g_ctrl = svc->createCharacteristic(
      CTRL_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  g_ctrl->setCallbacks(new CtrlCallbacks());

  NimBLECharacteristic *data = svc->createCharacteristic(
      DATA_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  data->setCallbacks(new DataCallbacks());

  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setName("TRMNL-X");
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();

  Log_info("BLE advertising as TRMNL-X");
}

// --------------------------------------------------------------- process ----
void ble_fw_process(void) {
  if (g_render_pending) {
    g_render_pending = false;

    Log_info("Rendering %u bytes", g_received);
    display_show_image(g_buf, (int)g_received, true);
    Log_info("Render complete");

    transfer_reset();
    notify(EV_DONE);
  }
  delay(10);
}
