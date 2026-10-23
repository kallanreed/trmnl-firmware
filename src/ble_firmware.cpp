#include "ble_firmware.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>

#include "battery.h"
#include "config.h"
#include "display.h"
#include "filesystem.h"
#include "pins.h"
#include "globals.h"
#include "power.h"
#include "esp_log.h"

// Deliberately not trmnl_log.h: ESP_LOGI(TAG, ) routes to ArduinoLog, which drops
// everything unless Log.begin() ran, and its LOG_STORE_ONLY default queues
// lines for an API submission that never happens without WiFi. The
// Log_*_serial variants return early past their free() calls (app_logger.cpp),
// leaking two allocations per line. ESP_LOG has neither problem.
static const char *TAG = "ble_fw";

#include "esp_heap_caps.h"
#include "esp_pm.h"

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
#define STAT_UUID "7a0b0004-5ec4-4d6b-9b1e-2f3a7c1d0004"

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

// A BEGIN that never reaches COMMIT -- client closed the tab, went out of
// range, crashed mid-send -- would otherwise hold its buffer (up to
// MAX_IMAGE_SIZE) until the next BEGIN or a disconnect. Abandon it instead,
// so the device returns to a clean state on its own rather than only when
// prompted.
static uint32_t g_last_progress_ms = 0;
#define TRANSFER_IDLE_TIMEOUT_MS 30000

static NimBLECharacteristic *g_ctrl = nullptr;
static NimBLECharacteristic *g_stat = nullptr;
// Held across panel refreshes. FastEPD's bbepWriteRow() paces the parallel
// bus with delayMicroseconds() per row; if DFS drops the CPU to min_freq
// mid-refresh those rows crawl and the 5s task watchdog fires. Pinning
// ESP_PM_CPU_FREQ_MAX for the duration is the targeted fix -- it keeps DFS
// savings everywhere except the one place that cannot tolerate them.
static esp_pm_lock_handle_t g_epd_lock = nullptr;
static uint32_t g_last_stat_ms = 0;

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
  ESP_LOGI(TAG, "BLE transfer failed, err=%u", err);
  transfer_reset();
  notify(EV_ERROR, err);
}

// ---------------------------------------------------------------- status ----
// Published so battery drain can be measured rather than guessed at. The
// gauge only refreshes on gaugeInit(), so take a fresh snapshot each time.
static void status_refresh(void) {
  if (!g_stat) return;

  battery().gaugeInit();
  const float volts = battery().readVoltage();
  const bool usb = (power().usbStatus() == UsbStatus::CONNECTED);
  const uint32_t up = millis() / 1000;
  const uint32_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
  const uint32_t dram_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;

  if (volts < 0) {
    ESP_LOGW(TAG, "gauge returned %.3f (battery_count=%d)", volts, (int)battery_count);
  }

  char json[160];
  snprintf(json, sizeof(json),
           "{\"v\":%.3f,\"usb\":%d,\"up\":%u,\"psram_kb\":%u,\"dram_kb\":%u}",
           volts, usb ? 1 : 0, up, psram_free, dram_free);

  g_stat->setValue((uint8_t *)json, strlen(json));
  g_stat->notify();
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
        g_last_progress_ms = millis();
        ESP_LOGI(TAG, "BLE BEGIN len=%u crc=%08x", g_expected_len, g_expected_crc);
        notify(EV_READY);
        break;
      }

      case OP_COMMIT: {
        if (!g_in_transfer) { fail(ERR_NOT_BEGUN); return; }
        if (g_received != g_expected_len) { fail(ERR_LEN_MISMATCH); return; }
        if (g_running_crc != g_expected_crc) {
          ESP_LOGI(TAG, "BLE CRC mismatch: got %08x want %08x", g_running_crc, g_expected_crc);
          fail(ERR_CRC);
          return;
        }
        ESP_LOGI(TAG, "BLE COMMIT ok, %u bytes", g_received);
        notify(EV_RENDERING);
        g_render_pending = true; // ble_fw_process() draws it
        break;
      }

      case OP_ABORT:
        ESP_LOGI(TAG, "BLE ABORT");
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
      ESP_LOGI(TAG, "BLE seq gap: got %u want %u", seq, g_next_seq);
      fail(ERR_SEQ);
      return;
    }
    if (g_received + n > g_expected_len) { fail(ERR_OVERFLOW); return; }

    memcpy(g_buf + g_received, payload, n);
    g_running_crc = crc32_update(g_running_crc, payload, n);
    g_received += n;
    g_next_seq++;
    g_last_progress_ms = millis();

    // Keep notification traffic light; roughly every 32 chunks.
    if ((g_next_seq & 0x1F) == 0) notify(EV_PROGRESS);
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
    ESP_LOGI(TAG, "BLE client connected");
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
    ESP_LOGI(TAG, "BLE client disconnected (reason %d)", reason);
    transfer_reset();
    NimBLEDevice::startAdvertising();
  }
};

// ------------------------------------------------------------------ init ----
void ble_fw_init(void) {
  Serial.begin(115200);
  ESP_LOGI(TAG, "TRMNL X BLE firmware starting");

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

  // BQ27427Battery::gaugeInit() bails out early when battery_count is
  // BATTERY_NONE, so the pack has to be detected first or every reading
  // comes back as the -1 sentinel. bl_init() does this at bl.cpp:225.
  battery_count = detect_battery_count();
  battery_charging = (power().chargingStatus() == ChargingStatus::CHARGING);
  ESP_LOGI(TAG, "Battery pack: %d cell(s), charging=%d",
           (int)battery_count, battery_charging ? 1 : 0);

  ESP_LOGI(TAG, "Display %ux%u ready", display_width(), display_height());

  // DFS only -- light sleep is deliberately off on this board.
  //
  // For BLE to survive light sleep the S3's BT controller needs a low-power
  // clock that keeps running: either EXT_32K_XTAL or RTC_SLOW. The X has no
  // 32.768kHz crystal (CONFIG_RTC_CLK_SRC_INT_RC=y, EXT_CRYS unset), and the
  // MAIN_XTAL source we must otherwise use cannot hold BLE timing across
  // light sleep -- advertising stops the moment USB is unplugged.
  //
  // RTC_SLOW would permit light sleep, but on internal RC its accuracy is
  // poor enough to make BLE connections unreliable. Advertising that works
  // beats advertising that saves power and does not answer.
  //
  // DFS plus BT_CTRL_MODEM_SLEEP still apply and cost nothing in function.
  esp_pm_config_t pm_cfg;
  pm_cfg.max_freq_mhz = 240;
  pm_cfg.min_freq_mhz = 80;
  pm_cfg.light_sleep_enable = false;
  esp_err_t pm_err = esp_pm_configure(&pm_cfg);
  ESP_LOGI(TAG, "esp_pm_configure: dfs %u-%uMHz light_sleep=%d err=%d",
           pm_cfg.min_freq_mhz, pm_cfg.max_freq_mhz,
           pm_cfg.light_sleep_enable, (int)pm_err);

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

  g_stat = svc->createCharacteristic(
      STAT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setName("TRMNL-X");
  adv->enableScanResponse(true);
  // 500-1000ms instead of NimBLE's ~20-100ms default. Units are 0.625ms.
  // Costs a second or so of discovery latency, saves a lot of radio time.
  adv->setMinInterval(800);
  adv->setMaxInterval(1600);
  NimBLEDevice::startAdvertising();

  status_refresh();
  ESP_LOGI(TAG, "BLE advertising as TRMNL-X");
}

// --------------------------------------------------------------- process ----
void ble_fw_process(void) {
  if (g_render_pending) {
    g_render_pending = false;

    ESP_LOGI(TAG, "Rendering %u bytes", g_received);
    const uint32_t t0 = millis();
    if (g_epd_lock) esp_pm_lock_acquire(g_epd_lock);
    display_show_image(g_buf, (int)g_received, true);
    // Deliberately NOT display_sleep(): on the X it calls bbep.deInit(),
    // which tears down the FastEPD driver without freeing the ~1.9MB of
    // buffers bbepInitPanel() allocated. The next fullUpdate() then hangs
    // on a dead driver, and re-running display_init() would leak instead.
    // It also saves nothing -- display_show_image() calls fullUpdate() with
    // bKeepOn=false (display.cpp:1963), and bbepFullUpdate() already cuts
    // the panel rails with bbepEinkPower(pState, 0) on its way out.
    if (g_epd_lock) esp_pm_lock_release(g_epd_lock);
    ESP_LOGI(TAG, "Render complete in %u ms", millis() - t0);

    transfer_reset();
    notify(EV_DONE);
    status_refresh();
  }

  if (g_in_transfer && !g_render_pending &&
      (millis() - g_last_progress_ms) > TRANSFER_IDLE_TIMEOUT_MS) {
    ESP_LOGW(TAG, "transfer idle %ums, abandoning %u/%u bytes",
             TRANSFER_IDLE_TIMEOUT_MS, g_received, g_expected_len);
    transfer_reset();
    notify(EV_ERROR, ERR_NOT_BEGUN);
  }

  if (millis() - g_last_stat_ms > 30000) {
    g_last_stat_ms = millis();
    status_refresh();
  }

  // Long enough for tickless idle to actually enter light sleep.
  delay(100);
}
