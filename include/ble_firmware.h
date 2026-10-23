#ifndef BLE_FIRMWARE_H
#define BLE_FIRMWARE_H

/**
 * BLE image transport for TRMNL X.
 *
 * Replaces the cloud/WiFi business logic in bl.cpp with a BLE GATT server.
 * A browser (Web Bluetooth) pushes an encoded image; we reassemble it in
 * PSRAM, verify a CRC32, and hand it to display_show_image(), which sniffs
 * the magic bytes and decodes PNG / JPEG / BMP / G5 itself.
 */

/** Hardware bring-up + BLE start. Call once from setup(). */
void ble_fw_init(void);

/** Pump; renders a completed transfer. Call from loop(). */
void ble_fw_process(void);

#endif // BLE_FIRMWARE_H
