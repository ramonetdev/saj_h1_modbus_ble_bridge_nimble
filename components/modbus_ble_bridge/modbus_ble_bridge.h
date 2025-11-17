#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/esp32_ble_client/ble_client.h"   // ✅ cliente BLE correcto
#include "modbus_types.h"

namespace esphome {
namespace modbus_ble_bridge {

class ModbusBleBridge : public Component {
 public:
  // --- Métodos principales ---
  void setup() override;
  void loop() override;
  void dump_config() override;

  // --- Funciones auxiliares ---
  void watchdog(uint64_t now);
  void heartbeat(uint64_t now);
  void softReconnect();
  void hardReset();
  void startModbusTCPServer();
  bool checkBLETimeout();
  void handleTCPConnection();
  void handleModbusTCP();
  void sendBLERequest(const modbus_saj::ModbusBLERequest &request);
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param);

  // --- Variables de estado ---
  uint16_t modbus_port_{502};              // Puerto TCP por defecto
  uint32_t ble_response_timeout_ms_{5000}; // Timeout BLE en ms
  uint32_t total_calls_{0};
  uint32_t total_errors_{0};
  uint32_t errlen_{0};

  bool waiting_ble_response{false};
  uint64_t waiting_since_{0};

#if defined(ARDUINO)
  WiFiServer* mb_server_{nullptr};
  WiFiClient client_;
#else
  int server_fd_{-1};
  int client_fd_{-1};
#endif

  // Última petición Modbus
  modbus_saj::ModbusTCPRequest modbus_tcp_request;

  // Buffer de petición TCP
  std::vector<uint8_t> modbus_request_v;

  // Cliente BLE del wrapper
  esphome::esp32_ble_client::BLEClient* parent_{nullptr};
  esphome::esp32_ble_client::BLECharacteristic* char_read_{nullptr};
  esphome::esp32_ble_client::BLECharacteristic* char_write_{nullptr};
};

}  // namespace modbus_ble_bridge
}  // namespace esphome
