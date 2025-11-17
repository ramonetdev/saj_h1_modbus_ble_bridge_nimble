#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "modbus_types.h"

// Forward declarations del cliente BLE
namespace esphome {
namespace esp32_ble_client {
  class BLEClient;
  class BLECharacteristic;
}
}

namespace esphome {
namespace modbus_ble_bridge {

class ModbusBleBridge : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

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

  uint16_t modbus_port_{502};
  uint32_t ble_response_timeout_ms_{5000};
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

  modbus_saj::ModbusTCPRequest modbus_tcp_request;
  std::vector<uint8_t> modbus_request_v;

  // Cliente BLE
  esphome::esp32_ble_client::BLEClient* parent_{nullptr};
  esphome::esp32_ble_client::BLECharacteristic* char_read_{nullptr};
  esphome::esp32_ble_client::BLECharacteristic* char_write_{nullptr};
};

}  // namespace modbus_ble_bridge
}  // namespace esphome
