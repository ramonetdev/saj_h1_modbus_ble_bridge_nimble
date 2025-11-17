#include <esphome/core/log.h>
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include <queue>
#include <algorithm>
#include "esphome/components/mqtt/mqtt_client.h"
#include "modbus_ble_bridge.h"


#define BLE_SERVICE 0xFFFF
#define BLE_CHAR_READ 0xFF02
#define BLE_CHAR_WRITE 0xFF01

namespace esphome {
namespace modbus_ble_bridge {

static const char *const TAG = "modbus_ble_bridge";

// Variables de salud/watchdog
unsigned long last_good_frame_ms_ = 0;
unsigned long last_heartbeat_ms_ = 0;
int reconnect_attempts_ = 0;
const int max_reconnect_attempts_ = 3;
const unsigned long stall_threshold_ms_ = 30000;    // 30 s sin respuesta
const unsigned long heartbeat_interval_ms_ = 10000; // cada 10 s

void ModbusBleBridge::setup() {
  ESP_LOGI(TAG, "Setting up Modbus BLE Bridge (ESPHome wrapper)");
  last_good_frame_ms_ = millis();
  this->total_calls_ = 0;
  this->total_errors_ = 0;
}

void ModbusBleBridge::loop() {
  uint64_t now = millis();
  startModbusTCPServer();
  checkBLETimeout();
  handleTCPConnection();
  handleModbusTCP();
  watchdog(now);
  heartbeat(now);
}

void ModbusBleBridge::watchdog(uint64_t now) {
  if (now - last_good_frame_ms_ > stall_threshold_ms_) {
    ESP_LOGW(TAG, "BLE stalled for %llu ms, attempting soft reconnect", now - last_good_frame_ms_);
    softReconnect();
  }
}

void ModbusBleBridge::heartbeat(uint64_t now) {
  if (now - last_heartbeat_ms_ > heartbeat_interval_ms_) {
    last_heartbeat_ms_ = now;
    unsigned long idle = now - last_good_frame_ms_;

    const char *status = "OK";
    if (idle > stall_threshold_ms_) status = "DEGRADED";
    if (!this->parent_ || !this->parent_->connected()) status = "LOST";

    ESP_LOGI(TAG, "[HB] status=%s idle_ms=%lu calls=%d errors=%d",
             status, idle, this->total_calls_, this->total_errors_);

    if (App.get_mqtt_client() != nullptr) {
      char payload[128];
      snprintf(payload, sizeof(payload),
               "{\"status\":\"%s\",\"idle_ms\":%lu,\"calls\":%d,\"errors\":%d}",
               status, idle, this->total_calls_, this->total_errors_);
      App.get_mqtt_client()->publish("saj/bridge/health", payload);
    }
  }
}

void ModbusBleBridge::softReconnect() {
  reconnect_attempts_++;
  ESP_LOGW(TAG, "Soft reconnect attempt %d", reconnect_attempts_);
  if (this->parent_ && this->parent_->connected()) {
    this->parent_->disconnect();
    delay(500);
    this->parent_->connect();
  }
  if (reconnect_attempts_ >= max_reconnect_attempts_) {
    ESP_LOGE(TAG, "Too many reconnect attempts, performing hard reset");
    hardReset();
  }
}

void ModbusBleBridge::hardReset() {
  ESP_LOGE(TAG, "Hard reset triggered");
#if defined(ARDUINO)
  if (this->client_ && this->client_.connected()) this->client_.stop();
#else
  if (this->client_fd_ >= 0) { ::close(this->client_fd_); this->client_fd_ = -1; }
#endif
  delay(200);
  esp_restart();
}

void ModbusBleBridge::sendBLERequest(const modbus_saj::ModbusBLERequest &request) {
  ESP_LOGD(TAG, "sendBLERequest invoked");

  if (!this->parent_ || !this->parent_->connected()) {
    ESP_LOGW(TAG, "BLE not connected when attempting to send request");
    return;
  }

  auto* char_write = this->parent_->get_characteristic(BLE_SERVICE, BLE_CHAR_WRITE);
  if (!char_write) {
    ESP_LOGE(TAG, "BLE write characteristic not found");
    return;
  }

  std::vector<uint8_t> request_frame = request.toBytes();
  char_write->write_value(request_frame.data(), request_frame.size(), ESP_GATT_WRITE_TYPE_NO_RSP);
  ESP_LOGD(TAG, "BLE request sent successfully");
}

void ModbusBleBridge::handleModbusTCP() {
  int avail = 0;
#if defined(ARDUINO)
  avail = this->client_.available();
#else
  if (this->client_fd_ >= 0) {
    uint8_t buf[260];
    int recv_bytes = ::recv(this->client_fd_, reinterpret_cast<char*>(buf), sizeof(buf), MSG_DONTWAIT);
    if (recv_bytes > 0) {
      avail = recv_bytes;
      modbus_request_v.assign(buf, buf + recv_bytes);
    } else if (recv_bytes == 0) {
      ESP_LOGI(TAG, "TCP client closed by peer");
      ::close(this->client_fd_);
      this->client_fd_ = -1;
      return;
    } else {
      avail = 0;
    }
  }
#endif

  if (!avail) return;
  if (this->waiting_ble_response) {
    checkBLETimeout();
    return;
  }

  ESP_LOGD(TAG, "handleModbusTCP: available bytes=%d", avail);
  if (modbus_request_v.size() < 12) {
    ESP_LOGW(TAG, "Incomplete Modbus TCP frame: %d bytes, aborting", modbus_request_v.size());
    return;
  }

#if defined(ARDUINO)
  this->client_.clear();
#endif

  modbus_tcp_request = modbus_saj::ModbusTCPRequest(modbus_request_v);
  modbus_saj::ModbusBLERequest ble_req(modbus_tcp_request);

  ESP_LOGD(TAG, "Sending BLE request (seq=%d)", ble_req.getBLETransactionId());
  this->sendBLERequest(ble_req);
  this->total_calls_++;
  this->waiting_ble_response = true;
  this->waiting_since_ = millis();
}

void ModbusBleBridge::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                          esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_NOTIFY_EVT: {
      last_good_frame_ms_ = millis();
      reconnect_attempts_ = 0;
      ESP_LOGI(TAG, "BLE notify received");
      // Aquí parseas la respuesta Modbus como antes
      break;
    }
    default:
      break;
  }
}

}  // namespace modbus_ble_bridge
}  // namespace esphome
