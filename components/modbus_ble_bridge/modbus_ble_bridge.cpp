#include <esphome/core/log.h>
#include <NimBLEDevice.h>
#include <queue>
#include <algorithm>
#include "esphome/components/mqtt/mqtt_client.h"

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

// Cliente NimBLE
static NimBLEClient* ble_client = nullptr;
static NimBLERemoteService* ble_service = nullptr;
static NimBLERemoteCharacteristic* char_read = nullptr;
static NimBLERemoteCharacteristic* char_write = nullptr;

void ModbusBleBridge::setup() {
  ESP_LOGI(TAG, "Setting up Modbus BLE Bridge (NimBLE)");
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P7);
  ble_client = NimBLEDevice::createClient();
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
    if (!ble_client || !ble_client->isConnected()) status = "LOST";

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
  if (ble_client && ble_client->isConnected()) {
    ble_client->disconnect();
    delay(500);
    ble_client->connect(ble_client->getPeerAddress());
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

  if (!ble_client || !ble_client->isConnected()) {
    ESP_LOGW(TAG, "BLE not connected when attempting to send request");
    return;
  }

  if (!char_write) {
    ESP_LOGE(TAG, "BLE write characteristic not found");
    return;
  }

  std::vector<uint8_t> request_frame = request.toBytes();
  if (!char_write->writeValue(request_frame.data(), request_frame.size(), false)) {
    ESP_LOGE(TAG, "BLE write failed");
    this->total_errors_++;
  } else {
    ESP_LOGD(TAG, "BLE request sent successfully");
  }
}

// Callback de notificación NimBLE
class NotifyCallback : public NimBLENotifierCallbacks {
  void onNotify(NimBLERemoteCharacteristic* c, uint8_t* data, size_t length, bool isNotify) override {
    last_good_frame_ms_ = millis();
    reconnect_attempts_ = 0;
    ESP_LOGI(TAG, "BLE notify received, length=%d", (int)length);

    // Validación y construcción de respuesta Modbus/TCP
    modbus_saj::ModbusBLEResponse ble_resp(data, length);
    modbus_saj::ModbusTCPResponse modbus_resp(modbus_tcp_request, ble_resp);
    std::vector<uint8_t> modbus_resp_bytes = modbus_resp.toBytes();

#if defined(ARDUINO)
    if (client_.connected()) {
      client_.write(modbus_resp_bytes.data(), modbus_resp_bytes.size());
      client_.stop();
    }
#else
    if (client_fd_ >= 0) {
      ::send(client_fd_, modbus_resp_bytes.data(), modbus_resp_bytes.size(), 0);
      ::close(client_fd_);
      client_fd_ = -1;
    }
#endif
    ESP_LOGD(TAG, "Sent Modbus/TCP response of %d bytes", (int)modbus_resp_bytes.size());
  }
};

}  // namespace modbus_ble_bridge
}  // namespace esphome
