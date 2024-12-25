#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#if defined(ESP32)
#include "driver/uart.h"
#endif

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

/*
  Command dumps from OVIEW:

  SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
  STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
  OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
  CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
  PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
  PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c
*/

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);
  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
      } else {
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Target position: %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {
#if defined(ESP32)
  uart_config_t uart_config = {
      .baud_rate = BAUD_WORK,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  uart_param_config(this->uart_num_, &uart_config);
  uart_driver_install(this->uart_num_, 256, 0, 0, NULL, 0);
  uart_set_pin(this->uart_num_, this->tx_pin_, this->rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#else
  _uart = uart_init(this->uart_num_, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, this->tx_pin_, 256, false);
#endif
}

void NiceBusT4::loop() {
  uint32_t now = millis();
  if ((now - this->last_update_) > 10000) {
    if (!this->init_ok) {
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00));
    } else {
      request_position();
    }
    this->last_update_ = now;
  }

  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  }

#if defined(ESP32)
  uint8_t c;
  size_t buffered_length;
  uart_get_buffered_data_len(this->uart_num_, &buffered_length);

  if (buffered_length > 0) {
    while (uart_read_bytes(this->uart_num_, &c, 1, 10 / portTICK_PERIOD_MS) > 0) {
      this->handle_char_(c);
      this->last_uart_byte_ = millis();
    }
  }
#else
  while (uart_rx_available(_uart) > 0) {
    uint8_t c = (uint8_t)uart_read_char(_uart);
    this->handle_char_(c);
    this->last_uart_byte_ = now;
  }
#endif

  if (this->ready_to_tx_ && !this->tx_buffer_.empty()) {
    this->send_array_cmd(this->tx_buffer_.front());
    this->tx_buffer_.pop();
    this->ready_to_tx_ = false;
  }
}

void NiceBusT4::handle_char_(uint8_t c) {
  if (this->rx_message_.size() > 256) {
    ESP_LOGW(TAG, "Buffer overflow detected. Clearing message buffer.");
    this->rx_message_.clear();
  }

  this->rx_message_.push_back(c);
  if (!this->validate_message_()) {
    this->rx_message_.clear();
  }
}

bool NiceBusT4::validate_message_() {
  uint8_t packet_size = this->rx_message_[2];
  uint8_t length = packet_size + 3;
  if (this->rx_message_.size() < length) return false;

  uint8_t crc1 = 0;
  for (size_t i = 3; i <= 8; i++) {
    crc1 ^= this->rx_message_[i];
  }
  if (this->rx_message_[9] != crc1) return false;

  uint8_t crc2 = 0;
  for (size_t i = 10; i < length - 1; i++) {
    crc2 ^= this->rx_message_[i];
  }
  if (this->rx_message_[length - 1] != crc2) return false;

  parse_status_packet(this->rx_message_);
  this->rx_message_.clear();
  return true;
}

void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
  if ((data[1] == 0x0d) && (data[13] == 0xFD)) {
    ESP_LOGE(TAG, "Command unavailable for this device");
  }

  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) {
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG, "Data string: %s", str.c_str());
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG, "HEX Data: %s", pretty_data.c_str());

    if ((data[6] == INF) && (data[9] == FOR_CU) && (data[11] == GET - 0x80) && (data[13] == NOERR)) {
      ESP_LOGI(TAG, "Response to request %X", data[10]);
      switch (data[10]) {
        case TYPE_M:
          switch (data[14]) {
            case SLIDING:
              this->class_gate_ = SLIDING;
              ESP_LOGD(TAG, "Gate type: Sliding");
              break;
            case SECTIONAL:
              this->class_gate_ = SECTIONAL;
              ESP_LOGD(TAG, "Gate type: Sectional");
              break;
            case SWING:
              this->class_gate_ = SWING;
              ESP_LOGD(TAG, "Gate type: Swing");
              break;
            case BARRIER:
              this->class_gate_ = BARRIER;
              ESP_LOGD(TAG, "Gate type: Barrier");
              break;
            case UPANDOVER:
              this->class_gate_ = UPANDOVER;
              ESP_LOGD(TAG, "Gate type: Up-and-over");
              break;
          }
          break;
        case INF_IO:
          switch (data[16]) {
            case 0x00:
              ESP_LOGI(TAG, "No limit switch triggered");
              break;
            case 0x01:
              ESP_LOGI(TAG, "Closed limit switch triggered");
              this->position = COVER_CLOSED;
              break;
            case 0x02:
              ESP_LOGI(TAG, "Open limit switch triggered");
              this->position = COVER_OPEN;
              break;
          }
          this->publish_state_if_changed();
          break;
        case MAX_OPN:
          if (is_walky) {
            this->_max_opn = data[15];
            this->_pos_opn = data[15];
          } else {
            this->_max_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Max encoder position: %d", this->_max_opn);
          break;
        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Closed position: %d", this->_pos_cls);
          break;
        case POS_MAX:
          if (((data[14] << 8) + data[15]) > 0x00) {
            this->_pos_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Open position: %d", this->_pos_opn);
          break;
        case CUR_POS:
          if (is_walky)
            update_position(data[15]);
          else
            update_position((data[14] << 8) + data[15]);
          break;
        case INF_STATUS:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "Gate opened");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_OPEN;
              if (this->_max_opn == 0) {
                this->_max_opn = this->_pos_opn = this->_pos_usl;
                ESP_LOGI(TAG, "Opened position calibrated");
              }
              break;
            case CLOSED:
              ESP_LOGI(TAG, "Gate closed");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_CLOSED;
              break;
            case STA_OPENING:
              ESP_LOGI(TAG, "Gate opening");
              this->current_operation = COVER_OPERATION_OPENING;
              break;
            case STA_CLOSING:
              ESP_LOGI(TAG, "Gate closing");
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
            case STOPPED:
              ESP_LOGI(TAG, "Gate stopped");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            default:
              ESP_LOGI(TAG, "Unknown status: %X", data[14]);
          }
          this->publish_state_if_changed();
          break;
      }
    }
  }
}

}  // namespace bus_t4
}  // namespace esphome
