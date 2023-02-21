#include "pn7160.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pn7160 {

static const char *const TAG = "pn7160";

void PN7160::nci_fsm_transition_() {
  switch (this->state_) {
    case PN7160State::RESET:
      if (this->reset_core_(true, true) != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to reset NCI core");
        this->mark_failed();
        this->state_ = PN7160State::FAILED;
      } else {
        this->state_ = PN7160State::INIT;
      }
      // fall through

    case PN7160State::INIT:
      if (this->init_core_(true) != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to init NCI core");
        this->mark_failed();
        this->state_ = PN7160State::FAILED;
      } else {
        this->state_ = PN7160State::CONFIG;
      }
      // fall through

    case PN7160State::CONFIG:
      if (this->send_config_() != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to send config");
        this->mark_failed();
        this->state_ = PN7160State::FAILED;
      } else {
        this->state_ = PN7160State::SET_MODE;
      }
      // fall through

    case PN7160State::SET_MODE:
      if (this->set_mode_(PN7160Mode::READ_WRITE) != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set mode");
        this->mark_failed();
        this->state_ = PN7160State::FAILED;
      } else {
        this->state_ = PN7160State::DISCOVERY;
      }
      return;

    case PN7160State::DISCOVERY:
      if (this->start_discovery_(PN7160Mode::READ_WRITE) != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to start discovery");
        this->mark_failed();
        this->state_ = PN7160State::FAILED;
      } else {
        this->state_ = PN7160State::WAITING_FOR_TAG;
      }
      return;

    case PN7160State::WAITING_FOR_REMOVAL:
      this->presence_check_();

    // These cases are waiting for NOTIFICATION messages
    case PN7160State::WAITING_FOR_TAG:
    case PN7160State::DEACTIVATING:
    case PN7160State::SELECTING:
      break;

    case PN7160State::FAILED:
    case PN7160State::NONE:
    default:
      return;
  }

  if (!this->irq_pin_->digital_read()) {
    // this->current_uid_ = {};
    return;  // No data to read
  }

  std::vector<uint8_t> response;
  if (!this->read_data_(response, 5, false)) {
    // No data
    // this->current_uid_ = {};
    return;
  }

  uint8_t mt = response[0] & MT_MASK;
  uint8_t gid = response[0] & GID_MASK;
  uint8_t oid = response[1] & OID_MASK;

  switch (mt) {
    case MT_CTRL_RESPONSE:
      ESP_LOGW(TAG, "Unimplemented response: GID: 0x%.2X  OID: 0x%.2X", gid, oid);
      break;

    case MT_CTRL_NOTIFICATION:
      if (gid == RF_GID) {
        switch (oid) {
          case RF_INTF_ACTIVATED_OID: {
            // a single endpoint was detected and automatically activated
            uint8_t interface = response[4];
            uint8_t protocol = response[5];
            uint8_t mode_tech = response[6];
            uint8_t max_size = response[7];
            // uint8_t initial_cred = response[8];
            // uint8_t rf_tech_params_len = response[9];
            // uint8_t rf_tech_params = response[10];

            this->current_protocol_ = protocol;
            ESP_LOGD(TAG, "Endpoint detected: interface: 0x%.2X, protocol: 0x%.2X, mode&tech: 0x%.2X, max payload: %u",
                     interface, protocol, mode_tech, max_size);
            auto tag = this->build_tag_(mode_tech, std::vector<uint8_t>(response.begin() + 10, response.end()));
            if (tag == nullptr) {
              ESP_LOGE(TAG, "Could not build tag!");
              return;
            }

            if (this->check_for_tag_(tag)) {
              ESP_LOGD(TAG, "I have seen this tag before!");
              this->state_ = PN7160State::WAITING_FOR_REMOVAL;
              return;
            }

            std::vector<uint8_t> write_data = {nfc::MIFARE_CMD_READ, nfc::MIFARE_ULTRALIGHT_DATA_START_PAGE};
            if (!this->write_data_and_read_(write_data, response, 50)) {
              ESP_LOGE(TAG, "Error reading tag data");
            } else {
              tag->set_ndef_message(make_unique<nfc::NdefMessage>(nfc::NdefMessage(write_data)));
              for (size_t i = 0; i < response.size(); i++) {
                ESP_LOGD(TAG, "Tag data read: 0x%.2x", response[i]);
              }
            }

            this->tag_.push_back(*tag.get());

            for (auto *trigger : this->triggers_ontag_) {
              trigger->process(tag);
            }

            this->state_ = PN7160State::NONE;
            this->set_timeout(200, [this]() { this->state_ = PN7160State::WAITING_FOR_REMOVAL; });
            return;
          }
          case RF_DISCOVER_OID: {
            ESP_LOGW(TAG, "One tag at a time, please");
            break;
          }
          case RF_DEACTIVATE_OID: {
            ESP_LOGW(TAG, "Deactivate OID received: type: 0x%.2X, reason: 0x%.2X", response[3], response[4]);
            if (this->next_function_ != nullptr) {
              this->next_function_();
              this->next_function_ = nullptr;
            }
            break;
            // if (response[3] == DEACTIVATION_TYPE_SLEEP) {
            // }
          }
          default:
            ESP_LOGW(TAG, "Unsupported RF OID received: 0x%.2X", oid);
        }
      } else if (gid == NCI_CORE_GID) {
        switch (oid) {
          case NCI_CORE_GENERIC_ERROR_OID:
            switch (response[3]) {
              case DISCOVERY_TARGET_ACTIVATION_FAILED:
                // Tag removed
                ESP_LOGD(TAG, "Purging all tags...");
                this->tag_.clear();
                if (!this->deactivate_(DEACTIVATION_TYPE_IDLE)) {
                  ESP_LOGE(TAG, "Error deactivating");
                }
                this->state_ = PN7160State::DISCOVERY;
                break;
            }
            break;

          default:
            ESP_LOGW(TAG, "Unsupported NCI Core OID received: 0x%.2X", oid);
        }
      }
      break;

    case MT_CTRL_COMMAND:

    default:
      break;
  }
}

void PN7160::setup() {
  this->dwl_req_pin_->setup();
  this->irq_pin_->setup();
  this->ven_pin_->setup();
  this->wkup_req_pin_->setup();
  this->spi_setup();

  this->nci_fsm_transition_();  // kick off reset & init processes
}

uint8_t PN7160::reset_core_(bool reset_config, bool power) {
  this->dwl_req_pin_->digital_write(false);
  delay(2);

  if (power) {
    this->ven_pin_->digital_write(true);
    delay(2);
    this->ven_pin_->digital_write(false);
    delay(2);
    this->ven_pin_->digital_write(true);
    delay(50);
  }

  std::vector<uint8_t> response;
  std::vector<uint8_t> write_data;

  if (reset_config)
    write_data.push_back(0x01);
  else
    write_data.push_back(0x00);

  if (!this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_RESET_OID, write_data, response, 50)) {
    ESP_LOGE(TAG, "Error sending reset command");
    return STATUS_FAILED;
  }

  if (response[3] != STATUS_OK) {
    ESP_LOGE(TAG, "Invalid reset response: 0x%.2X", response[3]);
    return response[3];
  }

  // there may also be a notification in response to the reset command. we don't care but still need to read it back
  this->read_data_(response, 50);

  return STATUS_OK;
}

uint8_t PN7160::init_core_(bool store_report) {
  std::vector<uint8_t> response;

  if (!this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_INIT_OID, {}, response)) {
    ESP_LOGE(TAG, "Error sending initialise command");
    return STATUS_FAILED;
  }

  if (response[3] != STATUS_OK) {
    ESP_LOGE(TAG, "Invalid initialise response: 0x%.2X", response[3]);
  } else if (store_report) {
    this->version_[0] = response[17 + response[8]];
    this->version_[1] = response[18 + response[8]];
    this->version_[2] = response[19 + response[8]];

    if (this->version_[0] == 0x08) {
      this->generation_ = 1;
    } else if (this->version_[0] == 0x10) {
      this->generation_ = 2;
    }

    ESP_LOGD(TAG, "Core Version: 0x%02X", this->version_[0]);
    ESP_LOGD(TAG, "Firmware version: 0x%02X.0x%02X", this->version_[1], this->version_[2]);
    ESP_LOGD(TAG, "Generation: %d", this->generation_);
  }

  return response[3];
}

uint8_t PN7160::send_config_() {
  // uint8_t currentTS[32] = __TIMESTAMP__;
  std::vector<uint8_t> response;

  if (!this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, CORE_CONFIG, sizeof(CORE_CONFIG), response)) {
    ESP_LOGE(TAG, "Error sending core config");
    return STATUS_FAILED;
  }

  response.clear();

  // if (!this->write_ctrl_and_read_(0x2f, 0x00, {0x01}, response)) {
  //   ESP_LOGE(TAG, "Error sending standby config");
  //   return STATUS_FAILED;
  // }

  // if (response[3] != 0x00) {
  //   ESP_LOGE(TAG, "Incorrect standby config response");
  //   return STATUS_FAILED;
  // }

  // response.clear();

  // std::vector<uint8_t> read_ts = {0x01, 0xA0, 0x14};
  // if (this->generation_ == 1)
  //   read_ts[2] = 0x0F;
  // if (!this->write_ctrl_and_read_(0x20, 0x03, read_ts, response)) {
  //   ESP_LOGE(TAG, "Error sending read timestamp config");
  //   return STATUS_FAILED;
  // }

  // if (response[3] != 0x00) {
  //   ESP_LOGE(TAG, "Incorrect timestamp response");
  //   return STATUS_FAILED;
  // }

  // if (!memcmp(&response[8], currentTS, sizeof(currentTS))) {
  //   // No need to update as settings could not have changed since last setup.
  // } else {
  // response.clear();

  if (!this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, CORE_CONF_EXTENSION,
                                  sizeof(CORE_CONF_EXTENSION), response)) {
    ESP_LOGE(TAG, "Error sending core config extension");
    return STATUS_FAILED;
  }

  response.clear();

  if (!this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, CLOCK_CONFIG, sizeof(CLOCK_CONFIG),
                                  response)) {
    ESP_LOGE(TAG, "Error sending clock config");
    return STATUS_FAILED;
  }

  response.clear();

  if (!this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, RF_TVDD_CONFIG, sizeof(RF_TVDD_CONFIG),
                                  response)) {
    ESP_LOGE(TAG, "Error sending RF TVDD config");
    return STATUS_FAILED;
  }

  // response.clear();

  // if (!this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, RF_CONF, sizeof(RF_CONF), response, 20)) {
  //   ESP_LOGE(TAG, "Error sending RF config");
  //   return STATUS_FAILED;
  // }

  // response.clear();

  // std::vector<uint8_t> write_ts = {0x01, 0xA0, 0x14, 0x20};
  // write_ts.resize(36);

  // if (this->generation_ == 1)
  //   write_ts[3] = 0x0F;
  // memcpy(&write_ts[4], currentTS, sizeof(currentTS));
  // if (!this->write_ctrl_and_read_(0x20, 0x02, write_ts, response)) {
  //   ESP_LOGE(TAG, "Error sending TS config");
  //   return STATUS_FAILED;
  // }

  // if (response[3] != 0x00 || response[4] != 0x00) {
  //   ESP_LOGE(TAG, "Incorrect TS config response");
  //   return STATUS_FAILED;
  // }

  // response.clear();
  // }

  // if (!this->reset_core_(false, false))
  //   return STATUS_FAILED;

  // if (!this->init_core_(response))
  //   return STATUS_FAILED;

  return STATUS_OK;
}

uint8_t PN7160::set_mode_(PN7160Mode mode) {
  if (mode == PN7160Mode::READ_WRITE) {
    std::vector<uint8_t> response;
    std::vector<uint8_t> write_data;

    write_data.resize(1 + sizeof(READ_WRITE_MODE));
    write_data[0] = sizeof(READ_WRITE_MODE) / 3;
    memcpy(&write_data[1], READ_WRITE_MODE, sizeof(READ_WRITE_MODE));

    if (!this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_MAP_OID, write_data, response, 10)) {
      ESP_LOGE(TAG, "Error sending discover map");
      return STATUS_FAILED;
    }
    return STATUS_OK;
  }
  return STATUS_FAILED;
}

uint8_t PN7160::start_discovery_(PN7160Mode mode) {
  std::vector<uint8_t> response;
  uint8_t length = sizeof(DISCOVERY_READ_WRITE);
  std::vector<uint8_t> write_data = std::vector<uint8_t>((length * 2) + 1);

  write_data[0] = length;
  for (uint8_t i = 0; i < length; i++) {
    write_data[(i * 2) + 1] = DISCOVERY_READ_WRITE[i];
    write_data[(i * 2) + 2] = 0x01;  // RF Technology and Mode will be executed in every discovery period
  }

  if (!this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_OID, write_data, response)) {
    ESP_LOGE(TAG, "Error sending discovery");
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

bool PN7160::deactivate_(uint8_t type) {
  std::vector<uint8_t> response;
  if (!this->write_ctrl_and_read_(RF_GID, RF_DEACTIVATE_OID, {type}, response, 20)) {
    ESP_LOGE(TAG, "Error sending deactivate");
    return false;
  }

  this->state_ = PN7160State::DEACTIVATING;
  return true;
}

std::unique_ptr<nfc::NfcTag> PN7160::build_tag_(uint8_t mode_tech, const std::vector<uint8_t> &data) {
  switch (mode_tech) {
    case (MODE_POLL | TECH_PASSIVE_NFCA): {
      uint8_t uid_length = data[2];
      std::vector<uint8_t> uid(data.begin() + 3, data.begin() + 3 + uid_length);
      return make_unique<nfc::NfcTag>(uid, nfc::MIFARE_CLASSIC);
    }
  }
  return nullptr;
}

bool PN7160::check_for_tag_(std::unique_ptr<nfc::NfcTag> &tag) {
  for (auto cur_tag : this->tag_) {
    if (tag->get_uid().size() == cur_tag.get_uid().size()) {  // if the uid sizes match, this is a tag of interest
      bool same_uid = true;
      for (size_t i = 0; i < cur_tag.get_uid().size(); i++) {
        // ESP_LOGD(TAG, "UID: %u", tag->get_uid()[i]);
        same_uid &= cur_tag.get_uid()[i] == tag->get_uid()[i];
      }
      if (same_uid) {
        return true;
      }
    }
  }
  return false;
}

void PN7160::select_tag_() {
  std::vector<uint8_t> response;
  uint8_t interface = INTF_UNDETERMINED;
  switch (this->current_protocol_) {
    case PROT_ISODEP:
      interface = INTF_ISODEP;
      break;
    case PROT_NFCDEP:
      interface = INTF_NFCDEP;
      break;
    case PROT_MIFARE:
      interface = INTF_TAGCMD;
      break;
    default:
      interface = INTF_FRAME;
      break;
  }
  if (!this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_SELECT_OID, {0x01, this->current_protocol_, interface},
                                  response)) {
    ESP_LOGE(TAG, "Error sending discovery select");
    return;
  }
  this->state_ = PN7160State::SELECTING;
}

bool PN7160::presence_check_() {
  std::vector<uint8_t> response;
  switch (this->current_protocol_) {
    case PROT_MIFARE: {
      this->deactivate_(DEACTIVATION_TYPE_SLEEP);
      this->next_function_ = std::bind(&PN7160::select_tag_, this);
    }
    case PROT_T2T: {
      this->deactivate_(DEACTIVATION_TYPE_SLEEP);
      this->next_function_ = std::bind(&PN7160::select_tag_, this);
    }
    default:
      break;
  }
  return false;
}

void PN7160::dump_config() {
  ESP_LOGCONFIG(TAG, "PN7160:");
  LOG_PIN("  CS Pin: ", this->cs_);
}

void PN7160::loop() { this->nci_fsm_transition_(); }

bool PN7160::write_ctrl_and_read_(uint8_t gid, uint8_t oid, const std::vector<uint8_t> &data,
                                  std::vector<uint8_t> &response, uint16_t timeout, bool warn) {
  return this->write_ctrl_and_read_(gid, oid, data.data(), data.size(), response, timeout, warn);
}

bool PN7160::write_ctrl_and_read_(uint8_t gid, uint8_t oid, const uint8_t *data, const uint8_t len,
                                  std::vector<uint8_t> &response, uint16_t timeout, bool warn) {
  if (gid != (gid & GID_MASK)) {
    ESP_LOGE(TAG, "Invalid GID");
    return false;
  }
  if (oid != (oid & OID_MASK)) {
    ESP_LOGE(TAG, "Invalid OID");
    return false;
  }

  std::vector<uint8_t> write_data = std::vector<uint8_t>(len + 3);
  write_data[0] = MT_CTRL_COMMAND | (gid & GID_MASK);
  write_data[1] = oid & OID_MASK;
  write_data[2] = len;

  if (len > 0) {
    for (uint8_t i = 0; i < len; i++) {
      write_data[i + 3] = data[i];
    }
  }

  if (!this->write_and_read_(write_data, response, timeout, warn)) {
    return false;
  }

  if ((response[0] & GID_MASK) != gid || (response[1] & OID_MASK) != oid) {
    ESP_LOGE(TAG, "Incorrect response: 0x%.2x 0x%.2x 0x%.2x ", response[0], response[1], response[2]);
    return false;
  }

  if (response[3] != STATUS_OK) {
    ESP_LOGE(TAG, "Error in response: %d", response[3]);
    return false;
  }

  return true;
}

bool PN7160::write_data_and_read_(std::vector<uint8_t> &data, std::vector<uint8_t> &response, uint16_t timeout,
                                  bool warn) {
  std::vector<uint8_t> write_data = std::vector<uint8_t>(data.size() + 3);
  write_data[0] = MT_DATA;
  write_data[1] = 0x00;
  write_data[2] = data.size();
  for (uint8_t i = 0; i < data.size(); i++) {
    write_data[i + 3] = data[i];
  }

  if (!this->write_and_read_(write_data, response, timeout, warn)) {
    ESP_LOGE(TAG, "Error requesting read from endpoint");
    return false;
  }

  if ((response[0] != (MT_CTRL_NOTIFICATION | NCI_CORE_GID)) || (response[1] != NCI_CORE_CONN_CREDITS_OID) ||
      (response[2] != 3)) {
    ESP_LOGE(TAG, "Incorrect response: 0x%.2x 0x%.2x 0x%.2x ", response[0], response[1], response[2]);
    return false;
  }

  if (!this->wait_for_irq_(timeout, false)) {
    return false;
  }

  if (!this->read_data_(response, timeout)) {
    ESP_LOGE(TAG, "Error reading data from endpoint");
    return false;
  }

  return true;
}

bool PN7160::write_and_read_(std::vector<uint8_t> &data, std::vector<uint8_t> &response, uint16_t timeout, bool warn) {
  if (!this->write_data_(data)) {
    ESP_LOGE(TAG, "Error sending data");
    return false;
  }

  if (!this->read_data_(response, timeout)) {
    if (warn)
      ESP_LOGE(TAG, "Error reading response");
    return false;
  }

  return true;
}

bool PN7160::write_data_(const std::vector<uint8_t> &data) {
  this->enable();
  this->write_byte(TDD_SPI_WRITE);  // send "transfer direction detector"
  this->write_array(data.data(), data.size());
  this->disable();
  return true;
}

bool PN7160::wait_for_irq_(uint16_t timeout, bool state, bool warn) {
  uint32_t start_time = millis();
  uint32_t count = 0;
  while (true) {
    if (this->irq_pin_->digital_read() == state) {
      return true;
    }
    count++;

    if (millis() - start_time > timeout) {
      if (warn)
        ESP_LOGW(TAG, "Timed out waiting for data (this can be normal)");
      return false;
    }
  }
}

bool PN7160::read_data_(std::vector<uint8_t> &data, uint16_t timeout, bool warn) {
  if (!this->wait_for_irq_(timeout, true, warn)) {
    return false;
  }

  data.resize(3);
  this->enable();
  this->write_byte(TDD_SPI_READ);  // send "transfer direction detector"
  this->read_array(data.data(), 3);

  uint8_t length = data[2];
  if (length > 0) {
    data.resize(length + 3);
    uint8_t start = 0;

    while (start < length) {
      if (!this->wait_for_irq_(timeout)) {
        return false;
      }

      uint8_t read_length = std::min(length - start, 10);
      this->read_array(data.data() + 3 + start, read_length);
      start += read_length;
    }
  }
  this->disable();
  return true;
}

}  // namespace pn7160
}  // namespace esphome
