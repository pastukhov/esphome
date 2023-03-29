#include "automation.h"
#include "pn7160.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pn7160 {

static const char *const TAG = "pn7160";

void PN7160::setup() {
  this->dwl_req_pin_->setup();
  this->irq_pin_->setup();
  this->ven_pin_->setup();
  this->wkup_req_pin_->setup();
  this->spi_setup();

  for (auto *bs : this->binary_sensors_) {
    bs->publish_initial_state(false);
  }

  this->nci_fsm_transition_();  // kick off reset & init processes
}

void PN7160::dump_config() {
  ESP_LOGCONFIG(TAG, "PN7160:");
  LOG_PIN("  CS Pin: ", this->cs_);

  for (auto *child : this->binary_sensors_) {
    LOG_BINARY_SENSOR("  ", "Tag", child);
  }
}

void PN7160::loop() {
  this->nci_fsm_transition_();
  this->purge_old_tags_();
}

void PN7160::set_tag_emulation_message(std::shared_ptr<nfc::NdefMessage> message) {
  this->card_emulation_message_ = message;
  ESP_LOGD(TAG, "Tag emulation message set");
}

void PN7160::set_tag_emulation_message(optional<std::string> message, optional<bool> include_android_app_record) {
  if (!message.has_value()) {
    return;
  }

  nfc::NdefMessage ndef_message;

  ndef_message.add_uri_record(message.value());

  if (!include_android_app_record.has_value() || include_android_app_record.value()) {
    std::string ext_record_type = "android.com:pkg";
    std::string ext_record_payload = "io.homeassistant.companion.android";
    nfc::NdefRecord ext_record;
    ext_record.set_tnf(nfc::TNF_EXTERNAL_TYPE);
    ext_record.set_type(ext_record_type);
    ext_record.set_payload(ext_record_payload);
    ndef_message.add_record(make_unique<nfc::NdefRecord>(ext_record));
  }

  this->card_emulation_message_ = make_unique<nfc::NdefMessage>(ndef_message);
  ESP_LOGD(TAG, "Tag emulation message set");
}

void PN7160::set_tag_emulation_off() {
  this->listening_enabled_ = false;
  this->config_update_pending_ = true;
  if (this->nci_state_ == NCIState::RFST_DISCOVERY) {
    if (this->deactivate_(DEACTIVATION_TYPE_IDLE, NFCC_FULL_TIMEOUT) == STATUS_OK) {
      this->nci_fsm_set_state_(NCIState::RFST_IDLE);
    } else {
      this->nci_fsm_set_state_(NCIState::NFCC_RESET);
    }
  }
  ESP_LOGD(TAG, "Tag emulation disabled");
}

void PN7160::set_tag_emulation_on() {
  if (this->card_emulation_message_ == nullptr) {
    ESP_LOGE(TAG, "No NDEF message is set; tag emulation cannot be enabled");
    return;
  }

  this->listening_enabled_ = true;
  this->config_update_pending_ = true;
  if (this->nci_state_ == NCIState::RFST_DISCOVERY) {
    if (this->deactivate_(DEACTIVATION_TYPE_IDLE, NFCC_FULL_TIMEOUT) == STATUS_OK) {
      this->nci_fsm_set_state_(NCIState::RFST_IDLE);
    } else {
      this->nci_fsm_set_state_(NCIState::NFCC_RESET);
    }
  }
  ESP_LOGD(TAG, "Tag emulation enabled");
}

void PN7160::read_mode() {
  this->next_task_ = EP_READ;
  ESP_LOGD(TAG, "Waiting to read next tag");
}

void PN7160::clean_mode() {
  this->next_task_ = EP_CLEAN;
  ESP_LOGD(TAG, "Waiting to clean next tag");
}

void PN7160::format_mode() {
  this->next_task_ = EP_FORMAT;
  ESP_LOGD(TAG, "Waiting to format next tag");
}

void PN7160::write_mode() {
  if (this->next_task_message_to_write_ == nullptr) {
    ESP_LOGW(TAG, "Message to write must be set before setting write mode");
    return;
  }

  this->next_task_ = EP_WRITE;
  ESP_LOGD(TAG, "Waiting to write next tag");
}

void PN7160::set_tag_write_message(std::shared_ptr<nfc::NdefMessage> message) {
  this->next_task_message_to_write_ = message;
  ESP_LOGD(TAG, "Message to write has been set");
}

void PN7160::set_tag_write_message(optional<std::string> message, optional<bool> include_android_app_record) {
  if (!message.has_value()) {
    return;
  }

  nfc::NdefMessage ndef_message;

  ndef_message.add_uri_record(message.value());

  if (!include_android_app_record.has_value() || include_android_app_record.value()) {
    std::string ext_record_type = "android.com:pkg";
    std::string ext_record_payload = "io.homeassistant.companion.android";
    nfc::NdefRecord ext_record;
    ext_record.set_tnf(nfc::TNF_EXTERNAL_TYPE);
    ext_record.set_type(ext_record_type);
    ext_record.set_payload(ext_record_payload);
    ndef_message.add_record(make_unique<nfc::NdefRecord>(ext_record));
  }

  this->next_task_message_to_write_ = make_unique<nfc::NdefMessage>(ndef_message);
  ESP_LOGD(TAG, "Message to write has been set");
}

void PN7160::init_failure_handler_() {
  ESP_LOGE(TAG, "Communication failure");
  if (this->fail_count_ < NFCC_MAX_COMM_FAILS) {
    ESP_LOGE(TAG, "Initialization attempt %u failed, retrying...", this->fail_count_++);
    this->nci_fsm_set_state_(NCIState::NFCC_RESET);
  } else {
    ESP_LOGE(TAG, "Too many initialization failures -- check device connections");
    this->mark_failed();
    this->nci_fsm_set_state_(NCIState::FAILED);
  }
}

uint8_t PN7160::reset_core_(const bool reset_config, const bool power) {
  this->dwl_req_pin_->digital_write(false);
  delay(NFCC_DEFAULT_TIMEOUT);

  if (power) {
    this->ven_pin_->digital_write(true);
    delay(NFCC_DEFAULT_TIMEOUT);
    this->ven_pin_->digital_write(false);
    delay(NFCC_DEFAULT_TIMEOUT);
    this->ven_pin_->digital_write(true);
    delay(NFCC_INIT_TIMEOUT);
  }

  std::vector<uint8_t> response;
  std::vector<uint8_t> write_data;

  if (reset_config) {
    write_data.push_back(0x01);
  } else {
    write_data.push_back(0x00);
  }

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_RESET_OID, write_data, response, NFCC_FULL_TIMEOUT) !=
      STATUS_OK) {
    ESP_LOGE(TAG, "Error sending reset command");
    return STATUS_FAILED;
  }

  if (response[3] != STATUS_OK) {
    ESP_LOGE(TAG, "Invalid reset response: 0x%02X", response[3]);
    return response[3];
  }

  // read reset notification
  if (this->read_data_(response, NFCC_INIT_TIMEOUT) != STATUS_OK) {
    ESP_LOGE(TAG, "Reset notification was not received");
    return STATUS_FAILED;
  }
  // verify reset notification
  if ((response[0] != MT_CTRL_NOTIFICATION) || (response[2] != 9) || (response[3] != 0x02)) {
    ESP_LOGE(TAG, "Reset notification was malformed");
    return STATUS_FAILED;
  }

  ESP_LOGD(TAG, "Configuration %s", response[4] ? "reset" : "retained");
  ESP_LOGD(TAG, "NCI version: %s", response[5] == 0x20 ? "2.0" : "1.0");
  ESP_LOGD(TAG, "Manufacturer ID: 0x%02X", response[6]);
  response.erase(response.begin(), response.begin() + 8);
  ESP_LOGD(TAG, "Manufacturer info: %s", nfc::format_bytes(response).c_str());

  return STATUS_OK;
}

uint8_t PN7160::init_core_(const bool store_report) {
  std::vector<uint8_t> response;

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_INIT_OID, {}, response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error sending initialise command");
    return STATUS_FAILED;
  }

  if (response[3] != STATUS_OK) {
    ESP_LOGE(TAG, "Invalid initialise response: 0x%02X", response[3]);
  } else if (store_report) {
    this->hw_version_ = response[17 + response[8]];
    this->rom_code_version_ = response[18 + response[8]];
    this->flash_major_version_ = response[19 + response[8]];
    this->flash_minor_version_ = response[20 + response[8]];
  }

  ESP_LOGD(TAG, "Hardware version: %u", this->hw_version_);
  ESP_LOGD(TAG, "ROM code version: %u", this->rom_code_version_);
  ESP_LOGD(TAG, "FLASH major version: %u", this->flash_major_version_);
  ESP_LOGD(TAG, "FLASH minor version: %u", this->flash_minor_version_);
  ESP_LOGD(TAG, "Features[0]: 0x%02X", response[4]);
  ESP_LOGD(TAG, "Features[1]: 0x%02X", response[5]);
  ESP_LOGD(TAG, "Features[2]: 0x%02X", response[6]);
  ESP_LOGD(TAG, "Features[3]: 0x%02X", response[7]);

  return response[3];
}

uint8_t PN7160::send_init_config_() {
  std::vector<uint8_t> response;

  if (this->write_ctrl_and_read_(NCI_PROPRIETARY_GID, NCI_CORE_SET_CONFIG_OID, {}, response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error enabling proprietary extensions");
    return STATUS_FAILED;
  }

  response.clear();

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, PMU_CFG, sizeof(PMU_CFG), response) !=
      STATUS_OK) {
    ESP_LOGE(TAG, "Error sending PMU config");
    return STATUS_FAILED;
  }

  return this->send_core_config_();
}

uint8_t PN7160::send_core_config_() {
  const uint8_t *core_config = CORE_CONFIG_SOLO;
  uint8_t core_config_length = sizeof(CORE_CONFIG_SOLO);
  std::vector<uint8_t> response;

  if (this->listening_enabled_ && this->polling_enabled_) {
    core_config = CORE_CONFIG_RW_CE;
    core_config_length = sizeof(CORE_CONFIG_RW_CE);
  }

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, core_config, core_config_length, response) !=
      STATUS_OK) {
    ESP_LOGE(TAG, "Error sending core config");
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

uint8_t PN7160::set_discover_map_() {
  std::vector<uint8_t> response;
  std::vector<uint8_t> write_data;

  write_data.resize(1 + sizeof(RF_DISCOVER_MAP_CONFIG));
  write_data[0] = sizeof(RF_DISCOVER_MAP_CONFIG) / 3;
  memcpy(&write_data[1], RF_DISCOVER_MAP_CONFIG, sizeof(RF_DISCOVER_MAP_CONFIG));

  if (this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_MAP_OID, write_data, response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error sending discover map poll config");
    return STATUS_FAILED;
  }
  return STATUS_OK;
}

uint8_t PN7160::set_listen_mode_routing_() {
  std::vector<uint8_t> response;

  if (this->write_ctrl_and_read_(RF_GID, RF_SET_LISTEN_MODE_ROUTING_OID, RF_LISTEN_MODE_ROUTING_CONFIG,
                                 sizeof(RF_LISTEN_MODE_ROUTING_CONFIG), response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error setting listen mode routing config");
    return STATUS_FAILED;
  }
  return STATUS_OK;
}

uint8_t PN7160::start_discovery_() {
  uint8_t length = sizeof(RF_DISCOVERY_CONFIG);
  const uint8_t *rf_discovery_config = RF_DISCOVERY_CONFIG;
  std::vector<uint8_t> response;

  if (!this->listening_enabled_) {
    length = sizeof(RF_DISCOVERY_POLL_CONFIG);
    rf_discovery_config = RF_DISCOVERY_POLL_CONFIG;
  } else if (!this->polling_enabled_) {
    length = sizeof(RF_DISCOVERY_LISTEN_CONFIG);
    rf_discovery_config = RF_DISCOVERY_LISTEN_CONFIG;
  }

  std::vector<uint8_t> write_data = std::vector<uint8_t>((length * 2) + 1);

  write_data[0] = length;
  for (uint8_t i = 0; i < length; i++) {
    write_data[(i * 2) + 1] = rf_discovery_config[i];
    write_data[(i * 2) + 2] = 0x01;  // RF Technology and Mode will be executed in every discovery period
  }

  if (this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_OID, write_data, response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error starting discovery poll");
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

uint8_t PN7160::deactivate_(const uint8_t type, const uint16_t timeout) {
  std::vector<uint8_t> response;
  if (this->write_ctrl_and_read_(RF_GID, RF_DEACTIVATE_OID, {type}, response, timeout) != STATUS_OK) {
    ESP_LOGE(TAG, "Error sending deactivate");
    return STATUS_FAILED;
  }

  this->nci_fsm_set_state_(NCIState::EP_DEACTIVATING);
  return STATUS_OK;
}

void PN7160::select_endpoint_() {
  if (!this->discovered_endpoint_.size()) {
    ESP_LOGE(TAG, "No cached tags to select!");
    return;
  }
  std::vector<uint8_t> response;
  std::vector<uint8_t> write_data = {this->discovered_endpoint_[0].id, this->discovered_endpoint_[0].protocol,
                                     0x01};  // that last byte is the interface ID
  for (size_t i = 0; i < this->discovered_endpoint_.size(); i++) {
    if (!this->discovered_endpoint_[i].trig_called) {
      write_data = {this->discovered_endpoint_[i].id, this->discovered_endpoint_[i].protocol,
                    0x01};  // that last byte is the interface ID
      this->selecting_endpoint_ = i;
      break;
    }
  }

  if (this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_SELECT_OID, write_data.data(), write_data.size(), response) !=
      STATUS_OK) {
    ESP_LOGE(TAG, "Error selecting endpoint");
  } else {
    this->nci_fsm_set_state_(NCIState::EP_SELECTING);
  }
}

uint8_t PN7160::read_endpoint_data_(nfc::NfcTag &tag) {
  uint8_t type = nfc::guess_tag_type(tag.get_uid().size());

  switch (type) {
    case nfc::TAG_TYPE_MIFARE_CLASSIC:
      ESP_LOGV(TAG, "Reading Mifare classic");
      return this->read_mifare_classic_tag_(tag);

    case nfc::TAG_TYPE_2:
      ESP_LOGV(TAG, "Reading Mifare ultralight");
      return this->read_mifare_ultralight_tag_(tag);

    case nfc::TAG_TYPE_UNKNOWN:
    default:
      ESP_LOGV(TAG, "Cannot determine tag type");
      break;
  }
  return STATUS_FAILED;
}

uint8_t PN7160::clean_endpoint_(std::vector<uint8_t> &uid) {
  uint8_t type = nfc::guess_tag_type(uid.size());
  switch (type) {
    case nfc::TAG_TYPE_MIFARE_CLASSIC:
      return this->format_mifare_classic_mifare_();

    case nfc::TAG_TYPE_2:
      return this->clean_mifare_ultralight_();

    default:
      ESP_LOGE(TAG, "Unsupported tag for formatting");
      break;
  }
  return STATUS_FAILED;
}

uint8_t PN7160::format_endpoint_(std::vector<uint8_t> &uid) {
  uint8_t type = nfc::guess_tag_type(uid.size());
  switch (type) {
    case nfc::TAG_TYPE_MIFARE_CLASSIC:
      return this->format_mifare_classic_ndef_();

    case nfc::TAG_TYPE_2:
      return this->clean_mifare_ultralight_();

    default:
      ESP_LOGE(TAG, "Unsupported tag for formatting");
      break;
  }
  return STATUS_FAILED;
}

uint8_t PN7160::write_endpoint_(std::vector<uint8_t> &uid, std::shared_ptr<nfc::NdefMessage> message) {
  uint8_t type = nfc::guess_tag_type(uid.size());
  switch (type) {
    case nfc::TAG_TYPE_MIFARE_CLASSIC:
      return this->write_mifare_classic_tag_(message);

    case nfc::TAG_TYPE_2:
      return this->write_mifare_ultralight_tag_(uid, message);

    default:
      ESP_LOGE(TAG, "Unsupported tag for formatting");
      break;
  }
  return STATUS_FAILED;
}

std::unique_ptr<nfc::NfcTag> PN7160::build_tag_(const uint8_t mode_tech, const std::vector<uint8_t> &data) {
  switch (mode_tech) {
    case (MODE_POLL | TECH_PASSIVE_NFCA): {
      uint8_t uid_length = data[2];
      if (!uid_length) {
        ESP_LOGE(TAG, "UID length cannot be zero");
        return nullptr;
      }
      std::vector<uint8_t> uid(data.begin() + 3, data.begin() + 3 + uid_length);
      auto tag_type_str =
          nfc::guess_tag_type(uid_length) == nfc::TAG_TYPE_MIFARE_CLASSIC ? nfc::MIFARE_CLASSIC : nfc::NFC_FORUM_TYPE_2;
      return make_unique<nfc::NfcTag>(uid, tag_type_str);
    }
  }
  return nullptr;
}

optional<size_t> PN7160::find_tag_uid_(const std::vector<uint8_t> &uid) {
  if (this->discovered_endpoint_.size()) {
    for (size_t i = 0; i < this->discovered_endpoint_.size(); i++) {
      auto existing_tag_uid = this->discovered_endpoint_[i].tag.get_uid();
      bool uid_match = (uid.size() == existing_tag_uid.size());

      if (uid_match) {
        for (size_t i = 0; i < uid.size(); i++) {
          uid_match &= (uid[i] == existing_tag_uid[i]);
        }
        if (uid_match) {
          return i;
        }
      }
    }
  }
  return nullopt;
}

void PN7160::purge_old_tags_() {
  for (size_t i = 0; i < this->discovered_endpoint_.size(); i++) {
    if (millis() - this->discovered_endpoint_[i].last_seen > this->tag_ttl_) {
      for (auto *trigger : this->triggers_ontagremoved_) {
        trigger->process(make_unique<nfc::NfcTag>(this->discovered_endpoint_[i].tag));
      }
      for (auto *bs : this->binary_sensors_) {
        bs->tag_off(this->discovered_endpoint_[i].tag.get_uid());
      }
      ESP_LOGW(TAG, "Tag %s removed from cache",
               nfc::format_bytes(this->discovered_endpoint_[i].tag.get_uid()).c_str());
      this->discovered_endpoint_.erase(this->discovered_endpoint_.begin() + i);
    }
  }
}

void PN7160::nci_fsm_transition_() {
  switch (this->nci_state_) {
    case NCIState::NFCC_RESET:
      if (this->reset_core_(true, true) != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to reset NCI core");
        this->init_failure_handler_();
      } else {
        this->nci_fsm_set_state_(NCIState::NFCC_INIT);
      }
      // fall through

    case NCIState::NFCC_INIT:
      if (this->init_core_(true) != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to initialise NCI core");
        this->init_failure_handler_();
      } else {
        this->nci_fsm_set_state_(NCIState::NFCC_CONFIG);
      }
      // fall through

    case NCIState::NFCC_CONFIG:
      if (this->send_init_config_() != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to send initial config");
        this->init_failure_handler_();
      } else {
        this->config_update_pending_ = false;
        this->nci_fsm_set_state_(NCIState::NFCC_SET_DISCOVER_MAP);
      }
      // fall through

    case NCIState::NFCC_SET_DISCOVER_MAP:
      if (this->set_discover_map_() != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set discover map");
      } else {
        this->nci_fsm_set_state_(NCIState::NFCC_SET_LISTEN_MODE_ROUTING);
      }
      // fall through

    case NCIState::NFCC_SET_LISTEN_MODE_ROUTING:
      if (this->set_listen_mode_routing_() != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set listen mode routing");
      } else {
        this->nci_fsm_set_state_(NCIState::RFST_IDLE);
      }
      // fall through

    case NCIState::RFST_IDLE:
      if (this->config_update_pending_) {
        if (this->send_core_config_() != STATUS_OK) {
          ESP_LOGE(TAG, "Failed to update core config");
        } else {
          this->config_update_pending_ = false;
        }
      }
      if (this->start_discovery_() != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to start discovery polling");
      } else {
        this->nci_fsm_set_state_(NCIState::RFST_DISCOVERY);
      }
      return;

    case NCIState::RFST_W4_HOST_SELECT:
      select_endpoint_();
      // fall through

    // All cases below are waiting for NOTIFICATION messages
    case NCIState::RFST_DISCOVERY:
    case NCIState::RFST_LISTEN_ACTIVE:
    case NCIState::RFST_LISTEN_SLEEP:
    case NCIState::RFST_POLL_ACTIVE:
    case NCIState::EP_SELECTING:
    case NCIState::EP_DEACTIVATING:
      if (this->irq_pin_->digital_read()) {
        this->process_message_();
      }
      break;

    case NCIState::FAILED:
    case NCIState::NONE:
    default:
      return;
  }
}

void PN7160::nci_fsm_set_state_(NCIState new_state) {
  ESP_LOGVV(TAG, "nci_fsm_set_state_(%u)", new_state);
  this->nci_state_ = new_state;
  this->last_nci_state_change_ = millis();
}

void PN7160::process_message_() {
  std::vector<uint8_t> response;
  if (this->read_data_(response, NFCC_DEFAULT_TIMEOUT, false) != STATUS_OK) {
    return;  // No data
  }

  uint8_t mt = response[0] & MT_MASK;
  uint8_t gid = response[0] & GID_MASK;
  uint8_t oid = response[1] & OID_MASK;
  // uint8_t length = response[2];

  switch (mt) {
    case MT_CTRL_NOTIFICATION:
      if (gid == RF_GID) {
        switch (oid) {
          case RF_INTF_ACTIVATED_OID:
            ESP_LOGVV(TAG, "RF_INTF_ACTIVATED_OID");
            this->process_rf_intf_activated_oid_(response);
            return;

          case RF_DISCOVER_OID:
            ESP_LOGVV(TAG, "RF_DISCOVER_OID");
            this->process_rf_discover_oid_(response);
            return;

          case RF_DEACTIVATE_OID:
            ESP_LOGVV(TAG, "RF_DEACTIVATE_OID: type: 0x%02X, reason: 0x%02X", response[3], response[4]);
            this->process_rf_deactivate_oid_(response);
            return;

          default:
            ESP_LOGW(TAG, "Unimplemented RF OID received: 0x%02X", oid);
        }
      } else if (gid == NCI_CORE_GID) {
        switch (oid) {
          case NCI_CORE_GENERIC_ERROR_OID:
            ESP_LOGW(TAG, "NCI_CORE_GENERIC_ERROR_OID:");
            switch (response[3]) {
              case DISCOVERY_ALREADY_STARTED:
                ESP_LOGW(TAG, "  DISCOVERY_ALREADY_STARTED");
                break;

              case DISCOVERY_TARGET_ACTIVATION_FAILED:
                // Tag removed too soon
                ESP_LOGW(TAG, "  DISCOVERY_TARGET_ACTIVATION_FAILED");
                if (this->nci_state_ == NCIState::EP_SELECTING) {
                  this->discovered_endpoint_.erase(this->discovered_endpoint_.begin() + this->selecting_endpoint_);
                  if (this->discovered_endpoint_.size() > 0) {
                    this->nci_fsm_set_state_(NCIState::RFST_W4_HOST_SELECT);
                  } else {
                    // unusual case, but it can happen and must be handled
                    this->deactivate_(DEACTIVATION_TYPE_IDLE);
                    this->nci_fsm_set_state_(NCIState::RFST_IDLE);
                  }
                }
                break;

              case DISCOVERY_TEAR_DOWN:
                ESP_LOGW(TAG, "  DISCOVERY_TEAR_DOWN");
                break;

              default:
                ESP_LOGW(TAG, "Unknown error: 0x%02X", response[3]);
                break;
            }
            break;

          default:
            ESP_LOGW(TAG, "Unimplemented NCI Core OID received: 0x%02X", oid);
        }
      } else {
        ESP_LOGW(TAG, "Unimplemented notification: %s", nfc::format_bytes(response).c_str());
      }
      break;

    case MT_CTRL_RESPONSE:
      ESP_LOGW(TAG, "Unimplemented GID: 0x%02X  OID: 0x%02X  Full response: %s", gid, oid,
               nfc::format_bytes(response).c_str());
      break;

    case MT_CTRL_COMMAND:
      ESP_LOGW(TAG, "Unimplemented command: %s", nfc::format_bytes(response).c_str());
      break;

    case MT_DATA:
      if ((response[0] == 0x00) && (response[1] == 0x00)) {
        this->process_data_message_(response);
      } else {
        ESP_LOGW(TAG, "Malformed data message: %s", nfc::format_bytes(response).c_str());
        this->deactivate_(DEACTIVATION_TYPE_IDLE);
      }
      break;

    default:
      ESP_LOGW(TAG, "Unimplemented message type: %s", nfc::format_bytes(response).c_str());
      break;
  }
}

void PN7160::process_rf_intf_activated_oid_(std::vector<uint8_t> &response) {  // an endpoint was activated
  uint8_t discovery_id = response[3];
  uint8_t interface = response[4];
  uint8_t protocol = response[5];
  uint8_t mode_tech = response[6];
  uint8_t max_size = response[7];
  // uint8_t initial_cred = response[8];
  // uint8_t rf_tech_params_len = response[9];
  // uint8_t rf_tech_params = response[10];

  ESP_LOGVV(TAG, "Endpoint activated -- interface: 0x%02X, protocol: 0x%02X, mode&tech: 0x%02X, max payload: %u",
            interface, protocol, mode_tech, max_size);

  if (mode_tech & MODE_LISTEN) {
    ESP_LOGVV(TAG, "Tag activated in listen mode");
    this->nci_fsm_set_state_(NCIState::RFST_LISTEN_ACTIVE);
    return;
  }

  this->nci_fsm_set_state_(NCIState::RFST_POLL_ACTIVE);
  auto incoming_tag = this->build_tag_(mode_tech, std::vector<uint8_t>(response.begin() + 10, response.end()));

  if (incoming_tag == nullptr) {
    ESP_LOGE(TAG, "Could not build tag");
  } else {
    auto tag_loc = this->find_tag_uid_(incoming_tag.get()->get_uid());
    if (tag_loc.has_value()) {
      this->discovered_endpoint_[tag_loc.value()].id = discovery_id;
      this->discovered_endpoint_[tag_loc.value()].protocol = protocol;
      this->discovered_endpoint_[tag_loc.value()].last_seen = millis();
      ESP_LOGVV(TAG, "Tag cache updated");
    } else {
      DiscoveredEndpoint disc_endpoint{discovery_id, protocol, millis(), *incoming_tag.get(), false};
      this->discovered_endpoint_.push_back(disc_endpoint);
      tag_loc = this->discovered_endpoint_.size() - 1;
      ESP_LOGVV(TAG, "Tag added to cache");
    }

    auto &working_endpoint = this->discovered_endpoint_[tag_loc.value()];

    switch (this->next_task_) {
      case EP_CLEAN:
        ESP_LOGD(TAG, "  Tag cleaning...");
        if (this->clean_endpoint_(working_endpoint.tag.get_uid()) != STATUS_OK) {
          ESP_LOGE(TAG, "  Tag was not fully cleaned successfully");
        }
        ESP_LOGD(TAG, "  Tag cleaned!");
        break;

      case EP_FORMAT:
        ESP_LOGD(TAG, "  Tag formatting...");
        if (this->format_endpoint_(working_endpoint.tag.get_uid()) != STATUS_OK) {
          ESP_LOGE(TAG, "Error formatting tag as NDEF");
        }
        ESP_LOGD(TAG, "  Tag formatted!");
        break;

      case EP_WRITE:
        if (this->next_task_message_to_write_ != nullptr) {
          ESP_LOGD(TAG, "  Tag writing...");
          ESP_LOGD(TAG, "  Tag formatting...");
          if (this->format_endpoint_(working_endpoint.tag.get_uid()) != STATUS_OK) {
            ESP_LOGE(TAG, "  Tag could not be formatted for writing");
          } else {
            ESP_LOGD(TAG, "  Writing NDEF data");
            if (this->write_endpoint_(working_endpoint.tag.get_uid(), this->next_task_message_to_write_) != STATUS_OK) {
              ESP_LOGE(TAG, "  Failed to write message to tag");
            }
            ESP_LOGD(TAG, "  Finished writing NDEF data");
            this->next_task_message_to_write_ = nullptr;
            this->on_finished_write_callback_.call();
          }
        }
        break;

      case EP_READ:
      default:
        if (!working_endpoint.trig_called) {
          ESP_LOGD(TAG, "Read tag type %s with UID %s", working_endpoint.tag.get_tag_type().c_str(),
                   nfc::format_uid(working_endpoint.tag.get_uid()).c_str());
          if (this->read_endpoint_data_(working_endpoint.tag) != STATUS_OK) {
            ESP_LOGW(TAG, "  Unable to read NDEF record(s)");
          } else if (working_endpoint.tag.has_ndef_message()) {
            const auto &message = working_endpoint.tag.get_ndef_message();
            const auto &records = message->get_records();
            ESP_LOGD(TAG, "  NDEF record(s):");
            for (const auto &record : records) {
              ESP_LOGD(TAG, "    %s - %s", record->get_type().c_str(), record->get_payload().c_str());
            }
          } else {
            ESP_LOGW(TAG, "  No NDEF records found");
          }
          for (auto *trigger : this->triggers_ontag_) {
            trigger->process(make_unique<nfc::NfcTag>(working_endpoint.tag));
          }
          for (auto *bs : this->binary_sensors_) {
            bs->tag_on(working_endpoint.tag.get_uid());
          }
          working_endpoint.trig_called = true;
          break;
        }
    }
    if (working_endpoint.tag.get_tag_type() == nfc::MIFARE_CLASSIC) {
      this->halt_mifare_classic_tag_();
    }
  }
  if (this->next_task_ != EP_READ) {
    this->read_mode();
  }

  this->deactivate_(DEACTIVATION_TYPE_IDLE);
}

void PN7160::process_rf_discover_oid_(std::vector<uint8_t> &response) {
  uint8_t mode_tech = response[5];
  auto incoming_tag = this->build_tag_(mode_tech, std::vector<uint8_t>(response.begin() + 7, response.end()));

  if (incoming_tag.get() == nullptr) {
    ESP_LOGE(TAG, "Could not build tag!");
  } else {
    auto tag_loc = this->find_tag_uid_(incoming_tag.get()->get_uid());
    if (tag_loc.has_value()) {
      this->discovered_endpoint_[tag_loc.value()].id = response[3];
      this->discovered_endpoint_[tag_loc.value()].protocol = response[4];
      this->discovered_endpoint_[tag_loc.value()].last_seen = millis();
      ESP_LOGVV(TAG, "Tag found & updated");
    } else {
      DiscoveredEndpoint disc_endpoint{response[3], response[4], millis(), *incoming_tag.get(), false};
      this->discovered_endpoint_.push_back(disc_endpoint);
      ESP_LOGVV(TAG, "Tag saved");
    }
  }

  if (response.back() != RF_DISCOVER_NTF_NT_MORE) {
    this->nci_fsm_set_state_(NCIState::RFST_W4_HOST_SELECT);
    ESP_LOGVV(TAG, "Discovered %u endpoints", this->discovered_endpoint_.size());
  }
}

void PN7160::process_rf_deactivate_oid_(std::vector<uint8_t> &response) {
  this->ce_state_ = CardEmulationState::CARD_EMU_IDLE;

  switch (response[3]) {
    case DEACTIVATION_TYPE_DISCOVERY:
      this->nci_fsm_set_state_(NCIState::RFST_DISCOVERY);
      break;

    case DEACTIVATION_TYPE_IDLE:
      this->nci_fsm_set_state_(NCIState::RFST_IDLE);
      break;

    case DEACTIVATION_TYPE_SLEEP:
    case DEACTIVATION_TYPE_SLEEP_AF:
      if (this->nci_state_ == NCIState::RFST_LISTEN_ACTIVE) {
        this->nci_fsm_set_state_(NCIState::RFST_LISTEN_SLEEP);
      } else if (this->nci_state_ == NCIState::RFST_POLL_ACTIVE) {
        this->nci_fsm_set_state_(NCIState::RFST_W4_HOST_SELECT);
      } else {
        this->nci_fsm_set_state_(NCIState::RFST_IDLE);
      }
      break;

    default:
      break;
  }
}

void PN7160::process_data_message_(std::vector<uint8_t> &response) {
  ESP_LOGVV(TAG, "Received data message: %s", nfc::format_bytes(response).c_str());

  std::vector<uint8_t> ndef_response;
  this->card_emu_t4t_get_response(response, ndef_response);

  uint16_t ndef_response_size = ndef_response.size();
  if (!ndef_response_size) {
    return;  // no message returned, we cannot respond
  }

  std::vector<uint8_t> write_data{MT_DATA, uint8_t((ndef_response_size & 0xFF00) >> 8),
                                  uint8_t(ndef_response_size & 0x00FF)};
  write_data.insert(write_data.end(), ndef_response.begin(), ndef_response.end());
  if (this->write_and_read_(write_data, response, NFCC_DEFAULT_TIMEOUT, false)) {
    ESP_LOGE(TAG, "Sending reply for card emulation failed");
  }
}

void PN7160::card_emu_t4t_get_response(std::vector<uint8_t> &response, std::vector<uint8_t> &ndef_response) {
  const uint8_t MSG_OFFSET = 3;

  if (this->card_emulation_message_ == nullptr) {
    ESP_LOGE(TAG, "No NDEF message is set; tag emulation not possible");
    ndef_response.clear();
    return;
  }

  if (equal(response.begin() + MSG_OFFSET, response.end(), std::begin(CARD_EMU_T4T_APP_SELECT))) {
    // CARD_EMU_T4T_APP_SELECT
    ESP_LOGVV(TAG, "CARD_EMU_NDEF_APP_SELECTED");
    this->ce_state_ = CardEmulationState::CARD_EMU_NDEF_APP_SELECTED;
    ndef_response.insert(ndef_response.begin(), std::begin(CARD_EMU_T4T_OK), std::end(CARD_EMU_T4T_OK));
  } else if (equal(response.begin() + MSG_OFFSET, response.end(), std::begin(CARD_EMU_T4T_CC_SELECT))) {
    // CARD_EMU_T4T_CC_SELECT
    if (this->ce_state_ == CardEmulationState::CARD_EMU_NDEF_APP_SELECTED) {
      ESP_LOGVV(TAG, "CARD_EMU_CC_SELECTED");
      this->ce_state_ = CardEmulationState::CARD_EMU_CC_SELECTED;
      ndef_response.insert(ndef_response.begin(), std::begin(CARD_EMU_T4T_OK), std::end(CARD_EMU_T4T_OK));
    }
  } else if (equal(response.begin() + MSG_OFFSET, response.end(), std::begin(CARD_EMU_T4T_NDEF_SELECT))) {
    // CARD_EMU_T4T_NDEF_SELECT
    ESP_LOGVV(TAG, "CARD_EMU_NDEF_SELECTED");
    this->ce_state_ = CardEmulationState::CARD_EMU_NDEF_SELECTED;
    ndef_response.insert(ndef_response.begin(), std::begin(CARD_EMU_T4T_OK), std::end(CARD_EMU_T4T_OK));
  } else if (equal(response.begin() + MSG_OFFSET, response.begin() + MSG_OFFSET + sizeof(CARD_EMU_T4T_READ),
                   std::begin(CARD_EMU_T4T_READ))) {
    // CARD_EMU_T4T_READ
    if (this->ce_state_ == CardEmulationState::CARD_EMU_CC_SELECTED) {
      // CARD_EMU_T4T_READ with CARD_EMU_CC_SELECTED
      ESP_LOGVV(TAG, "CARD_EMU_T4T_READ with CARD_EMU_CC_SELECTED");
      uint16_t offset = (response[MSG_OFFSET + 2] << 8) + response[MSG_OFFSET + 3];
      uint8_t length = response[MSG_OFFSET + 4];

      if (length <= (sizeof(CARD_EMU_T4T_CC) + offset + 2)) {
        ndef_response.insert(ndef_response.begin(), std::begin(CARD_EMU_T4T_CC) + offset,
                             std::begin(CARD_EMU_T4T_CC) + offset + length);
        ndef_response.insert(ndef_response.end(), std::begin(CARD_EMU_T4T_OK), std::end(CARD_EMU_T4T_OK));
      }
    } else if (this->ce_state_ == CardEmulationState::CARD_EMU_NDEF_SELECTED) {
      // CARD_EMU_T4T_READ with CARD_EMU_NDEF_SELECTED
      ESP_LOGVV(TAG, "CARD_EMU_T4T_READ with CARD_EMU_NDEF_SELECTED");
      auto ndef_message = this->card_emulation_message_->encode();
      uint16_t ndef_msg_size = ndef_message.size();
      uint16_t offset = (response[MSG_OFFSET + 2] << 8) + response[MSG_OFFSET + 3];
      uint8_t length = response[MSG_OFFSET + 4];

      ESP_LOGVV(TAG, "Encoded NDEF message: %s", nfc::format_bytes(ndef_message).c_str());

      if (length <= (ndef_msg_size + offset + 2)) {
        if (offset == 0) {
          ndef_response.resize(2);
          ndef_response[0] = (ndef_msg_size & 0xFF00) >> 8;
          ndef_response[1] = (ndef_msg_size & 0x00FF);
          if (length > 2) {
            ndef_response.insert(ndef_response.end(), ndef_message.begin(), ndef_message.begin() + length - 2);
          }
        } else if (offset == 1) {
          ndef_response.resize(1);
          ndef_response[0] = (ndef_msg_size & 0x00FF);
          if (length > 1) {
            ndef_response.insert(ndef_response.end(), ndef_message.begin(), ndef_message.begin() + length - 1);
          }
        } else {
          ndef_response.insert(ndef_response.end(), ndef_message.begin(), ndef_message.begin() + length);
        }

        ndef_response.insert(ndef_response.end(), std::begin(CARD_EMU_T4T_OK), std::end(CARD_EMU_T4T_OK));

        if ((offset + length) >= (ndef_msg_size + 2)) {
          ESP_LOGD(TAG, "NDEF message sent");
          this->on_emulated_tag_scan_callback_.call();
        }
      }
    }
  } else if (equal(response.begin() + MSG_OFFSET, response.begin() + MSG_OFFSET + sizeof(CARD_EMU_T4T_WRITE),
                   std::begin(CARD_EMU_T4T_WRITE))) {
    // CARD_EMU_T4T_WRITE
    if (this->ce_state_ == CardEmulationState::CARD_EMU_NDEF_SELECTED) {
      ESP_LOGVV(TAG, "CARD_EMU_T4T_WRITE");
      uint16_t offset = (response[MSG_OFFSET + 2] << 8) + response[MSG_OFFSET + 3];
      uint8_t length = response[MSG_OFFSET + 4];
      std::vector<uint8_t> ndef_msg_written;

      ndef_msg_written.insert(ndef_msg_written.end(), response.begin() + MSG_OFFSET + 5,
                              response.begin() + MSG_OFFSET + 5 + length);
      ESP_LOGW(TAG, "Received %u-byte NDEF message: %s", length, nfc::format_bytes(ndef_msg_written).c_str());
      ndef_response.insert(ndef_response.end(), std::begin(CARD_EMU_T4T_OK), std::end(CARD_EMU_T4T_OK));
    }
  }
}

uint8_t PN7160::write_ctrl_and_read_(const uint8_t gid, const uint8_t oid, const std::vector<uint8_t> &data,
                                     std::vector<uint8_t> &response, const uint16_t timeout, const bool warn) {
  return this->write_ctrl_and_read_(gid, oid, data.data(), data.size(), response, timeout, warn);
}

uint8_t PN7160::write_ctrl_and_read_(const uint8_t gid, const uint8_t oid, const uint8_t *data, const uint8_t len,
                                     std::vector<uint8_t> &response, const uint16_t timeout, const bool warn) {
  if (gid != (gid & GID_MASK)) {
    ESP_LOGE(TAG, "Invalid GID");
    return STATUS_FAILED;
  }
  if (oid != (oid & OID_MASK)) {
    ESP_LOGE(TAG, "Invalid OID");
    return STATUS_FAILED;
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

  if (this->write_and_read_(write_data, response, timeout, warn) != STATUS_OK) {
    return STATUS_FAILED;
  }

  if ((response[0] & GID_MASK) != gid || (response[1] & OID_MASK) != oid) {
    ESP_LOGE(TAG, "Incorrect response: 0x%02X 0x%02X 0x%02X ", response[0], response[1], response[2]);
    return STATUS_FAILED;
  }

  if (response[3] != STATUS_OK) {
    ESP_LOGE(TAG, "Error in response: 0x%02X", response[3]);
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

uint8_t PN7160::write_data_and_read_(const std::vector<uint8_t> &data, std::vector<uint8_t> &response,
                                     const uint16_t timeout, const bool warn) {
  std::vector<uint8_t> write_data{MT_DATA, 0x00, (uint8_t) data.size()};
  write_data.insert(write_data.end(), data.begin(), data.end());

  if (this->write_and_read_(write_data, response, timeout, warn) != STATUS_OK) {
    ESP_LOGE(TAG, "Error requesting read from endpoint");
    return STATUS_FAILED;
  }

  if ((response[0] != (MT_CTRL_NOTIFICATION | NCI_CORE_GID)) || (response[1] != NCI_CORE_CONN_CREDITS_OID) ||
      (response[2] != 3)) {
    ESP_LOGE(TAG, "Incorrect response: %s", nfc::format_bytes(response).c_str());
    return STATUS_FAILED;
  }

  if (this->wait_for_irq_(timeout, warn) != STATUS_OK) {
    return STATUS_FAILED;
  }

  if (this->read_data_(response, timeout) != STATUS_OK) {
    ESP_LOGE(TAG, "Error reading data from endpoint");
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

uint8_t PN7160::write_and_read_(const std::vector<uint8_t> &data, std::vector<uint8_t> &response,
                                const uint16_t timeout, const bool warn) {
  if (this->write_data_(data) != STATUS_OK) {
    ESP_LOGE(TAG, "Error sending data");
    return STATUS_FAILED;
  }

  if (this->read_data_(response, timeout) != STATUS_OK) {
    if (warn)
      ESP_LOGE(TAG, "Error reading response");
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

uint8_t PN7160::read_data_(std::vector<uint8_t> &data, const uint16_t timeout, const bool warn) {
  if (this->wait_for_irq_(timeout, true, warn) != STATUS_OK) {
    return STATUS_FAILED;
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
      if (this->wait_for_irq_(timeout) != STATUS_OK) {
        return STATUS_FAILED;
      }

      uint8_t read_length = std::min(length - start, 10);
      this->read_array(data.data() + 3 + start, read_length);
      start += read_length;
    }
  }
  this->disable();
  return STATUS_OK;
}

uint8_t PN7160::write_data_(const std::vector<uint8_t> &data) {
  this->enable();
  this->write_byte(TDD_SPI_WRITE);  // send "transfer direction detector"
  this->write_array(data.data(), data.size());
  this->disable();
  return STATUS_OK;
}

uint8_t PN7160::wait_for_irq_(uint16_t timeout, bool state, bool warn) {
  uint32_t start_time = millis();
  uint32_t count = 0;
  while (true) {
    if (this->irq_pin_->digital_read() == state) {
      return STATUS_OK;
    }
    count++;

    if (millis() - start_time > timeout) {
      if (warn)
        ESP_LOGW(TAG, "Timed out waiting for data (this can be normal)");
      return STATUS_FAILED;
    }
  }
}

bool PN7160BinarySensor::tag_match(const std::vector<uint8_t> &data) {
  if (data.size() != this->uid_.size()) {
    return false;
  }

  for (size_t i = 0; i < data.size(); i++) {
    if (data[i] != this->uid_[i]) {
      return false;
    }
  }

  return true;
}

}  // namespace pn7160
}  // namespace esphome
