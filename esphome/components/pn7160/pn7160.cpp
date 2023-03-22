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

  this->nci_fsm_transition_();  // kick off reset & init processes
}

void PN7160::dump_config() {
  ESP_LOGCONFIG(TAG, "PN7160:");
  LOG_PIN("  CS Pin: ", this->cs_);
}

void PN7160::loop() {
  this->purge_old_tags_();
  this->nci_fsm_transition_();
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

void PN7160::write_mode(nfc::NdefMessage *message) {
  this->next_task_ = EP_WRITE;
  this->next_task_message_to_write_ = message;
  ESP_LOGD(TAG, "Waiting to write next tag");
}

void PN7160::init_failure_handler_() {
  ESP_LOGE(TAG, "Communication failure");
  if (this->fail_count_ < NFCC_MAX_COMM_FAILS) {
    ESP_LOGE(TAG, "Initialization attempt %u failed, retrying...", this->fail_count_++);
    this->state_ = PN7160State::NFCC_RESET;
  } else {
    ESP_LOGE(TAG, "Too many initialization failures -- check device connections");
    this->mark_failed();
    this->state_ = PN7160State::FAILED;
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

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_RESET_OID, write_data, response, NFCC_INIT_TIMEOUT) !=
      STATUS_OK) {
    ESP_LOGE(TAG, "Error sending reset command");
    return STATUS_FAILED;
  }

  if (response[3] != STATUS_OK) {
    ESP_LOGE(TAG, "Invalid reset response: 0x%02X", response[3]);
    return response[3];
  }

  // there may also be a notification in response to the reset command. we don't care but still need to read it back
  this->read_data_(response);

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
  std::vector<uint8_t> response;

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, CORE_CONFIG, sizeof(CORE_CONFIG), response) !=
      STATUS_OK) {
    ESP_LOGE(TAG, "Error sending core config");
    return STATUS_FAILED;
  }

  response.clear();

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, CORE_CONF_EXTENSION,
                                 sizeof(CORE_CONF_EXTENSION), response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error sending core config extension");
    return STATUS_FAILED;
  }

  response.clear();

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, CLOCK_CONFIG, sizeof(CLOCK_CONFIG), response) !=
      STATUS_OK) {
    ESP_LOGE(TAG, "Error sending clock config");
    return STATUS_FAILED;
  }

  response.clear();

  if (this->write_ctrl_and_read_(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID, RF_TVDD_CONFIG, sizeof(RF_TVDD_CONFIG),
                                 response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error sending RF TVDD config");
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

uint8_t PN7160::set_mode_() {
  std::vector<uint8_t> response;
  std::vector<uint8_t> write_data;

  write_data.resize(1 + sizeof(READ_WRITE_MODE));
  write_data[0] = sizeof(READ_WRITE_MODE) / 3;
  memcpy(&write_data[1], READ_WRITE_MODE, sizeof(READ_WRITE_MODE));

  if (this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_MAP_OID, write_data, response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error sending discover map");
    return STATUS_FAILED;
  }
  return STATUS_OK;
}

uint8_t PN7160::start_discovery_() {
  std::vector<uint8_t> response;
  uint8_t length = sizeof(DISCOVERY_READ_WRITE);
  std::vector<uint8_t> write_data = std::vector<uint8_t>((length * 2) + 1);

  write_data[0] = length;
  for (uint8_t i = 0; i < length; i++) {
    write_data[(i * 2) + 1] = DISCOVERY_READ_WRITE[i];
    write_data[(i * 2) + 2] = 0x01;  // RF Technology and Mode will be executed in every discovery period
  }

  if (this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_OID, write_data, response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error starting discovery");
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

uint8_t PN7160::deactivate_(const uint8_t type) {
  std::vector<uint8_t> response;
  if (this->write_ctrl_and_read_(RF_GID, RF_DEACTIVATE_OID, {type}, response) != STATUS_OK) {
    ESP_LOGE(TAG, "Error sending deactivate");
    return STATUS_FAILED;
  }

  this->state_ = PN7160State::EP_DEACTIVATING;
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
    this->state_ = PN7160State::EP_SELECTING;
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

uint8_t PN7160::write_endpoint_(std::vector<uint8_t> &uid, nfc::NdefMessage *message) {
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
      ESP_LOGW(TAG, "Tag %s removed from cache",
               nfc::format_bytes(this->discovered_endpoint_[i].tag.get_uid()).c_str());
      this->discovered_endpoint_.erase(this->discovered_endpoint_.begin() + i);
    }
  }
}

void PN7160::nci_fsm_transition_() {
  switch (this->state_) {
    case PN7160State::NFCC_RESET:
      if (this->reset_core_(true, true) != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to reset NCI core");
        this->init_failure_handler_();
      } else {
        this->state_ = PN7160State::NFCC_INIT;
      }
      // fall through

    case PN7160State::NFCC_INIT:
      if (this->init_core_(true) != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to init NCI core");
        this->init_failure_handler_();
      } else {
        this->state_ = PN7160State::NFCC_CONFIG;
      }
      // fall through

    case PN7160State::NFCC_CONFIG:
      if (this->send_config_() != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to send config");
        this->init_failure_handler_();
      } else {
        this->state_ = PN7160State::NFCC_SET_MODE;
      }
      // fall through

    case PN7160State::NFCC_SET_MODE:
      if (this->set_mode_() != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set mode");
        this->init_failure_handler_();
      } else {
        this->state_ = PN7160State::RFST_IDLE;
      }
      return;

    case PN7160State::RFST_IDLE:
      if (this->start_discovery_() != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to start discovery");
      } else {
        this->state_ = PN7160State::RFST_DISCOVERY;
      }
      return;

    case PN7160State::RFST_W4_HOST_SELECT:
      select_endpoint_();
      // fall through

    // These cases are waiting for NOTIFICATION messages
    case PN7160State::EP_SELECTING:
    case PN7160State::RFST_DISCOVERY:
    case PN7160State::RFST_POLL_ACTIVE:
    case PN7160State::EP_DEACTIVATING:
      if (this->irq_pin_->digital_read()) {
        this->process_message_();
      }
      break;

    case PN7160State::FAILED:
    case PN7160State::NONE:
    default:
      return;
  }
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
          case RF_INTF_ACTIVATED_OID: {  // an endpoint was activated
            ESP_LOGVV(TAG, "RF_INTF_ACTIVATED_OID");
            this->state_ = PN7160State::RFST_POLL_ACTIVE;
            uint8_t discovery_id = response[3];
            uint8_t interface = response[4];
            uint8_t protocol = response[5];
            uint8_t mode_tech = response[6];
            uint8_t max_size = response[7];
            // uint8_t initial_cred = response[8];
            // uint8_t rf_tech_params_len = response[9];
            // uint8_t rf_tech_params = response[10];

            ESP_LOGVV(TAG,
                      "Endpoint activated -- interface: 0x%02X, protocol: 0x%02X, mode&tech: 0x%02X, max payload: %u",
                      interface, protocol, mode_tech, max_size);

            auto incoming_tag =
                this->build_tag_(mode_tech, std::vector<uint8_t>(response.begin() + 10, response.end()));

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
                      if (this->write_endpoint_(working_endpoint.tag.get_uid(), this->next_task_message_to_write_) !=
                          STATUS_OK) {
                        ESP_LOGE(TAG, "  Failed to write message to tag");
                      }
                      ESP_LOGD(TAG, "  Finished writing NDEF data");
                      delete this->next_task_message_to_write_;
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
                    working_endpoint.trig_called = true;
                    break;
                  }
              }
            }
            if (this->next_task_ != EP_READ) {
              this->read_mode();
            }

            this->deactivate_(DEACTIVATION_TYPE_IDLE);
            return;
          }
          case RF_DISCOVER_OID: {
            ESP_LOGVV(TAG, "RF_DISCOVER_OID");

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
              this->state_ = PN7160State::RFST_W4_HOST_SELECT;
              ESP_LOGVV(TAG, "Discovered %u endpoints", this->discovered_endpoint_.size());
            }
            break;
          }
          case RF_DEACTIVATE_OID: {
            ESP_LOGVV(TAG, "RF_DEACTIVATE_OID: type: 0x%02X, reason: 0x%02X", response[3], response[4]);
            switch (response[3]) {
              case DEACTIVATION_TYPE_DISCOVERY:
                this->state_ = PN7160State::RFST_DISCOVERY;
                break;

              case DEACTIVATION_TYPE_IDLE:
                this->state_ = PN7160State::RFST_IDLE;
                break;

              case DEACTIVATION_TYPE_SLEEP:
              case DEACTIVATION_TYPE_SLEEP_AF:
                this->state_ = PN7160State::RFST_W4_HOST_SELECT;
                break;

              default:
                break;
            }
            break;
          }
          default:
            ESP_LOGW(TAG, "Unsupported RF OID received: 0x%02X", oid);
        }
      } else if (gid == NCI_CORE_GID) {
        switch (oid) {
          case NCI_CORE_GENERIC_ERROR_OID:
            ESP_LOGW(TAG, "NCI_CORE_GENERIC_ERROR_OID: 0x%02X", response[3]);
            switch (response[3]) {
              case DISCOVERY_ALREADY_STARTED:
                ESP_LOGW(TAG, "  DISCOVERY_ALREADY_STARTED");
                break;

              case DISCOVERY_TARGET_ACTIVATION_FAILED:
                // Tag removed too soon
                ESP_LOGW(TAG, "  DISCOVERY_TARGET_ACTIVATION_FAILED");
                if (this->state_ == PN7160State::EP_SELECTING) {
                  this->discovered_endpoint_.erase(this->discovered_endpoint_.begin() + this->selecting_endpoint_);
                  if (this->discovered_endpoint_.size() > 0) {
                    this->state_ = PN7160State::RFST_W4_HOST_SELECT;
                  } else {
                    // unusual case, but it can happen and must be handled
                    this->deactivate_(DEACTIVATION_TYPE_IDLE);
                    this->state_ = PN7160State::RFST_IDLE;
                  }
                }
                break;

              case DISCOVERY_TEAR_DOWN:
                ESP_LOGW(TAG, "  DISCOVERY_TEAR_DOWN");
                break;

              default:
                break;
            }
            break;

          default:
            ESP_LOGW(TAG, "Unsupported NCI Core OID received: 0x%02X", oid);
        }
      } else {
        ESP_LOGW(TAG, "Unimplemented notification -- response: %s", nfc::format_bytes(response).c_str());
      }
      break;

    case MT_CTRL_RESPONSE:
      ESP_LOGW(TAG, "Unimplemented response: GID: 0x%02X  OID: 0x%02X", gid, oid);
      break;

    case MT_CTRL_COMMAND:
    default:
      ESP_LOGW(TAG, "Unimplemented command -- response: %s", nfc::format_bytes(response).c_str());
      break;
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

}  // namespace pn7160
}  // namespace esphome
