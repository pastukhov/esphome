#include "pn7160.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <sstream>

namespace esphome {
namespace pn7160 {

static const char *const TAG = "pn7160";

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
      select_tag_();
      // fall through

    // These cases are waiting for NOTIFICATION messages
    case PN7160State::RFST_POLL_ACTIVE:
    case PN7160State::RFST_DISCOVERY:
    case PN7160State::EP_DEACTIVATING:
    case PN7160State::EP_SELECTING:
      break;

    case PN7160State::FAILED:
    case PN7160State::NONE:
    default:
      return;
  }

  if (!this->irq_pin_->digital_read()) {
    return;  // No data to read
  }

  std::vector<uint8_t> response;
  if (!this->read_data_(response, 5, false)) {
    return;  // No data
  }

  uint8_t mt = response[0] & MT_MASK;
  uint8_t gid = response[0] & GID_MASK;
  uint8_t oid = response[1] & OID_MASK;
  // uint8_t length = response[2];

  switch (mt) {
    case MT_CTRL_RESPONSE:
      ESP_LOGW(TAG, "Unimplemented response: GID: 0x%.2X  OID: 0x%.2X", gid, oid);
      break;

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

            // this->current_protocol_ = protocol;
            ESP_LOGVV(TAG, "Endpoint detected: interface: 0x%.2X, protocol: 0x%.2X, mode&tech: 0x%.2X, max payload: %u",
                      interface, protocol, mode_tech, max_size);

            auto tag = this->build_tag_(mode_tech, std::vector<uint8_t>(response.begin() + 10, response.end()), true);

            if (tag == nullptr) {
              ESP_LOGE(TAG, "Could not build tag!");
            } else {
              auto tag_loc = this->find_tag_uid_(tag.get()->get_uid());
              if (tag_loc.has_value()) {
                this->discovered_endpoint_[tag_loc.value()].id = discovery_id;
                this->discovered_endpoint_[tag_loc.value()].protocol = protocol;
                this->discovered_endpoint_[tag_loc.value()].last_seen = millis();
                ESP_LOGVV(TAG, "Tag found & updated");
              } else {
                DiscoveredEndpoint disc_endpoint{discovery_id, protocol, millis(), *tag.get(), false};
                this->discovered_endpoint_.push_back(disc_endpoint);
                tag_loc = this->discovered_endpoint_.size() - 1;
                ESP_LOGVV(TAG, "Tag saved");
              }
              // TODO: read tag's NDEF message here if it is not already present
              if (!this->discovered_endpoint_[tag_loc.value()].trig_called) {
                for (auto *trigger : this->triggers_ontag_) {
                  trigger->process(tag);
                }
                this->discovered_endpoint_[tag_loc.value()].trig_called = true;
              }
            }

            // this->state_ = PN7160State::NONE;
            // this->set_timeout(200, [this]() { this->state_ = PN7160State::RFST_POLL_ACTIVE; });

            this->deactivate_(DEACTIVATION_TYPE_IDLE);
            return;
          }
          case RF_DISCOVER_OID: {
            ESP_LOGVV(TAG, "RF_DISCOVER_OID");

            uint8_t mode_tech = response[5];
            auto tag = this->build_tag_(mode_tech, std::vector<uint8_t>(response.begin() + 7, response.end()));

            if (tag.get() == nullptr) {
              ESP_LOGE(TAG, "Could not build tag!");
            } else {
              auto tag_loc = this->find_tag_uid_(tag.get()->get_uid());
              if (tag_loc.has_value()) {
                this->discovered_endpoint_[tag_loc.value()].id = response[3];
                this->discovered_endpoint_[tag_loc.value()].protocol = response[4];
                this->discovered_endpoint_[tag_loc.value()].last_seen = millis();
                ESP_LOGVV(TAG, "Tag found & updated");
              } else {
                DiscoveredEndpoint disc_endpoint{response[3], response[4], millis(), *tag.get(), false};
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
            ESP_LOGVV(TAG, "RF_DEACTIVATE_OID: type: 0x%.2X, reason: 0x%.2X", response[3], response[4]);
            // if (this->next_function_ != nullptr) {
            //   this->next_function_();
            //   this->next_function_ = nullptr;
            // }
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
            ESP_LOGW(TAG, "Unsupported RF OID received: 0x%.2X", oid);
        }
      } else if (gid == NCI_CORE_GID) {
        switch (oid) {
          case NCI_CORE_GENERIC_ERROR_OID:
            ESP_LOGW(TAG, "NCI_CORE_GENERIC_ERROR_OID: 0x%.2X", response[3]);
            switch (response[3]) {
              case DISCOVERY_TARGET_ACTIVATION_FAILED:
                // Tag removed
                // ESP_LOGD(TAG, "DISCOVERY_TARGET_ACTIVATION_FAILED");
                // ESP_LOGD(TAG, "Purging all tags...");
                // this->discovered_endpoint_.clear();
                // if (!this->deactivate_(DEACTIVATION_TYPE_IDLE)) {
                //   ESP_LOGE(TAG, "Error deactivating");
                // }
                // this->state_ = PN7160State::RFST_DISCOVERY;
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

optional<size_t> PN7160::find_discovery_id_(uint8_t id) {
  if (this->discovered_endpoint_.size()) {
    for (size_t i = 0; i < this->discovered_endpoint_.size(); i++) {
      if (this->discovered_endpoint_[i].id == id) {
        return i;
      }
    }
  }
  return nullopt;
}

optional<size_t> PN7160::find_tag_uid_(const std::vector<uint8_t> &uid) {
  if (this->discovered_endpoint_.size()) {
    for (size_t i = 0; i < this->discovered_endpoint_.size(); i++) {
      auto existing_tag_uid = this->discovered_endpoint_[i].tag.get_uid();
      bool tag_match = (uid.size() == existing_tag_uid.size());

      if (tag_match) {
        for (size_t i = 0; i < uid.size(); i++) {
          if (uid[i] != existing_tag_uid[i]) {
            tag_match = false;
          }
        }
        if (tag_match) {
          return i;
        }
      }
    }
  }
  return nullopt;
}

void PN7160::init_failure_handler_() {
  ESP_LOGE(TAG, "Communication failure");
  if (this->fail_count_ < MAX_FAILS) {
    ESP_LOGE(TAG, "Initialization attempt %u failed, retrying...", this->fail_count_++);
    this->state_ = PN7160State::NFCC_RESET;
  } else {
    ESP_LOGE(TAG, "Too many initialization failures -- check device connections");
    this->mark_failed();
    this->state_ = PN7160State::FAILED;
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

uint8_t PN7160::set_mode_() {
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

uint8_t PN7160::start_discovery_() {
  std::vector<uint8_t> response;
  uint8_t length = sizeof(DISCOVERY_READ_WRITE);
  std::vector<uint8_t> write_data = std::vector<uint8_t>((length * 2) + 1);

  write_data[0] = length;
  for (uint8_t i = 0; i < length; i++) {
    write_data[(i * 2) + 1] = DISCOVERY_READ_WRITE[i];
    write_data[(i * 2) + 2] = 0x01;  // RF Technology and Mode will be executed in every discovery period
  }

  if (!this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_OID, write_data, response)) {
    ESP_LOGE(TAG, "Error starting discovery");
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

  this->state_ = PN7160State::EP_DEACTIVATING;
  return true;
}

std::unique_ptr<nfc::NfcTag> PN7160::build_tag_(uint8_t mode_tech, const std::vector<uint8_t> &data, bool print_uid) {
  switch (mode_tech) {
    case (MODE_POLL | TECH_PASSIVE_NFCA): {
      uint8_t uid_length = data[2];
      if (!uid_length) {
        ESP_LOGW(TAG, "UID length cannot be zero!");
        return nullptr;
      }
      std::vector<uint8_t> uid(data.begin() + 3, data.begin() + 3 + uid_length);
      std::ostringstream tag_msg;
      auto tag_type_str =
          nfc::guess_tag_type(uid_length) == nfc::TAG_TYPE_MIFARE_CLASSIC ? nfc::MIFARE_CLASSIC : nfc::NFC_FORUM_TYPE_2;
      tag_msg << "Tag type " << tag_type_str << " with UID ";
      for (size_t i = 0; i < uid.size(); i++) {
        char buffer[6];
        sprintf(buffer, "%02X", uid[i]);
        if (i > 0) {
          tag_msg << "-";
        }
        tag_msg << buffer;
      }

      if (print_uid) {
        ESP_LOGD(TAG, "%s", tag_msg.str().c_str());
      }

      return make_unique<nfc::NfcTag>(uid, tag_type_str);
    }
  }
  return nullptr;
}

bool PN7160::check_for_tag_(std::unique_ptr<nfc::NfcTag> &tag) {
  for (auto endpoint : this->discovered_endpoint_) {
    if (tag->get_uid().size() == endpoint.tag.get_uid().size()) {  // if the uid sizes match, this is a tag of interest
      bool same_uid = true;
      for (size_t i = 0; i < endpoint.tag.get_uid().size(); i++) {
        // ESP_LOGD(TAG, "UID: %u", tag->get_uid()[i]);
        same_uid &= endpoint.tag.get_uid()[i] == tag->get_uid()[i];
      }
      if (same_uid) {
        return true;
      }
    }
  }
  return false;
}

void PN7160::select_tag_() {
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
      break;
    }
  }

  if (!this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_SELECT_OID, write_data.data(), write_data.size(), response)) {
    ESP_LOGE(TAG, "Error selecting endpoint");
  } else {
    this->state_ = PN7160State::EP_SELECTING;
  }
  // uint8_t interface = INTF_UNDETERMINED;
  // switch (this->current_protocol_) {
  //   case PROT_ISODEP:
  //     interface = INTF_ISODEP;
  //     break;
  //   case PROT_NFCDEP:
  //     interface = INTF_NFCDEP;
  //     break;
  //   case PROT_MIFARE:
  //     interface = INTF_TAGCMD;
  //     break;
  //   default:
  //     interface = INTF_FRAME;
  //     break;
  // }
  // if (!this->write_ctrl_and_read_(RF_GID, RF_DISCOVER_SELECT_OID, {0x01, this->current_protocol_, interface},
  //                                 response)) {
  //   ESP_LOGE(TAG, "Error sending discovery select");
  //   return;
  // }
}

// bool PN7160::presence_check_() {
//   std::vector<uint8_t> response;
//   switch (this->current_protocol_) {
//     case PROT_MIFARE: {
//       this->deactivate_(DEACTIVATION_TYPE_SLEEP);
//       // this->next_function_ = std::bind(&PN7160::select_tag_, this);
//     }
//     case PROT_T2T: {
//       this->deactivate_(DEACTIVATION_TYPE_SLEEP);
//       // this->next_function_ = std::bind(&PN7160::select_tag_, this);
//     }
//     default:
//       break;
//   }
//   return false;
// }

void PN7160::dump_config() {
  ESP_LOGCONFIG(TAG, "PN7160:");
  LOG_PIN("  CS Pin: ", this->cs_);
}

void PN7160::loop() {
  this->purge_old_tags_();
  this->nci_fsm_transition_();
}

void PN7160::purge_old_tags_() {
  std::ostringstream tag_msg;
  tag_msg << "Removing old tag UID from cache ";

  for (size_t i = 0; i < this->discovered_endpoint_.size(); i++) {
    if (millis() - this->discovered_endpoint_[i].last_seen > TAG_TTL) {
      auto uid = this->discovered_endpoint_[i].tag.get_uid();
      for (size_t i = 0; i < uid.size(); i++) {
        char buffer[6];
        sprintf(buffer, "%02X", uid[i]);
        if (i > 0) {
          tag_msg << "-";
        }
        tag_msg << buffer;
      }
      ESP_LOGW(TAG, "%s", tag_msg.str().c_str());
      this->discovered_endpoint_.erase(this->discovered_endpoint_.begin() + i);
    }
  }
}

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
