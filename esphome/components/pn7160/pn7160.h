#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/nfc/automation.h"
#include "esphome/components/nfc/nfc.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

#include <functional>

namespace esphome {
namespace pn7160 {

static const uint16_t NFCC_DEFAULT_TIMEOUT = 5;
static const uint16_t NFCC_FULL_TIMEOUT = 1000;
static const uint16_t NFCC_INIT_TIMEOUT = 50;
static const uint16_t NFCC_MFC_TIMEOUT = 15;

static const uint8_t NFCC_MAX_COMM_FAILS = 2;

static const uint8_t TDD_SPI_READ = 0xFF;
static const uint8_t TDD_SPI_WRITE = 0x0A;

static const uint8_t MT_MASK = 0xE0;

static const uint8_t MT_DATA = 0x00;               // For sending commands to NFC endpoint (tag)
static const uint8_t MT_CTRL_COMMAND = 0x20;       // For sending commands to NFCC
static const uint8_t MT_CTRL_RESPONSE = 0x40;      // Response from NFCC to commands
static const uint8_t MT_CTRL_NOTIFICATION = 0x60;  // Notification from NFCC

static const uint8_t GID_MASK = 0x0F;
static const uint8_t OID_MASK = 0x3F;

static const uint8_t NCI_CORE_GID = 0x0;
static const uint8_t NCI_PROPRIETARY_GID = 0xF;

static const uint8_t NCI_CORE_RESET_OID = 0x00;
static const uint8_t NCI_CORE_INIT_OID = 0x01;
static const uint8_t NCI_CORE_SET_CONFIG_OID = 0x02;
static const uint8_t NCI_CORE_GET_CONFIG_OID = 0x03;
static const uint8_t NCI_CORE_CONN_CREATE_OID = 0x04;
static const uint8_t NCI_CORE_CONN_CLOSE_OID = 0x05;
static const uint8_t NCI_CORE_CONN_CREDITS_OID = 0x06;
static const uint8_t NCI_CORE_GENERIC_ERROR_OID = 0x07;
static const uint8_t NCI_CORE_INTERFACE_ERROR_OID = 0x08;

static const uint8_t RF_GID = 0x1;

static const uint8_t RF_DISCOVER_MAP_OID = 0x00;
static const uint8_t RF_SET_LISTEN_MODE_ROUTING_OID = 0x01;
static const uint8_t RF_GET_LISTEN_MODE_ROUTING_OID = 0x02;
static const uint8_t RF_DISCOVER_OID = 0x03;
static const uint8_t RF_DISCOVER_SELECT_OID = 0x04;
static const uint8_t RF_INTF_ACTIVATED_OID = 0x05;
static const uint8_t RF_DEACTIVATE_OID = 0x06;
static const uint8_t RF_FIELD_INFO_OID = 0x07;
static const uint8_t RF_T3T_POLLING_OID = 0x08;
static const uint8_t RF_NFCEE_ACTION_OID = 0x09;
static const uint8_t RF_NFCEE_DISCOVERY_REQ_OID = 0x0A;
static const uint8_t RF_PARAMETER_UPDATE_OID = 0x0B;

static const uint8_t NFCEE_GID = 0x2;

static const uint8_t NFCEE_DISCOVER_OID = 0x00;
static const uint8_t NFCEE_MODE_SET_OID = 0x01;

static const uint8_t XCHG_DATA_OID = 0x10;
static const uint8_t MF_SECTORSEL_OID = 0x32;
static const uint8_t MFC_AUTHENTICATE_OID = 0x40;

static const uint8_t MFC_AUTHENTICATE_PARAM_KS_A = 0x00;  // key select A
static const uint8_t MFC_AUTHENTICATE_PARAM_KS_B = 0x80;  // key select B
static const uint8_t MFC_AUTHENTICATE_PARAM_EMBED_KEY = 0x10;

static const uint8_t MODE_MASK = 0xF0;
static const uint8_t MODE_LISTEN = 0x80;
static const uint8_t MODE_POLL = 0x00;

static const uint8_t TECH_PASSIVE_NFCA = 0;
static const uint8_t TECH_PASSIVE_NFCB = 1;
static const uint8_t TECH_PASSIVE_NFCF = 2;
static const uint8_t TECH_ACTIVE_NFCA = 3;
static const uint8_t TECH_ACTIVE_NFCF = 5;
static const uint8_t TECH_PASSIVE_15693 = 6;

static const uint8_t PROT_UNDETERMINED = 0x00;
static const uint8_t PROT_T1T = 0x01;
static const uint8_t PROT_T2T = 0x02;
static const uint8_t PROT_T3T = 0x03;
static const uint8_t PROT_ISODEP = 0x04;
static const uint8_t PROT_NFCDEP = 0x05;
static const uint8_t PROT_T5T = 0x06;
static const uint8_t PROT_MIFARE = 0x80;

static const uint8_t RF_DISCOVER_MAP_MODE_POLL = 0x1;
static const uint8_t RF_DISCOVER_MAP_MODE_LISTEN = 0x2;

static const uint8_t INTF_UNDETERMINED = 0x0;
static const uint8_t INTF_FRAME = 0x1;
static const uint8_t INTF_ISODEP = 0x2;
static const uint8_t INTF_NFCDEP = 0x3;
static const uint8_t INTF_TAGCMD = 0x80;

static const uint8_t DEACTIVATION_TYPE_IDLE = 0x00;
static const uint8_t DEACTIVATION_TYPE_SLEEP = 0x01;
static const uint8_t DEACTIVATION_TYPE_SLEEP_AF = 0x02;
static const uint8_t DEACTIVATION_TYPE_DISCOVERY = 0x03;

static const uint8_t STATUS_OK = 0x00;
static const uint8_t STATUS_REJECTED = 0x01;
static const uint8_t STATUS_RF_FRAME_CORRUPTED = 0x02;
static const uint8_t STATUS_FAILED = 0x03;
static const uint8_t STATUS_NOT_INITIALIZED = 0x04;
static const uint8_t STATUS_SYNTAX_ERROR = 0x05;
static const uint8_t STATUS_SEMANTIC_ERROR = 0x06;
static const uint8_t STATUS_INVALID_PARAM = 0x09;
static const uint8_t STATUS_MESSAGE_SIZE_EXCEEDED = 0x0A;

static const uint8_t DISCOVERY_ALREADY_STARTED = 0xA0;
static const uint8_t DISCOVERY_TARGET_ACTIVATION_FAILED = 0xA1;
static const uint8_t DISCOVERY_TEAR_DOWN = 0xA2;

static const uint8_t RF_DISCOVER_NTF_NT_LAST = 0x00;
static const uint8_t RF_DISCOVER_NTF_NT_LAST_RL = 0x01;
static const uint8_t RF_DISCOVER_NTF_NT_MORE = 0x02;

static const uint8_t CARD_EMU_T4T_APP_SELECT[] = {0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76,
                                                  0x00, 0x00, 0x85, 0x01, 0x01, 0x00};
static const uint8_t CARD_EMU_T4T_CC[] = {0x00, 0x0F, 0x20, 0x00, 0xFF, 0x00, 0xFF, 0x04,
                                          0x06, 0xE1, 0x04, 0x00, 0xFF, 0x00, 0x00};
static const uint8_t CARD_EMU_T4T_CC_SELECT[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03};
static const uint8_t CARD_EMU_T4T_NDEF_SELECT[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04};
static const uint8_t CARD_EMU_T4T_READ[] = {0x00, 0xB0};
static const uint8_t CARD_EMU_T4T_WRITE[] = {0x00, 0xD6};
static const uint8_t CARD_EMU_T4T_OK[] = {0x90, 0x00};
static const uint8_t CARD_EMU_T4T_NOK[] = {0x6A, 0x82};

static const uint8_t CORE_CONFIG_SOLO[] = {0x01,   // Number of parameter fields
                                           0x00,   // config param identifier (TOTAL_DURATION)
                                           0x02,   // length of value
                                           0x01,   // TOTAL_DURATION (low)...
                                           0x00};  // TOTAL_DURATION (high): 1 ms

static const uint8_t CORE_CONFIG_RW_CE[] = {0x01,   // Number of parameter fields
                                            0x00,   // config param identifier (TOTAL_DURATION)
                                            0x02,   // length of value
                                            0xF8,   // TOTAL_DURATION (low)...
                                            0x02};  // TOTAL_DURATION (high): 760 ms

static const uint8_t PMU_CFG[] = {
    0x01,        // Number of parameters
    0xA0, 0x0E,  // ext. tag
    11,          // length
    0x11,        // IRQ Enable: PVDD + temp sensor IRQs
    0x01,        // RFU
    0x01,        // Power and Clock Configuration, device on (CFG1)
    0x01,        // Power and Clock Configuration, device off (CFG1)
    0x00,        // RFU
    0x00,        // DC-DC 0
    0x00,        // DC-DC 1
    // 0x14,        // TXLDO (3.3V / 4.75V)
    // 0xBB,        // TXLDO (4.7V / 4.7V)
    0xFF,  // TXLDO (5.0V / 5.0V)
    0x00,  // RFU
    0xD0,  // TXLDO check
    0x0C,  // RFU
};

static const uint8_t RF_DISCOVER_MAP_CONFIG[] = {  // poll modes
    PROT_T1T,    RF_DISCOVER_MAP_MODE_POLL,
    INTF_FRAME,  // poll mode
    PROT_T2T,    RF_DISCOVER_MAP_MODE_POLL,
    INTF_FRAME,  // poll mode
    PROT_T3T,    RF_DISCOVER_MAP_MODE_POLL,
    INTF_FRAME,  // poll mode
    PROT_ISODEP, RF_DISCOVER_MAP_MODE_POLL | RF_DISCOVER_MAP_MODE_LISTEN,
    INTF_ISODEP,  // poll & listen mode
    PROT_MIFARE, RF_DISCOVER_MAP_MODE_POLL,
    INTF_TAGCMD};  // poll mode

static const uint8_t RF_DISCOVERY_LISTEN_CONFIG[] = {MODE_LISTEN | TECH_PASSIVE_NFCA,   // listen mode
                                                     MODE_LISTEN | TECH_PASSIVE_NFCB,   // listen mode
                                                     MODE_LISTEN | TECH_PASSIVE_NFCF};  // listen mode

static const uint8_t RF_DISCOVERY_POLL_CONFIG[] = {MODE_POLL | TECH_PASSIVE_NFCA,    // poll mode
                                                   MODE_POLL | TECH_PASSIVE_NFCB,    // poll mode
                                                   MODE_POLL | TECH_PASSIVE_NFCF,    // poll mode
                                                   MODE_POLL | TECH_PASSIVE_15693};  // poll mode

static const uint8_t RF_DISCOVERY_CONFIG[] = {MODE_POLL | TECH_PASSIVE_NFCA,     // poll mode
                                              MODE_POLL | TECH_PASSIVE_NFCB,     // poll mode
                                              MODE_POLL | TECH_PASSIVE_NFCF,     // poll mode
                                              MODE_POLL | TECH_PASSIVE_15693,    // poll mode
                                              MODE_LISTEN | TECH_PASSIVE_NFCA,   // listen mode
                                              MODE_LISTEN | TECH_PASSIVE_NFCB,   // listen mode
                                              MODE_LISTEN | TECH_PASSIVE_NFCF};  // listen mode

static const uint8_t RF_LISTEN_MODE_ROUTING_CONFIG[] = {0x00,  // "more" (another message is coming)
                                                        2,     // number of table entries
                                                        0x01,  // type = protocol-based
                                                        3,     // length
                                                        0,     // DH NFCEE ID, a static ID representing the DH-NFCEE
                                                        0x07,  // power state
                                                        PROT_ISODEP,  // protocol
                                                        0x00,         // type = technology-based
                                                        3,            // length
                                                        0,     // DH NFCEE ID, a static ID representing the DH-NFCEE
                                                        0x07,  // power state
                                                        TECH_PASSIVE_NFCA};  // technology

enum class CardEmulationState : uint8_t {
  CARD_EMU_IDLE,
  CARD_EMU_NDEF_APP_SELECTED,
  CARD_EMU_CC_SELECTED,
  CARD_EMU_NDEF_SELECTED,
  CARD_EMU_DESFIRE_PROD,
};

enum class NCIState : uint8_t {
  NONE = 0x00,
  NFCC_RESET,
  NFCC_INIT,
  NFCC_CONFIG,
  NFCC_SET_DISCOVER_MAP,
  NFCC_SET_LISTEN_MODE_ROUTING,
  RFST_IDLE,
  RFST_DISCOVERY,
  RFST_W4_ALL_DISCOVERIES,
  RFST_W4_HOST_SELECT,
  RFST_LISTEN_ACTIVE,
  RFST_LISTEN_SLEEP,
  RFST_POLL_ACTIVE,
  EP_DEACTIVATING,
  EP_SELECTING,
  FAILED = 0XFF,
};

struct DiscoveredEndpoint {
  uint8_t id;
  uint8_t protocol;
  uint32_t last_seen;
  nfc::NfcTag tag;
  bool trig_called;
};

class PN7160BinarySensor;

class PN7160 : public Component,
               public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                     spi::DATA_RATE_4MHZ> {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const { return setup_priority::DATA; }
  void loop() override;

  void set_dwl_req_pin(GPIOPin *dwl_req_pin) { this->dwl_req_pin_ = dwl_req_pin; }
  void set_irq_pin(GPIOPin *irq_pin) { this->irq_pin_ = irq_pin; }
  void set_ven_pin(GPIOPin *ven_pin) { this->ven_pin_ = ven_pin; }
  void set_wkup_req_pin(GPIOPin *wkup_req_pin) { this->wkup_req_pin_ = wkup_req_pin; }

  void set_tag_ttl(uint32_t ttl) { this->tag_ttl_ = ttl; }
  void set_tag_to_emulate(std::shared_ptr<nfc::NdefMessage> message);
  void set_tag_emulation_off();
  void set_tag_emulation_on();
  bool tag_emulation_enabled() { return this->listening_enabled_; }

  void register_tag(PN7160BinarySensor *tag) { this->binary_sensors_.push_back(tag); }
  void register_ontag_trigger(nfc::NfcOnTagTrigger *trig) { this->triggers_ontag_.push_back(trig); }
  void register_ontagremoved_trigger(nfc::NfcOnTagTrigger *trig) { this->triggers_ontagremoved_.push_back(trig); }

  void add_on_finished_write_callback(std::function<void()> callback) {
    this->on_finished_write_callback_.add(std::move(callback));
  }

  bool is_writing() { return this->next_task_ != EP_READ; };

  void read_mode();
  void clean_mode();
  void format_mode();
  void write_mode(nfc::NdefMessage *message);

 protected:
  void init_failure_handler_();

  uint8_t reset_core_(bool reset_config, bool power);
  uint8_t init_core_(bool store_report);
  uint8_t send_init_config_();
  uint8_t send_core_config_();

  uint8_t set_discover_map_();

  uint8_t set_listen_mode_routing_();

  uint8_t start_discovery_();
  uint8_t deactivate_(uint8_t type, uint16_t timeout = NFCC_DEFAULT_TIMEOUT);

  void select_endpoint_();

  uint8_t read_endpoint_data_(nfc::NfcTag &tag);
  uint8_t clean_endpoint_(std::vector<uint8_t> &uid);
  uint8_t format_endpoint_(std::vector<uint8_t> &uid);
  uint8_t write_endpoint_(std::vector<uint8_t> &uid, nfc::NdefMessage *message);

  std::unique_ptr<nfc::NfcTag> build_tag_(const uint8_t mode_tech, const std::vector<uint8_t> &data);
  optional<size_t> find_tag_uid_(const std::vector<uint8_t> &uid);
  void purge_old_tags_();

  /// advance controller state as required
  void nci_fsm_transition_();
  /// set new controller state
  void nci_fsm_set_state_(NCIState new_state);
  /// parse & process incoming messages from the NFCC
  void process_message_();
  void process_rf_intf_activated_oid_(std::vector<uint8_t> &response);
  void process_rf_discover_oid_(std::vector<uint8_t> &response);
  void process_rf_deactivate_oid_(std::vector<uint8_t> &response);
  void process_data_message_(std::vector<uint8_t> &response);

  void card_emu_t4t_get_response(std::vector<uint8_t> &response, std::vector<uint8_t> &ndef_response);

  uint8_t write_ctrl_and_read_(const uint8_t gid, const uint8_t oid, const std::vector<uint8_t> &data,
                               std::vector<uint8_t> &response, const uint16_t timeout = NFCC_DEFAULT_TIMEOUT,
                               const bool warn = true);
  uint8_t write_ctrl_and_read_(const uint8_t gid, const uint8_t oid, const uint8_t *data, uint8_t len,
                               std::vector<uint8_t> &response, const uint16_t timeout = NFCC_DEFAULT_TIMEOUT,
                               const bool warn = true);
  uint8_t write_data_and_read_(const std::vector<uint8_t> &data, std::vector<uint8_t> &response,
                               const uint16_t timeout = NFCC_DEFAULT_TIMEOUT, const bool warn = true);
  uint8_t write_and_read_(const std::vector<uint8_t> &data, std::vector<uint8_t> &response, const uint16_t timeout,
                          const bool warn);
  uint8_t read_data_(std::vector<uint8_t> &data, const uint16_t timeout = NFCC_DEFAULT_TIMEOUT, const bool warn = true);
  uint8_t write_data_(const std::vector<uint8_t> &data);
  uint8_t wait_for_irq_(uint16_t timeout = NFCC_DEFAULT_TIMEOUT, bool state = true, bool warn = true);

  uint8_t read_mifare_classic_tag_(nfc::NfcTag &tag);
  uint8_t read_mifare_classic_block_(const uint8_t block_num, std::vector<uint8_t> &data);
  uint8_t write_mifare_classic_block_(const uint8_t block_num, std::vector<uint8_t> &data);
  uint8_t auth_mifare_classic_block_(const uint8_t block_num, const uint8_t key_num, const uint8_t *key);
  uint8_t sect_to_auth(const uint8_t block_num);
  uint8_t format_mifare_classic_mifare_();
  uint8_t format_mifare_classic_ndef_();
  uint8_t write_mifare_classic_tag_(nfc::NdefMessage *message);
  uint8_t halt_mifare_classic_tag_();

  uint8_t read_mifare_ultralight_tag_(nfc::NfcTag &tag);
  uint8_t read_mifare_ultralight_bytes_(const uint8_t start_page, const uint16_t num_bytes, std::vector<uint8_t> &data);
  bool is_mifare_ultralight_formatted_();
  uint16_t read_mifare_ultralight_capacity_();
  uint8_t find_mifare_ultralight_ndef_(uint8_t &message_length, uint8_t &message_start_index);
  uint8_t write_mifare_ultralight_page_(const uint8_t page_num, std::vector<uint8_t> &write_data);
  uint8_t write_mifare_ultralight_tag_(std::vector<uint8_t> &uid, nfc::NdefMessage *message);
  uint8_t clean_mifare_ultralight_();

  enum NfcTask : uint8_t {
    EP_READ = 0,
    EP_CLEAN,
    EP_FORMAT,
    EP_WRITE,
  } next_task_{EP_READ};

  bool config_update_pending_{false};
  bool listening_enabled_{true};
  bool polling_enabled_{true};

  uint8_t fail_count_{0};
  uint32_t last_nci_state_change_{0};
  uint8_t selecting_endpoint_{0};
  uint32_t tag_ttl_{250};

  uint8_t hw_version_{0};
  uint8_t rom_code_version_{0};
  uint8_t flash_major_version_{0};
  uint8_t flash_minor_version_{0};

  GPIOPin *dwl_req_pin_;
  GPIOPin *irq_pin_;
  GPIOPin *ven_pin_;
  GPIOPin *wkup_req_pin_;

  CallbackManager<void()> on_finished_write_callback_;

  std::vector<DiscoveredEndpoint> discovered_endpoint_;

  CardEmulationState ce_state_{CardEmulationState::CARD_EMU_IDLE};
  NCIState nci_state_{NCIState::NFCC_RESET};

  std::shared_ptr<nfc::NdefMessage> card_emulation_message_;
  nfc::NdefMessage *next_task_message_to_write_;

  std::vector<PN7160BinarySensor *> binary_sensors_;
  std::vector<nfc::NfcOnTagTrigger *> triggers_ontag_;
  std::vector<nfc::NfcOnTagTrigger *> triggers_ontagremoved_;
};

class PN7160BinarySensor : public binary_sensor::BinarySensor {
 public:
  void set_uid(const std::vector<uint8_t> &uid) { uid_ = uid; }

  bool tag_match(const std::vector<uint8_t> &data);

  void tag_off(const std::vector<uint8_t> &data) {
    if (this->tag_match(data)) {
      this->publish_state(false);
    }
  }

  void tag_on(const std::vector<uint8_t> &data) {
    if (this->tag_match(data)) {
      this->publish_state(true);
    }
  }

 protected:
  std::vector<uint8_t> uid_;
};

class PN7160OnFinishedWriteTrigger : public Trigger<> {
 public:
  explicit PN7160OnFinishedWriteTrigger(PN7160 *parent) {
    parent->add_on_finished_write_callback([this]() { this->trigger(); });
  }
};

template<typename... Ts> class PN7160IsWritingCondition : public Condition<Ts...>, public Parented<PN7160> {
 public:
  bool check(Ts... x) override { return this->parent_->is_writing(); }
};

}  // namespace pn7160
}  // namespace esphome
