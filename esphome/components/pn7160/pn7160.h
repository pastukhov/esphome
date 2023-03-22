#pragma once

#include "esphome/components/spi/spi.h"
#include "esphome/components/nfc/automation.h"
#include "esphome/components/nfc/nfc.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

#include <functional>

namespace esphome {
namespace pn7160 {

static const uint8_t NFCC_DEFAULT_TIMEOUT = 5;
static const uint8_t NFCC_INIT_TIMEOUT = 50;
static const uint8_t NFCC_MFC_TIMEOUT = 10;

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

static const uint8_t PROPRIETARY_GID = 0xF;

static const uint8_t XCHG_DATA_OID = 0x10;
static const uint8_t MF_SECTORSEL_OID = 0x32;
static const uint8_t MFC_AUTHENTICATE_OID = 0x40;

static const uint8_t MFC_AUTHENTICATE_PARAM_KS_A = 0x00;  // key select A
static const uint8_t MFC_AUTHENTICATE_PARAM_KS_B = 0x80;  // key select B
static const uint8_t MFC_AUTHENTICATE_PARAM_EMBED_KEY = 0x10;

static const uint8_t CORE_CONFIG[] = {0x01,                     // Number of parameters
                                      0x00, 0x02, 0x00, 0x01};  // TOTAL_DURATION: 1ms

static const uint8_t CORE_CONF_EXTENSION[] = {
    0x03,                     // Number of parameters
    0xA0, 0x40, 0x01, 0x00,   // TAG_DETECTOR_CFG: Default
    0xA0, 0x41, 0x01, 0x04,   // TAG_DETECTOR_THRESHOLD_CFG: Default
    0xA0, 0x43, 0x01, 0x00};  // TAG_DETECTOR_FALLBACK_CNT_CFG: Hybrid mode disabled

static const uint8_t CLOCK_CONFIG[] = {0x01,                     // Number of parameters
                                       0xA0, 0x03, 0x01, 0x08};  // CLOCK_SEL_CFG: XTAL: 27.12MHz

static const uint8_t RF_CONF[] = {0x11,                                                   // Number of parameters
                                  0xA0, 0x0D, 0x06, 0x04, 0x35, 0x90, 0x01, 0xF4, 0x01,   //
                                  0xA0, 0x0D, 0x06, 0x06, 0x30, 0x01, 0x90, 0x03, 0x00,   //
                                  0xA0, 0x0D, 0x06, 0x06, 0x42, 0x02, 0x00, 0xFF, 0xFF,   //
                                  0xA0, 0x0D, 0x06, 0x20, 0x42, 0x88, 0x00, 0xFF, 0xFF,   //
                                  0xA0, 0x0D, 0x04, 0x22, 0x44, 0x23, 0x00,               //
                                  0xA0, 0x0D, 0x06, 0x22, 0x2D, 0x50, 0x34, 0x0C, 0x00,   //
                                  0xA0, 0x0D, 0x06, 0x32, 0x42, 0xF8, 0x00, 0xFF, 0xFF,   //
                                  0xA0, 0x0D, 0x06, 0x34, 0x2D, 0x24, 0x37, 0x0C, 0x00,   //
                                  0xA0, 0x0D, 0x06, 0x34, 0x33, 0x86, 0x80, 0x00, 0x70,   //
                                  0xA0, 0x0D, 0x04, 0x34, 0x44, 0x22, 0x00,               //
                                  0xA0, 0x0D, 0x06, 0x42, 0x2D, 0x15, 0x45, 0x0D, 0x00,   //
                                  0xA0, 0x0D, 0x04, 0x46, 0x44, 0x22, 0x00,               //
                                  0xA0, 0x0D, 0x06, 0x46, 0x2D, 0x05, 0x59, 0x0E, 0x00,   //
                                  0xA0, 0x0D, 0x06, 0x44, 0x42, 0x88, 0x00, 0xFF, 0xFF,   //
                                  0xA0, 0x0D, 0x06, 0x56, 0x2D, 0x05, 0x9F, 0x0C, 0x00,   //
                                  0xA0, 0x0D, 0x06, 0x54, 0x42, 0x88, 0x00, 0xFF, 0xFF,   //
                                  0xA0, 0x0D, 0x06, 0x0A, 0x33, 0x80, 0x86, 0x00, 0x70};  //

static const uint8_t RF_TVDD_CONFIG[] = {
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
    // 0xBB,  // TXLDO (4.7V / 4.7V)
    0xFF,  // TXLDO (5.0V / 5.0V)
    0x00,  // RFU
    0xD0,  // TXLDO check
    0x0C,  // RFU
};

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
static const uint8_t PROT_ISO15693 = 0x06;
static const uint8_t PROT_MIFARE = 0x80;

static const uint8_t RF_DISCOVER_MAP_MODE_POLL = 0x1;
static const uint8_t RF_DISCOVER_MAP_MODE_LISTEN = 0x2;

static const uint8_t INTF_UNDETERMINED = 0x0;
static const uint8_t INTF_FRAME = 0x1;
static const uint8_t INTF_ISODEP = 0x2;
static const uint8_t INTF_NFCDEP = 0x3;
static const uint8_t INTF_TAGCMD = 0x80;

static const uint8_t READ_WRITE_MODE[] = {PROT_T1T,    RF_DISCOVER_MAP_MODE_POLL, INTF_FRAME,    // Poll mode
                                          PROT_T2T,    RF_DISCOVER_MAP_MODE_POLL, INTF_FRAME,    // Poll mode
                                          PROT_T3T,    RF_DISCOVER_MAP_MODE_POLL, INTF_FRAME,    // Poll mode
                                          PROT_ISODEP, RF_DISCOVER_MAP_MODE_POLL, INTF_ISODEP,   // Poll mode
                                          PROT_MIFARE, RF_DISCOVER_MAP_MODE_POLL, INTF_TAGCMD};  // Poll mode

static const uint8_t DISCOVERY_READ_WRITE[] = {MODE_POLL | TECH_PASSIVE_NFCA,    //
                                               MODE_POLL | TECH_PASSIVE_NFCB,    //
                                               MODE_POLL | TECH_PASSIVE_NFCF,    //
                                               MODE_POLL | TECH_PASSIVE_15693};  //

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

enum class PN7160State : uint8_t {
  NONE = 0x00,
  NFCC_RESET,
  NFCC_INIT,
  NFCC_CONFIG,
  NFCC_SET_MODE,
  RFST_IDLE,
  RFST_DISCOVERY,
  RFST_W4_ALL_DISCOVERIES,
  RFST_W4_HOST_SELECT,
  RFST_POLL_ACTIVE,
  EP_DEACTIVATING,
  EP_SELECTING,
  FAILED = 0XFF,
};

enum PN7160Mode : uint8_t {
  CARD_EMULATION = 1 << 0,
  P2P = 1 << 1,
  READ_WRITE = 1 << 2,
};

struct DiscoveredEndpoint {
  uint8_t id;
  uint8_t protocol;
  uint32_t last_seen;
  nfc::NfcTag tag;
  bool trig_called;
};

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

  // void register_tag(PN532BinarySensor *tag) { this->binary_sensors_.push_back(tag); }
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
  uint8_t send_config_();

  uint8_t set_mode_();

  uint8_t start_discovery_();
  uint8_t deactivate_(uint8_t type);

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
  /// parse & process incoming messages from the NFCC
  void process_message_();

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

  uint8_t read_mifare_ultralight_tag_(nfc::NfcTag &tag);
  uint8_t read_mifare_ultralight_bytes_(const uint8_t start_page, const uint16_t num_bytes, std::vector<uint8_t> &data);
  bool is_mifare_ultralight_formatted_();
  uint16_t read_mifare_ultralight_capacity_();
  uint8_t find_mifare_ultralight_ndef_(uint8_t &message_length, uint8_t &message_start_index);
  uint8_t write_mifare_ultralight_page_(const uint8_t page_num, std::vector<uint8_t> &write_data);
  uint8_t write_mifare_ultralight_tag_(std::vector<uint8_t> &uid, nfc::NdefMessage *message);
  uint8_t clean_mifare_ultralight_();

  enum NfcTask {
    EP_READ = 0,
    EP_CLEAN,
    EP_FORMAT,
    EP_WRITE,
  } next_task_{EP_READ};

  GPIOPin *dwl_req_pin_;
  GPIOPin *irq_pin_;
  GPIOPin *ven_pin_;
  GPIOPin *wkup_req_pin_;

  uint8_t fail_count_{0};
  uint8_t generation_{0};
  uint8_t selecting_endpoint_{0};
  uint8_t tag_ttl_{250};
  uint8_t version_[3];

  CallbackManager<void()> on_finished_write_callback_;

  std::vector<DiscoveredEndpoint> discovered_endpoint_;

  PN7160State state_{PN7160State::NFCC_RESET};

  nfc::NdefMessage *next_task_message_to_write_;

  std::vector<nfc::NfcOnTagTrigger *> triggers_ontag_;
  std::vector<nfc::NfcOnTagTrigger *> triggers_ontagremoved_;
};

}  // namespace pn7160
}  // namespace esphome
