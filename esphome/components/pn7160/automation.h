#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/pn7160/pn7160.h"

namespace esphome {
namespace pn7160 {

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

class PN7160OnEmulatedTagScanTrigger : public Trigger<> {
 public:
  explicit PN7160OnEmulatedTagScanTrigger(PN7160 *parent) {
    parent->add_on_emulated_tag_scan_callback([this]() { this->trigger(); });
  }
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

template<typename... Ts> class SetCleanModeAction : public Action<Ts...> {
 public:
  explicit SetCleanModeAction(PN7160 *a_pn7160) : pn7160_(a_pn7160) {}

  void play(Ts... x) override { this->pn7160_->clean_mode(); }

 protected:
  PN7160 *pn7160_;
};

template<typename... Ts> class SetFormatModeAction : public Action<Ts...> {
 public:
  explicit SetFormatModeAction(PN7160 *a_pn7160) : pn7160_(a_pn7160) {}

  void play(Ts... x) override { this->pn7160_->format_mode(); }

 protected:
  PN7160 *pn7160_;
};

template<typename... Ts> class SetReadModeAction : public Action<Ts...> {
 public:
  explicit SetReadModeAction(PN7160 *a_pn7160) : pn7160_(a_pn7160) {}

  void play(Ts... x) override { this->pn7160_->read_mode(); }

 protected:
  PN7160 *pn7160_;
};

template<typename... Ts> class SetWriteModeAction : public Action<Ts...> {
 public:
  explicit SetWriteModeAction(PN7160 *a_pn7160) : pn7160_(a_pn7160) {}

  void play(Ts... x) override { this->pn7160_->write_mode(); }

 protected:
  PN7160 *pn7160_;
};

}  // namespace pn7160
}  // namespace esphome
