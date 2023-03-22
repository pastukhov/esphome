#include <memory>

#include "pn7160.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pn7160 {

static const char *const TAG = "pn7160.mifare_ultralight";

uint8_t PN7160::read_mifare_ultralight_tag_(nfc::NfcTag &tag) {
  if (!this->is_mifare_ultralight_formatted_()) {
    ESP_LOGW(TAG, "Not NDEF formatted");
    return STATUS_FAILED;
  }

  uint8_t message_length;
  uint8_t message_start_index;
  if (this->find_mifare_ultralight_ndef_(message_length, message_start_index) != STATUS_OK) {
    ESP_LOGW(TAG, "Couldn't find NDEF message");
    return STATUS_FAILED;
  }
  ESP_LOGVV(TAG, "NDEF message length: %d, start: %d", message_length, message_start_index);

  if (message_length == 0) {
    return STATUS_FAILED;
  }

  std::vector<uint8_t> data;
  if (read_mifare_ultralight_bytes_(nfc::MIFARE_ULTRALIGHT_DATA_START_PAGE, message_length, data) != STATUS_OK) {
    ESP_LOGE(TAG, "Error reading tag data");
    return STATUS_FAILED;
  }

  data.erase(data.begin(), data.begin() + message_start_index);

  tag.set_ndef_message(make_unique<nfc::NdefMessage>(data));

  return STATUS_OK;
}

uint8_t PN7160::read_mifare_ultralight_bytes_(uint8_t start_page, uint16_t num_bytes, std::vector<uint8_t> &data) {
  const uint8_t read_increment = nfc::MIFARE_ULTRALIGHT_READ_SIZE * nfc::MIFARE_ULTRALIGHT_PAGE_SIZE;
  std::vector<uint8_t> data_out = {nfc::MIFARE_CMD_READ, start_page};
  std::vector<uint8_t> pages_in;

  for (size_t i = 0; i * read_increment < num_bytes; i++) {
    data_out[1] = i * nfc::MIFARE_ULTRALIGHT_READ_SIZE + start_page;
    if (this->write_data_and_read_(data_out, pages_in, NFCC_DEFAULT_TIMEOUT, false) != STATUS_OK) {
      ESP_LOGE(TAG, "Error reading tag data");
      return STATUS_FAILED;
    }
    // if the subtraction rolls over the integer type, we'll append the full block of data we read;
    //  otherwise, only append the necessary portion of the data we read
    uint16_t bytes_offset = (i + 1) * read_increment;
    auto pages_in_end_itr =
        num_bytes - bytes_offset <= num_bytes ? pages_in.end() - 1 : pages_in.end() - bytes_offset - num_bytes + 1;
    data.insert(data.end(), pages_in.begin() + 3, pages_in_end_itr);
  }

  ESP_LOGVV(TAG, "Data read: %s", nfc::format_bytes(data).c_str());

  return STATUS_OK;
}

bool PN7160::is_mifare_ultralight_formatted_() {
  std::vector<uint8_t> data;
  if (this->read_mifare_ultralight_bytes_(nfc::MIFARE_ULTRALIGHT_DATA_START_PAGE, nfc::MIFARE_ULTRALIGHT_PAGE_SIZE,
                                          data) == STATUS_OK) {
    return !(data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF && data[3] == 0xFF);
  }
  return true;
}

uint16_t PN7160::read_mifare_ultralight_capacity_() {
  std::vector<uint8_t> data;
  if (this->read_mifare_ultralight_bytes_(3, nfc::MIFARE_ULTRALIGHT_PAGE_SIZE, data) == STATUS_OK) {
    ESP_LOGVV(TAG, "Tag capacity is %u bytes", data[2] * 8U);
    return data[2] * 8U;
  }
  return 0;
}

uint8_t PN7160::find_mifare_ultralight_ndef_(uint8_t &message_length, uint8_t &message_start_index) {
  std::vector<uint8_t> data;
  if (this->read_mifare_ultralight_bytes_(nfc::MIFARE_ULTRALIGHT_DATA_START_PAGE, nfc::MIFARE_ULTRALIGHT_PAGE_SIZE * 2,
                                          data) != STATUS_OK) {
    return STATUS_FAILED;
  }

  if (data[0] == 0x03) {
    message_length = data[1];
    message_start_index = 2;
    return STATUS_OK;
  } else if (data[5] == 0x03) {
    message_length = data[6];
    message_start_index = 7;
    return STATUS_OK;
  }
  return STATUS_FAILED;
}

uint8_t PN7160::write_mifare_ultralight_tag_(std::vector<uint8_t> &uid, nfc::NdefMessage *message) {
  uint32_t capacity = this->read_mifare_ultralight_capacity_();

  auto encoded = message->encode();

  uint32_t message_length = encoded.size();
  uint32_t buffer_length = nfc::get_mifare_ultralight_buffer_size(message_length);

  if (buffer_length > capacity) {
    ESP_LOGE(TAG, "Message length exceeds tag capacity %u > %u", buffer_length, capacity);
    return STATUS_FAILED;
  }

  encoded.insert(encoded.begin(), 0x03);
  if (message_length < 255) {
    encoded.insert(encoded.begin() + 1, message_length);
  } else {
    encoded.insert(encoded.begin() + 1, 0xFF);
    encoded.insert(encoded.begin() + 2, (message_length >> 8) & 0xFF);
    encoded.insert(encoded.begin() + 2, message_length & 0xFF);
  }
  encoded.push_back(0xFE);

  encoded.resize(buffer_length, 0);

  uint32_t index = 0;
  uint8_t current_page = nfc::MIFARE_ULTRALIGHT_DATA_START_PAGE;

  while (index < buffer_length) {
    std::vector<uint8_t> data(encoded.begin() + index, encoded.begin() + index + nfc::MIFARE_ULTRALIGHT_PAGE_SIZE);
    if (this->write_mifare_ultralight_page_(current_page, data) != STATUS_OK) {
      return STATUS_FAILED;
    }
    index += nfc::MIFARE_ULTRALIGHT_PAGE_SIZE;
    current_page++;
  }
  return STATUS_OK;
}

uint8_t PN7160::clean_mifare_ultralight_() {
  uint32_t capacity = this->read_mifare_ultralight_capacity_();
  uint8_t pages = (capacity / nfc::MIFARE_ULTRALIGHT_PAGE_SIZE) + nfc::MIFARE_ULTRALIGHT_DATA_START_PAGE;

  std::vector<uint8_t> blank_data = {0x00, 0x00, 0x00, 0x00};

  for (int i = nfc::MIFARE_ULTRALIGHT_DATA_START_PAGE; i < pages; i++) {
    if (this->write_mifare_ultralight_page_(i, blank_data) != STATUS_OK) {
      return STATUS_FAILED;
    }
  }
  return STATUS_OK;
}

uint8_t PN7160::write_mifare_ultralight_page_(uint8_t page_num, std::vector<uint8_t> &write_data) {
  std::vector<uint8_t> data_out = {nfc::MIFARE_CMD_WRITE_ULTRALIGHT, page_num};
  std::vector<uint8_t> response;
  data_out.insert(data_out.end(), write_data.begin(), write_data.end());
  if (this->write_data_and_read_(data_out, response, NFCC_DEFAULT_TIMEOUT, false) != STATUS_OK) {
    ESP_LOGE(TAG, "Error writing page %u", page_num);
    return STATUS_FAILED;
  }
  return STATUS_OK;
}

}  // namespace pn7160
}  // namespace esphome
