#include <memory>

#include "pn7160.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pn7160 {

static const char *const TAG = "pn7160.mifare_classic";

uint8_t PN7160::read_mifare_classic_tag_(nfc::NfcTag &tag) {
  uint8_t current_block = 4;
  uint8_t message_start_index = 0;
  uint32_t message_length = 0;

  if (this->auth_mifare_classic_block_(current_block, nfc::MIFARE_CMD_AUTH_A, nfc::NDEF_KEY) != STATUS_OK) {
    ESP_LOGE(TAG, "Tag auth failed while attempting to read tag data");
    return STATUS_FAILED;
  }
  std::vector<uint8_t> data;

  if (this->read_mifare_classic_block_(current_block, data) == STATUS_OK) {
    if (!nfc::decode_mifare_classic_tlv(data, message_length, message_start_index)) {
      return STATUS_FAILED;
    }
  } else {
    ESP_LOGE(TAG, "Failed to read block %d", current_block);
    return STATUS_FAILED;
  }

  uint32_t index = 0;
  uint32_t buffer_size = nfc::get_mifare_classic_buffer_size(message_length);
  std::vector<uint8_t> buffer;

  while (index < buffer_size) {
    if (nfc::mifare_classic_is_first_block(current_block)) {
      if (this->auth_mifare_classic_block_(current_block, nfc::MIFARE_CMD_AUTH_A, nfc::NDEF_KEY) != STATUS_OK) {
        ESP_LOGE(TAG, "Error, Block authentication failed for %d", current_block);
      }
    }
    std::vector<uint8_t> block_data;
    if (this->read_mifare_classic_block_(current_block, block_data) != STATUS_OK) {
      ESP_LOGE(TAG, "Error reading block %d", current_block);
    } else {
      buffer.insert(buffer.end(), block_data.begin(), block_data.end());
    }

    index += nfc::MIFARE_CLASSIC_BLOCK_SIZE;
    current_block++;

    if (nfc::mifare_classic_is_trailer_block(current_block)) {
      current_block++;
    }
  }
  buffer.erase(buffer.begin(), buffer.begin() + message_start_index);

  tag.set_ndef_message(make_unique<nfc::NdefMessage>(buffer));

  return STATUS_OK;
}

uint8_t PN7160::read_mifare_classic_block_(uint8_t block_num, std::vector<uint8_t> &data) {
  std::vector<uint8_t> data_out = {XCHG_DATA_OID, nfc::MIFARE_CMD_READ, block_num};
  std::vector<uint8_t> response;

  ESP_LOGVV(TAG, "Read XCHG_DATA_REQ: %s", nfc::format_bytes(data_out).c_str());
  if (this->write_data_and_read_(data_out, response, NFCC_DEFAULT_TIMEOUT, false) != STATUS_OK) {
    ESP_LOGE(TAG, "Timeout reading tag data");
    return STATUS_FAILED;
  }

  if ((response[0] != MT_DATA) || (response[3] != XCHG_DATA_OID) || (response[2] != 18)) {
    ESP_LOGE(TAG, "MFC read block failed - block 0x%02x", block_num);
    ESP_LOGW(TAG, "Read response: %s", nfc::format_bytes(response).c_str());
    return STATUS_FAILED;
  }

  data.insert(data.begin(), response.begin() + 4, response.end() - 1);

  ESP_LOGV(TAG, " Block %u: %s", block_num, nfc::format_bytes(data).c_str());
  return STATUS_OK;
}

uint8_t PN7160::auth_mifare_classic_block_(uint8_t block_num, uint8_t key_num, const uint8_t *key) {
  std::vector<uint8_t> data_out = {MFC_AUTHENTICATE_OID, this->sect_to_auth(block_num), key_num};
  std::vector<uint8_t> response;

  switch (key_num) {
    case nfc::MIFARE_CMD_AUTH_A:
      data_out.back() = MFC_AUTHENTICATE_PARAM_KS_A;
      break;

    case nfc::MIFARE_CMD_AUTH_B:
      data_out.back() = MFC_AUTHENTICATE_PARAM_KS_B;
      break;

    default:
      break;
  }

  if (key != nullptr) {
    data_out.back() |= MFC_AUTHENTICATE_PARAM_EMBED_KEY;
    data_out.insert(data_out.end(), key, key + 6);
  }

  ESP_LOGVV(TAG, "MFC_AUTHENTICATE_REQ: %s", nfc::format_bytes(data_out).c_str());
  if (this->write_data_and_read_(data_out, response, NFCC_DEFAULT_TIMEOUT, false) != STATUS_OK) {
    ESP_LOGE(TAG, "Sending MFC_AUTHENTICATE_REQ failed");
    return STATUS_FAILED;
  }
  if ((response[0] != MT_DATA) || (response[3] != MFC_AUTHENTICATE_OID) || (response[4] != STATUS_OK)) {
    ESP_LOGE(TAG, "MFC authentication failed - block 0x%02x", block_num);
    ESP_LOGVV(TAG, "MFC_AUTHENTICATE_RSP: %s", nfc::format_bytes(response).c_str());
    return STATUS_FAILED;
  }

  ESP_LOGV(TAG, "MFC block %u authentication succeeded", block_num);
  return STATUS_OK;
}

uint8_t PN7160::sect_to_auth(const uint8_t block_num) {
  const uint8_t first_high_block = nfc::MIFARE_CLASSIC_BLOCKS_PER_SECT_LOW * nfc::MIFARE_CLASSIC_16BLOCK_SECT_START;
  if (block_num >= first_high_block) {
    return ((block_num - first_high_block) / nfc::MIFARE_CLASSIC_BLOCKS_PER_SECT_HIGH) +
           nfc::MIFARE_CLASSIC_16BLOCK_SECT_START;
  }
  return block_num / nfc::MIFARE_CLASSIC_BLOCKS_PER_SECT_LOW;
}

uint8_t PN7160::format_mifare_classic_mifare_() {
  std::vector<uint8_t> blank_buffer(
      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
  std::vector<uint8_t> trailer_buffer(
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x80, 0x69, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});

  auto status = STATUS_OK;

  for (int block = 0; block < 64; block += 4) {
    if (this->auth_mifare_classic_block_(block + 3, nfc::MIFARE_CMD_AUTH_B, nfc::DEFAULT_KEY) != STATUS_OK) {
      continue;
    }
    if (block != 0) {
      if (this->write_mifare_classic_block_(block, blank_buffer) != STATUS_OK) {
        ESP_LOGE(TAG, "Unable to write block %d", block);
        status = STATUS_FAILED;
      }
    }
    if (this->write_mifare_classic_block_(block + 1, blank_buffer) != STATUS_OK) {
      ESP_LOGE(TAG, "Unable to write block %d", block + 1);
      status = STATUS_FAILED;
    }
    if (this->write_mifare_classic_block_(block + 2, blank_buffer) != STATUS_OK) {
      ESP_LOGE(TAG, "Unable to write block %d", block + 2);
      status = STATUS_FAILED;
    }
    if (this->write_mifare_classic_block_(block + 3, trailer_buffer) != STATUS_OK) {
      ESP_LOGE(TAG, "Unable to write block %d", block + 3);
      status = STATUS_FAILED;
    }
  }

  return status;
}

uint8_t PN7160::format_mifare_classic_ndef_() {
  std::vector<uint8_t> empty_ndef_message(
      {0x03, 0x03, 0xD0, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
  std::vector<uint8_t> blank_block(
      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
  std::vector<uint8_t> block_1_data(
      {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1});
  std::vector<uint8_t> block_2_data(
      {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1});
  std::vector<uint8_t> block_3_trailer(
      {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
  std::vector<uint8_t> ndef_trailer(
      {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});

  if (this->auth_mifare_classic_block_(0, nfc::MIFARE_CMD_AUTH_B, nfc::DEFAULT_KEY) != STATUS_OK) {
    ESP_LOGE(TAG, "Unable to authenticate block 0 for formatting!");
    return STATUS_FAILED;
  }
  if (this->write_mifare_classic_block_(1, block_1_data) != STATUS_OK) {
    return STATUS_FAILED;
  }
  if (this->write_mifare_classic_block_(2, block_2_data) != STATUS_OK) {
    return STATUS_FAILED;
  }
  if (this->write_mifare_classic_block_(3, block_3_trailer) != STATUS_OK) {
    return STATUS_FAILED;
  }

  ESP_LOGD(TAG, "Sector 0 formatted to NDEF");

  auto status = STATUS_OK;

  for (int block = 4; block < 64; block += 4) {
    if (this->auth_mifare_classic_block_(block + 3, nfc::MIFARE_CMD_AUTH_B, nfc::DEFAULT_KEY) != STATUS_OK) {
      return STATUS_FAILED;
    }
    if (block == 4) {
      if (this->write_mifare_classic_block_(block, empty_ndef_message) != STATUS_OK) {
        ESP_LOGE(TAG, "Unable to write block %d", block);
        status = STATUS_FAILED;
      }
    } else {
      if (this->write_mifare_classic_block_(block, blank_block) != STATUS_OK) {
        ESP_LOGE(TAG, "Unable to write block %d", block);
        status = STATUS_FAILED;
      }
    }
    if (this->write_mifare_classic_block_(block + 1, blank_block) != STATUS_OK) {
      ESP_LOGE(TAG, "Unable to write block %d", block + 1);
      status = STATUS_FAILED;
    }
    if (this->write_mifare_classic_block_(block + 2, blank_block) != STATUS_OK) {
      ESP_LOGE(TAG, "Unable to write block %d", block + 2);
      status = STATUS_FAILED;
    }
    if (this->write_mifare_classic_block_(block + 3, ndef_trailer) != STATUS_OK) {
      ESP_LOGE(TAG, "Unable to write trailer block %d", block + 3);
      status = STATUS_FAILED;
    }
  }
  return status;
}

uint8_t PN7160::write_mifare_classic_block_(uint8_t block_num, std::vector<uint8_t> &write_data) {
  std::vector<uint8_t> data_out = {XCHG_DATA_OID, nfc::MIFARE_CMD_WRITE, block_num};
  std::vector<uint8_t> response;

  ESP_LOGVV(TAG, "Write XCHG_DATA_REQ 1: %s", nfc::format_bytes(data_out).c_str());
  if (this->write_data_and_read_(data_out, response, NFCC_DEFAULT_TIMEOUT, false) != STATUS_OK) {
    ESP_LOGE(TAG, "Sending XCHG_DATA_REQ failed");
    return STATUS_FAILED;
  }
  // write command part two
  data_out = {XCHG_DATA_OID};
  data_out.insert(data_out.end(), write_data.begin(), write_data.end());

  ESP_LOGVV(TAG, "Write XCHG_DATA_REQ 2: %s", nfc::format_bytes(data_out).c_str());
  if (this->write_data_and_read_(data_out, response, NFCC_MFC_TIMEOUT, false) != STATUS_OK) {
    ESP_LOGE(TAG, "MFC XCHG_DATA timed out waiting for XCHG_DATA_RSP during block write");
    return STATUS_FAILED;
  }

  if ((response[0] != MT_DATA) || (response[3] != XCHG_DATA_OID) || (response[4] != nfc::MIFARE_CMD_ACK)) {
    ESP_LOGE(TAG, "MFC write block failed - block 0x%02x", block_num);
    ESP_LOGW(TAG, "Write response: %s", nfc::format_bytes(response).c_str());
    return STATUS_FAILED;
  }

  return STATUS_OK;
}

uint8_t PN7160::write_mifare_classic_tag_(std::shared_ptr<nfc::NdefMessage> message) {
  auto encoded = message->encode();

  uint32_t message_length = encoded.size();
  uint32_t buffer_length = nfc::get_mifare_classic_buffer_size(message_length);

  encoded.insert(encoded.begin(), 0x03);
  if (message_length < 255) {
    encoded.insert(encoded.begin() + 1, message_length);
  } else {
    encoded.insert(encoded.begin() + 1, 0xFF);
    encoded.insert(encoded.begin() + 2, (message_length >> 8) & 0xFF);
    encoded.insert(encoded.begin() + 3, message_length & 0xFF);
  }
  encoded.push_back(0xFE);

  encoded.resize(buffer_length, 0);

  uint32_t index = 0;
  uint8_t current_block = 4;

  while (index < buffer_length) {
    if (nfc::mifare_classic_is_first_block(current_block)) {
      if (this->auth_mifare_classic_block_(current_block, nfc::MIFARE_CMD_AUTH_A, nfc::NDEF_KEY) != STATUS_OK) {
        return STATUS_FAILED;
      }
    }

    std::vector<uint8_t> data(encoded.begin() + index, encoded.begin() + index + nfc::MIFARE_CLASSIC_BLOCK_SIZE);
    if (this->write_mifare_classic_block_(current_block, data) != STATUS_OK) {
      return STATUS_FAILED;
    }
    index += nfc::MIFARE_CLASSIC_BLOCK_SIZE;
    current_block++;

    if (nfc::mifare_classic_is_trailer_block(current_block)) {
      // Skipping as cannot write to trailer
      current_block++;
    }
  }
  return STATUS_OK;
}

uint8_t PN7160::halt_mifare_classic_tag_() {
  std::vector<uint8_t> data_out = {XCHG_DATA_OID, nfc::MIFARE_CMD_HALT, 0};
  std::vector<uint8_t> response;

  ESP_LOGVV(TAG, "Halt XCHG_DATA_REQ: %s", nfc::format_bytes(data_out).c_str());
  if (this->write_data_and_read_(data_out, response, NFCC_MFC_TIMEOUT, false) != STATUS_OK) {
    ESP_LOGE(TAG, "Sending halt XCHG_DATA_REQ failed");
    return STATUS_FAILED;
  }
  return STATUS_OK;
}

}  // namespace pn7160
}  // namespace esphome
