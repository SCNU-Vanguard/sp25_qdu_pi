#include "qdu_shared_topic_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace io::qdu
{
namespace
{
uint32_t read_u32_le(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
}

void write_u32_le(std::vector<uint8_t> & data, std::size_t offset, uint32_t value)
{
  for (std::size_t i = 0; i < 4; ++i) data[offset + i] = static_cast<uint8_t>(value >> (i * 8U));
}

void append_float_le(std::vector<uint8_t> & output, float value)
{
  static_assert(sizeof(float) == 4, "QDU SharedTopic requires 32-bit float");
  static_assert(std::numeric_limits<float>::is_iec559, "QDU SharedTopic requires IEEE-754 float");
  uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  output.push_back(static_cast<uint8_t>(bits));
  output.push_back(static_cast<uint8_t>(bits >> 8U));
  output.push_back(static_cast<uint8_t>(bits >> 16U));
  output.push_back(static_cast<uint8_t>(bits >> 24U));
}

uint32_t read_u24_le(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U);
}
}  // namespace

uint8_t crc8(const uint8_t * data, std::size_t size)
{
  uint8_t result = 0xFF;
  for (std::size_t i = 0; i < size; ++i) {
    result ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      result = (result & 0x01U) != 0U ? static_cast<uint8_t>((result >> 1U) ^ 0x8CU)
                                     : static_cast<uint8_t>(result >> 1U);
    }
  }
  return result;
}

uint32_t topic_crc32(const uint8_t * data, std::size_t size)
{
  uint32_t result = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    result ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      result = (result & 0x01U) != 0U ? (result >> 1U) ^ 0xEDB88320U : result >> 1U;
    }
  }
  return result;
}

uint32_t topic_crc32(const std::string & topic_name)
{
  return topic_crc32(reinterpret_cast<const uint8_t *>(topic_name.data()), topic_name.size());
}

std::vector<uint8_t> target_euler_payload(
  float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc)
{
  std::vector<uint8_t> result;
  result.reserve(sizeof(float) * 9U);
  append_float_le(result, pitch);
  append_float_le(result, pitch);
  append_float_le(result, yaw);
  append_float_le(result, pitch_vel);
  append_float_le(result, pitch_vel);
  append_float_le(result, yaw_vel);
  append_float_le(result, pitch_acc);
  append_float_le(result, pitch_acc);
  append_float_le(result, yaw_acc);
  return result;
}

std::vector<uint8_t> pack(
  uint32_t topic_key, uint64_t timestamp_us, const uint8_t * payload, std::size_t payload_size)
{
  if (payload_size > 0xFFFFFFU) throw std::invalid_argument("QDU payload exceeds 24-bit length");
  if ((timestamp_us >> 48U) != 0U) throw std::invalid_argument("QDU timestamp exceeds 48 bits");
  if (payload_size != 0U && payload == nullptr) throw std::invalid_argument("QDU payload is null");

  std::vector<uint8_t> result(kPacketOverhead + payload_size, 0);
  result[0] = kPacketPrefix;
  result[1] = static_cast<uint8_t>(payload_size);
  result[2] = static_cast<uint8_t>(payload_size >> 8U);
  result[3] = static_cast<uint8_t>(payload_size >> 16U);
  write_u32_le(result, 4, topic_key);
  for (std::size_t i = 0; i < 6; ++i) result[8 + i] = static_cast<uint8_t>(timestamp_us >> (i * 8U));
  result[14] = kPacketVersion;
  result[15] = crc8(result.data(), 15);
  if (payload_size != 0U) std::copy(payload, payload + payload_size, result.begin() + kHeaderSize);
  result.back() = crc8(result.data(), kHeaderSize + payload_size);
  return result;
}

Parser::Parser(std::size_t max_payload_size) : max_payload_size_(max_payload_size)
{
  if (max_payload_size_ == 0U || max_payload_size_ > 0xFFFFFFU)
    throw std::invalid_argument("invalid QDU parser payload limit");
  buffer_.reserve(max_payload_size_ + kPacketOverhead);
}

void Parser::feed(const uint8_t * data, std::size_t size, const PacketCallback & callback)
{
  if (data == nullptr && size != 0U) throw std::invalid_argument("QDU parser input is null");
  if (size == 0U) return;
  buffer_.insert(buffer_.end(), data, data + size);

  while (true) {
    const auto prefix = std::find(buffer_.begin(), buffer_.end(), kPacketPrefix);
    if (prefix != buffer_.begin()) {
      const auto discarded = static_cast<uint64_t>(std::distance(buffer_.begin(), prefix));
      stats_.discarded_bytes += discarded;
      buffer_.erase(buffer_.begin(), prefix);
    }
    if (buffer_.size() < kHeaderSize) return;

    if (crc8(buffer_.data(), 15) != buffer_[15]) {
      ++stats_.header_crc_errors;
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }
    if (buffer_[14] != kPacketVersion) {
      ++stats_.version_errors;
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t payload_size = read_u24_le(buffer_.data() + 1);
    if (payload_size > max_payload_size_) {
      ++stats_.oversize_errors;
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t packet_size = kPacketOverhead + payload_size;
    if (buffer_.size() < packet_size) return;
    if (crc8(buffer_.data(), packet_size - 1U) != buffer_[packet_size - 1U]) {
      ++stats_.payload_crc_errors;
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }

    Packet packet;
    packet.topic_crc32 = read_u32_le(buffer_.data() + 4);
    for (std::size_t i = 0; i < 6; ++i)
      packet.timestamp_us |= static_cast<uint64_t>(buffer_[8 + i]) << (i * 8U);
    packet.payload.assign(buffer_.begin() + kHeaderSize, buffer_.begin() + kHeaderSize + payload_size);
    callback(packet);
    ++stats_.accepted_packets;
    buffer_.erase(buffer_.begin(), buffer_.begin() + packet_size);
  }
}

void Parser::reset()
{
  buffer_.clear();
}

const ParserStats & Parser::stats() const
{
  return stats_;
}

}  // namespace io::qdu
