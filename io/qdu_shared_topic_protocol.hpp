#ifndef IO__QDU_SHARED_TOPIC_PROTOCOL_HPP
#define IO__QDU_SHARED_TOPIC_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace io::qdu
{

// [9.21-QDU] These constants are copied from the QDU libxr Topic wire contract.
constexpr uint8_t kPacketPrefix = 0x5A;
constexpr uint8_t kPacketVersion = 0x01;
constexpr std::size_t kHeaderSize = 16;
constexpr std::size_t kPacketOverhead = 17;
constexpr std::size_t kDefaultMaxPayloadSize = 4096;

struct Packet
{
  uint32_t topic_crc32{};
  uint64_t timestamp_us{};
  std::vector<uint8_t> payload;
};

struct ParserStats
{
  uint64_t accepted_packets{};
  uint64_t discarded_bytes{};
  uint64_t header_crc_errors{};
  uint64_t payload_crc_errors{};
  uint64_t version_errors{};
  uint64_t oversize_errors{};
};

// [9.21-QDU] libxr uses CRC-8/MAXIM with init 0xFF and no final xor.
uint8_t crc8(const uint8_t * data, std::size_t size);

// [9.21-QDU] Topic keys are CRC-32/ISO-HDLC state with init 0xFFFFFFFF and no final xor.
uint32_t topic_crc32(const uint8_t * data, std::size_t size);
uint32_t topic_crc32(const std::string & topic_name);

// [9.21-QDU] Map SP25 yaw/pitch planning values to QDU AimerHostGimbalTarget's 9 floats.
std::vector<uint8_t> target_euler_payload(
  float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc);

// [9.21-QDU] Build the exact libxr frame: 16-byte header, payload, trailing CRC8.
std::vector<uint8_t> pack(
  uint32_t topic_key, uint64_t timestamp_us, const uint8_t * payload, std::size_t payload_size);

inline std::vector<uint8_t> pack(
  const std::string & topic_name, uint64_t timestamp_us, const uint8_t * payload,
  std::size_t payload_size)
{
  return pack(topic_crc32(topic_name), timestamp_us, payload, payload_size);
}

class Parser
{
public:
  using PacketCallback = std::function<void(const Packet &)>;

  explicit Parser(std::size_t max_payload_size = kDefaultMaxPayloadSize);

  void feed(const uint8_t * data, std::size_t size, const PacketCallback & callback);
  void reset();
  const ParserStats & stats() const;

private:
  std::size_t max_payload_size_;
  std::vector<uint8_t> buffer_;
  ParserStats stats_;
};

}  // namespace io::qdu

#endif  // IO__QDU_SHARED_TOPIC_PROTOCOL_HPP
