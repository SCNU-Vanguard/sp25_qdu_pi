#include "io/qdu_shared_topic_protocol.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

void append(std::vector<uint8_t> & destination, const std::vector<uint8_t> & source)
{
  destination.insert(destination.end(), source.begin(), source.end());
}
}  // namespace

int main()
{
  using io::qdu::Packet;
  using io::qdu::Parser;

  try {
    // [9.21-QDU] Fixed values were generated independently from the supplied libxr polynomial.
    require(io::qdu::topic_crc32("target_euler") == 0x6F0986CBU, "target_euler CRC32");
    require(io::qdu::topic_crc32("fire_notify") == 0x0291AD6BU, "fire_notify CRC32");
    require(io::qdu::topic_crc32("ahrs_quaternion") == 0xA6351AEAU, "ahrs CRC32");
    require(io::qdu::topic_crc32("gimbal_quat") == 0xBD0471CCU, "gimbal CRC32");

    const std::vector<uint8_t> expected_target_payload{
      0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x80, 0x3F,
      0x00, 0x00, 0xA0, 0x40, 0x00, 0x00, 0xA0, 0x40, 0x00, 0x00, 0x00, 0x40,
      0x00, 0x00, 0xC0, 0x40, 0x00, 0x00, 0xC0, 0x40, 0x00, 0x00, 0x40, 0x40};
    require(
      io::qdu::target_euler_payload(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F) ==
        expected_target_payload,
      "target_euler field order");

    const std::vector<uint8_t> payload{0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0xC0};
    const std::vector<uint8_t> expected{
      0x5A, 0x08, 0x00, 0x00, 0xCB, 0x86, 0x09, 0x6F, 0x06, 0x05, 0x04, 0x03,
      0x02, 0x01, 0x01, 0x15, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0xC0,
      0xCB};
    const auto encoded = io::qdu::pack(
      "target_euler", 0x010203040506ULL, payload.data(), payload.size());
    require(encoded == expected, "byte-exact QDU packet fixture");

    Parser parser(64);
    std::vector<Packet> packets;
    auto collect = [&](const Packet & packet) { packets.push_back(packet); };

    // [9.21-QDU] Every possible two-part split must reassemble into one packet.
    for (std::size_t split = 1; split < encoded.size(); ++split) {
      parser.reset();
      packets.clear();
      parser.feed(encoded.data(), split, collect);
      require(packets.empty(), "fragment emitted early");
      parser.feed(encoded.data() + split, encoded.size() - split, collect);
      require(packets.size() == 1, "fragmented packet not emitted once");
      require(packets[0].topic_crc32 == 0x6F0986CBU, "parsed topic key");
      require(packets[0].timestamp_us == 0x010203040506ULL, "parsed timestamp");
      require(packets[0].payload == payload, "parsed payload");
    }

    // [9.21-QDU] Noise, a damaged frame, and concatenated valid frames must resynchronize.
    auto corrupted = encoded;
    corrupted[18] ^= 0x40U;
    std::vector<uint8_t> stream{0x00, 0x11, 0x22, 0x5B};
    append(stream, corrupted);
    append(stream, encoded);
    append(stream, encoded);
    parser.reset();
    packets.clear();
    parser.feed(stream.data(), stream.size(), collect);
    require(packets.size() == 2, "parser failed to recover after damaged payload");
    require(parser.stats().payload_crc_errors >= 1, "payload CRC error not counted");
    require(parser.stats().discarded_bytes >= 4, "noise bytes not counted");

    // [9.21-QDU] A valid-looking oversized header must be rejected without allocating its length.
    std::vector<uint8_t> oversized(16, 0);
    oversized[0] = 0x5A;
    oversized[1] = 65;
    oversized[14] = 1;
    oversized[15] = io::qdu::crc8(oversized.data(), 15);
    append(oversized, encoded);
    parser.reset();
    packets.clear();
    parser.feed(oversized.data(), oversized.size(), collect);
    require(packets.size() == 1, "parser failed to recover after oversized header");
    require(parser.stats().oversize_errors >= 1, "oversize error not counted");

    std::cout << "QDU SharedTopic protocol tests passed\n";
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "QDU SharedTopic protocol test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
