#include "support/replay_file.hpp"
#include "support/sha256.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using x30_plane_seg_core::test_support::ReplayFormatError;
using x30_plane_seg_core::test_support::Sha256Digest;

constexpr std::size_t kBodyHashOffset = 88U;
constexpr std::size_t kHeaderBytes = 128U;
constexpr std::size_t kSensorLookXOffset = kHeaderBytes + 244U;

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::string digestHex(const Sha256Digest & digest)
{
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    output << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return output.str();
}

std::vector<std::uint8_t> readBytes(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  require(static_cast<bool>(stream), "unable to open fixture for format test");
  const std::streamoff end = stream.tellg();
  require(end >= 0, "unable to determine fixture size");
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(end));
  stream.seekg(0, std::ios::beg);
  if (!bytes.empty()) {
    stream.read(
      reinterpret_cast<char *>(bytes.data()),
      static_cast<std::streamsize>(bytes.size()));
  }
  require(stream.good() || stream.eof(), "unable to read fixture bytes");
  require(static_cast<std::size_t>(stream.gcount()) == bytes.size(), "short fixture read");
  return bytes;
}

void writeBytes(
  const std::filesystem::path & path, const std::vector<std::uint8_t> & bytes)
{
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  require(static_cast<bool>(stream), "unable to create temporary replay fixture");
  stream.write(
    reinterpret_cast<const char *>(bytes.data()),
    static_cast<std::streamsize>(bytes.size()));
  require(static_cast<bool>(stream), "unable to write temporary replay fixture");
}

void refreshBodyHash(std::vector<std::uint8_t> & bytes)
{
  require(bytes.size() >= kHeaderBytes, "fixture is too short to refresh body hash");
  const Sha256Digest digest = x30_plane_seg_core::test_support::sha256(
    bytes.data() + kHeaderBytes, bytes.size() - kHeaderBytes);
  std::copy(digest.begin(), digest.end(), bytes.begin() + kBodyHashOffset);
}

void expectFormatFailure(
  const std::filesystem::path & temporary_path,
  const std::vector<std::uint8_t> & bytes,
  const std::string & expected_fragment)
{
  writeBytes(temporary_path, bytes);
  bool failed_as_expected = false;
  try {
    static_cast<void>(x30_plane_seg_core::test_support::loadReplayFile(
      temporary_path.string()));
  } catch (const ReplayFormatError & error) {
    failed_as_expected = std::string(error.what()).find(expected_fragment) !=
      std::string::npos;
  }
  std::error_code ignored;
  std::filesystem::remove(temporary_path, ignored);
  require(
    failed_as_expected,
    "corrupt replay did not fail with diagnostic containing: " + expected_fragment);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) {
      throw std::runtime_error("usage: x30_plane_seg_core_replay_format_test fixture.x30rpl");
    }

    const std::string empty;
    require(
      digestHex(x30_plane_seg_core::test_support::sha256(empty.data(), empty.size())) ==
      "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
      "SHA-256 empty-string vector failed");
    const std::string abc = "abc";
    require(
      digestHex(x30_plane_seg_core::test_support::sha256(abc.data(), abc.size())) ==
      "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
      "SHA-256 abc vector failed");

    const auto replay = x30_plane_seg_core::test_support::loadReplayFile(argv[1]);
    require(replay.frames.size() == 12U, "committed replay must contain 12 frames");

    const std::filesystem::path temporary_path =
      std::filesystem::temp_directory_path() / "x30_plane_seg_corrupt_replay.x30rpl";
    const std::vector<std::uint8_t> original = readBytes(argv[1]);

    std::vector<std::uint8_t> corrupt_body = original;
    corrupt_body.back() ^= 0x01U;
    expectFormatFailure(temporary_path, corrupt_body, "body_sha256");

    std::vector<std::uint8_t> truncated = original;
    truncated.pop_back();
    expectFormatFailure(temporary_path, truncated, "file_bytes");

    std::vector<std::uint8_t> invalid_sensor_pose = original;
    require(
      kSensorLookXOffset + sizeof(float) <= invalid_sensor_pose.size(),
      "sensor pose offset is outside fixture");
    std::fill_n(invalid_sensor_pose.begin() + kSensorLookXOffset, sizeof(float), 0U);
    refreshBodyHash(invalid_sensor_pose);
    expectFormatFailure(temporary_path, invalid_sensor_pose, "sensor pose");
  } catch (const std::exception & error) {
    std::cerr << "x30 replay format test failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
