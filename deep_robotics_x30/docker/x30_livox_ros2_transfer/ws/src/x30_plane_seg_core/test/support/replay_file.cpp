#include "support/replay_file.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace x30_plane_seg_core
{
namespace test_support
{
namespace
{

constexpr std::uint64_t kHeaderBytes = 128U;
constexpr std::uint64_t kFrameRecordBytes = 384U;
constexpr std::uint32_t kHeaderFlags = 3U;
constexpr std::uint32_t kKnownFrameFlags =
  kReplayHasExactTf | kReplayExpectMissingTf | kReplayHasFactoryOracle |
  kReplayHasCoreOracle | kReplayRunCore;
constexpr std::array<std::uint8_t, 8> kMagic{{
  'X', '3', '0', 'R', 'P', 'L', 'Y', 0U,
}};

static_assert(sizeof(float) == 4U, "replay schema requires 32-bit float");
static_assert(sizeof(double) == 8U, "replay schema requires 64-bit double");
static_assert(
  std::numeric_limits<float>::is_iec559,
  "replay schema requires IEC 60559 float");
static_assert(
  std::numeric_limits<double>::is_iec559,
  "replay schema requires IEC 60559 double");

[[noreturn]] void fail(const std::string & message)
{
  throw ReplayFormatError(message);
}

std::string framePrefix(const std::size_t index, const std::string & case_name)
{
  std::ostringstream stream;
  stream << "frame " << index;
  if (!case_name.empty()) {
    stream << " (" << case_name << ')';
  }
  stream << ": ";
  return stream.str();
}

std::uint64_t checkedAdd(
  const std::uint64_t left, const std::uint64_t right, const char * const field)
{
  if (right > std::numeric_limits<std::uint64_t>::max() - left) {
    fail(std::string(field) + " overflows uint64");
  }
  return left + right;
}

std::uint64_t checkedMultiply(
  const std::uint64_t left, const std::uint64_t right, const char * const field)
{
  if (left != 0U && right > std::numeric_limits<std::uint64_t>::max() / left) {
    fail(std::string(field) + " overflows uint64");
  }
  return left * right;
}

class Cursor
{
public:
  Cursor(
    const std::vector<std::uint8_t> & bytes, const std::uint64_t begin,
    const std::uint64_t length, std::string context)
  : bytes_(bytes), position_(begin), end_(checkedAdd(begin, length, "cursor range")),
    context_(std::move(context))
  {
    if (end_ > bytes_.size()) {
      fail(context_ + "range is outside the file");
    }
  }

  std::uint8_t readU8(const char * const field)
  {
    require(1U, field);
    return bytes_[static_cast<std::size_t>(position_++)];
  }

  std::uint16_t readU16(const char * const field)
  {
    require(2U, field);
    const std::uint16_t value =
      static_cast<std::uint16_t>(bytes_[at(0U)]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes_[at(1U)]) << 8U);
    position_ += 2U;
    return value;
  }

  std::uint32_t readU32(const char * const field)
  {
    require(4U, field);
    std::uint32_t value = 0U;
    for (std::size_t index = 0; index < 4U; ++index) {
      value |= static_cast<std::uint32_t>(bytes_[at(index)]) << (index * 8U);
    }
    position_ += 4U;
    return value;
  }

  std::uint64_t readU64(const char * const field)
  {
    require(8U, field);
    std::uint64_t value = 0U;
    for (std::size_t index = 0; index < 8U; ++index) {
      value |= static_cast<std::uint64_t>(bytes_[at(index)]) << (index * 8U);
    }
    position_ += 8U;
    return value;
  }

  float readF32(const char * const field)
  {
    const std::uint32_t bits = readU32(field);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  double readF64(const char * const field)
  {
    const std::uint64_t bits = readU64(field);
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  Sha256Digest readDigest(const char * const field)
  {
    require(Sha256Digest{}.size(), field);
    Sha256Digest value{};
    std::memcpy(value.data(), bytes_.data() + at(0U), value.size());
    position_ += value.size();
    return value;
  }

  std::array<std::uint8_t, 8> readMagic()
  {
    require(kMagic.size(), "magic");
    std::array<std::uint8_t, 8> value{};
    std::memcpy(value.data(), bytes_.data() + at(0U), value.size());
    position_ += value.size();
    return value;
  }

  std::string readCaseName()
  {
    constexpr std::size_t kCaseNameBytes = 64U;
    require(kCaseNameBytes, "case_name");
    const auto * const begin = bytes_.data() + at(0U);
    const auto * const terminator = std::find(begin, begin + kCaseNameBytes, 0U);
    if (terminator == begin + kCaseNameBytes) {
      fail(context_ + "case_name is not NUL terminated");
    }
    if (terminator == begin) {
      fail(context_ + "case_name is empty");
    }
    if (!std::all_of(terminator, begin + kCaseNameBytes, [](const std::uint8_t value) {
        return value == 0U;
      }))
    {
      fail(context_ + "case_name has non-zero bytes after its terminator");
    }
    const std::string value(
      reinterpret_cast<const char *>(begin),
      static_cast<std::size_t>(terminator - begin));
    position_ += kCaseNameBytes;
    return value;
  }

  ReplayBlobRef readBlobRef(const char * const field)
  {
    ReplayBlobRef reference;
    reference.offset = readU64(field);
    reference.bytes = readU64(field);
    return reference;
  }

  void requireEnd() const
  {
    if (position_ != end_) {
      fail(context_ + "record parser did not consume the declared byte count");
    }
  }

private:
  void require(const std::uint64_t count, const char * const field) const
  {
    if (position_ > end_ || count > end_ - position_) {
      fail(context_ + field + " is truncated");
    }
  }

  std::size_t at(const std::size_t relative) const
  {
    return static_cast<std::size_t>(position_) + relative;
  }

  const std::vector<std::uint8_t> & bytes_;
  std::uint64_t position_;
  std::uint64_t end_;
  std::string context_;
};

bool allFinite(const std::array<double, 3> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
      return std::isfinite(value);
    });
}

bool allFinite(const std::array<double, 4> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
      return std::isfinite(value);
    });
}

bool allFinite(const std::array<float, 3> & values)
{
  return std::all_of(values.begin(), values.end(), [](const float value) {
      return std::isfinite(value);
    });
}

template<typename Scalar, std::size_t Size>
bool isNormalizable(const std::array<Scalar, Size> & values)
{
  Scalar scale = Scalar{0};
  for (const Scalar value : values) {
    scale = std::max(scale, std::abs(value));
  }
  if (!std::isfinite(scale) || scale == Scalar{0}) {
    return false;
  }

  Scalar scaled_norm_squared = Scalar{0};
  for (const Scalar value : values) {
    const Scalar scaled = value / scale;
    scaled_norm_squared += scaled * scaled;
  }
  return std::isfinite(scaled_norm_squared) && scaled_norm_squared > Scalar{0};
}

bool geometryLengthMatches(
  const double length, const std::uint32_t size, const double resolution)
{
  const double expected = static_cast<double>(size) * resolution;
  const double scale = std::max({1.0, std::abs(length), std::abs(expected)});
  return std::isfinite(expected) && std::abs(length - expected) <= 1.0e-9 * scale;
}

void validateAbsentBlob(
  const ReplayBlobRef & reference, const std::string & prefix, const char * const name)
{
  if (reference.offset != 0U || reference.bytes != 0U) {
    fail(prefix + name + " must use an all-zero BlobRef when absent");
  }
}

struct BlobRange
{
  std::uint64_t begin;
  std::uint64_t end;
  std::string name;
};

void validateBlob(
  const ReplayBlobRef & reference, const std::uint64_t expected_bytes,
  const ReplayHeader & header, const std::string & name,
  std::vector<BlobRange> & ranges)
{
  if (reference.bytes != expected_bytes) {
    std::ostringstream stream;
    stream << name << " byte count is " << reference.bytes << ", expected " << expected_bytes;
    fail(stream.str());
  }
  if (expected_bytes == 0U) {
    if (reference.offset != 0U) {
      fail(name + " has a non-zero offset for an empty blob");
    }
    return;
  }
  if (reference.offset < header.payload_offset) {
    fail(name + " starts before payload_offset");
  }
  const std::uint64_t end = checkedAdd(reference.offset, reference.bytes, name.c_str());
  if (end > header.file_bytes) {
    fail(name + " extends past file_bytes");
  }
  ranges.push_back(BlobRange{reference.offset, end, name});
}

bool hostIsLittleEndian() noexcept
{
  const std::uint16_t value = 1U;
  std::uint8_t first_byte = 0U;
  std::memcpy(&first_byte, &value, sizeof(first_byte));
  return first_byte == 1U;
}

std::vector<float> decodeFloatBlob(
  const std::vector<std::uint8_t> & file_bytes, const ReplayBlobRef & reference,
  const std::size_t value_count)
{
  std::vector<float> values(value_count);
  if (value_count == 0U) {
    return values;
  }

  const auto * const source = file_bytes.data() + static_cast<std::size_t>(reference.offset);
  if (hostIsLittleEndian()) {
    std::memcpy(values.data(), source, static_cast<std::size_t>(reference.bytes));
    return values;
  }

  for (std::size_t index = 0; index < value_count; ++index) {
    std::uint32_t bits = 0U;
    for (std::size_t byte = 0; byte < sizeof(float); ++byte) {
      bits |= static_cast<std::uint32_t>(source[index * sizeof(float) + byte]) << (byte * 8U);
    }
    std::memcpy(&values[index], &bits, sizeof(values[index]));
  }
  return values;
}

void validateFrameScalars(const ReplayFrame & frame, const std::size_t frame_index)
{
  const std::string prefix = framePrefix(frame_index, frame.case_name);
  if ((frame.flags & ~kKnownFrameFlags) != 0U) {
    fail(prefix + "unknown frame flag bits are set");
  }
  if (!frame.hasFlag(kReplayRunCore)) {
    fail(prefix + "RUN_CORE is required");
  }
  if (frame.hasFlag(kReplayHasCoreOracle)) {
    fail(prefix + "HAS_CORE_ORACLE is unsupported in schema v1 fixtures");
  }
  if (frame.expected_core_count != std::numeric_limits<std::uint32_t>::max()) {
    fail(prefix + "expected_core_count must be UINT32_MAX without a core oracle");
  }
  validateAbsentBlob(frame.core_oracle_blob, prefix, "core oracle");

  const bool has_exact_tf = frame.hasFlag(kReplayHasExactTf);
  const bool expects_missing_tf = frame.hasFlag(kReplayExpectMissingTf);
  if (has_exact_tf == expects_missing_tf) {
    fail(prefix + "exactly one of HAS_EXACT_TF and EXPECT_MISSING_TF must be set");
  }

  if (frame.size_x == 0U || frame.size_y == 0U) {
    fail(prefix + "grid dimensions must be non-zero");
  }
  if (frame.outer_start_index >= frame.size_x || frame.inner_start_index >= frame.size_y) {
    fail(prefix + "grid circular-buffer start index is outside its dimension");
  }
  if (!std::isfinite(frame.resolution) || frame.resolution <= 0.0 ||
    !std::isfinite(frame.length_x) || frame.length_x <= 0.0 ||
    !std::isfinite(frame.length_y) || frame.length_y <= 0.0)
  {
    fail(prefix + "grid resolution and lengths must be finite and positive");
  }
  if (!geometryLengthMatches(frame.length_x, frame.size_x, frame.resolution) ||
    !geometryLengthMatches(frame.length_y, frame.size_y, frame.resolution))
  {
    fail(prefix + "grid lengths do not match dimensions times resolution");
  }
  if (!allFinite(frame.center) || !allFinite(frame.orientation_xyzw) ||
    !isNormalizable(frame.orientation_xyzw))
  {
    fail(prefix + "grid pose must be finite with a normalizable quaternion");
  }
  if (!allFinite(frame.world_to_base_translation) ||
    !allFinite(frame.world_to_base_rotation_xyzw) ||
    !isNormalizable(frame.world_to_base_rotation_xyzw))
  {
    fail(prefix + "world-to-base pose must be finite with a normalizable quaternion");
  }
  if (expects_missing_tf &&
    (frame.world_to_base_translation != std::array<double, 3>{{0.0, 0.0, 0.0}} ||
    frame.world_to_base_rotation_xyzw != std::array<double, 4>{{0.0, 0.0, 0.0, 1.0}}))
  {
    fail(prefix + "missing TF must preserve zero translation and identity rotation");
  }
  if (!allFinite(frame.sensor_origin) || !allFinite(frame.sensor_look_direction) ||
    !isNormalizable(frame.sensor_look_direction))
  {
    fail(prefix + "sensor pose must be finite with a non-zero look direction");
  }
  if (frame.sensor_origin != std::array<float, 3>{{0.0F, 0.0F, 0.0F}} ||
    frame.sensor_look_direction != std::array<float, 3>{{1.0F, 0.0F, 0.0F}})
  {
    fail(prefix + "schema-v1 sensor pose must be origin=(0,0,0), look=(1,0,0)");
  }

  if (frame.accessibility_threshold != 0.9F ||
    frame.downsample_resolution_m != 0.01F ||
    frame.max_angle_from_horizontal_deg != 45.0F)
  {
    fail(prefix + "core configuration does not match the schema-v1 factory constants");
  }
  if (frame.factory_mode_9 || frame.debug) {
    fail(prefix + "schema-v1 replay requires factory_mode_9=0 and debug=0");
  }

  const std::uint64_t cell_count = checkedMultiply(
    frame.size_x, frame.size_y, "grid cell count");
  if (frame.expected_retained_count > cell_count) {
    fail(prefix + "expected retained count exceeds the input cell count");
  }
  if (!frame.hasFlag(kReplayHasFactoryOracle)) {
    if (frame.factory_point_count != 0U) {
      fail(prefix + "factory points are present without HAS_FACTORY_ORACLE");
    }
    validateAbsentBlob(frame.factory_xyz_blob, prefix, "factory XYZ");
  }
  if (frame.factory_point_count % 4U != 0U) {
    fail(prefix + "factory point count is not divisible into four-point groups");
  }
}

std::vector<std::uint8_t> readFile(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    fail("unable to open replay fixture: " + path);
  }
  const std::streamoff end = stream.tellg();
  if (end < 0) {
    fail("unable to determine replay fixture size: " + path);
  }
  const auto file_size = static_cast<std::uint64_t>(end);
  if (file_size > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    fail("replay fixture is too large for this process: " + path);
  }

  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(file_size));
  stream.seekg(0, std::ios::beg);
  if (!bytes.empty()) {
    stream.read(
      reinterpret_cast<char *>(bytes.data()),
      static_cast<std::streamsize>(bytes.size()));
  }
  if (!stream || static_cast<std::size_t>(stream.gcount()) != bytes.size()) {
    fail("unable to read the complete replay fixture: " + path);
  }
  return bytes;
}

ReplayHeader parseHeader(const std::vector<std::uint8_t> & bytes)
{
  if (bytes.size() < kHeaderBytes) {
    fail("replay fixture is shorter than the 128-byte header");
  }
  Cursor cursor(bytes, 0U, kHeaderBytes, "header: ");
  if (cursor.readMagic() != kMagic) {
    fail("header: magic is not X30RPLY\\0");
  }

  ReplayHeader header;
  header.major = cursor.readU16("major");
  header.minor = cursor.readU16("minor");
  const std::uint32_t header_bytes = cursor.readU32("header_bytes");
  header.flags = cursor.readU32("flags");
  header.frame_count = cursor.readU32("frame_count");
  const std::uint32_t frame_record_bytes = cursor.readU32("frame_record_bytes");
  const std::uint32_t reserved = cursor.readU32("reserved");
  const std::uint64_t frame_table_offset = cursor.readU64("frame_table_offset");
  header.payload_offset = cursor.readU64("payload_offset");
  header.file_bytes = cursor.readU64("file_bytes");
  header.source_manifest_sha256 = cursor.readDigest("source_manifest_sha256");
  header.body_sha256 = cursor.readDigest("body_sha256");
  header.absolute_tolerance = cursor.readF32("absolute_tolerance");
  header.relative_tolerance = cursor.readF32("relative_tolerance");
  cursor.requireEnd();

  if (header.major != 1U || header.minor != 0U) {
    fail("header: only replay schema 1.0 is supported");
  }
  if (header_bytes != kHeaderBytes || frame_record_bytes != kFrameRecordBytes) {
    fail("header: fixed header or frame record byte count is invalid");
  }
  if (header.flags != kHeaderFlags) {
    fail("header: flags must declare little-endian IEC 60559 payloads");
  }
  if (reserved != 0U) {
    fail("header: reserved field is non-zero");
  }
  if (header.frame_count == 0U) {
    fail("header: frame_count must be non-zero");
  }
  if (frame_table_offset != kHeaderBytes) {
    fail("header: frame_table_offset must be 128");
  }
  if (header.file_bytes != bytes.size()) {
    fail("header: file_bytes does not match the physical file size");
  }
  const std::uint64_t table_bytes = checkedMultiply(
    header.frame_count, kFrameRecordBytes, "frame table byte count");
  const std::uint64_t table_end = checkedAdd(
    frame_table_offset, table_bytes, "frame table end");
  if (table_end > header.file_bytes || header.payload_offset < table_end ||
    header.payload_offset > header.file_bytes)
  {
    fail("header: frame table or payload offset is outside its allowed range");
  }
  if (header.absolute_tolerance != 1.0e-5F || header.relative_tolerance != 1.0e-5F) {
    fail("header: schema-v1 tolerances must both equal 1e-5");
  }

  const Sha256Digest actual_body_hash = sha256(
    bytes.data() + static_cast<std::size_t>(kHeaderBytes),
    bytes.size() - static_cast<std::size_t>(kHeaderBytes));
  if (actual_body_hash != header.body_sha256) {
    fail("header: SHA256 of bytes [128,file_bytes) does not match body_sha256");
  }
  return header;
}

ReplayFrame parseFrame(
  const std::vector<std::uint8_t> & bytes, const std::size_t frame_index)
{
  const std::uint64_t record_offset = checkedAdd(
    kHeaderBytes, checkedMultiply(frame_index, kFrameRecordBytes, "frame offset"),
    "frame offset");
  Cursor cursor(
    bytes, record_offset, kFrameRecordBytes,
    "frame " + std::to_string(frame_index) + ": ");

  ReplayFrame frame;
  frame.case_name = cursor.readCaseName();
  frame.stamp_ns = cursor.readU64("stamp_ns");
  frame.selected_index = cursor.readU32("selected_index");
  frame.flags = cursor.readU32("flags");
  frame.size_x = cursor.readU32("size_x");
  frame.size_y = cursor.readU32("size_y");
  frame.outer_start_index = cursor.readU32("outer_start_index");
  frame.inner_start_index = cursor.readU32("inner_start_index");
  frame.resolution = cursor.readF64("resolution");
  frame.length_x = cursor.readF64("length_x");
  frame.length_y = cursor.readF64("length_y");
  for (double & value : frame.center) {
    value = cursor.readF64("center");
  }
  for (double & value : frame.orientation_xyzw) {
    value = cursor.readF64("orientation_xyzw");
  }
  for (double & value : frame.world_to_base_translation) {
    value = cursor.readF64("world_to_base_translation");
  }
  for (double & value : frame.world_to_base_rotation_xyzw) {
    value = cursor.readF64("world_to_base_rotation_xyzw");
  }
  for (float & value : frame.sensor_origin) {
    value = cursor.readF32("sensor_origin");
  }
  for (float & value : frame.sensor_look_direction) {
    value = cursor.readF32("sensor_look_direction");
  }
  frame.accessibility_threshold = cursor.readF32("accessibility_threshold");
  frame.downsample_resolution_m = cursor.readF32("downsample_resolution_m");
  frame.max_angle_from_horizontal_deg = cursor.readF32("max_angle_from_horizontal_deg");
  const std::uint8_t factory_mode_9 = cursor.readU8("factory_mode_9");
  const std::uint8_t debug = cursor.readU8("debug");
  const std::uint16_t reserved_16 = cursor.readU16("reserved_16");
  frame.expected_retained_count = cursor.readU32("expected_retained_count");
  frame.expected_core_count = cursor.readU32("expected_core_count");
  frame.factory_point_count = cursor.readU32("factory_point_count");
  const std::uint32_t reserved_32 = cursor.readU32("reserved_32");
  frame.elevation_blob = cursor.readBlobRef("elevation_blob");
  frame.accessibility_blob = cursor.readBlobRef("accessibility_blob");
  frame.factory_xyz_blob = cursor.readBlobRef("factory_xyz_blob");
  frame.core_oracle_blob = cursor.readBlobRef("core_oracle_blob");
  frame.frame_input_sha256 = cursor.readDigest("frame_input_sha256");
  cursor.requireEnd();

  const std::string prefix = framePrefix(frame_index, frame.case_name);
  if (factory_mode_9 > 1U || debug > 1U) {
    fail(prefix + "boolean fields must be encoded as 0 or 1");
  }
  frame.factory_mode_9 = factory_mode_9 != 0U;
  frame.debug = debug != 0U;
  if (reserved_16 != 0U || reserved_32 != 0U) {
    fail(prefix + "reserved frame fields must be zero");
  }
  validateFrameScalars(frame, frame_index);
  return frame;
}

}  // namespace

ReplayFile loadReplayFile(const std::string & path)
{
  const std::vector<std::uint8_t> bytes = readFile(path);
  ReplayFile replay;
  replay.header = parseHeader(bytes);
  replay.frames.reserve(replay.header.frame_count);

  std::vector<BlobRange> ranges;
  ranges.reserve(static_cast<std::size_t>(replay.header.frame_count) * 3U);
  for (std::size_t index = 0; index < replay.header.frame_count; ++index) {
    ReplayFrame frame = parseFrame(bytes, index);
    const std::string prefix = framePrefix(index, frame.case_name);
    const std::uint64_t cell_count = checkedMultiply(
      frame.size_x, frame.size_y, "grid cell count");
    const std::uint64_t layer_bytes = checkedMultiply(
      cell_count, sizeof(float), "grid layer byte count");
    const std::uint64_t factory_value_count = checkedMultiply(
      frame.factory_point_count, 3U, "factory XYZ value count");
    const std::uint64_t factory_bytes = checkedMultiply(
      factory_value_count, sizeof(float), "factory XYZ byte count");

    validateBlob(
      frame.elevation_blob, layer_bytes, replay.header,
      prefix + "elevation", ranges);
    validateBlob(
      frame.accessibility_blob, layer_bytes, replay.header,
      prefix + "accessibility", ranges);
    validateBlob(
      frame.factory_xyz_blob, factory_bytes, replay.header,
      prefix + "factory XYZ", ranges);

    replay.frames.push_back(std::move(frame));
  }

  std::sort(ranges.begin(), ranges.end(), [](const BlobRange & left, const BlobRange & right) {
      if (left.begin != right.begin) {
        return left.begin < right.begin;
      }
      return left.end < right.end;
    });
  for (std::size_t index = 1; index < ranges.size(); ++index) {
    if (ranges[index].begin < ranges[index - 1U].end) {
      fail(
        ranges[index - 1U].name + " overlaps " + ranges[index].name);
    }
  }
  std::uint64_t covered_until = replay.header.payload_offset;
  for (const BlobRange & range : ranges) {
    if (range.begin != covered_until) {
      fail(range.name + " does not immediately follow the preceding payload blob");
    }
    covered_until = range.end;
  }
  if (covered_until != replay.header.file_bytes) {
    fail("payload blobs do not cover the complete declared payload");
  }

  for (std::size_t index = 0; index < replay.frames.size(); ++index) {
    ReplayFrame & frame = replay.frames[index];
    Sha256 input_hasher;
    input_hasher.update(
      bytes.data() + static_cast<std::size_t>(frame.elevation_blob.offset),
      static_cast<std::size_t>(frame.elevation_blob.bytes));
    input_hasher.update(
      bytes.data() + static_cast<std::size_t>(frame.accessibility_blob.offset),
      static_cast<std::size_t>(frame.accessibility_blob.bytes));
    if (frame.factory_xyz_blob.bytes != 0U) {
      input_hasher.update(
        bytes.data() + static_cast<std::size_t>(frame.factory_xyz_blob.offset),
        static_cast<std::size_t>(frame.factory_xyz_blob.bytes));
    }
    if (input_hasher.finish() != frame.frame_input_sha256) {
      fail(framePrefix(index, frame.case_name) + "frame_input_sha256 does not match its blobs");
    }

    const std::uint64_t cell_count = checkedMultiply(
      frame.size_x, frame.size_y, "grid cell count");
    const std::uint64_t factory_value_count = checkedMultiply(
      frame.factory_point_count, 3U, "factory XYZ value count");
    if (cell_count > std::numeric_limits<std::size_t>::max() ||
      factory_value_count > std::numeric_limits<std::size_t>::max())
    {
      fail(framePrefix(index, frame.case_name) + "decoded vector size exceeds size_t");
    }
    frame.elevation = decodeFloatBlob(
      bytes, frame.elevation_blob, static_cast<std::size_t>(cell_count));
    frame.accessibility = decodeFloatBlob(
      bytes, frame.accessibility_blob, static_cast<std::size_t>(cell_count));
    frame.factory_xyz = decodeFloatBlob(
      bytes, frame.factory_xyz_blob, static_cast<std::size_t>(factory_value_count));

    if (!std::all_of(frame.factory_xyz.begin(), frame.factory_xyz.end(), [](const float value) {
        return std::isfinite(value);
      }))
    {
      fail(framePrefix(index, frame.case_name) + "factory XYZ contains a non-finite value");
    }
  }

  return replay;
}

}  // namespace test_support
}  // namespace x30_plane_seg_core
