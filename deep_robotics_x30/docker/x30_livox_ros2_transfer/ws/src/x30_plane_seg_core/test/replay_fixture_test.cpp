#include "support/replay_file.hpp"

#include "x30_plane_seg_core/grid_map_adapter.hpp"
#include "x30_plane_seg_core/plane_seg_core.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>

namespace
{

using x30_plane_seg_core::CandidateBlock;
using x30_plane_seg_core::CoreConfig;
using x30_plane_seg_core::CoreResult;
using x30_plane_seg_core::Float32ArrayView;
using x30_plane_seg_core::GridMapInput;
using x30_plane_seg_core::GridMapQuaternion;
using x30_plane_seg_core::GridMapVector3;
using x30_plane_seg_core::PlaneSegCore;
using x30_plane_seg_core::test_support::ReplayFrame;
using x30_plane_seg_core::test_support::Sha256;
using x30_plane_seg_core::test_support::Sha256Digest;

constexpr std::size_t kExpectedFrameCount = 12U;
constexpr std::size_t kExpectedInputCellCount = 10000U;
constexpr float kExpectedBlockHeight = 0.142875F;
constexpr float kBlockHeightTolerance = 1.0e-6F;

struct CommandLineOptions
{
  std::string fixture_path{"test/fixtures/x30_plane_seg_replay_v1.x30rpl"};
  std::size_t expected_frame_count{kExpectedFrameCount};
  std::string expected_output_sha256;
  bool trace_frames{false};
};

std::string normalizeSha256Hex(const std::string & value, const char * const option)
{
  if (value.size() != 64U) {
    throw std::runtime_error(std::string(option) + " requires 64 hexadecimal characters");
  }
  std::string normalized;
  normalized.reserve(value.size());
  for (const unsigned char character : value) {
    if (!std::isxdigit(character)) {
      throw std::runtime_error(std::string(option) + " contains a non-hexadecimal character");
    }
    normalized.push_back(static_cast<char>(std::tolower(character)));
  }
  return normalized;
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

std::size_t parsePositiveSize(const std::string & value, const char * const option)
{
  std::size_t consumed = 0U;
  unsigned long long parsed = 0U;
  try {
    parsed = std::stoull(value, &consumed, 10);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string(option) + " requires a positive integer");
  }
  if (consumed != value.size() || parsed == 0U ||
    parsed > static_cast<unsigned long long>(std::numeric_limits<std::uint32_t>::max()))
  {
    throw std::runtime_error(std::string(option) + " requires a positive uint32 value");
  }
  return static_cast<std::size_t>(parsed);
}

CommandLineOptions parseCommandLine(const int argc, char ** argv)
{
  CommandLineOptions options;
  bool fixture_was_set = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--expected-frames") {
      if (++index >= argc) {
        throw std::runtime_error("--expected-frames requires a value");
      }
      options.expected_frame_count = parsePositiveSize(argv[index], "--expected-frames");
      continue;
    }
    if (argument == "--expected-output-sha256") {
      if (++index >= argc) {
        throw std::runtime_error("--expected-output-sha256 requires a value");
      }
      options.expected_output_sha256 = normalizeSha256Hex(
        argv[index], "--expected-output-sha256");
      continue;
    }
    if (argument == "--trace-frames") {
      options.trace_frames = true;
      continue;
    }
    if (!argument.empty() && argument.front() == '-') {
      throw std::runtime_error("unknown option: " + argument);
    }
    if (fixture_was_set) {
      throw std::runtime_error("only one replay fixture path may be supplied");
    }
    options.fixture_path = argument;
    fixture_was_set = true;
  }
  return options;
}

[[noreturn]] void fail(const ReplayFrame & frame, const std::string & message)
{
  throw std::runtime_error(frame.case_name + ": " + message);
}

void require(
  const ReplayFrame & frame, const bool condition, const std::string & message)
{
  if (!condition) {
    fail(frame, message);
  }
}

float canonicalZero(const float value)
{
  return value == 0.0F ? 0.0F : value;
}

struct CanonicalCandidate
{
  int type{0};
  std::array<float, 3> size{};
  std::array<float, 3> translation{};
  std::array<float, 4> quaternion_xyzw{};
  std::vector<std::array<float, 3>> hull;
};

bool finite(const std::array<float, 3> & values)
{
  return std::all_of(values.begin(), values.end(), [](const float value) {
      return std::isfinite(value);
    });
}

bool finite(const std::array<float, 4> & values)
{
  return std::all_of(values.begin(), values.end(), [](const float value) {
      return std::isfinite(value);
    });
}

bool shouldNegateQuaternion(const std::array<float, 4> & xyzw)
{
  const std::array<float, 4> canonical_order{{xyzw[3], xyzw[0], xyzw[1], xyzw[2]}};
  for (const float value : canonical_order) {
    if (value < 0.0F) {
      return true;
    }
    if (value > 0.0F) {
      return false;
    }
  }
  return false;
}

CanonicalCandidate canonicalize(
  const ReplayFrame & frame, const CandidateBlock & candidate)
{
  CanonicalCandidate output;
  output.type = candidate.type;
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    output.size[axis] = canonicalZero(candidate.size[static_cast<Eigen::Index>(axis)]);
    output.translation[axis] = canonicalZero(
      candidate.pose.translation()[static_cast<Eigen::Index>(axis)]);
  }
  require(frame, finite(output.size), "candidate size contains a non-finite value");
  require(
    frame, finite(output.translation),
    "candidate translation contains a non-finite value");
  require(frame, candidate.pose.rotation().allFinite(), "candidate rotation is non-finite");

  Eigen::Quaternionf quaternion(candidate.pose.rotation());
  require(
    frame, std::isfinite(quaternion.norm()) && quaternion.norm() > 1.0e-6F,
    "candidate rotation cannot be converted to a normalizable quaternion");
  quaternion.normalize();
  output.quaternion_xyzw = {{
    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w(),
  }};
  if (shouldNegateQuaternion(output.quaternion_xyzw)) {
    for (float & value : output.quaternion_xyzw) {
      value = -value;
    }
  }
  for (float & value : output.quaternion_xyzw) {
    value = canonicalZero(value);
  }
  require(
    frame, finite(output.quaternion_xyzw),
    "candidate quaternion contains a non-finite value");

  output.hull.reserve(candidate.hull.size());
  for (const Eigen::Vector3f & point : candidate.hull) {
    std::array<float, 3> xyz{{
      canonicalZero(point.x()), canonicalZero(point.y()), canonicalZero(point.z()),
    }};
    require(frame, finite(xyz), "candidate hull contains a non-finite point");
    output.hull.push_back(xyz);
  }
  std::sort(output.hull.begin(), output.hull.end());
  return output;
}

std::vector<CanonicalCandidate> canonicalize(
  const ReplayFrame & frame, const std::vector<CandidateBlock> & candidates)
{
  std::vector<CanonicalCandidate> output;
  output.reserve(candidates.size());
  for (const CandidateBlock & candidate : candidates) {
    output.push_back(canonicalize(frame, candidate));
  }
  std::sort(
    output.begin(), output.end(),
    [](const CanonicalCandidate & left, const CanonicalCandidate & right) {
      return std::tie(
        left.type, left.size, left.translation, left.quaternion_xyzw, left.hull) <
             std::tie(
        right.type, right.size, right.translation, right.quaternion_xyzw, right.hull);
    });
  return output;
}

std::string jsonEscape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20U) {
          output << "\\u00" << std::hex << std::setw(2) << std::setfill('0') <<
            static_cast<unsigned int>(character) << std::dec << std::setfill(' ');
        } else {
          output << static_cast<char>(character);
        }
        break;
    }
  }
  return output.str();
}

template<std::size_t Size>
void appendArray(std::ostream & output, const std::array<float, Size> & values)
{
  output << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << values[index];
  }
  output << ']';
}

std::string frameJson(
  const ReplayFrame & frame, const CoreResult & result,
  const std::vector<CanonicalCandidate> & candidates)
{
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<float>::max_digits10);
  output << "{\"case\":\"" << jsonEscape(frame.case_name) <<
    "\",\"stamp_ns\":" << frame.stamp_ns <<
    ",\"selected_index\":" << frame.selected_index <<
    ",\"retained\":" << result.retained_sample_count <<
    ",\"candidate_count\":" << candidates.size() <<
    ",\"factory_group_count\":" << frame.factory_point_count / 4U <<
    ",\"candidates\":[";
  for (std::size_t candidate_index = 0; candidate_index < candidates.size();
    ++candidate_index)
  {
    if (candidate_index != 0U) {
      output << ',';
    }
    const CanonicalCandidate & candidate = candidates[candidate_index];
    output << "{\"type\":" << candidate.type << ",\"size\":";
    appendArray(output, candidate.size);
    output << ",\"translation\":";
    appendArray(output, candidate.translation);
    output << ",\"quaternion_xyzw\":";
    appendArray(output, candidate.quaternion_xyzw);
    output << ",\"hull\":[";
    for (std::size_t point_index = 0; point_index < candidate.hull.size(); ++point_index) {
      if (point_index != 0U) {
        output << ',';
      }
      appendArray(output, candidate.hull[point_index]);
    }
    output << "]}";
  }
  output << "]}";
  return output.str();
}

std::string runFrame(const ReplayFrame & frame)
{
  const std::uint64_t cell_count =
    static_cast<std::uint64_t>(frame.size_x) * static_cast<std::uint64_t>(frame.size_y);
  require(frame, cell_count == kExpectedInputCellCount, "input grid must contain 10000 cells");
  require(
    frame, frame.elevation.size() == kExpectedInputCellCount &&
    frame.accessibility.size() == kExpectedInputCellCount,
    "decoded grid layer size mismatch");

  GridMapInput input{};
  input.size_x = frame.size_x;
  input.size_y = frame.size_y;
  input.resolution = frame.resolution;
  input.length_x = frame.length_x;
  input.length_y = frame.length_y;
  input.center = GridMapVector3{frame.center[0], frame.center[1], frame.center[2]};
  input.orientation = GridMapQuaternion{
    frame.orientation_xyzw[0], frame.orientation_xyzw[1],
    frame.orientation_xyzw[2], frame.orientation_xyzw[3]};
  input.outer_start_index = frame.outer_start_index;
  input.inner_start_index = frame.inner_start_index;
  input.elevation = Float32ArrayView{frame.elevation.data(), frame.elevation.size()};
  input.accessibility = Float32ArrayView{
    frame.accessibility.data(), frame.accessibility.size()};

  const auto adapted = x30_plane_seg_core::adaptGridMap(input);
  require(
    frame, adapted.success,
    std::string("GridMap adapter failed: ") +
    (adapted.diagnostic == nullptr ? "no diagnostic" : adapted.diagnostic));
  require(
    frame, adapted.samples.size() == kExpectedInputCellCount,
    "GridMap adapter did not return 10000 input cells");

  CoreConfig config;
  config.accessibility_threshold = frame.accessibility_threshold;
  config.factory_mode_9 = frame.factory_mode_9;
  config.debug = frame.debug;
  config.downsample_resolution_m = frame.downsample_resolution_m;
  config.max_angle_from_horizontal_deg = frame.max_angle_from_horizontal_deg;
  const PlaneSegCore core(config);
  const Eigen::Vector3f sensor_origin(
    frame.sensor_origin[0], frame.sensor_origin[1], frame.sensor_origin[2]);
  const Eigen::Vector3f sensor_look(
    frame.sensor_look_direction[0], frame.sensor_look_direction[1],
    frame.sensor_look_direction[2]);

  std::srand(1);
  const CoreResult result = core.process(adapted.samples, sensor_origin, sensor_look);
  require(frame, result.success, "PlaneSegCore::process failed: " + result.diagnostic);
  require(
    frame, result.input_sample_count == kExpectedInputCellCount,
    "core input sample count mismatch");
  require(
    frame, result.retained_sample_count == frame.expected_retained_count,
    "core retained sample count mismatch");
  require(
    frame, result.parity_stage == x30_plane_seg_core::ParityStage::kCoreRectangles &&
    std::string(PlaneSegCore::parityStageName(result.parity_stage)) == "core_rectangles",
    "core parity stage is not core_rectangles");

  if (frame.expected_core_count != std::numeric_limits<std::uint32_t>::max()) {
    require(
      frame, result.candidates.size() == frame.expected_core_count,
      "core candidate count differs from the frozen oracle");
  }
  for (const CandidateBlock & candidate : result.candidates) {
    require(frame, candidate.type == 0, "candidate block type is not factory type 0");
    require(
      frame, std::isfinite(candidate.size.z()) &&
      std::abs(candidate.size.z() - kExpectedBlockHeight) <= kBlockHeightTolerance,
      "candidate block height differs from 0.142875 m");
  }

  return frameJson(frame, result, canonicalize(frame, result.candidates));
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const CommandLineOptions options = parseCommandLine(argc, argv);
    const auto replay = x30_plane_seg_core::test_support::loadReplayFile(
      options.fixture_path);
    if (replay.frames.size() != options.expected_frame_count) {
      std::ostringstream message;
      message << "replay fixture contains " << replay.frames.size() <<
        " frames, expected " << options.expected_frame_count;
      throw std::runtime_error(message.str());
    }

    std::size_t missing_tf_count = 0U;
    Sha256 output_hasher;
    for (std::size_t index = 0U; index < replay.frames.size(); ++index) {
      const ReplayFrame & frame = replay.frames[index];
      if (!frame.hasFlag(x30_plane_seg_core::test_support::kReplayRunCore)) {
        fail(frame, "RUN_CORE flag is missing");
      }
      if (frame.hasFlag(x30_plane_seg_core::test_support::kReplayExpectMissingTf)) {
        ++missing_tf_count;
        require(
          frame, !frame.hasFlag(x30_plane_seg_core::test_support::kReplayHasExactTf),
          "missing-TF frame is also marked HAS_EXACT_TF");
      }
      if (options.trace_frames) {
        std::cerr << "[x30-replay] frame=" << index <<
          " case=" << frame.case_name <<
          " selected_index=" << frame.selected_index <<
          " stamp_ns=" << frame.stamp_ns << '\n';
      }
      const std::string line = runFrame(frame);
      std::cout << line << '\n';
      output_hasher.update(line.data(), line.size());
      constexpr char newline = '\n';
      output_hasher.update(&newline, 1U);
    }
    if (missing_tf_count != 1U) {
      std::ostringstream message;
      message << "replay fixture preserves " << missing_tf_count <<
        " missing-TF frames, expected exactly 1";
      throw std::runtime_error(message.str());
    }
    const std::string output_hash = digestHex(output_hasher.finish());
    if (!options.expected_output_sha256.empty() &&
      output_hash != options.expected_output_sha256)
    {
      throw std::runtime_error(
              "canonical JSONL SHA256 is " + output_hash + ", expected " +
              options.expected_output_sha256);
    }
  } catch (const std::exception & error) {
    std::cerr << "x30 replay fixture failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
