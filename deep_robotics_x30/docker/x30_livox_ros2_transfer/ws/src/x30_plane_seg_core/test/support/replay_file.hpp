#ifndef X30_PLANE_SEG_CORE__TEST__SUPPORT__REPLAY_FILE_HPP_
#define X30_PLANE_SEG_CORE__TEST__SUPPORT__REPLAY_FILE_HPP_

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "support/sha256.hpp"

namespace x30_plane_seg_core
{
namespace test_support
{

constexpr std::uint32_t kReplayHasExactTf = 1U << 0U;
constexpr std::uint32_t kReplayExpectMissingTf = 1U << 1U;
constexpr std::uint32_t kReplayHasFactoryOracle = 1U << 2U;
constexpr std::uint32_t kReplayHasCoreOracle = 1U << 3U;
constexpr std::uint32_t kReplayRunCore = 1U << 4U;

struct ReplayBlobRef
{
  std::uint64_t offset{0};
  std::uint64_t bytes{0};
};

struct ReplayHeader
{
  std::uint16_t major{0};
  std::uint16_t minor{0};
  std::uint32_t flags{0};
  std::uint32_t frame_count{0};
  std::uint64_t payload_offset{0};
  std::uint64_t file_bytes{0};
  Sha256Digest source_manifest_sha256{};
  Sha256Digest body_sha256{};
  float absolute_tolerance{0.0F};
  float relative_tolerance{0.0F};
};

struct ReplayFrame
{
  std::string case_name;
  std::uint64_t stamp_ns{0};
  std::uint32_t selected_index{0};
  std::uint32_t flags{0};
  std::uint32_t size_x{0};
  std::uint32_t size_y{0};
  std::uint32_t outer_start_index{0};
  std::uint32_t inner_start_index{0};
  double resolution{0.0};
  double length_x{0.0};
  double length_y{0.0};
  std::array<double, 3> center{};
  std::array<double, 4> orientation_xyzw{};
  std::array<double, 3> world_to_base_translation{};
  std::array<double, 4> world_to_base_rotation_xyzw{};
  std::array<float, 3> sensor_origin{};
  std::array<float, 3> sensor_look_direction{};
  float accessibility_threshold{0.0F};
  float downsample_resolution_m{0.0F};
  float max_angle_from_horizontal_deg{0.0F};
  bool factory_mode_9{false};
  bool debug{false};
  std::uint32_t expected_retained_count{0};
  std::uint32_t expected_core_count{0};
  std::uint32_t factory_point_count{0};
  ReplayBlobRef elevation_blob;
  ReplayBlobRef accessibility_blob;
  ReplayBlobRef factory_xyz_blob;
  ReplayBlobRef core_oracle_blob;
  Sha256Digest frame_input_sha256{};
  std::vector<float> elevation;
  std::vector<float> accessibility;
  std::vector<float> factory_xyz;

  bool hasFlag(std::uint32_t flag) const noexcept
  {
    return (flags & flag) != 0U;
  }
};

struct ReplayFile
{
  ReplayHeader header;
  std::vector<ReplayFrame> frames;
};

class ReplayFormatError : public std::runtime_error
{
public:
  explicit ReplayFormatError(const std::string & message)
  : std::runtime_error(message)
  {
  }
};

ReplayFile loadReplayFile(const std::string & path);

}  // namespace test_support
}  // namespace x30_plane_seg_core

#endif  // X30_PLANE_SEG_CORE__TEST__SUPPORT__REPLAY_FILE_HPP_
