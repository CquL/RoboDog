#ifndef X30_PLANE_SEG_CORE__PLANE_SEG_CORE_HPP_
#define X30_PLANE_SEG_CORE__PLANE_SEG_CORE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace x30_plane_seg_core
{

constexpr float kFactoryAccessibilityThreshold = 0.9F;
constexpr int kFactoryDefaultMinPoints = 20;
constexpr int kFactoryMode9MinPoints = 40;

enum class ParityStage
{
  kCoreRectangles,
};

struct TerrainSample
{
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};
  float accessibility{0.0F};
};

struct CoreConfig
{
  float accessibility_threshold{kFactoryAccessibilityThreshold};
  bool factory_mode_9{false};
  bool debug{false};
  float downsample_resolution_m{0.01F};
  float max_angle_from_horizontal_deg{45.0F};
};

struct CandidateBlock
{
  int type{0};
  Eigen::Vector3f size{Eigen::Vector3f::Zero()};
  Eigen::Isometry3f pose{Eigen::Isometry3f::Identity()};
  std::vector<Eigen::Vector3f> hull;
  Eigen::Vector2d gravity_center{Eigen::Vector2d::Zero()};
  std::vector<Eigen::Vector3f> contained_points;
};

struct CoreResult
{
  bool success{false};
  ParityStage parity_stage{ParityStage::kCoreRectangles};
  std::size_t input_sample_count{0};
  std::size_t retained_sample_count{0};
  std::vector<CandidateBlock> candidates;
  std::string diagnostic;
};

class PlaneSegCore
{
public:
  explicit PlaneSegCore(CoreConfig config = CoreConfig{});

  CoreResult process(
    const std::vector<TerrainSample> & samples,
    const Eigen::Vector3f & sensor_origin,
    const Eigen::Vector3f & sensor_look_direction) const;

  static std::vector<TerrainSample> filterAccessibleSamples(
    const std::vector<TerrainSample> & samples,
    float accessibility_threshold = kFactoryAccessibilityThreshold);

  static int minimumPlanePoints(bool factory_mode_9);
  static const char * parityStageName(ParityStage stage);

private:
  CoreConfig config_;
};

}  // namespace x30_plane_seg_core

#endif  // X30_PLANE_SEG_CORE__PLANE_SEG_CORE_HPP_
