#include "x30_plane_seg_core/plane_seg_core.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

#include "plane_seg/BlockFitter.hpp"

namespace x30_plane_seg_core
{

PlaneSegCore::PlaneSegCore(CoreConfig config)
: config_(std::move(config))
{
}

std::vector<TerrainSample> PlaneSegCore::filterAccessibleSamples(
  const std::vector<TerrainSample> & samples,
  const float accessibility_threshold)
{
  std::vector<TerrainSample> filtered;
  filtered.reserve(samples.size());

  for (const TerrainSample & sample : samples) {
    if (!sample.position.allFinite()) {
      continue;
    }
    if (!std::isfinite(sample.accessibility)) {
      continue;
    }
    if (sample.accessibility > accessibility_threshold) {
      continue;
    }
    filtered.push_back(sample);
  }

  return filtered;
}

int PlaneSegCore::minimumPlanePoints(const bool factory_mode_9)
{
  return factory_mode_9 ? kFactoryMode9MinPoints : kFactoryDefaultMinPoints;
}

const char * PlaneSegCore::parityStageName(const ParityStage stage)
{
  switch (stage) {
    case ParityStage::kCoreRectangles:
      return "core_rectangles";
  }
  return "unknown";
}

CoreResult PlaneSegCore::process(
  const std::vector<TerrainSample> & samples,
  const Eigen::Vector3f & sensor_origin,
  const Eigen::Vector3f & sensor_look_direction) const
{
  CoreResult result;
  result.input_sample_count = samples.size();

  if (!sensor_origin.allFinite() || !sensor_look_direction.allFinite() ||
    sensor_look_direction.norm() < 1.0e-6F)
  {
    result.diagnostic = "invalid sensor pose";
    return result;
  }

  const std::vector<TerrainSample> filtered = filterAccessibleSamples(
    samples, config_.accessibility_threshold);
  result.retained_sample_count = filtered.size();

  const int minimum_points = minimumPlanePoints(config_.factory_mode_9);
  if (filtered.size() < static_cast<std::size_t>(minimum_points)) {
    result.diagnostic = "not enough accessible terrain samples";
    return result;
  }

  planeseg::LabeledCloud::Ptr cloud(new planeseg::LabeledCloud());
  cloud->reserve(filtered.size());
  for (std::size_t index = 0; index < filtered.size(); ++index) {
    planeseg::Point point;
    point.getVector3fMap() = filtered[index].position;
    point.label = static_cast<std::uint32_t>(index);
    cloud->push_back(point);
  }

  planeseg::BlockFitter fitter;
  fitter.setSensorPose(sensor_origin, sensor_look_direction.normalized());
  fitter.setCloud(cloud);
  fitter.setDownsampleResolution(config_.downsample_resolution_m);
  fitter.setRemoveGround(false);
  fitter.setMaxAngleFromHorizontal(config_.max_angle_from_horizontal_deg);
  fitter.setComputeVerticalPlane(false);
  fitter.setMinPoint(minimum_points);
  fitter.setDebug(config_.debug);

  const planeseg::BlockFitter::Result core_result = fitter.go();
  result.success = core_result.mSuccess;
  result.candidates.reserve(core_result.mBlocks.size());
  for (std::size_t index = 0U; index < core_result.mBlocks.size(); ++index) {
    const planeseg::BlockFitter::Block & block = core_result.mBlocks[index];
    CandidateBlock candidate;
    candidate.type = block.type;
    candidate.size = block.mSize;
    candidate.pose = block.mPose;
    candidate.hull = block.mHull;
    if (index < core_result.mGravityCenters.size()) {
      candidate.gravity_center = core_result.mGravityCenters[index];
    }
    if (index < core_result.mPointClouds.size() && core_result.mPointClouds[index]) {
      candidate.contained_points.reserve(core_result.mPointClouds[index]->size());
      for (const pcl::PointXYZ & point : *core_result.mPointClouds[index]) {
        candidate.contained_points.emplace_back(point.x, point.y, point.z);
      }
    }
    result.candidates.push_back(std::move(candidate));
  }

  result.diagnostic = result.success ?
    "core rectangle extraction completed; X30 quadrangle post-processing is pending" :
    "core rectangle extraction failed";
  return result;
}

}  // namespace x30_plane_seg_core
