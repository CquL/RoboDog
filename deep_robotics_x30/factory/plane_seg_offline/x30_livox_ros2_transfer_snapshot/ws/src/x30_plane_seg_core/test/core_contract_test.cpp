#include "x30_plane_seg_core/plane_seg_core.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{

bool check(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "x30_plane_seg_core contract failure: " << message << std::endl;
  }
  return condition;
}

}  // namespace

int main()
{
  using x30_plane_seg_core::PlaneSegCore;
  using x30_plane_seg_core::TerrainSample;

  bool passed = true;
  passed &= check(
    PlaneSegCore::minimumPlanePoints(false) == 20,
    "normal minimum point count must be 20");
  passed &= check(
    PlaneSegCore::minimumPlanePoints(true) == 40,
    "factory mode 9 minimum point count must be 40");
  passed &= check(
    std::string(PlaneSegCore::parityStageName(
      x30_plane_seg_core::ParityStage::kCoreRectangles)) == "core_rectangles",
    "parity stage must not claim final quadrangles");

  std::vector<TerrainSample> samples(4);
  samples[0].position = Eigen::Vector3f(0.0F, 0.0F, 0.0F);
  samples[0].accessibility = 0.9F;
  samples[1].position = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
  samples[1].accessibility = std::nextafter(0.9F, 1.0F);
  samples[2].position = Eigen::Vector3f(2.0F, 0.0F, 0.0F);
  samples[2].accessibility = 0.0F;
  samples[3].position = Eigen::Vector3f(
    std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);
  samples[3].accessibility = 0.0F;

  const auto filtered = PlaneSegCore::filterAccessibleSamples(samples);
  passed &= check(filtered.size() == 2, "accessibility filter count mismatch");
  passed &= check(
    filtered.size() > 0 && filtered[0].position.x() == 0.0F,
    "accessibility exactly 0.9 must survive");
  passed &= check(
    filtered.size() > 1 && filtered[1].position.x() == 2.0F,
    "finite accessible sample must survive");

  PlaneSegCore core;
  const auto result = core.process(
    filtered, Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitX());
  passed &= check(!result.success, "undersized cloud must fail");
  passed &= check(result.input_sample_count == 2, "input count mismatch");
  passed &= check(result.retained_sample_count == 2, "retained count mismatch");
  passed &= check(
    result.diagnostic == "not enough accessible terrain samples",
    "undersized-cloud diagnostic mismatch");

  std::vector<TerrainSample> synthetic_plane;
  synthetic_plane.reserve(21 * 21);
  for (int row = 0; row < 21; ++row) {
    for (int column = 0; column < 21; ++column) {
      TerrainSample sample;
      sample.position = Eigen::Vector3f(
        0.03F * static_cast<float>(column),
        0.03F * static_cast<float>(row),
        0.0F);
      sample.accessibility = 0.0F;
      synthetic_plane.push_back(sample);
    }
  }

  const auto plane_result = core.process(
    synthetic_plane,
    Eigen::Vector3f(0.0F, 0.0F, 1.0F),
    Eigen::Vector3f::UnitX());
  passed &= check(
    plane_result.success,
    "synthetic plane must traverse the PCL segmentation path");
  passed &= check(
    !plane_result.candidates.empty(),
    "synthetic plane must produce a core rectangle candidate");
  passed &= check(
    plane_result.retained_sample_count == synthetic_plane.size(),
    "synthetic plane retained count mismatch");

  return passed ? 0 : 1;
}
