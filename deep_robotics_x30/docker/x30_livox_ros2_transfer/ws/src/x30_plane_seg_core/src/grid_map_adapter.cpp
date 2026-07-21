#include "x30_plane_seg_core/grid_map_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

namespace x30_plane_seg_core
{
namespace
{

constexpr double kGeometryRelativeTolerance = 1.0e-9;

GridMapAdapterResult failure(const char * diagnostic) noexcept
{
  GridMapAdapterResult result;
  result.diagnostic = diagnostic;
  return result;
}

bool lengthMatches(
  const double length, const std::size_t size, const double resolution) noexcept
{
  const double expected = static_cast<double>(size) * resolution;
  if (!std::isfinite(expected)) {
    return false;
  }

  const double scale = std::max({1.0, std::abs(length), std::abs(expected)});
  return std::abs(length - expected) <= kGeometryRelativeTolerance * scale;
}

std::size_t shiftedIndex(
  const std::size_t index, const std::size_t start, const std::size_t size) noexcept
{
  const std::size_t distance_to_end = size - start;
  return index < distance_to_end ? index + start : index - distance_to_end;
}

}  // namespace

GridMapAdapterResult adaptGridMap(const GridMapInput & input) noexcept
{
  if (input.size_x == 0U || input.size_y == 0U) {
    return failure("grid dimensions must be non-zero");
  }
  if (input.size_x > std::numeric_limits<std::size_t>::max() / input.size_y) {
    return failure("grid dimensions overflow the cell count");
  }

  const std::size_t cell_count = input.size_x * input.size_y;
  if (!std::isfinite(input.resolution) || input.resolution <= 0.0) {
    return failure("grid resolution must be finite and positive");
  }
  if (!std::isfinite(input.length_x) || input.length_x <= 0.0 ||
    !std::isfinite(input.length_y) || input.length_y <= 0.0)
  {
    return failure("grid lengths must be finite and positive");
  }
  if (!lengthMatches(input.length_x, input.size_x, input.resolution) ||
    !lengthMatches(input.length_y, input.size_y, input.resolution))
  {
    return failure("grid lengths do not match size times resolution");
  }

  if (input.elevation.size != cell_count || input.accessibility.size != cell_count) {
    return failure("grid layer lengths must equal the cell count");
  }
  if (input.elevation.data == nullptr || input.accessibility.data == nullptr) {
    return failure("grid layer data pointers must be non-null");
  }
  if (input.outer_start_index >= input.size_x ||
    input.inner_start_index >= input.size_y)
  {
    return failure("grid start indices are outside their dimensions");
  }

  if (!std::isfinite(input.center.x) || !std::isfinite(input.center.y) ||
    !std::isfinite(input.center.z))
  {
    return failure("grid center must be finite");
  }
  if (!std::isfinite(input.orientation.x) || !std::isfinite(input.orientation.y) ||
    !std::isfinite(input.orientation.z) || !std::isfinite(input.orientation.w))
  {
    return failure("grid orientation must be finite");
  }

  const double quaternion_scale = std::max(
    {
      std::abs(input.orientation.x), std::abs(input.orientation.y),
      std::abs(input.orientation.z), std::abs(input.orientation.w)});
  if (quaternion_scale == 0.0) {
    return failure("grid orientation quaternion must be normalizable");
  }

  const double scaled_x = input.orientation.x / quaternion_scale;
  const double scaled_y = input.orientation.y / quaternion_scale;
  const double scaled_z = input.orientation.z / quaternion_scale;
  const double scaled_w = input.orientation.w / quaternion_scale;
  const double scaled_norm = std::sqrt(
    scaled_x * scaled_x + scaled_y * scaled_y +
    scaled_z * scaled_z + scaled_w * scaled_w);
  if (!std::isfinite(scaled_norm) || scaled_norm == 0.0) {
    return failure("grid orientation quaternion must be normalizable");
  }

  const Eigen::Quaterniond orientation(
    scaled_w / scaled_norm, scaled_x / scaled_norm,
    scaled_y / scaled_norm, scaled_z / scaled_norm);

  GridMapAdapterResult result;
  try {
    result.samples.resize(cell_count);

    for (std::size_t y = 0; y < input.size_y; ++y) {
      const std::size_t raw_y = shiftedIndex(
        y, input.inner_start_index, input.size_y);
      const double local_y =
        input.length_y * 0.5 - (static_cast<double>(y) + 0.5) * input.resolution;

      for (std::size_t x = 0; x < input.size_x; ++x) {
        const std::size_t raw_x = shiftedIndex(
          x, input.outer_start_index, input.size_x);
        const std::size_t raw_index = raw_x + raw_y * input.size_x;
        const std::size_t output_index = x + y * input.size_x;
        const double local_x =
          input.length_x * 0.5 - (static_cast<double>(x) + 0.5) * input.resolution;

        // Pose transforms the planar cell center; factory elevation is already absolute Z.
        const Eigen::Vector3d rotated = orientation * Eigen::Vector3d(local_x, local_y, 0.0);
        const double world_x = input.center.x + rotated.x();
        const double world_y = input.center.y + rotated.y();
        const double float_limit = static_cast<double>(std::numeric_limits<float>::max());
        if (!std::isfinite(world_x) || !std::isfinite(world_y) ||
          std::abs(world_x) > float_limit || std::abs(world_y) > float_limit)
        {
          result.samples.clear();
          result.diagnostic = "transformed grid XY is outside the float32 range";
          return result;
        }

        TerrainSample & sample = result.samples[output_index];
        sample.position = Eigen::Vector3f(
          static_cast<float>(world_x), static_cast<float>(world_y),
          input.elevation.data[raw_index]);
        sample.accessibility = input.accessibility.data[raw_index];
      }
    }
  } catch (...) {
    result.samples.clear();
    result.diagnostic = "unable to allocate or populate terrain samples";
    return result;
  }

  result.success = true;
  result.diagnostic = "ok";
  return result;
}

}  // namespace x30_plane_seg_core
