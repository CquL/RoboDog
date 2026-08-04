#include "x30_plane_seg_core/grid_map_adapter.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <type_traits>

namespace
{

using x30_plane_seg_core::Float32ArrayView;
using x30_plane_seg_core::GridMapInput;
using x30_plane_seg_core::GridMapQuaternion;
using x30_plane_seg_core::GridMapVector3;

static_assert(sizeof(float) == 4U, "GridMap layers require 32-bit float storage");
static_assert(std::is_standard_layout<GridMapInput>::value, "GridMapInput must be POD");
static_assert(std::is_trivial<GridMapInput>::value, "GridMapInput must be POD");

bool check(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "grid map adapter failure: " << message << std::endl;
  }
  return condition;
}

bool near(const float actual, const float expected, const float tolerance = 1.0e-5F)
{
  return std::abs(actual - expected) <= tolerance;
}

template<std::size_t Size>
GridMapInput makeInput(
  const std::array<float, Size> & elevation,
  const std::array<float, Size> & accessibility,
  const std::size_t size_x,
  const std::size_t size_y)
{
  GridMapInput input{};
  input.size_x = size_x;
  input.size_y = size_y;
  input.resolution = 1.0;
  input.length_x = static_cast<double>(size_x);
  input.length_y = static_cast<double>(size_y);
  input.center = GridMapVector3{0.0, 0.0, 0.0};
  input.orientation = GridMapQuaternion{0.0, 0.0, 0.0, 1.0};
  input.elevation = Float32ArrayView{elevation.data(), elevation.size()};
  input.accessibility = Float32ArrayView{accessibility.data(), accessibility.size()};
  return input;
}

bool expectFailure(const GridMapInput & input, const char * message)
{
  const auto result = x30_plane_seg_core::adaptGridMap(input);
  return check(!result.success, message) &&
         check(result.samples.empty(), "failed conversion must not return partial samples") &&
         check(
    result.diagnostic != nullptr && result.diagnostic[0] != '\0',
    "failed conversion must include a diagnostic");
}

bool testCircularBufferAndCoordinateDirection()
{
  const std::array<float, 6> elevation{{0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F}};
  const std::array<float, 6> accessibility{{
    100.0F, 101.0F, 102.0F, 103.0F, 104.0F, 105.0F}};
  GridMapInput input = makeInput(elevation, accessibility, 3U, 2U);
  input.center.z = 50.0;
  input.outer_start_index = 1U;
  input.inner_start_index = 1U;

  const auto result = x30_plane_seg_core::adaptGridMap(input);
  bool passed = true;
  passed &= check(result.success, "valid circular grid must convert");
  passed &= check(result.samples.size() == 6U, "all circular grid cells must be retained");
  if (result.samples.size() != 6U) {
    return false;
  }

  const std::array<float, 6> expected_z{{4.0F, 5.0F, 3.0F, 1.0F, 2.0F, 0.0F}};
  const std::array<float, 6> expected_accessibility{{
    104.0F, 105.0F, 103.0F, 101.0F, 102.0F, 100.0F}};
  for (std::size_t index = 0; index < result.samples.size(); ++index) {
    passed &= check(
      result.samples[index].position.z() == expected_z[index],
      "circular elevation unpack order mismatch");
    passed &= check(
      result.samples[index].accessibility == expected_accessibility[index],
      "circular accessibility unpack order mismatch");
  }

  passed &= check(near(result.samples[0].position.x(), 1.0F), "logical X must start positive");
  passed &= check(near(result.samples[1].position.x(), 0.0F), "logical X midpoint mismatch");
  passed &= check(near(result.samples[2].position.x(), -1.0F), "logical X must decrease");
  passed &= check(near(result.samples[0].position.y(), 0.5F), "logical Y must start positive");
  passed &= check(near(result.samples[3].position.y(), -0.5F), "logical Y must decrease");
  passed &= check(
    result.samples[0].position.z() == 4.0F,
    "center Z must not be added to absolute elevation");
  return passed;
}

bool testYawTranslationAndQuaternionNormalization()
{
  const std::array<float, 2> elevation{{7.0F, 8.0F}};
  const std::array<float, 2> accessibility{{0.25F, 0.75F}};
  GridMapInput input = makeInput(elevation, accessibility, 2U, 1U);
  input.center = GridMapVector3{10.0, -2.0, 100.0};
  const double root_two = std::sqrt(2.0);
  input.orientation = GridMapQuaternion{0.0, 0.0, root_two, root_two};

  const auto result = x30_plane_seg_core::adaptGridMap(input);
  bool passed = true;
  passed &= check(result.success, "finite non-unit quaternion must normalize");
  passed &= check(result.samples.size() == 2U, "yaw test sample count mismatch");
  if (result.samples.size() != 2U) {
    return false;
  }

  passed &= check(near(result.samples[0].position.x(), 10.0F), "yaw first X mismatch");
  passed &= check(near(result.samples[0].position.y(), -1.5F), "yaw first Y mismatch");
  passed &= check(near(result.samples[1].position.x(), 10.0F), "yaw second X mismatch");
  passed &= check(near(result.samples[1].position.y(), -2.5F), "yaw second Y mismatch");
  passed &= check(result.samples[0].position.z() == 7.0F, "yaw must not alter elevation");
  passed &= check(result.samples[1].accessibility == 0.75F, "accessibility must be unchanged");
  return passed;
}

bool testNanElevationIsRetained()
{
  const std::array<float, 1> elevation{{std::numeric_limits<float>::quiet_NaN()}};
  const std::array<float, 1> accessibility{{1.25F}};
  const GridMapInput input = makeInput(elevation, accessibility, 1U, 1U);

  const auto result = x30_plane_seg_core::adaptGridMap(input);
  return check(result.success, "NaN elevation must not fail adaptation") &&
         check(result.samples.size() == 1U, "NaN elevation cell must be retained") &&
         check(std::isnan(result.samples[0].position.z()), "NaN elevation must remain NaN") &&
         check(result.samples[0].accessibility == 1.25F, "accessibility must remain unchanged");
}

bool testInvalidInputs()
{
  const std::array<float, 6> elevation{{0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F}};
  const std::array<float, 6> accessibility{{0.0F, 0.1F, 0.2F, 0.3F, 0.4F, 0.5F}};
  const GridMapInput valid = makeInput(elevation, accessibility, 3U, 2U);
  bool passed = true;

  GridMapInput invalid = valid;
  invalid.size_x = 0U;
  passed &= expectFailure(invalid, "zero dimension must fail");

  invalid = valid;
  invalid.size_x = std::numeric_limits<std::size_t>::max();
  invalid.size_y = 2U;
  passed &= expectFailure(invalid, "overflowing dimensions must fail");

  invalid = valid;
  invalid.resolution = 0.0;
  passed &= expectFailure(invalid, "zero resolution must fail");

  invalid = valid;
  invalid.resolution = std::numeric_limits<double>::quiet_NaN();
  passed &= expectFailure(invalid, "non-finite resolution must fail");

  invalid = valid;
  invalid.length_x = 3.25;
  passed &= expectFailure(invalid, "inconsistent grid length must fail");

  invalid = valid;
  invalid.elevation.size = 5U;
  passed &= expectFailure(invalid, "short elevation array must fail");

  invalid = valid;
  invalid.accessibility.size = 7U;
  passed &= expectFailure(invalid, "long accessibility array must fail");

  invalid = valid;
  invalid.elevation.data = nullptr;
  passed &= expectFailure(invalid, "null elevation array must fail");

  invalid = valid;
  invalid.outer_start_index = 3U;
  passed &= expectFailure(invalid, "outer start index at size must fail");

  invalid = valid;
  invalid.inner_start_index = 2U;
  passed &= expectFailure(invalid, "inner start index at size must fail");

  invalid = valid;
  invalid.center.y = std::numeric_limits<double>::infinity();
  passed &= expectFailure(invalid, "non-finite center must fail");

  invalid = valid;
  invalid.orientation.x = std::numeric_limits<double>::quiet_NaN();
  passed &= expectFailure(invalid, "non-finite quaternion must fail");

  invalid = valid;
  invalid.orientation = GridMapQuaternion{0.0, 0.0, 0.0, 0.0};
  passed &= expectFailure(invalid, "zero quaternion must fail");

  return passed;
}

}  // namespace

int main()
{
  bool passed = true;
  passed &= testCircularBufferAndCoordinateDirection();
  passed &= testYawTranslationAndQuaternionNormalization();
  passed &= testNanElevationIsRetained();
  passed &= testInvalidInputs();
  return passed ? 0 : 1;
}
