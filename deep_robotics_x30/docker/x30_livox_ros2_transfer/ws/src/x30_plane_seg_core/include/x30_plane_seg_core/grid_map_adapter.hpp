#ifndef X30_PLANE_SEG_CORE__GRID_MAP_ADAPTER_HPP_
#define X30_PLANE_SEG_CORE__GRID_MAP_ADAPTER_HPP_

#include <cstddef>
#include <type_traits>
#include <vector>

#include "x30_plane_seg_core/plane_seg_core.hpp"

namespace x30_plane_seg_core
{

struct GridMapVector3
{
  double x;
  double y;
  double z;
};

struct GridMapQuaternion
{
  // Components are stored in x, y, z, w order.
  double x;
  double y;
  double z;
  double w;
};

struct Float32ArrayView
{
  const float * data;
  std::size_t size;
};

// A non-owning POD view of the fields needed from a GridMap message.
struct GridMapInput
{
  std::size_t size_x;
  std::size_t size_y;
  double resolution;
  double length_x;
  double length_y;
  GridMapVector3 center;
  GridMapQuaternion orientation;
  std::size_t outer_start_index;
  std::size_t inner_start_index;
  Float32ArrayView elevation;
  Float32ArrayView accessibility;
};

static_assert(sizeof(float) == 4U, "GridMap layers require 32-bit float storage");
static_assert(
  std::is_standard_layout<GridMapInput>::value && std::is_trivial<GridMapInput>::value,
  "GridMapInput must remain a POD type");

struct GridMapAdapterResult
{
  bool success{false};
  std::vector<TerrainSample> samples;
  const char * diagnostic{"grid map adaptation has not run"};
};

GridMapAdapterResult adaptGridMap(const GridMapInput & input) noexcept;

}  // namespace x30_plane_seg_core

#endif  // X30_PLANE_SEG_CORE__GRID_MAP_ADAPTER_HPP_
