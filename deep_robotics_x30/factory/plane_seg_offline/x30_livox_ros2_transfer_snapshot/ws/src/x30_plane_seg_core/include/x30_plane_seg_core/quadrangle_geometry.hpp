#ifndef X30_PLANE_SEG_CORE__QUADRANGLE_GEOMETRY_HPP_
#define X30_PLANE_SEG_CORE__QUADRANGLE_GEOMETRY_HPP_

#include <array>

#include <Eigen/Geometry>

#include "x30_plane_seg_core/plane_seg_core.hpp"

namespace x30_plane_seg_core
{

using Quadrangle = std::array<Eigen::Vector3f, 4>;

struct CandidateTopRectangleResult
{
  bool success{false};
  Quadrangle points{};
  const char * diagnostic{"candidate top rectangle has not run"};
};

float signedAreaXY(const Quadrangle & points) noexcept;
bool isClockwiseXY(const Quadrangle & points) noexcept;

// Recovers the top face of a BlockFitter candidate. This is an intermediate
// geometric seed, not the final X30 stair quadrangle post-processing result.
CandidateTopRectangleResult candidateTopRectangle(
  const CandidateBlock & candidate,
  float minimum_edge_m = 1.0e-6F) noexcept;

}  // namespace x30_plane_seg_core

#endif  // X30_PLANE_SEG_CORE__QUADRANGLE_GEOMETRY_HPP_
