#include "x30_plane_seg_core/quadrangle_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace x30_plane_seg_core
{

float signedAreaXY(const Quadrangle & points) noexcept
{
  float twice_area = 0.0F;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Eigen::Vector3f & current = points[index];
    const Eigen::Vector3f & next = points[(index + 1U) % points.size()];
    twice_area += current.x() * next.y() - next.x() * current.y();
  }
  return 0.5F * twice_area;
}

bool isClockwiseXY(const Quadrangle & points) noexcept
{
  return signedAreaXY(points) < 0.0F;
}

CandidateTopRectangleResult candidateTopRectangle(
  const CandidateBlock & candidate,
  const float minimum_edge_m) noexcept
{
  CandidateTopRectangleResult result;

  if (!std::isfinite(minimum_edge_m) || minimum_edge_m <= 0.0F) {
    result.diagnostic = "minimum edge must be finite and positive";
    return result;
  }
  if (!candidate.size.allFinite() || !candidate.pose.matrix().allFinite()) {
    result.diagnostic = "candidate size or pose is non-finite";
    return result;
  }
  if (candidate.size.x() <= minimum_edge_m || candidate.size.y() <= minimum_edge_m ||
    candidate.size.z() < 0.0F)
  {
    result.diagnostic = "candidate has a degenerate size";
    return result;
  }
  if (candidate.hull.size() < 3U ||
    !std::all_of(candidate.hull.begin(), candidate.hull.end(), [](const Eigen::Vector3f & point) {
      return point.allFinite();
    }))
  {
    result.diagnostic = "candidate hull is empty, undersized, or non-finite";
    return result;
  }

  const float half_x = 0.5F * candidate.size.x();
  const float half_y = 0.5F * candidate.size.y();
  const float top_z = 0.5F * candidate.size.z();
  const std::array<Eigen::Vector3f, 4> local_points{{
    Eigen::Vector3f(-half_x, -half_y, top_z),
    Eigen::Vector3f(half_x, -half_y, top_z),
    Eigen::Vector3f(half_x, half_y, top_z),
    Eigen::Vector3f(-half_x, half_y, top_z),
  }};

  for (std::size_t index = 0; index < local_points.size(); ++index) {
    result.points[index] = candidate.pose * local_points[index];
  }

  const float projected_area = signedAreaXY(result.points);
  if (!std::isfinite(projected_area) ||
    std::abs(projected_area) <= minimum_edge_m * minimum_edge_m)
  {
    result.diagnostic = "candidate top face is degenerate in XY";
    return result;
  }
  if (projected_area > 0.0F) {
    std::swap(result.points[1], result.points[3]);
  }

  result.success = true;
  result.diagnostic = "candidate top rectangle recovered in clockwise XY order";
  return result;
}

}  // namespace x30_plane_seg_core
