#include "x30_plane_seg_core/quadrangle_geometry.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include <Eigen/Geometry>

namespace
{

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(1);
  }
}

x30_plane_seg_core::CandidateBlock validCandidate()
{
  x30_plane_seg_core::CandidateBlock candidate;
  candidate.size = Eigen::Vector3f(2.0F, 1.0F, 0.2F);
  candidate.pose = Eigen::Isometry3f::Identity();
  candidate.pose.translation() = Eigen::Vector3f(0.25F, -0.5F, -0.1F);
  candidate.hull = {
    Eigen::Vector3f(-0.75F, -1.0F, 0.0F),
    Eigen::Vector3f(1.25F, -1.0F, 0.0F),
    Eigen::Vector3f(1.25F, 0.0F, 0.0F),
    Eigen::Vector3f(-0.75F, 0.0F, 0.0F),
  };
  return candidate;
}

}  // namespace

int main()
{
  using x30_plane_seg_core::candidateTopRectangle;
  using x30_plane_seg_core::isClockwiseXY;
  using x30_plane_seg_core::signedAreaXY;

  const auto identity = candidateTopRectangle(validCandidate());
  require(identity.success, identity.diagnostic);
  require(isClockwiseXY(identity.points), "top rectangle must use clockwise XY order");
  require(
    std::abs(signedAreaXY(identity.points) + 2.0F) < 1.0e-5F,
    "identity candidate projected area mismatch");
  for (const Eigen::Vector3f & point : identity.points) {
    require(std::abs(point.z()) < 1.0e-6F, "top face did not undo half-height pose shift");
  }

  auto tilted_candidate = validCandidate();
  tilted_candidate.pose.linear() =
    Eigen::AngleAxisf(0.3F, Eigen::Vector3f::UnitY()).toRotationMatrix();
  const auto tilted = candidateTopRectangle(tilted_candidate);
  require(tilted.success, tilted.diagnostic);
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  for (const Eigen::Vector3f & point : tilted.points) {
    center += point;
  }
  center *= 0.25F;
  const Eigen::Vector3f expected_center = tilted_candidate.pose.translation() +
    tilted_candidate.pose.rotation().col(2) * tilted_candidate.size.z() * 0.5F;
  require(
    (center - expected_center).norm() < 1.0e-5F,
    "tilted top center does not match pose plus half-height normal");

  auto empty_hull = validCandidate();
  empty_hull.hull.clear();
  require(
    !candidateTopRectangle(empty_hull).success,
    "empty hull candidate must be rejected");

  auto zero_width = validCandidate();
  zero_width.size.x() = 0.0F;
  require(
    !candidateTopRectangle(zero_width).success,
    "zero-width candidate must be rejected");

  auto non_finite = validCandidate();
  non_finite.pose.translation().x() = std::numeric_limits<float>::quiet_NaN();
  require(
    !candidateTopRectangle(non_finite).success,
    "non-finite candidate must be rejected");

  std::cout << "X30 candidate top rectangle tests passed." << std::endl;
  return 0;
}
