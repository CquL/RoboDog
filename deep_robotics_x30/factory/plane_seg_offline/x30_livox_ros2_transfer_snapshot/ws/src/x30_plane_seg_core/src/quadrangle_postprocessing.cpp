#include "x30_plane_seg_core/quadrangle_postprocessing.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

#include <Eigen/QR>

namespace x30_plane_seg_core
{
namespace
{

Eigen::Vector2d edgeXY(
  const Eigen::Vector3f & first, const Eigen::Vector3f & second)
{
  return (first - second).head<2>().cast<double>();
}

bool normalize(Eigen::Vector2d & value, const double minimum_norm)
{
  if (!value.allFinite()) {
    return false;
  }
  const double norm = value.norm();
  if (!std::isfinite(norm) || norm <= minimum_norm) {
    return false;
  }
  value /= norm;
  return true;
}

bool finitePositive(const float value)
{
  return std::isfinite(value) && value > 0.0F;
}

double cross2D(
  const Eigen::Vector2d & first, const Eigen::Vector2d & second)
{
  return first.x() * second.y() - first.y() * second.x();
}

bool pointOnSegment(
  const Eigen::Vector2d & point,
  const Eigen::Vector2d & first,
  const Eigen::Vector2d & second,
  const double epsilon)
{
  if (std::abs(cross2D(second - first, point - first)) > epsilon) {
    return false;
  }
  return point.x() >= std::min(first.x(), second.x()) - epsilon &&
         point.x() <= std::max(first.x(), second.x()) + epsilon &&
         point.y() >= std::min(first.y(), second.y()) - epsilon &&
         point.y() <= std::max(first.y(), second.y()) + epsilon;
}

bool pointInsideOrOnQuadrangle(
  const Eigen::Vector2d & point,
  const Quadrangle & quadrangle,
  const double epsilon)
{
  bool has_positive = false;
  bool has_negative = false;
  for (std::size_t index = 0U; index < quadrangle.size(); ++index) {
    const Eigen::Vector2d first =
      quadrangle[index].head<2>().cast<double>();
    const Eigen::Vector2d second =
      quadrangle[(index + 1U) % quadrangle.size()].head<2>().cast<double>();
    if (pointOnSegment(point, first, second, epsilon)) {
      return true;
    }
    const double side = cross2D(second - first, point - first);
    has_positive = has_positive || side > epsilon;
    has_negative = has_negative || side < -epsilon;
    if (has_positive && has_negative) {
      return false;
    }
  }
  return true;
}

std::vector<Eigen::Vector2d> sampleQuadrangleBoundary(
  const Quadrangle & quadrangle,
  const double sample_step_m,
  const double sample_count_epsilon)
{
  std::vector<Eigen::Vector2d> samples;
  for (std::size_t index = 0U; index < quadrangle.size(); ++index) {
    const Eigen::Vector2d start =
      quadrangle[index].head<2>().cast<double>();
    const Eigen::Vector2d end =
      quadrangle[(index + 1U) % quadrangle.size()].head<2>().cast<double>();
    const Eigen::Vector2d delta = end - start;
    const double length = delta.norm();
    const int sample_count = static_cast<int>(
      length / sample_step_m + sample_count_epsilon);
    if (sample_count <= 0 || !std::isfinite(length) || length <= 0.0) {
      continue;
    }
    const Eigen::Vector2d step = delta / length * sample_step_m;
    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
      samples.push_back(
        start + static_cast<double>(sample_index) * step);
    }
  }
  return samples;
}

bool sampledBoundaryIntrudes(
  const Quadrangle & quadrangle,
  const std::vector<Eigen::Vector2d> & sampled_boundary)
{
  constexpr double kBoundaryEpsilon = 1.0e-9;
  return std::any_of(
    sampled_boundary.begin(), sampled_boundary.end(),
    [&](const Eigen::Vector2d & point) {
      return pointInsideOrOnQuadrangle(
        point, quadrangle, kBoundaryEpsilon);
    });
}

Eigen::Vector2d directionAlignedWithStair(
  const Quadrangle & quadrangle,
  const Eigen::Vector2d & stair_direction,
  const double minimum_edge_m)
{
  Eigen::Vector2d first = edgeXY(quadrangle[0], quadrangle[1]);
  Eigen::Vector2d second = edgeXY(quadrangle[1], quadrangle[2]);
  if (!normalize(first, minimum_edge_m) ||
    !normalize(second, minimum_edge_m))
  {
    return Eigen::Vector2d::Zero();
  }
  Eigen::Vector2d selected =
    std::abs(first.dot(stair_direction)) >=
    std::abs(second.dot(stair_direction)) ? first : second;
  if (selected.dot(stair_direction) < 0.0) {
    selected = -selected;
  }
  return selected;
}

IndexedQuadrangle quadrangleFromContainedPoints(
  const IndexedQuadrangle & source,
  const Eigen::Vector2d & direction,
  const float fallback_mean_z)
{
  std::vector<Eigen::Vector3f> fallback_points;
  const std::vector<Eigen::Vector3f> * source_points = &source.contained_points;
  if (source_points->empty()) {
    fallback_points.assign(source.points.begin(), source.points.end());
    source_points = &fallback_points;
  }

  Eigen::Vector2d axis = direction;
  normalize(axis, std::numeric_limits<double>::epsilon());
  const Eigen::Vector2d lateral(-axis.y(), axis.x());

  double minimum_axis = std::numeric_limits<double>::infinity();
  double maximum_axis = -std::numeric_limits<double>::infinity();
  double minimum_lateral = std::numeric_limits<double>::infinity();
  double maximum_lateral = -std::numeric_limits<double>::infinity();
  for (const Eigen::Vector3f & point : *source_points) {
    const Eigen::Vector2d xy = point.head<2>().cast<double>();
    const double along_axis = axis.dot(xy);
    const double along_lateral = lateral.dot(xy);
    minimum_axis = std::min(minimum_axis, along_axis);
    maximum_axis = std::max(maximum_axis, along_axis);
    minimum_lateral = std::min(minimum_lateral, along_lateral);
    maximum_lateral = std::max(maximum_lateral, along_lateral);
  }

  float mean_z = fallback_mean_z;
  if (!source_points->empty()) {
    mean_z = 0.0F;
    for (const Eigen::Vector3f & point : *source_points) {
      mean_z += point.z();
    }
    mean_z /= static_cast<float>(source_points->size());
  }

  const auto worldPoint =
    [&](const double along_axis, const double along_lateral) {
      const Eigen::Vector2d xy = axis * along_axis + lateral * along_lateral;
      return Eigen::Vector3f(
        static_cast<float>(xy.x()), static_cast<float>(xy.y()), mean_z);
    };

  IndexedQuadrangle output;
  output.source_candidate_index = source.source_candidate_index;
  output.contained_points = *source_points;
  output.merge_count = source.merge_count;
  output.points = {{
    worldPoint(minimum_axis, minimum_lateral),
    worldPoint(minimum_axis, maximum_lateral),
    worldPoint(maximum_axis, maximum_lateral),
    worldPoint(maximum_axis, minimum_lateral),
  }};
  if (!isClockwiseXY(output.points)) {
    std::swap(output.points[1], output.points[3]);
  }
  return output;
}

IndexedQuadrangle correctedFromContainedPoints(
  const CandidateBlock & candidate,
  const IndexedQuadrangle & seed,
  const Eigen::Vector2d & direction)
{
  IndexedQuadrangle source = seed;
  source.contained_points =
    candidate.contained_points.empty() ? candidate.hull : candidate.contained_points;
  float mean_z = 0.0F;
  for (const Eigen::Vector3f & point : seed.points) {
    mean_z += point.z();
  }
  mean_z *= 0.25F;
  return quadrangleFromContainedPoints(source, direction, mean_z);
}

std::vector<std::size_t> sideDistanceOutlierIndices(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const Eigen::Vector2d & axis,
  const double threshold_m)
{
  std::vector<std::size_t> indices;
  for (std::size_t index = 0U; index < quadrangles.size(); ++index) {
    bool all_far = true;
    bool same_side = true;
    bool first_side = false;
    bool has_first_side = false;
    for (const Eigen::Vector3f & point : quadrangles[index].points) {
      const Eigen::Vector2d xy = point.head<2>().cast<double>();
      const double projection = axis.dot(xy);
      const double squared_distance =
        std::max(0.0, xy.squaredNorm() - projection * projection);
      if (std::sqrt(squared_distance) <= threshold_m) {
        all_far = false;
      }
      const bool left_side =
        axis.y() * xy.x() - axis.x() * xy.y() > 0.0;
      if (!has_first_side) {
        first_side = left_side;
        has_first_side = true;
      } else if (left_side != first_side) {
        same_side = false;
      }
    }
    if (all_far && same_side) {
      indices.push_back(index);
    }
  }
  return indices;
}

struct VectorizedQuadrangle
{
  std::size_t vector_index{0U};
  double short_edge_m{0.0};
  double long_edge_m{0.0};
  std::vector<Eigen::Vector2d> sampled_boundary;
};

std::vector<Eigen::Vector2d> sampleQuadrangleBoundaryForIntrusion(
  const Quadrangle & quadrangle, const double grid_resolution_m)
{
  std::vector<Eigen::Vector2d> samples;
  for (std::size_t index = 0U; index < quadrangle.size(); ++index) {
    const Eigen::Vector2d start =
      quadrangle[index].head<2>().cast<double>();
    const Eigen::Vector2d end =
      quadrangle[(index + 1U) % quadrangle.size()].head<2>().cast<double>();
    const Eigen::Vector2d delta = end - start;
    const double length = delta.norm();
    if (!std::isfinite(length) || length <= 0.0) {
      continue;
    }
    const int sample_count =
      static_cast<int>(length / grid_resolution_m + 1.0);
    const Eigen::Vector2d step =
      delta / length * grid_resolution_m;
    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
      samples.push_back(
        start + static_cast<double>(sample_index) * step);
    }
  }
  return samples;
}

bool sampledBoundaryEntersQuadrangle(
  const Quadrangle & quadrangle,
  const std::vector<Eigen::Vector2d> & sampled_boundary,
  const double minimum_edge_m)
{
  const Eigen::Vector2d origin =
    quadrangle[1].head<2>().cast<double>();
  Eigen::Vector2d first_axis =
    quadrangle[0].head<2>().cast<double>() - origin;
  Eigen::Vector2d second_axis =
    quadrangle[2].head<2>().cast<double>() - origin;
  const double first_length = first_axis.norm();
  const double second_length = second_axis.norm();
  if (!std::isfinite(first_length) || !std::isfinite(second_length) ||
    first_length <= minimum_edge_m || second_length <= minimum_edge_m)
  {
    return false;
  }
  first_axis /= first_length;
  second_axis /= second_length;

  for (const Eigen::Vector2d & point : sampled_boundary) {
    const Eigen::Vector2d local = point - origin;
    const double first_coordinate = first_axis.dot(local);
    const double second_coordinate = second_axis.dot(local);
    if (first_coordinate >= 0.0 && first_coordinate <= first_length &&
      second_coordinate >= 0.0 && second_coordinate <= second_length)
    {
      return true;
    }
  }
  return false;
}

struct RepairHypothesis
{
  Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
  Eigen::Vector2d x_axis{Eigen::Vector2d::UnitX()};
  Eigen::Vector2d y_axis{Eigen::Vector2d::UnitY()};
  double x_max{0.0};
  double x_min{0.0};
  double y_min{0.0};
  double y_max{0.0};
  std::size_t score{0U};
};

double computeRepairYNorm(
  const std::vector<Eigen::Vector2d> & local_boundary,
  const double reference_length)
{
  double minimum_y = 5.0;
  bool found = false;
  for (const Eigen::Vector2d & point : local_boundary) {
    if (point.x() < 0.0 || point.x() > reference_length ||
      point.y() < 0.0 || point.y() > 5.0)
    {
      continue;
    }
    minimum_y = std::min(minimum_y, point.y());
    found = true;
  }
  return found ? std::min(5.0, minimum_y) : 5.0;
}

std::pair<double, double> computeRepairXBounds(
  const std::vector<Eigen::Vector2d> & local_boundary,
  const double reference_length,
  const double y_norm)
{
  if (reference_length >= 10.0) {
    return {reference_length, 0.0};
  }
  std::vector<Eigen::Vector2d> y_filtered;
  for (const Eigen::Vector2d & point : local_boundary) {
    if (point.y() >= 0.0 && point.y() <= y_norm) {
      y_filtered.push_back(point);
    }
  }
  if (y_filtered.empty()) {
    const double half_length = reference_length * 0.5;
    return {half_length + 5.0, half_length - 5.0};
  }

  const double upper_limit = reference_length * 0.5 + 5.0;
  double x_max = upper_limit;
  for (const Eigen::Vector2d & point : y_filtered) {
    if (point.x() >= reference_length && point.x() <= upper_limit) {
      x_max = std::min(x_max, point.x());
    }
  }

  const double lower_limit = reference_length * 0.5 - 5.0;
  double x_min = lower_limit;
  for (const Eigen::Vector2d & point : y_filtered) {
    if (point.x() >= lower_limit && point.x() <= 0.0) {
      x_min = std::max(x_min, point.x());
    }
  }
  return {x_max, x_min};
}

std::size_t cutRepairHypothesisByContainedPoints(
  const std::vector<Eigen::Vector2d> & local_contained_points,
  const double y_norm,
  const bool expand_to_08,
  RepairHypothesis & hypothesis)
{
  std::vector<Eigen::Vector2d> retained;
  for (const Eigen::Vector2d & point : local_contained_points) {
    if (point.y() >= 0.0 && point.y() <= y_norm &&
      point.x() >= hypothesis.x_min && point.x() <= hypothesis.x_max)
    {
      retained.push_back(point);
    }
  }
  if (retained.empty()) {
    return 0U;
  }

  hypothesis.x_min = retained.front().x();
  hypothesis.x_max = retained.front().x();
  hypothesis.y_min = retained.front().y();
  hypothesis.y_max = retained.front().y();
  for (const Eigen::Vector2d & point : retained) {
    hypothesis.x_min = std::min(hypothesis.x_min, point.x());
    hypothesis.x_max = std::max(hypothesis.x_max, point.x());
    hypothesis.y_min = std::min(hypothesis.y_min, point.y());
    hypothesis.y_max = std::max(hypothesis.y_max, point.y());
  }

  if (expand_to_08) {
    const double width = hypothesis.x_max - hypothesis.x_min;
    const double height = hypothesis.y_max - hypothesis.y_min;
    if (width < height &&
      width >= 0.03 && width <= 0.4 &&
      height >= 0.3 && height < 0.8)
    {
      const double center = 0.5 * (hypothesis.y_min + hypothesis.y_max);
      hypothesis.y_min = center - 0.4;
      hypothesis.y_max = center + 0.4;
    } else if (width >= height &&
      height >= 0.03 && height <= 0.4 &&
      width >= 0.3 && width < 0.8)
    {
      const double center = 0.5 * (hypothesis.x_min + hypothesis.x_max);
      hypothesis.x_min = center - 0.4;
      hypothesis.x_max = center + 0.4;
    }
  }
  return retained.size();
}

Quadrangle repairHypothesisPoints(
  const RepairHypothesis & hypothesis,
  const float height,
  const double minimum_forward_extent_m)
{
  const double y_low = std::min(
    hypothesis.y_min,
    hypothesis.y_max - minimum_forward_extent_m);
  const auto world_point =
    [&](const double x, const double y) {
      const Eigen::Vector2d xy =
        hypothesis.origin + hypothesis.x_axis * x + hypothesis.y_axis * y;
      return Eigen::Vector3f(
        static_cast<float>(xy.x()),
        static_cast<float>(xy.y()),
        height);
    };
  return {{
    world_point(hypothesis.x_max, y_low),
    world_point(hypothesis.x_min, y_low),
    world_point(hypothesis.x_min, hypothesis.y_max),
    world_point(hypothesis.x_max, hypothesis.y_max),
  }};
}

void appendUniqueIndex(
  std::vector<std::size_t> & indices, const std::size_t index)
{
  if (std::find(indices.begin(), indices.end(), index) == indices.end()) {
    indices.push_back(index);
  }
}

double meanQuadrangleHeight(const IndexedQuadrangle & quadrangle)
{
  double height = 0.0;
  for (const Eigen::Vector3f & point : quadrangle.points) {
    height += static_cast<double>(point.z());
  }
  return height * 0.25;
}

struct DeletionRemapResult
{
  std::size_t deleted_count{0U};
  std::vector<std::size_t> deleted_source_candidate_indices;
};

DeletionRemapResult deleteQuadranglesAndRemapIndices(
  std::vector<IndexedQuadrangle> & quadrangles,
  std::vector<std::size_t> deletion_indices,
  const std::vector<std::vector<std::size_t> *> & tracked_index_vectors)
{
  DeletionRemapResult result;
  std::sort(deletion_indices.begin(), deletion_indices.end());
  deletion_indices.erase(
    std::unique(deletion_indices.begin(), deletion_indices.end()),
    deletion_indices.end());
  deletion_indices.erase(
    std::remove_if(
      deletion_indices.begin(), deletion_indices.end(),
      [&](const std::size_t index) {return index >= quadrangles.size();}),
    deletion_indices.end());
  if (deletion_indices.empty()) {
    return result;
  }

  constexpr std::size_t kDeletedIndex =
    std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> old_to_new(quadrangles.size(), kDeletedIndex);
  std::vector<IndexedQuadrangle> compacted;
  compacted.reserve(quadrangles.size() - deletion_indices.size());
  std::size_t deletion_position = 0U;
  for (std::size_t old_index = 0U; old_index < quadrangles.size();
    ++old_index)
  {
    if (deletion_position < deletion_indices.size() &&
      deletion_indices[deletion_position] == old_index)
    {
      result.deleted_source_candidate_indices.push_back(
        quadrangles[old_index].source_candidate_index);
      ++deletion_position;
      continue;
    }
    old_to_new[old_index] = compacted.size();
    compacted.push_back(std::move(quadrangles[old_index]));
  }
  quadrangles = std::move(compacted);
  result.deleted_count = deletion_indices.size();

  for (std::vector<std::size_t> * indices : tracked_index_vectors) {
    if (indices == nullptr) {
      continue;
    }
    std::vector<std::size_t> remapped;
    remapped.reserve(indices->size());
    for (const std::size_t old_index : *indices) {
      if (old_index < old_to_new.size() &&
        old_to_new[old_index] != kDeletedIndex)
      {
        remapped.push_back(old_to_new[old_index]);
      }
    }
    *indices = std::move(remapped);
  }
  return result;
}

}  // namespace

StairPoseEstimate estimateStairPose(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const float minimum_long_edge_m,
  const float minimum_short_edge_m,
  const float minimum_edge_m)
{
  StairPoseEstimate result;
  if (!finitePositive(minimum_long_edge_m) ||
    !finitePositive(minimum_short_edge_m) ||
    !finitePositive(minimum_edge_m))
  {
    result.diagnostic = "stair pose thresholds must be finite and positive";
    return result;
  }

  std::vector<const IndexedQuadrangle *> eligible;
  eligible.reserve(quadrangles.size());
  for (const IndexedQuadrangle & quadrangle : quadrangles) {
    const double first_length = edgeXY(
      quadrangle.points[0], quadrangle.points[1]).norm();
    const double second_length = edgeXY(
      quadrangle.points[1], quadrangle.points[2]).norm();
    if (!std::isfinite(first_length) || !std::isfinite(second_length)) {
      continue;
    }
    const double long_edge = std::max(first_length, second_length);
    const double short_edge = std::min(first_length, second_length);
    if (long_edge > minimum_long_edge_m && short_edge > minimum_short_edge_m) {
      eligible.push_back(&quadrangle);
    }
  }
  result.eligible_quadrangle_count = eligible.size();
  if (eligible.size() <= 1U) {
    result.diagnostic = "factory stair pose requires more than one eligible quadrangle";
    return result;
  }

  Eigen::MatrixXf coefficients(
    static_cast<Eigen::Index>(eligible.size() * 4U), 3);
  Eigen::VectorXf observations =
    Eigen::VectorXf::Constant(coefficients.rows(), -1.0F);
  Eigen::Index row = 0;
  for (const IndexedQuadrangle * quadrangle : eligible) {
    for (const Eigen::Vector3f & point : quadrangle->points) {
      coefficients.row(row++) = point.transpose();
    }
  }

  Eigen::Vector3f plane =
    coefficients.colPivHouseholderQr().solve(observations);
  if (!plane.allFinite()) {
    result.diagnostic = "stair pose least-squares solution is non-finite";
    return result;
  }
  if (plane.z() < 0.0F) {
    plane = -plane;
  }

  result.direction = Eigen::Vector2d(
    -static_cast<double>(plane.x() * plane.z()),
    -static_cast<double>(plane.y() * plane.z()));
  if (!normalize(result.direction, static_cast<double>(minimum_edge_m))) {
    result.direction.setZero();
    result.diagnostic = "stair pose horizontal gradient is degenerate";
    return result;
  }

  result.success = true;
  result.diagnostic = "factory least-squares stair pose recovered";
  return result;
}

PoseCorrectionResult correctQuadranglePoses(
  const std::vector<CandidateBlock> & candidates,
  const PoseCorrectionConfig & config)
{
  PoseCorrectionResult result;
  result.input_candidate_count = candidates.size();
  if (!finitePositive(config.minimum_long_edge_m) ||
    !finitePositive(config.minimum_short_edge_m) ||
    !finitePositive(config.consensus_cosine) ||
    config.consensus_cosine > 1.0F ||
    config.minimum_quadrangle_count == 0U ||
    !finitePositive(config.minimum_edge_m))
  {
    result.diagnostic = "invalid pose correction configuration";
    return result;
  }

  std::vector<IndexedQuadrangle> seeds;
  std::vector<const CandidateBlock *> seed_candidates;
  seeds.reserve(candidates.size());
  seed_candidates.reserve(candidates.size());
  for (std::size_t index = 0U; index < candidates.size(); ++index) {
    const CandidateTopRectangleResult top =
      candidateTopRectangle(candidates[index], config.minimum_edge_m);
    const std::vector<Eigen::Vector3f> & contained =
      candidates[index].contained_points.empty() ?
      candidates[index].hull : candidates[index].contained_points;
    if (!top.success || contained.size() < 3U ||
      !std::all_of(
        contained.begin(), contained.end(),
        [](const Eigen::Vector3f & point) {return point.allFinite();}))
    {
      ++result.rejected_candidate_count;
      continue;
    }
    IndexedQuadrangle seed;
    seed.source_candidate_index = index;
    seed.points = top.points;
    seeds.push_back(std::move(seed));
    seed_candidates.push_back(&candidates[index]);
  }
  result.valid_seed_count = seeds.size();

  const StairPoseEstimate stair_pose = estimateStairPose(
    seeds, config.minimum_long_edge_m, config.minimum_short_edge_m,
    config.minimum_edge_m);
  result.initial_stair_direction = stair_pose.direction;
  result.eligible_stair_pose_count = stair_pose.eligible_quadrangle_count;
  if (!stair_pose.success) {
    result.diagnostic = stair_pose.diagnostic;
    return result;
  }
  if (seeds.size() < config.minimum_quadrangle_count) {
    result.diagnostic = "factory pose correction minimum quadrangle count was not met";
    return result;
  }

  std::vector<Eigen::Vector2d> oriented_directions;
  oriented_directions.reserve(seeds.size());
  for (const IndexedQuadrangle & seed : seeds) {
    Eigen::Vector2d first = edgeXY(seed.points[0], seed.points[1]);
    Eigen::Vector2d second = edgeXY(seed.points[1], seed.points[2]);
    if (!normalize(first, config.minimum_edge_m) ||
      !normalize(second, config.minimum_edge_m))
    {
      oriented_directions.push_back(Eigen::Vector2d::Zero());
      continue;
    }
    Eigen::Vector2d selected =
      std::abs(first.dot(stair_pose.direction)) >=
      std::abs(second.dot(stair_pose.direction)) ? first : second;
    if (selected.dot(stair_pose.direction) < 0.0) {
      selected = -selected;
    }
    oriented_directions.push_back(selected);
  }

  Eigen::Vector2d average_direction = Eigen::Vector2d::Zero();
  for (const Eigen::Vector2d & direction : oriented_directions) {
    average_direction += direction;
  }
  average_direction /= static_cast<double>(oriented_directions.size());

  Eigen::Vector2d consensus = Eigen::Vector2d::Zero();
  for (const Eigen::Vector2d & direction : oriented_directions) {
    if (average_direction.dot(direction) > config.consensus_cosine) {
      consensus += direction;
      ++result.consensus_direction_count;
    }
  }
  if (result.consensus_direction_count <= 1U) {
    result.diagnostic = "factory pose correction direction consensus was not met";
    return result;
  }
  consensus /= static_cast<double>(result.consensus_direction_count);
  if (!normalize(consensus, config.minimum_edge_m)) {
    result.diagnostic = "factory pose correction consensus is degenerate";
    return result;
  }
  result.corrected_stair_direction = consensus;

  result.quadrangles.reserve(seeds.size());
  for (std::size_t index = 0U; index < seeds.size(); ++index) {
    result.quadrangles.push_back(
      correctedFromContainedPoints(*seed_candidates[index], seeds[index], consensus));
  }
  result.success = true;
  result.diagnostic =
    "computeStairPose and correctQuadPose prefix completed; later factory stages remain";
  return result;
}

CutByXResult cutQuadranglesByX(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const Eigen::Vector2d & stair_direction,
  const int target_index,
  const CutByXConfig & config)
{
  CutByXResult result;
  result.input_quadrangle_count = quadrangles.size();
  result.quadrangles = quadrangles;
  if (!finitePositive(config.minimum_remaining_depth_m) ||
    !finitePositive(config.step_m) ||
    !finitePositive(config.removed_point_ratio) ||
    config.maximum_removed_points == 0U ||
    !finitePositive(config.minimum_edge_m))
  {
    result.diagnostic = "invalid cutByX configuration";
    return result;
  }
  Eigen::Vector2d normalized_stair_direction = stair_direction;
  if (!normalize(
      normalized_stair_direction, static_cast<double>(config.minimum_edge_m)))
  {
    result.diagnostic = "cutByX stair direction is degenerate";
    return result;
  }
  if (target_index < -1 ||
    target_index >= static_cast<int>(quadrangles.size()))
  {
    result.diagnostic = "cutByX target index is outside the quadrangle vector";
    return result;
  }

  for (std::size_t index = 0U; index < result.quadrangles.size(); ++index) {
    if (target_index >= 0 && static_cast<int>(index) != target_index) {
      continue;
    }
    ++result.processed_quadrangle_count;
    IndexedQuadrangle & quadrangle = result.quadrangles[index];
    if (quadrangle.contained_points.empty()) {
      continue;
    }

    Quadrangle points = quadrangle.points;
    Eigen::Vector2d first = edgeXY(points[0], points[1]);
    Eigen::Vector2d second = edgeXY(points[1], points[2]);
    if (!normalize(first, config.minimum_edge_m) ||
      !normalize(second, config.minimum_edge_m))
    {
      continue;
    }
    if (std::abs(first.dot(normalized_stair_direction)) >=
      std::abs(second.dot(normalized_stair_direction)))
    {
      points = {{points[1], points[2], points[3], points[0]}};
    }

    std::vector<bool> keep(quadrangle.contained_points.size(), true);
    bool accepted_any_cut = false;
    for (int side = 0; side < 2; ++side) {
      Eigen::Vector3f & first_point = points[0];
      Eigen::Vector3f & second_point = points[1];
      const Eigen::Vector3f third_point = points[2];
      const Eigen::Vector3f fourth_point = points[3];
      const Eigen::Vector2d boundary =
        edgeXY(second_point, first_point);
      const double boundary_length = boundary.norm();
      const std::size_t maximum_removed = std::min(
        static_cast<std::size_t>(std::floor(
          boundary_length /
          static_cast<double>(config.minimum_remaining_depth_m) *
          static_cast<double>(config.removed_point_ratio))),
        config.maximum_removed_points);

      const Eigen::Vector3f depth_delta = fourth_point - first_point;
      double remaining_depth = depth_delta.head<2>().cast<double>().norm();
      if (remaining_depth > config.minimum_remaining_depth_m) {
        const Eigen::Vector3f step =
          depth_delta * static_cast<float>(
          static_cast<double>(config.step_m) / remaining_depth);
        while (remaining_depth > config.minimum_remaining_depth_m) {
          first_point += step;
          second_point += step;
          remaining_depth -= config.step_m;

          std::vector<std::size_t> removed_this_step;
          for (std::size_t point_index = 0U;
            point_index < quadrangle.contained_points.size(); ++point_index)
          {
            if (!keep[point_index]) {
              continue;
            }
            const Eigen::Vector2d delta =
              (quadrangle.contained_points[point_index] - first_point)
              .head<2>().cast<double>();
            if (
              boundary.y() * delta.x() - boundary.x() * delta.y() > 0.0)
            {
              keep[point_index] = false;
              removed_this_step.push_back(point_index);
            }
            if (removed_this_step.size() > maximum_removed) {
              break;
            }
          }
          if (removed_this_step.size() > maximum_removed) {
            for (const std::size_t removed_index : removed_this_step) {
              keep[removed_index] = true;
            }
            first_point -= step;
            second_point -= step;
            remaining_depth += config.step_m;
            break;
          }
          accepted_any_cut = true;
        }
      }
      points = {{third_point, fourth_point, first_point, second_point}};
    }

    if (!accepted_any_cut) {
      continue;
    }
    quadrangle.points = points;
    Eigen::Vector3f replacement = Eigen::Vector3f::Zero();
    for (std::size_t point_index = keep.size(); point_index > 0U; --point_index) {
      if (keep[point_index - 1U]) {
        replacement = quadrangle.contained_points[point_index - 1U];
        break;
      }
    }
    for (std::size_t point_index = 0U; point_index < keep.size(); ++point_index) {
      if (!keep[point_index]) {
        quadrangle.contained_points[point_index] = replacement;
      }
    }
    ++result.changed_quadrangle_count;
  }

  result.success = true;
  result.diagnostic =
    "factory cutByX two-sided 3 cm scan completed";
  return result;
}

Eigen::Vector2d computeQuadrangleDistanceVectorXY(
  const Quadrangle & first,
  const Quadrangle & second,
  const float boundary_sample_step_m,
  const float boundary_sample_count_epsilon)
{
  if (!finitePositive(boundary_sample_step_m) ||
    !std::isfinite(boundary_sample_count_epsilon) ||
    boundary_sample_count_epsilon < 0.0F)
  {
    return Eigen::Vector2d(100.0, 0.0);
  }
  const std::vector<Eigen::Vector2d> first_boundary =
    sampleQuadrangleBoundary(
    first, boundary_sample_step_m, boundary_sample_count_epsilon);
  const std::vector<Eigen::Vector2d> second_boundary =
    sampleQuadrangleBoundary(
    second, boundary_sample_step_m, boundary_sample_count_epsilon);
  if (sampledBoundaryIntrudes(first, second_boundary) ||
    sampledBoundaryIntrudes(second, first_boundary))
  {
    return Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d best(100.0, 0.0);
  double best_squared_distance = 10000.0;
  for (const Eigen::Vector2d & first_point : first_boundary) {
    for (const Eigen::Vector2d & second_point : second_boundary) {
      const Eigen::Vector2d candidate = first_point - second_point;
      const double squared_distance = candidate.squaredNorm();
      if (squared_distance < best_squared_distance) {
        best_squared_distance = squared_distance;
        best = candidate;
      }
    }
  }
  return best;
}

SameStairMergeResult mergeQuadranglesAtSameStair(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const Eigen::Vector2d & stair_direction,
  const SameStairMergeConfig & config)
{
  SameStairMergeResult result;
  result.input_quadrangle_count = quadrangles.size();
  if (!finitePositive(config.boundary_sample_step_m) ||
    !std::isfinite(config.boundary_sample_count_epsilon) ||
    config.boundary_sample_count_epsilon < 0.0F ||
    !finitePositive(config.maximum_height_difference_m) ||
    !finitePositive(config.direction_cosine) ||
    config.direction_cosine > 1.0F ||
    !finitePositive(config.maximum_distance_m) ||
    !finitePositive(config.maximum_along_stair_distance_m) ||
    !finitePositive(config.minimum_edge_m))
  {
    result.diagnostic = "invalid same-stair merge configuration";
    return result;
  }

  Eigen::Vector2d normalized_stair_direction = stair_direction;
  if (!normalize(
      normalized_stair_direction, static_cast<double>(config.minimum_edge_m)))
  {
    result.diagnostic = "same-stair merge direction is degenerate";
    return result;
  }

  std::vector<IndexedQuadrangle> merged = quadrangles;
  std::vector<bool> active(merged.size(), true);
  std::vector<Eigen::Vector2d> directions;
  std::vector<float> mean_heights;
  directions.reserve(merged.size());
  mean_heights.reserve(merged.size());
  for (IndexedQuadrangle & quadrangle : merged) {
    quadrangle.merge_count = 1U;
    directions.push_back(
      directionAlignedWithStair(
        quadrangle.points, normalized_stair_direction,
        config.minimum_edge_m));
    float mean_z = 0.0F;
    for (const Eigen::Vector3f & point : quadrangle.points) {
      mean_z += point.z();
    }
    mean_heights.push_back(mean_z * 0.25F);
  }

  for (std::size_t first_index = 0U;
    first_index < merged.size(); ++first_index)
  {
    if (!active[first_index] ||
      directions[first_index].squaredNorm() == 0.0)
    {
      continue;
    }
    for (std::size_t second_index = first_index + 1U;
      second_index < merged.size(); ++second_index)
    {
      if (!active[second_index] ||
        directions[second_index].squaredNorm() == 0.0)
      {
        continue;
      }
      if (std::abs(mean_heights[first_index] - mean_heights[second_index]) >
        config.maximum_height_difference_m)
      {
        continue;
      }
      if (std::abs(
          directions[first_index].dot(directions[second_index])) <
        config.direction_cosine)
      {
        continue;
      }
      const Eigen::Vector2d distance = computeQuadrangleDistanceVectorXY(
        merged[first_index].points, merged[second_index].points,
        config.boundary_sample_step_m,
        config.boundary_sample_count_epsilon);
      if (distance.norm() > config.maximum_distance_m ||
        std::abs(distance.dot(normalized_stair_direction)) >
        config.maximum_along_stair_distance_m)
      {
        continue;
      }

      IndexedQuadrangle combined = merged[first_index];
      combined.contained_points.insert(
        combined.contained_points.end(),
        merged[second_index].contained_points.begin(),
        merged[second_index].contained_points.end());
      const std::size_t first_count = merged[first_index].merge_count;
      const std::size_t second_count = merged[second_index].merge_count;
      const std::size_t combined_count = first_count + second_count;
      Eigen::Vector2d combined_direction =
        (directions[first_index] * static_cast<double>(first_count) +
        directions[second_index] * static_cast<double>(second_count)) /
        static_cast<double>(combined_count);
      if (!combined_direction.allFinite() ||
        combined_direction.norm() <= config.minimum_edge_m)
      {
        continue;
      }
      combined.merge_count = combined_count;
      const float fallback_mean_z =
        (mean_heights[first_index] * static_cast<float>(first_count) +
        mean_heights[second_index] * static_cast<float>(second_count)) /
        static_cast<float>(combined.merge_count);
      combined = quadrangleFromContainedPoints(
        combined, combined_direction, fallback_mean_z);

      merged[first_index] = std::move(combined);
      directions[first_index] = combined_direction;
      active[second_index] = false;
      ++result.merged_pair_count;
    }
  }

  result.quadrangles.reserve(
    merged.size() - result.merged_pair_count);
  for (std::size_t index = 0U; index < merged.size(); ++index) {
    if (active[index]) {
      result.quadrangles.push_back(std::move(merged[index]));
    }
  }
  for (std::size_t index = 0U; index < result.quadrangles.size(); ++index) {
    if (result.quadrangles[index].merge_count <= 1U) {
      continue;
    }
    const CutByXResult cut = cutQuadranglesByX(
      result.quadrangles, normalized_stair_direction,
      static_cast<int>(index));
    if (!cut.success) {
      result.diagnostic = "post-merge cutByX failed: " + cut.diagnostic;
      return result;
    }
    result.post_merge_cut_changed_count += cut.changed_quadrangle_count;
    result.quadrangles = cut.quadrangles;
  }
  result.output_quadrangle_count = result.quadrangles.size();
  result.success = true;
  result.diagnostic =
    "factory same-stair merge and post-merge cutByX completed";
  return result;
}

SideDistanceFilterResult ignoreUnnecessaryQuadrangles(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const Eigen::Vector2d & stair_direction,
  const float side_distance_threshold_m,
  const float minimum_edge_m)
{
  SideDistanceFilterResult result;
  result.input_quadrangle_count = quadrangles.size();
  if (!finitePositive(side_distance_threshold_m) ||
    !finitePositive(minimum_edge_m))
  {
    result.diagnostic = "side-distance thresholds must be finite and positive";
    return result;
  }

  Eigen::Vector2d original_axis = stair_direction;
  if (!normalize(original_axis, static_cast<double>(minimum_edge_m))) {
    result.diagnostic = "side-distance stair direction is degenerate";
    return result;
  }
  const Eigen::Vector2d perpendicular_axis(
    original_axis.y(), -original_axis.x());
  const std::vector<std::size_t> original_removals =
    sideDistanceOutlierIndices(
    quadrangles, original_axis,
    static_cast<double>(side_distance_threshold_m));
  const std::vector<std::size_t> perpendicular_removals =
    sideDistanceOutlierIndices(
    quadrangles, perpendicular_axis,
    static_cast<double>(side_distance_threshold_m));

  result.original_axis_removal_count = original_removals.size();
  result.perpendicular_axis_removal_count = perpendicular_removals.size();
  const bool use_perpendicular =
    original_removals.size() > perpendicular_removals.size();
  result.selected_stair_direction =
    use_perpendicular ? perpendicular_axis : original_axis;
  const std::vector<std::size_t> & removals =
    use_perpendicular ? perpendicular_removals : original_removals;

  std::vector<bool> remove(quadrangles.size(), false);
  for (const std::size_t index : removals) {
    remove[index] = true;
    result.removed_source_candidate_indices.push_back(
      quadrangles[index].source_candidate_index);
  }
  result.quadrangles.reserve(quadrangles.size() - removals.size());
  for (std::size_t index = 0U; index < quadrangles.size(); ++index) {
    if (!remove[index]) {
      result.quadrangles.push_back(quadrangles[index]);
    }
  }

  result.success = true;
  result.diagnostic =
    use_perpendicular ?
    "factory side-distance filter selected the perpendicular stair axis" :
    "factory side-distance filter retained the original stair axis";
  return result;
}

IntrusionClassificationResult classifyQuadrangleIntrusions(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const float grid_resolution_m,
  const float maximum_primary_short_edge_m,
  const float minimum_primary_long_edge_m,
  const float minimum_edge_m)
{
  IntrusionClassificationResult result;
  result.input_quadrangle_count = quadrangles.size();
  if (!finitePositive(grid_resolution_m) ||
    !finitePositive(maximum_primary_short_edge_m) ||
    !finitePositive(minimum_primary_long_edge_m) ||
    !finitePositive(minimum_edge_m))
  {
    result.diagnostic =
      "intrusion classification thresholds must be finite and positive";
    return result;
  }

  std::vector<VectorizedQuadrangle> records;
  records.reserve(quadrangles.size());
  for (std::size_t index = 0U; index < quadrangles.size(); ++index) {
    const double first_length =
      edgeXY(quadrangles[index].points[0], quadrangles[index].points[1]).norm();
    const double second_length =
      edgeXY(quadrangles[index].points[1], quadrangles[index].points[2]).norm();
    if (!std::isfinite(first_length) || !std::isfinite(second_length) ||
      first_length <= minimum_edge_m || second_length <= minimum_edge_m)
    {
      result.diagnostic =
        "intrusion classification encountered a degenerate quadrangle";
      return result;
    }
    VectorizedQuadrangle record;
    record.vector_index = index;
    record.short_edge_m = std::min(first_length, second_length);
    record.long_edge_m = std::max(first_length, second_length);
    record.sampled_boundary = sampleQuadrangleBoundaryForIntrusion(
      quadrangles[index].points, grid_resolution_m);
    records.push_back(std::move(record));
  }

  std::sort(
    records.begin(), records.end(),
    [](const VectorizedQuadrangle & first,
      const VectorizedQuadrangle & second)
    {
      return first.short_edge_m < second.short_edge_m;
    });

  std::vector<VectorizedQuadrangle> primary;
  std::vector<VectorizedQuadrangle> deferred;
  primary.reserve(records.size());
  deferred.reserve(records.size());
  for (VectorizedQuadrangle & record : records) {
    if (record.short_edge_m > maximum_primary_short_edge_m ||
      record.long_edge_m < minimum_primary_long_edge_m)
    {
      deferred.push_back(std::move(record));
    } else {
      primary.push_back(std::move(record));
    }
  }
  records.clear();
  records.reserve(primary.size() + deferred.size());
  records.insert(
    records.end(),
    std::make_move_iterator(primary.begin()),
    std::make_move_iterator(primary.end()));
  records.insert(
    records.end(),
    std::make_move_iterator(deferred.begin()),
    std::make_move_iterator(deferred.end()));

  result.sorted_vector_indices.reserve(records.size());
  for (const VectorizedQuadrangle & record : records) {
    result.sorted_vector_indices.push_back(record.vector_index);
  }
  if (!records.empty()) {
    result.non_intruding_vector_indices.push_back(records.front().vector_index);
  }
  for (std::size_t current_position = 1U;
    current_position < records.size(); ++current_position)
  {
    const VectorizedQuadrangle & current = records[current_position];
    bool intruded = false;
    for (std::size_t previous_position = 0U;
      previous_position < current_position; ++previous_position)
    {
      if (sampledBoundaryEntersQuadrangle(
          quadrangles[current.vector_index].points,
          records[previous_position].sampled_boundary,
          minimum_edge_m))
      {
        intruded = true;
        break;
      }
    }
    if (!intruded) {
      result.non_intruding_vector_indices.push_back(current.vector_index);
    } else if (current.short_edge_m > maximum_primary_short_edge_m) {
      result.intruded_long_edge_vector_indices.push_back(current.vector_index);
    } else {
      result.intruded_short_edge_vector_indices.push_back(current.vector_index);
    }
  }

  result.success = true;
  result.diagnostic =
    "factory pcl2vectors, sortVectors, and configIntrusions stages completed";
  return result;
}

double computeQuadrangleReferenceDistanceM(
  const Quadrangle & first,
  const Quadrangle & second,
  const float sample_step_m,
  const float sample_count_epsilon)
{
  if (!finitePositive(sample_step_m) ||
    !std::isfinite(sample_count_epsilon) ||
    sample_count_epsilon < 0.0F)
  {
    return 100.0;
  }
  const std::vector<Eigen::Vector2d> first_boundary =
    sampleQuadrangleBoundary(first, sample_step_m, sample_count_epsilon);
  const std::vector<Eigen::Vector2d> second_boundary =
    sampleQuadrangleBoundary(second, sample_step_m, sample_count_epsilon);
  if (first_boundary.empty() || second_boundary.empty()) {
    return 100.0;
  }

  double minimum_squared_distance = 10000.0;
  for (const Eigen::Vector2d & first_point : first_boundary) {
    for (const Eigen::Vector2d & second_point : second_boundary) {
      minimum_squared_distance = std::min(
        minimum_squared_distance,
        (first_point - second_point).squaredNorm());
    }
  }
  return std::sqrt(minimum_squared_distance);
}

ReferenceSelectionResult selectQuadrangleReference(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const std::vector<std::size_t> & reference_vector_indices,
  const std::size_t target_vector_index,
  const float grid_resolution_m,
  const Eigen::Vector2d & grid_map_position_xy,
  const bool fallback,
  const float maximum_candidate_distance_m,
  const float near_distance_m,
  const float minimum_edge_m)
{
  ReferenceSelectionResult result;
  result.updated_reference_vector_indices = reference_vector_indices;
  if (target_vector_index >= quadrangles.size()) {
    result.diagnostic = "reference target index is outside the quadrangle vector";
    return result;
  }
  if (!finitePositive(grid_resolution_m) ||
    !finitePositive(maximum_candidate_distance_m) ||
    !finitePositive(near_distance_m) ||
    !finitePositive(minimum_edge_m) ||
    !grid_map_position_xy.allFinite())
  {
    result.diagnostic =
      "reference selection thresholds and map position must be finite";
    return result;
  }
  for (const std::size_t index : reference_vector_indices) {
    if (index >= quadrangles.size()) {
      result.diagnostic = "reference index is outside the quadrangle vector";
      return result;
    }
  }

  std::vector<float> heights;
  std::vector<std::vector<Eigen::Vector2d>> boundaries;
  heights.reserve(quadrangles.size());
  boundaries.reserve(quadrangles.size());
  for (const IndexedQuadrangle & quadrangle : quadrangles) {
    float height = 0.0F;
    for (const Eigen::Vector3f & point : quadrangle.points) {
      if (!point.allFinite()) {
        result.diagnostic = "reference quadrangle contains a non-finite point";
        return result;
      }
      height += point.z();
    }
    heights.push_back(height * 0.25F);
    boundaries.push_back(
      sampleQuadrangleBoundaryForIntrusion(
        quadrangle.points, grid_resolution_m));
    if (boundaries.back().empty()) {
      result.diagnostic = "reference quadrangle has an empty sampled boundary";
      return result;
    }
  }

  std::vector<std::pair<std::size_t, float>> lower;
  std::vector<std::pair<std::size_t, float>> upper_or_equal;
  for (const std::size_t reference : reference_vector_indices) {
    if (heights[reference] < heights[target_vector_index]) {
      lower.emplace_back(reference, heights[reference]);
    } else {
      upper_or_equal.emplace_back(reference, heights[reference]);
    }
  }
  std::sort(
    lower.begin(), lower.end(),
    [](const auto & first, const auto & second) {
      return first.second > second.second;
    });
  std::sort(
    upper_or_equal.begin(), upper_or_equal.end(),
    [](const auto & first, const auto & second) {
      return first.second < second.second;
    });

  const auto intrudes =
    [&](const std::size_t rectangle, const std::size_t sampled_boundary) {
      return sampledBoundaryEntersQuadrangle(
        quadrangles[rectangle].points, boundaries[sampled_boundary],
        minimum_edge_m);
    };
  const auto related =
    [&](const std::size_t reference) {
      return intrudes(target_vector_index, reference) ||
             intrudes(reference, target_vector_index) ||
             computeQuadrangleReferenceDistanceM(
        quadrangles[reference].points,
        quadrangles[target_vector_index].points) <=
             maximum_candidate_distance_m;
    };
  const auto first_related =
    [&](const std::vector<std::pair<std::size_t, float>> & candidates) {
      for (const auto & candidate : candidates) {
        if (related(candidate.first)) {
          return static_cast<int>(candidate.first);
        }
      }
      return -1;
    };

  const int lower_reference = first_related(lower);
  const int upper_reference = first_related(upper_or_equal);
  if (lower_reference < 0 && upper_reference < 0) {
    result.updated_reference_vector_indices.push_back(target_vector_index);
    result.target_added_to_reference_indices = true;
    result.success = true;
    result.diagnostic =
      "target had no related reference and was promoted to the reference set";
    return result;
  }
  if (lower_reference < 0 || upper_reference < 0) {
    result.selected_reference_vector_index =
      lower_reference >= 0 ? lower_reference : upper_reference;
    result.success = true;
    result.diagnostic = "single-sided reference selected";
    return result;
  }

  const bool lower_intrudes = intrudes(
    target_vector_index, static_cast<std::size_t>(lower_reference));
  const bool upper_intrudes = intrudes(
    target_vector_index, static_cast<std::size_t>(upper_reference));
  if (!lower_intrudes && !upper_intrudes && !fallback) {
    result.deferred_for_fallback = true;
    result.success = true;
    result.diagnostic = "two-sided non-intruding target deferred for fallback";
    return result;
  }

  const double lower_distance =
    lower_intrudes ? 0.0 : computeQuadrangleReferenceDistanceM(
    quadrangles[static_cast<std::size_t>(lower_reference)].points,
    quadrangles[target_vector_index].points);
  const double upper_distance =
    upper_intrudes ? 0.0 : computeQuadrangleReferenceDistanceM(
    quadrangles[static_cast<std::size_t>(upper_reference)].points,
    quadrangles[target_vector_index].points);
  const bool lower_near = lower_distance < near_distance_m;
  const bool upper_near = upper_distance < near_distance_m;
  if (lower_near != upper_near) {
    result.selected_reference_vector_index =
      lower_near ? lower_reference : upper_reference;
    result.success = true;
    result.diagnostic = "only one two-sided reference met the near threshold";
    return result;
  }
  if (!lower_near) {
    result.selected_reference_vector_index =
      upper_distance <= lower_distance ? upper_reference : lower_reference;
    result.success = true;
    result.diagnostic = "two distant references resolved by shortest distance";
    return result;
  }

  const std::array<int, 2> references{{lower_reference, upper_reference}};
  int best_reference = -1;
  std::size_t best_boundary_index = 0U;
  double best_squared_distance = 10000.0;
  for (const int reference : references) {
    const std::vector<Eigen::Vector2d> & boundary =
      boundaries[static_cast<std::size_t>(reference)];
    std::size_t nearest_index = 0U;
    double nearest_squared_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < boundary.size(); ++index) {
      const double squared_distance =
        (boundary[index] - grid_map_position_xy).squaredNorm();
      if (squared_distance < nearest_squared_distance) {
        nearest_squared_distance = squared_distance;
        nearest_index = index;
      }
    }
    if (nearest_squared_distance < best_squared_distance) {
      best_squared_distance = nearest_squared_distance;
      best_reference = reference;
      best_boundary_index = nearest_index;
    }
    const Eigen::Vector2d & nearest =
      boundaries[static_cast<std::size_t>(best_reference)][best_boundary_index];
    const float reference_height =
      heights[static_cast<std::size_t>(best_reference)];
    result.constraint_points.emplace_back(
      static_cast<float>(nearest.x()),
      static_cast<float>(nearest.y()),
      reference_height);
    result.constraint_points.emplace_back(
      static_cast<float>(grid_map_position_xy.x()),
      static_cast<float>(grid_map_position_xy.y()),
      reference_height);
  }
  result.selected_reference_vector_index = best_reference;
  result.success = true;
  result.diagnostic =
    "two near references resolved by GridMap position with factory constraints";
  return result;
}

QuadrangleUpdateResult computeAndUpdateQuadranglesFromReference(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const std::vector<std::size_t> & active_reference_vector_indices,
  const std::size_t destination_vector_index,
  const std::size_t store_vector_index,
  const std::size_t reference_vector_index,
  const float grid_resolution_m,
  const bool emit_all,
  const bool expand_to_08,
  const float minimum_output_extent_m,
  const float minimum_forward_extent_m,
  const float minimum_edge_m)
{
  QuadrangleUpdateResult result;
  result.quadrangles = quadrangles;
  if (destination_vector_index >= quadrangles.size() ||
    reference_vector_index >= quadrangles.size())
  {
    result.diagnostic = "quadrangle update index is outside the vector";
    return result;
  }
  if (store_vector_index != destination_vector_index &&
    store_vector_index != quadrangles.size())
  {
    result.diagnostic =
      "append store index must equal the current quadrangle count";
    return result;
  }
  if (!finitePositive(grid_resolution_m) ||
    !finitePositive(minimum_output_extent_m) ||
    !finitePositive(minimum_forward_extent_m) ||
    !finitePositive(minimum_edge_m))
  {
    result.diagnostic = "quadrangle update thresholds must be finite and positive";
    return result;
  }
  for (const std::size_t index : active_reference_vector_indices) {
    if (index >= quadrangles.size()) {
      result.diagnostic = "active reference index is outside the vector";
      return result;
    }
  }
  const IndexedQuadrangle & destination =
    quadrangles[destination_vector_index];
  if (destination.contained_points.empty()) {
    result.diagnostic = "destination contained-point cloud is empty";
    if (!emit_all && store_vector_index == destination_vector_index) {
      result.error_vector_indices.push_back(destination_vector_index);
    } else if (emit_all) {
      result.failed_reference_vector_indices.push_back(reference_vector_index);
    }
    return result;
  }
  if (!std::all_of(
      destination.contained_points.begin(),
      destination.contained_points.end(),
      [](const Eigen::Vector3f & point) {return point.allFinite();}))
  {
    result.diagnostic = "destination contained-point cloud is non-finite";
    return result;
  }

  std::vector<Eigen::Vector2d> world_boundary;
  for (const std::size_t index : active_reference_vector_indices) {
    if (index == reference_vector_index) {
      continue;
    }
    const std::vector<Eigen::Vector2d> boundary =
      sampleQuadrangleBoundaryForIntrusion(
      quadrangles[index].points, grid_resolution_m);
    world_boundary.insert(
      world_boundary.end(), boundary.begin(), boundary.end());
  }

  const Quadrangle & reference = quadrangles[reference_vector_index].points;
  Quadrangle ordered_reference;
  if (edgeXY(reference[1], reference[2]).norm() >
    edgeXY(reference[0], reference[1]).norm())
  {
    ordered_reference = {{
      reference[0], reference[3], reference[2], reference[1]}};
  } else {
    ordered_reference = {{
      reference[1], reference[0], reference[3], reference[2]}};
  }

  std::array<RepairHypothesis, 4> hypotheses;
  for (std::size_t index = 0U; index < hypotheses.size(); ++index) {
    const Eigen::Vector2d first =
      ordered_reference[index].head<2>().cast<double>();
    const Eigen::Vector2d second =
      ordered_reference[(index + 1U) % ordered_reference.size()]
      .head<2>().cast<double>();
    Eigen::Vector2d x_axis = first - second;
    const double reference_length = x_axis.norm();
    if (!normalize(x_axis, minimum_edge_m)) {
      result.diagnostic = "reference quadrangle contains a degenerate edge";
      return result;
    }
    const Eigen::Vector2d y_axis(-x_axis.y(), x_axis.x());

    RepairHypothesis & hypothesis = hypotheses[index];
    hypothesis.origin = second;
    hypothesis.x_axis = x_axis;
    hypothesis.y_axis = y_axis;
    std::vector<Eigen::Vector2d> local_boundary;
    local_boundary.reserve(world_boundary.size());
    for (const Eigen::Vector2d & point : world_boundary) {
      const Eigen::Vector2d relative = point - hypothesis.origin;
      local_boundary.emplace_back(
        hypothesis.x_axis.dot(relative),
        hypothesis.y_axis.dot(relative));
    }
    const double y_norm = computeRepairYNorm(
      local_boundary, reference_length);
    const auto x_bounds = computeRepairXBounds(
      local_boundary, reference_length, y_norm);
    hypothesis.x_max = x_bounds.first;
    hypothesis.x_min = x_bounds.second;
    hypothesis.y_min = 0.0;
    hypothesis.y_max = y_norm;

    std::vector<Eigen::Vector2d> local_contained_points;
    local_contained_points.reserve(destination.contained_points.size());
    for (const Eigen::Vector3f & point : destination.contained_points) {
      const Eigen::Vector2d relative =
        point.head<2>().cast<double>() - hypothesis.origin;
      local_contained_points.emplace_back(
        hypothesis.x_axis.dot(relative),
        hypothesis.y_axis.dot(relative));
    }
    hypothesis.score = cutRepairHypothesisByContainedPoints(
      local_contained_points, y_norm, expand_to_08, hypothesis);
    result.hypothesis_scores[index] = hypothesis.score;
  }

  float destination_height = 0.0F;
  for (const Eigen::Vector3f & point : destination.points) {
    destination_height += point.z();
  }
  destination_height *= 0.25F;
  const IndexedQuadrangle destination_template = destination;
  const auto commit =
    [&](const std::size_t hypothesis_index,
      const std::size_t commit_index,
      QuadrangleUpdateResult & output) {
      IndexedQuadrangle repaired = destination_template;
      repaired.points = repairHypothesisPoints(
        hypotheses[hypothesis_index], destination_height,
        minimum_forward_extent_m);
      repaired.merge_count = 1U;
      if (commit_index == destination_vector_index) {
        output.quadrangles[commit_index] = std::move(repaired);
      } else {
        output.quadrangles.push_back(std::move(repaired));
      }
      output.registered_vector_indices.push_back(commit_index);
      ++output.emitted_quadrangle_count;
    };

  if (!emit_all) {
    std::size_t best_index = 0U;
    for (std::size_t index = 1U; index < hypotheses.size(); ++index) {
      if (hypotheses[index].score > hypotheses[best_index].score) {
        best_index = index;
      }
    }
    if (hypotheses[best_index].score == 0U) {
      if (destination_vector_index == store_vector_index) {
        result.error_vector_indices.push_back(destination_vector_index);
      }
      result.diagnostic =
        "all reference-based quadrangle hypotheses were empty";
      return result;
    }
    commit(best_index, store_vector_index, result);
    result.success = true;
    result.diagnostic = "earliest maximum-scoring quadrangle hypothesis committed";
    return result;
  }

  for (std::size_t index = 0U; index < hypotheses.size(); ++index) {
    const RepairHypothesis & hypothesis = hypotheses[index];
    if (hypothesis.score == 0U ||
      std::abs(hypothesis.x_max - hypothesis.x_min) <
      minimum_output_extent_m ||
      std::abs(hypothesis.y_max - hypothesis.y_min) <
      minimum_output_extent_m)
    {
      continue;
    }
    const std::size_t commit_index =
      result.emitted_quadrangle_count == 0U &&
      store_vector_index == destination_vector_index ?
      store_vector_index : result.quadrangles.size();
    commit(index, commit_index, result);
  }
  if (result.emitted_quadrangle_count == 0U) {
    result.failed_reference_vector_indices.push_back(reference_vector_index);
    result.diagnostic =
      "no emit-all quadrangle hypothesis met the 8 cm extent gate";
    return result;
  }

  result.success = true;
  result.diagnostic = "all valid reference-based quadrangle hypotheses committed";
  return result;
}

FactoryFinalSuffixResult runFactoryFinalSuffix(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const FactoryFinalSuffixConfig & config)
{
  FactoryFinalSuffixResult result;
  result.input_quadrangle_count = quadrangles.size();
  if (!finitePositive(config.grid_resolution_m) ||
    !config.grid_map_position_xy.allFinite() ||
    !finitePositive(config.wide_initial_height_tolerance_m) ||
    !finitePositive(config.wide_height_tolerance_step_m) ||
    !finitePositive(config.wide_maximum_height_tolerance_m) ||
    config.wide_initial_height_tolerance_m >
    config.wide_maximum_height_tolerance_m ||
    !std::isfinite(config.degenerate_first_edge_squared_m2) ||
    config.degenerate_first_edge_squared_m2 <= 0.0)
  {
    result.diagnostic = "invalid factory final suffix configuration";
    return result;
  }
  if (config.enable_pitch_gated_is_error_quad) {
    result.diagnostic =
      "pitch-gated isErrorQuad is unsupported until its factory predicate "
      "is fully evidenced";
    return result;
  }

  std::vector<IndexedQuadrangle> working = quadrangles;
  result.classification = classifyQuadrangleIntrusions(
    working, config.grid_resolution_m);
  if (!result.classification.success) {
    result.diagnostic = result.classification.diagnostic;
    return result;
  }

  std::vector<std::size_t> active_reference_indices =
    result.classification.non_intruding_vector_indices;
  std::vector<std::size_t> deferred_indices;
  std::vector<std::size_t> successful_reference_indices;
  std::vector<std::size_t> repaired_target_indices;
  std::vector<std::size_t> wide_considered_reference_indices;
  std::vector<std::size_t> wide_failed_reference_indices;
  std::vector<std::size_t> staged_promoted_indices;
  std::vector<std::size_t> error_indices;

  const auto process_narrow_target =
    [&](const std::size_t target_index, const bool fallback) {
      const ReferenceSelectionResult reference = selectQuadrangleReference(
        working, active_reference_indices, target_index,
        config.grid_resolution_m, config.grid_map_position_xy, fallback);
      if (!reference.success) {
        result.diagnostic = reference.diagnostic;
        return false;
      }
      active_reference_indices = reference.updated_reference_vector_indices;
      if (reference.deferred_for_fallback) {
        if (fallback) {
          result.diagnostic =
            "factory fallback unexpectedly deferred a narrow target";
          return false;
        }
        deferred_indices.push_back(target_index);
        return true;
      }
      if (reference.selected_reference_vector_index < 0) {
        return true;
      }

      if (!fallback) {
        ++result.narrow_normal_attempt_count;
      }
      const std::size_t reference_index = static_cast<std::size_t>(
        reference.selected_reference_vector_index);
      QuadrangleUpdateResult update =
        computeAndUpdateQuadranglesFromReference(
        working, active_reference_indices, target_index, target_index,
        reference_index, config.grid_resolution_m, false);
      working = std::move(update.quadrangles);
      if (!update.success) {
        if (update.error_vector_indices.empty()) {
          result.diagnostic = update.diagnostic;
          return false;
        }
        error_indices.insert(
          error_indices.end(),
          update.error_vector_indices.begin(),
          update.error_vector_indices.end());
        return true;
      }

      for (const std::size_t registered_index :
        update.registered_vector_indices)
      {
        appendUniqueIndex(active_reference_indices, registered_index);
      }
      successful_reference_indices.push_back(reference_index);
      repaired_target_indices.push_back(target_index);
      if (fallback) {
        ++result.fallback_success_count;
      } else {
        ++result.narrow_normal_success_count;
      }
      return true;
    };

  for (const std::size_t target_index :
    result.classification.intruded_short_edge_vector_indices)
  {
    if (!process_narrow_target(target_index, false)) {
      return result;
    }
  }

  const std::vector<std::size_t> fallback_targets = deferred_indices;
  result.fallback_target_count = fallback_targets.size();
  for (const std::size_t target_index : fallback_targets) {
    if (!process_narrow_target(target_index, true)) {
      return result;
    }
  }

  result.wide_target_count =
    result.classification.intruded_long_edge_vector_indices.size();
  for (const std::size_t target_index :
    result.classification.intruded_long_edge_vector_indices)
  {
    if (target_index >= working.size()) {
      result.diagnostic =
        "wide target index is outside the quadrangle vector";
      return result;
    }
    const double target_height = meanQuadrangleHeight(working[target_index]);
    if (!std::isfinite(target_height)) {
      result.diagnostic = "wide target height is non-finite";
      return result;
    }

    bool target_succeeded = false;
    std::vector<std::size_t> staged_indices;
    for (double tolerance =
      static_cast<double>(config.wide_initial_height_tolerance_m);
      tolerance <=
      static_cast<double>(config.wide_maximum_height_tolerance_m) + 1.0e-9;
      tolerance +=
      static_cast<double>(config.wide_height_tolerance_step_m))
    {
      std::vector<std::size_t> eligible_references;
      for (const std::size_t reference_index : active_reference_indices) {
        if (reference_index == target_index ||
          reference_index >= working.size())
        {
          continue;
        }
        const double reference_height =
          meanQuadrangleHeight(working[reference_index]);
        if (!std::isfinite(reference_height)) {
          result.diagnostic = "wide reference height is non-finite";
          return result;
        }
        // The recovered AArch64 comparison accepts a reference only when its
        // height difference is strictly below the current tolerance.
        if (std::abs(reference_height - target_height) < tolerance) {
          eligible_references.push_back(reference_index);
        }
      }

      for (const std::size_t reference_index : eligible_references) {
        ++result.wide_reference_attempt_count;
        wide_considered_reference_indices.push_back(reference_index);
        const std::size_t store_index =
          target_succeeded ? working.size() : target_index;
        QuadrangleUpdateResult update =
          computeAndUpdateQuadranglesFromReference(
          working, active_reference_indices, target_index, store_index,
          reference_index, config.grid_resolution_m, true);
        working = std::move(update.quadrangles);
        if (!update.success) {
          if (update.failed_reference_vector_indices.empty()) {
            result.diagnostic = update.diagnostic;
            return result;
          }
          wide_failed_reference_indices.insert(
            wide_failed_reference_indices.end(),
            update.failed_reference_vector_indices.begin(),
            update.failed_reference_vector_indices.end());
          continue;
        }

        target_succeeded = true;
        successful_reference_indices.push_back(reference_index);
        staged_indices.insert(
          staged_indices.end(),
          update.registered_vector_indices.begin(),
          update.registered_vector_indices.end());
      }
      if (target_succeeded) {
        break;
      }
    }

    if (!target_succeeded) {
      error_indices.push_back(target_index);
      continue;
    }
    ++result.wide_successful_target_count;
    for (const std::size_t staged_index : staged_indices) {
      appendUniqueIndex(active_reference_indices, staged_index);
      staged_promoted_indices.push_back(staged_index);
    }
    result.staged_promotion_count += staged_indices.size();
  }

  const std::vector<std::vector<std::size_t> *> tracked_indices{
    &active_reference_indices,
    &deferred_indices,
    &successful_reference_indices,
    &repaired_target_indices,
    &wide_considered_reference_indices,
    &wide_failed_reference_indices,
    &staged_promoted_indices,
  };
  const DeletionRemapResult error_deletion =
    deleteQuadranglesAndRemapIndices(
    working, error_indices, tracked_indices);
  result.deleted_error_quadrangle_count = error_deletion.deleted_count;
  result.deleted_source_candidate_indices.insert(
    result.deleted_source_candidate_indices.end(),
    error_deletion.deleted_source_candidate_indices.begin(),
    error_deletion.deleted_source_candidate_indices.end());

  std::vector<std::size_t> degenerate_indices;
  for (const std::size_t active_index : active_reference_indices) {
    if (active_index >= working.size()) {
      result.diagnostic =
        "active reference index is outside the compacted quadrangle vector";
      return result;
    }
    const double first_edge_squared =
      edgeXY(
      working[active_index].points[0],
      working[active_index].points[1]).squaredNorm();
    if (first_edge_squared <
      config.degenerate_first_edge_squared_m2)
    {
      appendUniqueIndex(degenerate_indices, active_index);
    }
  }
  const DeletionRemapResult degenerate_deletion =
    deleteQuadranglesAndRemapIndices(
    working, degenerate_indices, tracked_indices);
  result.deleted_degenerate_quadrangle_count =
    degenerate_deletion.deleted_count;
  result.deleted_source_candidate_indices.insert(
    result.deleted_source_candidate_indices.end(),
    degenerate_deletion.deleted_source_candidate_indices.begin(),
    degenerate_deletion.deleted_source_candidate_indices.end());

  result.active_reference_vector_indices = active_reference_indices;
  result.deferred_vector_indices = deferred_indices;
  result.successful_reference_vector_indices =
    successful_reference_indices;
  result.repaired_target_vector_indices = repaired_target_indices;
  result.wide_considered_reference_vector_indices =
    wide_considered_reference_indices;
  result.wide_failed_reference_vector_indices =
    wide_failed_reference_indices;
  result.staged_promoted_vector_indices = staged_promoted_indices;
  result.staged_tail_count = staged_promoted_indices.size();
  result.working_quadrangles = working;
  result.final_quadrangles.reserve(active_reference_indices.size());
  for (const std::size_t active_index : active_reference_indices) {
    if (active_index >= working.size()) {
      result.diagnostic =
        "final active reference index is outside the quadrangle vector";
      return result;
    }
    result.final_quadrangles.push_back(working[active_index]);
  }

  result.success = true;
  result.diagnostic =
    "factory final suffix completed with pitch-gated isErrorQuad disabled";
  return result;
}

}  // namespace x30_plane_seg_core
