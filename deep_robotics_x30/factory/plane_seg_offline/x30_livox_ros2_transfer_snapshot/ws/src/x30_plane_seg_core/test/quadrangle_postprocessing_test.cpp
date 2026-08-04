#include "x30_plane_seg_core/quadrangle_postprocessing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

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

x30_plane_seg_core::IndexedQuadrangle slopedQuadrangle(
  const std::size_t index, const float minimum_x)
{
  const auto point = [](const float x, const float y) {
      return Eigen::Vector3f(x, y, -0.5F * x - 0.5F);
    };
  x30_plane_seg_core::IndexedQuadrangle quadrangle;
  quadrangle.source_candidate_index = index;
  quadrangle.points = {{
    point(minimum_x, -0.5F),
    point(minimum_x, 0.5F),
    point(minimum_x + 0.3F, 0.5F),
    point(minimum_x + 0.3F, -0.5F),
  }};
  return quadrangle;
}

x30_plane_seg_core::CandidateBlock stairCandidate(
  const float center_x, const float top_z)
{
  x30_plane_seg_core::CandidateBlock candidate;
  candidate.size = Eigen::Vector3f(0.3F, 1.0F, 0.2F);
  candidate.pose = Eigen::Isometry3f::Identity();
  candidate.pose.translation() = Eigen::Vector3f(center_x, 0.0F, top_z - 0.1F);
  candidate.hull = {
    Eigen::Vector3f(center_x - 0.14F, -0.48F, top_z),
    Eigen::Vector3f(center_x - 0.12F, 0.49F, top_z),
    Eigen::Vector3f(center_x + 0.13F, 0.47F, top_z),
    Eigen::Vector3f(center_x + 0.14F, -0.46F, top_z),
  };
  candidate.contained_points = candidate.hull;
  return candidate;
}

x30_plane_seg_core::IndexedQuadrangle flatQuadrangle(
  const std::size_t index, const float center_x, const float center_y)
{
  x30_plane_seg_core::IndexedQuadrangle quadrangle;
  quadrangle.source_candidate_index = index;
  quadrangle.points = {{
    Eigen::Vector3f(center_x - 0.1F, center_y - 0.1F, 0.0F),
    Eigen::Vector3f(center_x - 0.1F, center_y + 0.1F, 0.0F),
    Eigen::Vector3f(center_x + 0.1F, center_y + 0.1F, 0.0F),
    Eigen::Vector3f(center_x + 0.1F, center_y - 0.1F, 0.0F),
  }};
  quadrangle.contained_points.assign(
    quadrangle.points.begin(), quadrangle.points.end());
  quadrangle.contained_points.push_back(
    Eigen::Vector3f(center_x, center_y, 0.0F));
  return quadrangle;
}

x30_plane_seg_core::IndexedQuadrangle terrainQuadrangle(
  const std::size_t index,
  const float minimum_x,
  const float maximum_x,
  const float minimum_y,
  const float maximum_y,
  const float height)
{
  x30_plane_seg_core::IndexedQuadrangle quadrangle;
  quadrangle.source_candidate_index = index;
  quadrangle.points = {{
    Eigen::Vector3f(minimum_x, minimum_y, height),
    Eigen::Vector3f(minimum_x, maximum_y, height),
    Eigen::Vector3f(maximum_x, maximum_y, height),
    Eigen::Vector3f(maximum_x, minimum_y, height),
  }};
  for (float x = minimum_x; x <= maximum_x + 1.0e-6F; x += 0.05F) {
    for (float y = minimum_y; y <= maximum_y + 1.0e-6F; y += 0.05F) {
      quadrangle.contained_points.emplace_back(x, y, height);
    }
  }
  return quadrangle;
}

}  // namespace

int main()
{
  using x30_plane_seg_core::PoseCorrectionConfig;
  using x30_plane_seg_core::FactoryFinalSuffixConfig;
  using x30_plane_seg_core::computeQuadrangleDistanceVectorXY;
  using x30_plane_seg_core::classifyQuadrangleIntrusions;
  using x30_plane_seg_core::computeAndUpdateQuadranglesFromReference;
  using x30_plane_seg_core::computeQuadrangleReferenceDistanceM;
  using x30_plane_seg_core::correctQuadranglePoses;
  using x30_plane_seg_core::cutQuadranglesByX;
  using x30_plane_seg_core::estimateStairPose;
  using x30_plane_seg_core::ignoreUnnecessaryQuadrangles;
  using x30_plane_seg_core::mergeQuadranglesAtSameStair;
  using x30_plane_seg_core::runFactoryFinalSuffix;
  using x30_plane_seg_core::selectQuadrangleReference;

  const std::vector<x30_plane_seg_core::IndexedQuadrangle> exact_plane{
    slopedQuadrangle(0U, -1.0F),
    slopedQuadrangle(1U, -0.4F),
  };
  const auto estimate = estimateStairPose(exact_plane);
  require(estimate.success, estimate.diagnostic);
  require(
    estimate.eligible_quadrangle_count == 2U,
    "both exact-plane quadrangles must be eligible");
  require(
    std::abs(estimate.direction.dot(Eigen::Vector2d::UnitX())) > 0.999,
    "least-squares stair direction must align with the X axis");

  std::vector<x30_plane_seg_core::CandidateBlock> candidates;
  for (int index = 0; index < 5; ++index) {
    candidates.push_back(stairCandidate(
      -0.8F + 0.4F * static_cast<float>(index),
      -0.7F + 0.18F * static_cast<float>(index)));
  }
  const auto corrected = correctQuadranglePoses(candidates);
  require(corrected.success, corrected.diagnostic);
  require(corrected.valid_seed_count == candidates.size(), "valid seed count mismatch");
  require(
    corrected.eligible_stair_pose_count == candidates.size(),
    "eligible stair-pose count mismatch");
  require(
    corrected.consensus_direction_count == candidates.size(),
    "all synthetic directions must enter the consensus");
  require(
    corrected.quadrangles.size() == candidates.size(),
    "corrected quadrangle count mismatch");
  require(
    std::abs(corrected.corrected_stair_direction.norm() - 1.0) < 1.0e-6,
    "corrected stair direction must be normalized");
  for (const auto & quadrangle : corrected.quadrangles) {
    const Eigen::Vector3f first_edge =
      quadrangle.points[1] - quadrangle.points[0];
    const Eigen::Vector3f second_edge =
      quadrangle.points[2] - quadrangle.points[1];
    require(
      std::abs(first_edge.head<2>().dot(second_edge.head<2>())) < 1.0e-5F,
      "corrected quadrangle edges must be orthogonal");
    for (const Eigen::Vector3f & point : quadrangle.points) {
      require(
        std::abs(point.z() - quadrangle.points[0].z()) < 1.0e-6F,
        "corrected quadrangle must use the seed mean height");
    }
  }

  PoseCorrectionConfig insufficient_config;
  insufficient_config.minimum_quadrangle_count = 6U;
  const auto insufficient = correctQuadranglePoses(candidates, insufficient_config);
  require(!insufficient.success, "minimum quadrangle guard must reject five candidates");

  auto degenerate = candidates;
  degenerate[0].hull.clear();
  degenerate[0].contained_points.clear();
  const auto with_rejection = correctQuadranglePoses(degenerate);
  require(
    with_rejection.rejected_candidate_count == 1U,
    "invalid source candidate must be reported");

  const std::vector<x30_plane_seg_core::IndexedQuadrangle> side_distance_input{
    flatQuadrangle(10U, 1.0F, 0.0F),
    flatQuadrangle(11U, 1.0F, 0.6F),
  };
  const auto side_filtered = ignoreUnnecessaryQuadrangles(
    side_distance_input, Eigen::Vector2d::UnitX());
  require(side_filtered.success, side_filtered.diagnostic);
  require(
    side_filtered.original_axis_removal_count == 1U,
    "original stair axis must classify the lateral outlier");
  require(
    side_filtered.perpendicular_axis_removal_count == 2U,
    "perpendicular axis must classify both forward quadrangles");
  require(
    side_filtered.selected_stair_direction.dot(Eigen::Vector2d::UnitX()) > 0.999,
    "factory filter must retain the orientation requiring fewer removals");
  require(
    side_filtered.quadrangles.size() == 1U &&
    side_filtered.quadrangles.front().source_candidate_index == 10U,
    "factory side-distance filter must preserve the centered quadrangle");
  require(
    side_filtered.removed_source_candidate_indices ==
    std::vector<std::size_t>{11U},
    "factory side-distance filter must report the removed source index");

  const auto first_step =
    terrainQuadrangle(20U, -0.5F, 0.5F, -0.25F, -0.05F, -0.5F);
  const auto second_step =
    terrainQuadrangle(21U, -0.5F, 0.5F, 0.0F, 0.2F, -0.5F);
  const Eigen::Vector2d separated_distance =
    computeQuadrangleDistanceVectorXY(first_step.points, second_step.points);
  require(
    std::abs(separated_distance.norm() - 0.05) < 1.0e-5,
    "factory sampled distance must recover the 5 cm lateral gap");
  require(
    computeQuadrangleDistanceVectorXY(
      first_step.points, first_step.points).norm() < 1.0e-9,
    "overlapping quadrangles must return a zero distance vector");

  const auto cut = cutQuadranglesByX(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{first_step},
    Eigen::Vector2d::UnitX());
  require(cut.success, cut.diagnostic);
  require(
    cut.processed_quadrangle_count == 1U,
    "cutByX must process the requested quadrangle");

  const auto merged = mergeQuadranglesAtSameStair(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      first_step, second_step},
    Eigen::Vector2d::UnitX());
  require(merged.success, merged.diagnostic);
  require(merged.merged_pair_count == 1U, "same stair pair must merge");
  require(merged.output_quadrangle_count == 1U, "merge output count mismatch");
  require(
    merged.quadrangles.front().merge_count == 2U,
    "merged quadrangle must retain its source count");

  auto higher_step = second_step;
  for (Eigen::Vector3f & point : higher_step.points) {
    point.z() += 0.1F;
  }
  for (Eigen::Vector3f & point : higher_step.contained_points) {
    point.z() += 0.1F;
  }
  const auto height_guard = mergeQuadranglesAtSameStair(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      first_step, higher_step},
    Eigen::Vector2d::UnitX());
  require(height_guard.success, height_guard.diagnostic);
  require(
    height_guard.merged_pair_count == 0U &&
    height_guard.output_quadrangle_count == 2U,
    "10 cm height difference must block same-stair merge");

  const auto height_chain_first =
    terrainQuadrangle(22U, -0.5F, 0.5F, -0.30F, -0.10F, 0.00F);
  const auto height_chain_second =
    terrainQuadrangle(23U, -0.5F, 0.5F, -0.05F, 0.15F, 0.06F);
  const auto height_chain_third =
    terrainQuadrangle(24U, -0.5F, 0.5F, 0.20F, 0.40F, 0.09F);
  const auto height_chain = mergeQuadranglesAtSameStair(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      height_chain_first, height_chain_second, height_chain_third},
    Eigen::Vector2d::UnitX());
  require(height_chain.success, height_chain.diagnostic);
  require(
    height_chain.merged_pair_count == 1U &&
    height_chain.output_quadrangle_count == 2U,
    "factory merge must retain the first source height for later height gates");

  const auto deferred_small =
    terrainQuadrangle(30U, -0.2F, 0.2F, -0.05F, 0.05F, 0.0F);
  const auto deferred_large =
    terrainQuadrangle(31U, -0.4F, 0.4F, -0.3F, 0.3F, 0.0F);
  const auto primary =
    terrainQuadrangle(32U, -0.5F, 0.5F, -0.1F, 0.1F, 0.0F);
  const auto sorted = classifyQuadrangleIntrusions(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      deferred_small, deferred_large, primary},
    0.05F);
  require(sorted.success, sorted.diagnostic);
  require(
    sorted.sorted_vector_indices == std::vector<std::size_t>({2U, 0U, 1U}),
    "sortVectors must retain the primary rectangle before deferred rectangles");
  require(
    sorted.non_intruding_vector_indices ==
    std::vector<std::size_t>({2U, 0U}),
    "non-intruding indices must include the first and separate nested rectangle");
  require(
    sorted.intruded_long_edge_vector_indices ==
    std::vector<std::size_t>({1U}),
    "large short-edge rectangle must enter the long-edge intrusion group");

  const auto narrow_inner =
    terrainQuadrangle(40U, -0.4F, 0.4F, -0.1F, 0.1F, 0.0F);
  const auto narrow_outer =
    terrainQuadrangle(41U, -0.5F, 0.5F, -0.15F, 0.15F, 0.0F);
  const auto short_intrusion = classifyQuadrangleIntrusions(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      narrow_inner, narrow_outer},
    0.05F);
  require(short_intrusion.success, short_intrusion.diagnostic);
  require(
    short_intrusion.non_intruding_vector_indices ==
    std::vector<std::size_t>({0U}),
    "sorted first rectangle must be accepted unconditionally");
  require(
    short_intrusion.intruded_short_edge_vector_indices ==
    std::vector<std::size_t>({1U}),
    "nested rectangle with a short edge <= 0.45 m must enter the short group");

  require(
    std::abs(
      computeQuadrangleReferenceDistanceM(
        first_step.points, second_step.points) - 0.05) < 1.0e-5,
    "reference selection must use its independent 5 cm sampled distance");

  const auto lower_overlap =
    terrainQuadrangle(50U, -0.5F, 0.5F, -0.1F, 0.1F, 0.0F);
  const auto target_overlap =
    terrainQuadrangle(51U, -0.5F, 0.5F, -0.1F, 0.1F, 0.5F);
  const auto upper_overlap =
    terrainQuadrangle(52U, -0.5F, 0.5F, -0.1F, 0.1F, 1.0F);
  const auto overlapping_reference = selectQuadrangleReference(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      lower_overlap, target_overlap, upper_overlap},
    std::vector<std::size_t>{0U, 2U},
    1U, 0.03F, Eigen::Vector2d::Zero(), false);
  require(overlapping_reference.success, overlapping_reference.diagnostic);
  require(
    overlapping_reference.selected_reference_vector_index == 0,
    "equal GridMap-center distance must retain the lower reference");
  require(
    overlapping_reference.constraint_points.size() == 4U,
    "two near references must append four factory constraint points");
  require(
    std::all_of(
      overlapping_reference.constraint_points.begin(),
      overlapping_reference.constraint_points.end(),
      [](const Eigen::Vector3f & point) {
        return std::abs(point.z()) < 1.0e-6F;
      }),
    "factory running-best quirk must duplicate the lower reference constraints");

  const auto lower_adjacent =
    terrainQuadrangle(53U, -0.35F, -0.15F, -0.1F, 0.1F, 0.0F);
  const auto target_adjacent =
    terrainQuadrangle(54U, -0.1F, 0.1F, -0.1F, 0.1F, 0.5F);
  const auto upper_adjacent =
    terrainQuadrangle(55U, 0.15F, 0.35F, -0.1F, 0.1F, 1.0F);
  const std::vector<x30_plane_seg_core::IndexedQuadrangle> adjacent_references{
    lower_adjacent, target_adjacent, upper_adjacent};
  const auto deferred_reference = selectQuadrangleReference(
    adjacent_references, std::vector<std::size_t>{0U, 2U},
    1U, 0.03F, Eigen::Vector2d::Zero(), false);
  require(deferred_reference.success, deferred_reference.diagnostic);
  require(
    deferred_reference.deferred_for_fallback &&
    deferred_reference.selected_reference_vector_index == -1,
    "two-sided non-intruding target must defer during the first pass");
  const auto fallback_reference = selectQuadrangleReference(
    adjacent_references, std::vector<std::size_t>{0U, 2U},
    1U, 0.03F, Eigen::Vector2d::Zero(), true);
  require(fallback_reference.success, fallback_reference.diagnostic);
  require(
    fallback_reference.selected_reference_vector_index == 0,
    "fallback tie must retain the lower reference nearest the GridMap center");

  const auto unrelated_reference = selectQuadrangleReference(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      terrainQuadrangle(56U, -2.1F, -1.9F, -0.1F, 0.1F, 0.0F),
      target_adjacent,
      terrainQuadrangle(57U, 1.9F, 2.1F, -0.1F, 0.1F, 1.0F)},
    std::vector<std::size_t>{0U, 2U},
    1U, 0.03F, Eigen::Vector2d::Zero(), false);
  require(unrelated_reference.success, unrelated_reference.diagnostic);
  require(
    unrelated_reference.target_added_to_reference_indices &&
    unrelated_reference.updated_reference_vector_indices.back() == 1U,
    "unrelated target must be promoted into the reference set");

  const auto repair_reference =
    terrainQuadrangle(60U, -0.5F, 0.5F, -0.1F, 0.1F, 0.0F);
  const auto repair_target =
    terrainQuadrangle(61U, -0.4F, 0.4F, 0.2F, 0.4F, 0.2F);
  const std::vector<x30_plane_seg_core::IndexedQuadrangle> repair_input{
    repair_reference, repair_target};
  const auto repaired = computeAndUpdateQuadranglesFromReference(
    repair_input, std::vector<std::size_t>{0U},
    1U, 1U, 0U, 0.03F, false);
  require(repaired.success, repaired.diagnostic);
  require(
    repaired.emitted_quadrangle_count == 1U &&
    repaired.registered_vector_indices == std::vector<std::size_t>{1U},
    "normal reference repair must overwrite and register the destination");
  require(
    repaired.quadrangles.size() == repair_input.size(),
    "normal reference repair must not append a quadrangle");
  for (const Eigen::Vector3f & point : repaired.quadrangles[1].points) {
    require(
      std::abs(point.z() - 0.2F) < 1.0e-6F,
      "reference repair must retain the destination height");
  }

  const auto emitted_all = computeAndUpdateQuadranglesFromReference(
    repair_input, std::vector<std::size_t>{0U},
    1U, 1U, 0U, 0.03F, true);
  require(emitted_all.success, emitted_all.diagnostic);
  require(
    emitted_all.emitted_quadrangle_count >= 1U,
    "wide reference repair must emit at least one valid hypothesis");

  auto empty_repair_input = repair_input;
  empty_repair_input[1].contained_points.clear();
  const auto empty_repair = computeAndUpdateQuadranglesFromReference(
    empty_repair_input, std::vector<std::size_t>{0U},
    1U, 1U, 0U, 0.03F, false);
  require(!empty_repair.success, "empty contained cloud must fail repair");
  require(
    empty_repair.error_vector_indices == std::vector<std::size_t>{1U},
    "normal repair failure must mark the destination as an error");

  const auto suffix_narrow_reference =
    terrainQuadrangle(70U, -0.5F, 0.5F, -0.05F, 0.05F, 0.0F);
  const auto suffix_narrow_target =
    terrainQuadrangle(71U, -0.6F, 0.6F, -0.10F, 0.10F, 0.2F);
  const auto narrow_suffix = runFactoryFinalSuffix(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      suffix_narrow_reference, suffix_narrow_target});
  require(narrow_suffix.success, narrow_suffix.diagnostic);
  require(
    narrow_suffix.narrow_normal_attempt_count == 1U &&
    narrow_suffix.narrow_normal_success_count == 1U,
    "factory suffix must repair one short-edge intrusion in the normal pass");
  require(
    narrow_suffix.fallback_target_count == 0U &&
    narrow_suffix.wide_target_count == 0U,
    "normal narrow repair must not enter fallback or wide repair");
  require(
    narrow_suffix.final_quadrangles.size() == 2U,
    "normal narrow repair must promote the repaired target to final output");

  auto fallback_failed_intruder =
    terrainQuadrangle(82U, -0.5F, 0.5F, -0.25F, -0.05F, 0.5F);
  fallback_failed_intruder.contained_points.clear();
  const std::vector<x30_plane_seg_core::IndexedQuadrangle> fallback_input{
    terrainQuadrangle(80U, -0.5F, 0.5F, -0.30F, -0.20F, 0.0F),
    terrainQuadrangle(81U, -0.5F, 0.5F, 0.25F, 0.40F, 1.0F),
    fallback_failed_intruder,
    terrainQuadrangle(83U, -0.5F, 0.5F, -0.10F, 0.20F, 0.5F),
  };
  const auto fallback_suffix = runFactoryFinalSuffix(fallback_input);
  require(fallback_suffix.success, fallback_suffix.diagnostic);
  require(
    fallback_suffix.fallback_target_count == 1U &&
    fallback_suffix.fallback_success_count == 1U,
    "two-sided non-intruding target must be repaired by the fallback pass");
  require(
    fallback_suffix.deleted_error_quadrangle_count == 1U &&
    std::find(
      fallback_suffix.deleted_source_candidate_indices.begin(),
      fallback_suffix.deleted_source_candidate_indices.end(), 82U) !=
    fallback_suffix.deleted_source_candidate_indices.end(),
    "failed narrow intruder must be removed during error cleanup");
  require(
    fallback_suffix.active_reference_vector_indices ==
    std::vector<std::size_t>({0U, 1U, 2U}),
    "fallback output indices must be remapped after deleting an earlier target");

  const auto suffix_wide_reference =
    terrainQuadrangle(90U, -0.5F, 0.5F, -0.10F, 0.10F, 0.0F);
  const auto suffix_wide_target =
    terrainQuadrangle(91U, -0.6F, 0.6F, -0.35F, 0.35F, 0.1F);
  const auto wide_suffix = runFactoryFinalSuffix(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      suffix_wide_reference, suffix_wide_target});
  require(wide_suffix.success, wide_suffix.diagnostic);
  require(
    wide_suffix.wide_target_count == 1U &&
    wide_suffix.wide_successful_target_count == 1U,
    "factory suffix must repair one long-edge intrusion in the wide pass");
  require(
    wide_suffix.wide_reference_attempt_count >= 1U &&
    wide_suffix.staged_promotion_count >= 1U,
    "wide repair must attempt a reference and promote staged outputs");
  require(
    wide_suffix.staged_tail_count ==
    wide_suffix.staged_promoted_vector_indices.size(),
    "staged tail count must describe the surviving promoted index tail");
  require(
    wide_suffix.staged_tail_count <= wide_suffix.staged_promotion_count,
    "surviving staged tail cannot exceed the number of promotions");
  require(
    wide_suffix.final_quadrangles.size() >= 2U,
    "wide staged outputs must enter the final active set");

  const auto late_wide_suffix = runFactoryFinalSuffix(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      suffix_wide_reference,
      terrainQuadrangle(92U, -0.6F, 0.6F, -0.35F, 0.35F, 2.9F),
    });
  require(late_wide_suffix.success, late_wide_suffix.diagnostic);
  require(
    late_wide_suffix.wide_successful_target_count == 1U &&
    late_wide_suffix.wide_reference_attempt_count == 1U,
    "wide repair must reach the recovered 2.95 m tolerance before success");

  auto suffix_error_target =
    terrainQuadrangle(101U, -0.6F, 0.6F, -0.10F, 0.10F, 0.2F);
  suffix_error_target.contained_points.clear();
  const auto deletion_suffix = runFactoryFinalSuffix(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      terrainQuadrangle(100U, -0.5F, 0.5F, -0.05F, 0.05F, 0.0F),
      suffix_error_target,
      terrainQuadrangle(102U, 1.5F, 2.5F, -0.075F, 0.075F, 0.0F),
    });
  require(deletion_suffix.success, deletion_suffix.diagnostic);
  require(
    deletion_suffix.deleted_error_quadrangle_count == 1U,
    "failed destination must be deleted by the first cleanup pass");
  require(
    deletion_suffix.active_reference_vector_indices ==
    std::vector<std::size_t>({0U, 1U}),
    "surviving active indices must be compacted after error deletion");
  require(
    deletion_suffix.final_quadrangles.size() == 2U &&
    deletion_suffix.final_quadrangles[0].source_candidate_index == 100U &&
    deletion_suffix.final_quadrangles[1].source_candidate_index == 102U,
    "error deletion must preserve surviving quadrangles and provenance");

  const auto degenerate_suffix = runFactoryFinalSuffix(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      terrainQuadrangle(
        110U, -0.5F, 0.5F, -0.00025F, 0.00025F, 0.0F),
      terrainQuadrangle(111U, 1.5F, 2.5F, -0.10F, 0.10F, 0.0F),
    });
  require(degenerate_suffix.success, degenerate_suffix.diagnostic);
  require(
    degenerate_suffix.deleted_degenerate_quadrangle_count == 1U,
    "factory first-edge squared-length cleanup must remove the tiny edge");
  require(
    degenerate_suffix.active_reference_vector_indices ==
    std::vector<std::size_t>{0U} &&
    degenerate_suffix.final_quadrangles.size() == 1U &&
    degenerate_suffix.final_quadrangles.front().source_candidate_index == 111U,
    "degenerate cleanup must remap the surviving active index to zero");

  FactoryFinalSuffixConfig unsupported_pitch_config;
  unsupported_pitch_config.enable_pitch_gated_is_error_quad = true;
  const auto unsupported_pitch_suffix = runFactoryFinalSuffix(
    std::vector<x30_plane_seg_core::IndexedQuadrangle>{
      suffix_narrow_reference},
    unsupported_pitch_config);
  require(
    !unsupported_pitch_suffix.success &&
    unsupported_pitch_suffix.diagnostic.find("unsupported") !=
    std::string::npos,
    "unsupported pitch-gated isErrorQuad must fail instead of guessing");

  std::cout << "X30 quadrangle pose-correction tests passed." << std::endl;
  return 0;
}
