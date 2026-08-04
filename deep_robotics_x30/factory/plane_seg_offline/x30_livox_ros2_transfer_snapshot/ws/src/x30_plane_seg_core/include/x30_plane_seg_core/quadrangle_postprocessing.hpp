#ifndef X30_PLANE_SEG_CORE__QUADRANGLE_POSTPROCESSING_HPP_
#define X30_PLANE_SEG_CORE__QUADRANGLE_POSTPROCESSING_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "x30_plane_seg_core/plane_seg_core.hpp"
#include "x30_plane_seg_core/quadrangle_geometry.hpp"

namespace x30_plane_seg_core
{

constexpr float kFactoryStairPoseMinimumLongEdgeM = 0.8F;
constexpr float kFactoryStairPoseMinimumShortEdgeM = 0.1F;
constexpr float kFactoryPoseConsensusCosine = 0.939692620786F;
constexpr std::size_t kFactoryCorrectMinimumSize = 4U;
constexpr float kFactoryBoundarySampleStepM = 0.03F;
constexpr float kFactoryBoundarySampleCountEpsilon = 0.0001F;
constexpr float kFactoryMergeMaximumHeightDifferenceM = 0.061F;
constexpr float kFactoryMergeDirectionCosine = 0.965925826289F;
constexpr float kFactoryMergeMaximumDistanceM = 0.3F;
constexpr float kFactoryMergeMaximumAlongStairDistanceM = 0.1F;
constexpr float kFactoryCutMinimumRemainingDepthM = 0.05F;
constexpr float kFactoryCutStepM = 0.03F;
constexpr float kFactoryCutRemovedPointRatio = 0.261F;
constexpr std::size_t kFactoryCutMaximumRemovedPoints = 9U;
constexpr float kFactorySideDistanceThresholdM = 0.22F;
constexpr float kFactorySortMaximumPrimaryShortEdgeM = 0.45F;
constexpr float kFactorySortMinimumPrimaryLongEdgeM = 0.50F;
constexpr float kFactoryReferenceDistanceSampleStepM = 0.05F;
constexpr float kFactoryReferenceDistanceSampleCountEpsilon = 0.0001F;
constexpr float kFactoryReferenceMaximumCandidateDistanceM = 0.30F;
constexpr float kFactoryReferenceNearDistanceM = 0.18F;
constexpr float kFactoryRepairMinimumOutputExtentM = 0.08F;
constexpr float kFactoryRepairMinimumForwardExtentM = 0.10F;
constexpr float kFactoryWideInitialHeightToleranceM = 0.25F;
constexpr float kFactoryWideHeightToleranceStepM = 0.10F;
constexpr float kFactoryWideMaximumHeightToleranceM = 2.95F;
constexpr double kFactoryDegenerateFirstEdgeSquaredM2 = 1.0e-6;

struct IndexedQuadrangle
{
  std::size_t source_candidate_index{0U};
  Quadrangle points{};
  std::vector<Eigen::Vector3f> contained_points;
  std::size_t merge_count{1U};
};

struct StairPoseEstimate
{
  bool success{false};
  Eigen::Vector2d direction{Eigen::Vector2d::Zero()};
  std::size_t eligible_quadrangle_count{0U};
  std::string diagnostic{"stair pose estimate has not run"};
};

struct PoseCorrectionConfig
{
  float minimum_long_edge_m{kFactoryStairPoseMinimumLongEdgeM};
  float minimum_short_edge_m{kFactoryStairPoseMinimumShortEdgeM};
  float consensus_cosine{kFactoryPoseConsensusCosine};
  std::size_t minimum_quadrangle_count{kFactoryCorrectMinimumSize};
  float minimum_edge_m{1.0e-6F};
};

struct PoseCorrectionResult
{
  bool success{false};
  Eigen::Vector2d initial_stair_direction{Eigen::Vector2d::Zero()};
  Eigen::Vector2d corrected_stair_direction{Eigen::Vector2d::Zero()};
  std::size_t input_candidate_count{0U};
  std::size_t valid_seed_count{0U};
  std::size_t eligible_stair_pose_count{0U};
  std::size_t consensus_direction_count{0U};
  std::size_t rejected_candidate_count{0U};
  std::vector<IndexedQuadrangle> quadrangles;
  std::string diagnostic{"quadrangle pose correction has not run"};
};

struct SideDistanceFilterResult
{
  bool success{false};
  Eigen::Vector2d selected_stair_direction{Eigen::Vector2d::Zero()};
  std::size_t input_quadrangle_count{0U};
  std::size_t original_axis_removal_count{0U};
  std::size_t perpendicular_axis_removal_count{0U};
  std::vector<std::size_t> removed_source_candidate_indices;
  std::vector<IndexedQuadrangle> quadrangles;
  std::string diagnostic{"side-distance filter has not run"};
};

struct SameStairMergeConfig
{
  float boundary_sample_step_m{kFactoryBoundarySampleStepM};
  float boundary_sample_count_epsilon{kFactoryBoundarySampleCountEpsilon};
  float maximum_height_difference_m{kFactoryMergeMaximumHeightDifferenceM};
  float direction_cosine{kFactoryMergeDirectionCosine};
  float maximum_distance_m{kFactoryMergeMaximumDistanceM};
  float maximum_along_stair_distance_m{
    kFactoryMergeMaximumAlongStairDistanceM};
  float minimum_edge_m{1.0e-6F};
};

struct SameStairMergeResult
{
  bool success{false};
  std::size_t input_quadrangle_count{0U};
  std::size_t output_quadrangle_count{0U};
  std::size_t merged_pair_count{0U};
  std::size_t post_merge_cut_changed_count{0U};
  std::vector<IndexedQuadrangle> quadrangles;
  std::string diagnostic{"same-stair merge has not run"};
};

struct CutByXConfig
{
  float minimum_remaining_depth_m{kFactoryCutMinimumRemainingDepthM};
  float step_m{kFactoryCutStepM};
  float removed_point_ratio{kFactoryCutRemovedPointRatio};
  std::size_t maximum_removed_points{kFactoryCutMaximumRemovedPoints};
  float minimum_edge_m{1.0e-6F};
};

struct CutByXResult
{
  bool success{false};
  std::size_t input_quadrangle_count{0U};
  std::size_t processed_quadrangle_count{0U};
  std::size_t changed_quadrangle_count{0U};
  std::vector<IndexedQuadrangle> quadrangles;
  std::string diagnostic{"cutByX has not run"};
};

struct IntrusionClassificationResult
{
  bool success{false};
  std::size_t input_quadrangle_count{0U};
  std::vector<std::size_t> sorted_vector_indices;
  std::vector<std::size_t> non_intruding_vector_indices;
  std::vector<std::size_t> intruded_short_edge_vector_indices;
  std::vector<std::size_t> intruded_long_edge_vector_indices;
  std::string diagnostic{"quadrangle intrusion classification has not run"};
};

struct ReferenceSelectionResult
{
  bool success{false};
  int selected_reference_vector_index{-1};
  bool target_added_to_reference_indices{false};
  bool deferred_for_fallback{false};
  std::vector<std::size_t> updated_reference_vector_indices;
  std::vector<Eigen::Vector3f> constraint_points;
  std::string diagnostic{"reference selection has not run"};
};

struct QuadrangleUpdateResult
{
  bool success{false};
  std::size_t emitted_quadrangle_count{0U};
  std::array<std::size_t, 4> hypothesis_scores{{0U, 0U, 0U, 0U}};
  std::vector<IndexedQuadrangle> quadrangles;
  std::vector<std::size_t> registered_vector_indices;
  std::vector<std::size_t> error_vector_indices;
  std::vector<std::size_t> failed_reference_vector_indices;
  std::string diagnostic{"reference-based quadrangle update has not run"};
};

struct FactoryFinalSuffixConfig
{
  float grid_resolution_m{0.03F};
  Eigen::Vector2d grid_map_position_xy{Eigen::Vector2d::Zero()};
  float wide_initial_height_tolerance_m{
    kFactoryWideInitialHeightToleranceM};
  float wide_height_tolerance_step_m{
    kFactoryWideHeightToleranceStepM};
  float wide_maximum_height_tolerance_m{
    kFactoryWideMaximumHeightToleranceM};
  double degenerate_first_edge_squared_m2{
    kFactoryDegenerateFirstEdgeSquaredM2};

  // The outer pitch gate and isErrorQuad() business predicate are not enabled
  // until their complete factory semantics are evidenced. Setting this true
  // deliberately rejects the run instead of applying a guessed filter.
  bool enable_pitch_gated_is_error_quad{false};
};

struct FactoryFinalSuffixResult
{
  bool success{false};
  std::size_t input_quadrangle_count{0U};
  // Classification preserves the pre-repair/pre-deletion vector positions.
  // All standalone index vectors below are remapped to working_quadrangles.
  IntrusionClassificationResult classification;
  std::size_t narrow_normal_attempt_count{0U};
  std::size_t narrow_normal_success_count{0U};
  std::size_t fallback_target_count{0U};
  std::size_t fallback_success_count{0U};
  std::size_t wide_target_count{0U};
  std::size_t wide_reference_attempt_count{0U};
  std::size_t wide_successful_target_count{0U};
  std::size_t staged_promotion_count{0U};
  std::size_t staged_tail_count{0U};
  std::size_t deleted_error_quadrangle_count{0U};
  std::size_t deleted_degenerate_quadrangle_count{0U};
  bool pitch_gated_is_error_quad_applied{false};
  std::vector<std::size_t> active_reference_vector_indices;
  std::vector<std::size_t> deferred_vector_indices;
  std::vector<std::size_t> successful_reference_vector_indices;
  std::vector<std::size_t> repaired_target_vector_indices;
  std::vector<std::size_t> wide_considered_reference_vector_indices;
  std::vector<std::size_t> wide_failed_reference_vector_indices;
  std::vector<std::size_t> staged_promoted_vector_indices;
  std::vector<std::size_t> deleted_source_candidate_indices;
  std::vector<IndexedQuadrangle> working_quadrangles;
  std::vector<IndexedQuadrangle> final_quadrangles;
  std::string diagnostic{"factory final suffix has not run"};
};

// Reproduces the factory computeStairPose() stage recovered from the AArch64
// binary. Eligible quadrangles are fitted to a*x + b*y + c*z = -1 and the
// normalized horizontal plane gradient is returned.
StairPoseEstimate estimateStairPose(
  const std::vector<IndexedQuadrangle> & quadrangles,
  float minimum_long_edge_m = kFactoryStairPoseMinimumLongEdgeM,
  float minimum_short_edge_m = kFactoryStairPoseMinimumShortEdgeM,
  float minimum_edge_m = 1.0e-6F);

// Reproduces the evidence-backed computeStairPose() + correctQuadPose() prefix
// of the factory post-processing chain. It does not claim the later cut,
// merge, intrusion, reference, or temporal repair stages.
PoseCorrectionResult correctQuadranglePoses(
  const std::vector<CandidateBlock> & candidates,
  const PoseCorrectionConfig & config = PoseCorrectionConfig{});

// Reproduces the factory cutByX() scan. target_index=-1 processes every
// quadrangle; a non-negative index processes only that vector position.
CutByXResult cutQuadranglesByX(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const Eigen::Vector2d & stair_direction,
  int target_index = -1,
  const CutByXConfig & config = CutByXConfig{});

// Reproduces the distance vector used by the factory merge stage. Each edge is
// sampled every 3 cm; overlapping quadrangles return zero, otherwise the
// shortest sampled boundary vector (first - second) is returned.
Eigen::Vector2d computeQuadrangleDistanceVectorXY(
  const Quadrangle & first,
  const Quadrangle & second,
  float boundary_sample_step_m = kFactoryBoundarySampleStepM,
  float boundary_sample_count_epsilon =
  kFactoryBoundarySampleCountEpsilon);

// Reproduces the evidence-backed mergeQuadsAtSameStair() decision and merge
// loop, including the factory's second cutByX() pass for merged quadrangles.
SameStairMergeResult mergeQuadranglesAtSameStair(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const Eigen::Vector2d & stair_direction,
  const SameStairMergeConfig & config = SameStairMergeConfig{});

// Reproduces the factory ignoreUnnecessaryQuads() stage. The factory evaluates
// the stair axis and its perpendicular, removes quadrangles whose four corners
// all lie on the same side farther than side_distance_threshold_m from the
// axis through the origin, then keeps the orientation requiring fewer removals.
SideDistanceFilterResult ignoreUnnecessaryQuadrangles(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const Eigen::Vector2d & stair_direction,
  float side_distance_threshold_m = kFactorySideDistanceThresholdM,
  float minimum_edge_m = 1.0e-6F);

// Reproduces the factory pcl2vectors() -> sortVectors() -> configIntrusions()
// suffix prefix. Indices are positions in the input quadrangle vector, matching
// the stripped factory containers. No quadrangle geometry is modified.
IntrusionClassificationResult classifyQuadrangleIntrusions(
  const std::vector<IndexedQuadrangle> & quadrangles,
  float grid_resolution_m,
  float maximum_primary_short_edge_m =
  kFactorySortMaximumPrimaryShortEdgeM,
  float minimum_primary_long_edge_m =
  kFactorySortMinimumPrimaryLongEdgeM,
  float minimum_edge_m = 1.0e-6F);

// Reproduces Pass::computeDistance(int,int): each quadrangle boundary is
// sampled at a fixed 5 cm interval and the shortest scalar distance is
// returned. This is intentionally distinct from the 3 cm merge vector and the
// GridMap-resolution intrusion boundary.
double computeQuadrangleReferenceDistanceM(
  const Quadrangle & first,
  const Quadrangle & second,
  float sample_step_m = kFactoryReferenceDistanceSampleStepM,
  float sample_count_epsilon =
  kFactoryReferenceDistanceSampleCountEpsilon);

// Reproduces getReferenceIndice(). The returned reference list captures the
// factory side effect when an unrelated target is promoted to a reference.
// Constraint points are appended only when both a lower and an upper reference
// are nearer than 0.18 m.
ReferenceSelectionResult selectQuadrangleReference(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const std::vector<std::size_t> & reference_vector_indices,
  std::size_t target_vector_index,
  float grid_resolution_m,
  const Eigen::Vector2d & grid_map_position_xy,
  bool fallback,
  float maximum_candidate_distance_m =
  kFactoryReferenceMaximumCandidateDistanceM,
  float near_distance_m = kFactoryReferenceNearDistanceM,
  float minimum_edge_m = 1.0e-6F);

// Reproduces computeAndUpdateNewQuad(). emit_all=false selects the earliest
// maximum-scoring orientation. emit_all=true emits every hypothesis with at
// least one contained point and both extents >= 8 cm. Appending requires
// store_vector_index == quadrangles.size().
QuadrangleUpdateResult computeAndUpdateQuadranglesFromReference(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const std::vector<std::size_t> & active_reference_vector_indices,
  std::size_t destination_vector_index,
  std::size_t store_vector_index,
  std::size_t reference_vector_index,
  float grid_resolution_m,
  bool emit_all,
  bool expand_to_08 = false,
  float minimum_output_extent_m = kFactoryRepairMinimumOutputExtentM,
  float minimum_forward_extent_m = kFactoryRepairMinimumForwardExtentM,
  float minimum_edge_m = 1.0e-6F);

// Runs the recovered factory final suffix over a stable vector-index model:
// classification -> narrow normal -> fallback -> wide height expansion
// (0.25 m through 2.95 m by default) -> staged promotion -> error
// deletion/remap -> first-edge degenerate cleanup. The pitch-gated
// isErrorQuad() stage remains explicitly unsupported and disabled because its
// complete business predicate has not yet been recovered.
FactoryFinalSuffixResult runFactoryFinalSuffix(
  const std::vector<IndexedQuadrangle> & quadrangles,
  const FactoryFinalSuffixConfig & config = FactoryFinalSuffixConfig{});

}  // namespace x30_plane_seg_core

#endif  // X30_PLANE_SEG_CORE__QUADRANGLE_POSTPROCESSING_HPP_
