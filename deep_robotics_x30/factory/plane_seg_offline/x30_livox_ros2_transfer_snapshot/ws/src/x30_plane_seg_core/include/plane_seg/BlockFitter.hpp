#ifndef X30_PLANE_SEG_CORE__PLANE_SEG__BLOCK_FITTER_HPP_
#define X30_PLANE_SEG_CORE__PLANE_SEG__BLOCK_FITTER_HPP_

#include <vector>

#include "plane_seg/Types.hpp"

namespace planeseg
{

class BlockFitter
{
public:
  enum RectangleFitAlgorithm
  {
    MinimumArea,
    ClosestToPriorSize,
    MaximumHullPointOverlap
  };

  struct Block
  {
    int type{0};  // 0: horizontal, 1: vertical
    Eigen::Vector3f mSize;
    Eigen::Isometry3f mPose;
    std::vector<Eigen::Vector3f> mHull;
  };

  struct Result
  {
    bool mSuccess{false};
    std::vector<Block> mBlocks;
    Eigen::Vector4f mGroundPlane;
    std::vector<Eigen::Vector3f> mGroundPolygon;
    std::vector<Eigen::Vector2d> mGravityCenters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mPointClouds;
  };

  BlockFitter();

  void setSensorPose(
    const Eigen::Vector3f & origin,
    const Eigen::Vector3f & look_direction);
  void setBlockDimensions(const Eigen::Vector3f & dimensions);
  void setDownsampleResolution(float resolution);
  void setRemoveGround(bool remove_ground);
  void setGroundBand(float minimum_z, float maximum_z);
  void setHeightBand(float minimum_height, float maximum_height);
  void setMaxRange(float range);
  void setMaxAngleFromHorizontal(float degrees);
  void setMaxAngleOfPlaneSegmenter(float degrees);
  void setAreaThresholds(float minimum, float maximum);
  void setRectangleFitAlgorithm(RectangleFitAlgorithm algorithm);
  void setDebug(bool debug);
  void setCloud(const LabeledCloud::Ptr & cloud);
  void setComputeVerticalPlane(bool compute_vertical_plane);
  void setMinPoint(int minimum_points);

  Result go();

protected:
  Eigen::Vector3f mOrigin;
  Eigen::Vector3f mLookDir;
  Eigen::Vector3f mBlockDimensions;
  float mDownsampleResolution;
  bool mRemoveGround;
  int mMinPoint;
  float mMinGroundZ;
  float mMaxGroundZ;
  float mMinHeightAboveGround;
  float mMaxHeightAboveGround;
  float mMaxRange;
  float mMaxAngleFromHorizontal;
  float mMaxAngleOfPlaneSegmenter;
  float mAreaThreshMin;
  float mAreaThreshMax;
  RectangleFitAlgorithm mRectangleFitAlgorithm;
  LabeledCloud::Ptr mCloud;
  bool mDebug;
  bool mComputeVerticalPlane;
};

}  // namespace planeseg

#endif  // X30_PLANE_SEG_CORE__PLANE_SEG__BLOCK_FITTER_HPP_
