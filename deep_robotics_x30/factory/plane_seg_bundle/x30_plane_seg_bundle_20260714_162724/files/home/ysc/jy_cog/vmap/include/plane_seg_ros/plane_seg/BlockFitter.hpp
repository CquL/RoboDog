#ifndef _planeseg_BlockFitter_hpp_
#define _planeseg_BlockFitter_hpp_

#include "Types.hpp"

namespace planeseg {

class BlockFitter {
public:
  enum RectangleFitAlgorithm {
    MinimumArea,
    ClosestToPriorSize,
    MaximumHullPointOverlap
  };

  struct Block {
    int type; // 0:horizontal  1:vertical
    Eigen::Vector3f mSize;
    Eigen::Isometry3f mPose;
    std::vector<Eigen::Vector3f> mHull;
  };
  struct Result {
    bool mSuccess;
    std::vector<Block> mBlocks;
    Eigen::Vector4f mGroundPlane;
    std::vector<Eigen::Vector3f> mGroundPolygon;
    std::vector<Eigen::Vector2d> mGravityCenters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mPointClouds;
  };

  LabeledCloud::Ptr cloud;
  LabeledCloud::Ptr cloud2;

public:
  BlockFitter();

  // iLookDir is a viewing normal in computer vision coordinates
  // i.e. z is forward
  void setSensorPose(const Eigen::Vector3f& iOrigin,
                     const Eigen::Vector3f& iLookDir);
  void setBlockDimensions(const Eigen::Vector3f& iDimensions);
  void setDownsampleResolution(const float iRes);
  void setRemoveGround(const bool iVal);
  void setGroundBand(const float iMinZ, const float iMaxZ);
  void setHeightBand(const float iMinHeight, const float iMaxHeight);
  void setMaxRange(const float iRange);
  void setMaxAngleFromHorizontal(const float iDegrees);
  void setMaxAngleOfPlaneSegmenter(const float iDegrees);
  // void setMinAngleOfPlaneSegmenter(const float iDegrees);
  void setAreaThresholds(const float iMin, const float iMax);
  void setRectangleFitAlgorithm(const RectangleFitAlgorithm iAlgo);
  void setDebug(const bool iVal);
  void setCloud(const LabeledCloud::Ptr& iCloud);
  void setComputeVerticalPlane(const bool iComputeVerticalPlane);
  void setMinPoint(const int iMinPoint);

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
  // float mMinAngleOfPlaneSegmenter;
  float mAreaThreshMin;
  float mAreaThreshMax;
  RectangleFitAlgorithm mRectangleFitAlgorithm;
  LabeledCloud::Ptr mCloud;
  bool mDebug;
  bool mComputeVerticalPlane;
};

}

#endif
