#ifndef X30_PLANE_SEG_CORE__PLANE_SEG__PLANE_SEGMENTER_HPP_
#define X30_PLANE_SEG_CORE__PLANE_SEG__PLANE_SEGMENTER_HPP_

#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "plane_seg/Types.hpp"

namespace planeseg
{

class PlaneSegmenter
{
public:
  struct Result
  {
    std::vector<int> mLabels;
    std::unordered_map<int, Eigen::Vector4f> mPlanes;
  };

  PlaneSegmenter();

  void setData(
    const LabeledCloud::Ptr & cloud,
    const NormalCloud::Ptr & normals);
  void setMaxError(float error);
  void setMaxAngle(float angle_degrees);
  void setMaxAngleToFloor(float angle_degrees);
  void setSearchRadius(float radius);
  void setMinPoints(int minimum_points);

  Result go();

protected:
  LabeledCloud::Ptr mCloud;
  NormalCloud::Ptr mNormals;
  float mMaxError;
  float mMaxAngle;
  float mMaxAngleToFloor;
  float mSearchRadius;
  int mMinPoints;
};

}  // namespace planeseg

#endif  // X30_PLANE_SEG_CORE__PLANE_SEG__PLANE_SEGMENTER_HPP_
