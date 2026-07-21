#ifndef X30_PLANE_SEG_CORE__PLANE_SEG__INCREMENTAL_PLANE_ESTIMATOR_HPP_
#define X30_PLANE_SEG_CORE__PLANE_SEG__INCREMENTAL_PLANE_ESTIMATOR_HPP_

#include <vector>

#include "plane_seg/Types.hpp"

namespace planeseg
{

class IncrementalPlaneEstimator
{
protected:
  std::vector<Eigen::Vector3f> mPoints;
  Eigen::Vector3d mSum;
  Eigen::Matrix3d mSumSquared;
  int mCount;

  Eigen::Vector4f getPlane(
    const Eigen::Vector3d & sum,
    const Eigen::Matrix3d & sum_squared,
    double count);

  inline float computeError(
    const Eigen::Vector4f & plane,
    const Eigen::Vector3f & point)
  {
    const float error = point.dot(plane.head<3>()) + plane[3];
    return error * error;
  }

public:
  IncrementalPlaneEstimator();

  void reset();
  int getNumPoints() const;
  void addPoint(const Eigen::Vector3f & point);

  std::vector<float> computeErrors(
    const Eigen::Vector4f & plane,
    const std::vector<Eigen::Vector3f> & points);

  bool tryPoint(const Eigen::Vector3f & point, float maximum_error);
  bool tryPoint(
    const Eigen::Vector3f & point,
    const Eigen::Vector3f & normal,
    float maximum_error,
    float maximum_angle,
    float maximum_angle_to_floor);

  Eigen::Vector4f getCurrentPlane();
};

}  // namespace planeseg

#endif  // X30_PLANE_SEG_CORE__PLANE_SEG__INCREMENTAL_PLANE_ESTIMATOR_HPP_
