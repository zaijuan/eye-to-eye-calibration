#include <opencv2/core/eigen.hpp>


inline Eigen::Vector2d cv2eigen(const cv::Point2d& p_cv)
{
  return Eigen::Vector2d(p_cv.x, p_cv.y);
}

inline Eigen::Vector3d cv2eigen(const cv::Point3d& p_cv)
{
  return Eigen::Vector3d(p_cv.x, p_cv.y, p_cv.z);
}

inline Eigen::Vector3d cv2eigen(const cv::Vec3d& v_cv)
{
  return Eigen::Vector3d(v_cv[0], v_cv[1], v_cv[2]);
}

inline Eigen::Quaterniond cv2eigen(const cv::Vec4d& q_cv)
{
  return Eigen::Quaterniond(q_cv[0], q_cv[1], q_cv[2], q_cv[3]);
}
