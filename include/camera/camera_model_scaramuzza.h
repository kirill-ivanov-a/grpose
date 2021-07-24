#ifndef GRPOSE_CAMERA_CAMERAMODELSCARAMUZZA_
#define GRPOSE_CAMERA_CAMERAMODELSCARAMUZZA_

#include <string>

#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "types.h"

namespace grpose {

struct CameraModelScaramuzzaSettings {
  static constexpr int default_map_polynomial_degree = 10;
  int map_polynomial_degree = default_map_polynomial_degree;

  static constexpr int default_unmap_polynomial_degree = 6;
  int unmap_polynomial_degree = default_unmap_polynomial_degree;

  static constexpr int default_map_polynomial_points = 2000;
  int map_polynomial_points = default_map_polynomial_points;

  static constexpr int default_unmap_polynomial_points = 2000;
  int unmap_polynomial_points = default_unmap_polynomial_points;

  static constexpr double default_initial_max_angle = 100.0 * (M_PI / 180.0);
  double initial_max_angle = default_initial_max_angle;

  enum CenterShift { kNoShift, kMinus0_5, kPlus0_5 };
  static constexpr CenterShift default_center_shift = kNoShift;
  CenterShift center_shift = default_center_shift;

  static constexpr bool default_debug_output = false;
  bool debug_output = default_debug_output;

  static constexpr bool default_is_deterministic = true;
  bool is_deterministic = default_is_deterministic;
};

class CameraModelScaramuzza {
 public:
  enum InputType { kPolynomialMap, kPolynomialUnmap };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraModelScaramuzza(int width, int height, double scale,
                        const Vector2 &center, const VectorX &unmapPolyCoeffs,
                        const CameraModelScaramuzzaSettings &settings = {});

  /**
   * Initialize from a filename. For concrete file structure, please consult the
   * source of this constructor.
   */
  CameraModelScaramuzza(int width, int height, const std::string &calibFileName,
                        InputType type,
                        const CameraModelScaramuzzaSettings &settings = {});

  /**
   * Initialize the model as a pinhole camera.
   * Note that it is a special case of the more general Scaramuzza model we use.
   */
  CameraModelScaramuzza(int width, int height, double f, double cx, double cy,
                        const CameraModelScaramuzzaSettings &settings = {});

  /**
   * Generic unmapping (image plane -> bearing vector). Was used to interface
   * with ceres's automatic differentiation, can be used again later.
   * @tparam T float, double or ceres::Jet
   * @param point 2 coordinates of a point on the image
   * @return 3D-direction in the camera frame, **UNNORMALIZED**
   */
  template <typename T>
  Eigen::Matrix<T, 3, 1> Unmap(const T *point) const {
    using Vector3t = Eigen::Matrix<T, 3, 1>;
    using Vector2t = Eigen::Matrix<T, 2, 1>;
    using VectorXt = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    Eigen::Map<const Vector2t> pt_(point);
    Vector2t pt = pt_;

    pt[1] = (pt[1] - principalPoint_[1]) / fy_;
    pt[0] = (pt[0] - skew_ * pt[1] - principalPoint_[0]) / fx_;

    VectorXt p = unmapPolyCoeffs_.cast<T>();

    T rho2 = pt.squaredNorm();
    T rho1 = sqrt(rho2);

    T z = p[0];
    T rhoN = rho2;
    for (int i = 1; i < unmapPolyDeg_; i += 2) {
      z += rhoN * p[i];
      if (i + 1 < unmapPolyDeg_) z += rhoN * rho1 * p[i + 1];
      rhoN *= rho2;
    }

    Vector3t res(pt[0], pt[1], z);
    return res;
  }

  /**
   * Generic mapping (bearing vector -> image plane). Was used to interface with
   * ceres's automatic differentiation, can be used again later.
   * @tparam T float, double or ceres::Jet
   * @param direction 3 coordinates of a direction in the camera frame (norm not
   * important)
   * @return 2D-point on the image
   *
   * WARNING: as for now, it can only be compiled with T=double because of the
   * temporary incorporation of the Unified camera model.
   */
  template <typename T>
  Eigen::Matrix<T, 2, 1> Map(const T *direction) const {
    using Vector3t = Eigen::Matrix<T, 3, 1>;
    using Vector2t = Eigen::Matrix<T, 2, 1>;
    using VectorXt = Eigen::Matrix<T, Eigen::Dynamic, 1>;

    Eigen::Map<const Vector3t> pt_(direction);
    Vector3t pt = pt_;
    VectorXt p = mapPolyCoeffs_.cast<T>();

    T angle = atan2(pt.template head<2>().norm(), pt[2]);

    T r = p[0];
    T angleN = angle;
    for (int i = 1; i < p.rows(); ++i) {
      r += p[i] * angleN;
      angleN *= angle;
    }

    Vector2t res = pt.template head<2>().normalized() * r;
    res[0] = T(fx_) * res[0] + T(skew_) * res[1] + T(principalPoint_[0]);
    res[1] = T(fy_) * res[1] + T(principalPoint_[1]);
    return res;
  }

  /**
   * Check if the direction can be mapped by this camera model. Note that some
   * directions (most notably, looking straight backward) are outside of the FoV
   * of the camera, and as the mapping polynomial was not fitted to work with
   * those, it may produce incorrect results and even map to the image! Check
   * this before mapping if you are unsure.
   */
  template <typename T>
  bool isMappable(const Eigen::Matrix<T, 3, 1> &ray) const {
    T angle = atan2(ray.template head<2>().norm(), ray[2]);
    return angle < maxAngle_;
  }

  inline Vector2 principalPoint() const { return principalPoint_; }
  inline double minZ() const { return minZ_; }
  inline double maxAngle() const { return maxAngle_; }
  inline VectorX mapPolyCoeffs() const { return mapPolyCoeffs_; }
  inline VectorX unmapPolyCoeffs() const { return unmapPolyCoeffs_; }
  inline double fx() const { return fx_; }
  inline double fy() const { return fy_; }
  inline double skew() const { return skew_; }

 private:
  double getImgRadiusByAngle(double observeAngle) const;
  void setImageCenter(const Vector2 &imcenter);
  void setMapPolyCoeffs();
  void setUnmapPolyCoeffs();
  void readUnmap(std::istream &is);
  void readMap(std::istream &is);

  inline double calcUnmapPoly(double r) const {
    double rN = r * r;
    double res = unmapPolyCoeffs_[0];
    for (int i = 1; i < unmapPolyDeg_; ++i) {
      res += unmapPolyCoeffs_[i] * rN;
      rN *= r;
    }
    return res;
  }

  inline double calcMapPoly(double funcVal) const {
    double funcValN = funcVal;
    double res = mapPolyCoeffs_[0];
    for (int i = 1; i < mapPolyCoeffs_.rows(); ++i) {
      res += mapPolyCoeffs_[i] * funcValN;
      funcValN *= funcVal;
    }
    return res;
  }
  void recalcMaxRadius();
  void recalcBoundaries();

  int width_, height_;
  int unmapPolyDeg_;
  VectorX unmapPolyCoeffs_;
  double fx_, fy_;
  Vector2 principalPoint_;
  double skew_;
  double radius_;
  double minZ_;
  double maxAngle_;
  VectorX mapPolyCoeffs_;
  CameraModelScaramuzzaSettings settings_;
};

}  // namespace grpose

#endif
