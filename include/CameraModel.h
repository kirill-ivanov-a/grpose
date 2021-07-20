#ifndef INCLUDE_CAMERAMODEL
#define INCLUDE_CAMERAMODEL

#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>
#include "types.h"

namespace grpose {

struct CameraModelSettings {
  static constexpr int default_mapPolyDegree = 10;
  int mapPolyDegree = default_mapPolyDegree;

  static constexpr int default_unmapPolyDegree = 6;
  int unmapPolyDegree = default_unmapPolyDegree;

  static constexpr int default_mapPolyPoints = 2000;
  int mapPolyPoints = default_mapPolyPoints;

  static constexpr int default_unmapPolyPoints = 2000;
  int unmapPolyPoints = default_unmapPolyPoints;

  static constexpr double default_magicMaxAngle = 100.0 * (M_PI / 180.0);
  double magicMaxAngle = default_magicMaxAngle;

  enum CenterShift { NO_SHIFT, MINUS_05, PLUS_05 };
  static constexpr CenterShift default_centerShift = NO_SHIFT;
  CenterShift centerShift = default_centerShift;

  static constexpr bool default_debugOutput = false;
  bool debugOutput = default_debugOutput;

  static constexpr bool default_isDeterministic = true;
  bool isDeterministic = default_isDeterministic;
};

class CameraModel {
 public:
  using CamPyr = StdVectorA<CameraModel>;

  enum InputType { POLY_UNMAP, POLY_MAP };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraModel(int width, int height, double scale, const Vector2 &center,
              const VectorX &unmapPolyCoeffs,
              const CameraModelSettings &settings = {});

  /**
   * Initialize from a filename. For concrete file structure, please consult the
   * source of this constructor.
   */
  CameraModel(int width, int height, const std::string &calibFileName,
              InputType type, const CameraModelSettings &settings = {});

  /**
   * Initialize the model as a pinhole camera.
   * Note that it is a special case of the more general Scaramuzza model we use.
   */
  CameraModel(int width, int height, double f, double cx, double cy,
              const CameraModelSettings &settings = {});

  /**
   * Generic unmapping (image plane -> bearing vector). Was used to interface
   * with ceres's automatic differentiation, can be used again later.
   * @tparam T float, double or ceres::Jet
   * @param point 2 coordinates of a point on the image
   * @return 3D-direction in the camera frame, **UNNORMALIZED**
   *
   * WARNING: as for now, it can only be compiled with T=double because of the
   * temporary incorporation of the Unified camera model.
   */
  template <typename T>
  Eigen::Matrix<T, 3, 1> unmap(const T *point) const {
    using Vector3t = Eigen::Matrix<T, 3, 1>;
    using Vector2t = Eigen::Matrix<T, 2, 1>;
    using VectorXt = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    Eigen::Map<const Vector2t> pt_(point);
    Vector2t pt = pt_;

    pt[1] = (pt[1] - mPrincipalPoint[1]) / mFy;
    pt[0] = (pt[0] - mSkew * pt[1] - mPrincipalPoint[0]) / mFx;

    VectorXt p = mUnmapPolyCoeffs.cast<T>();

    T rho2 = pt.squaredNorm();
    T rho1 = sqrt(rho2);

    T z = p[0];
    T rhoN = rho2;
    for (int i = 1; i < mUnmapPolyDeg; i += 2) {
      z += rhoN * p[i];
      if (i + 1 < mUnmapPolyDeg) z += rhoN * rho1 * p[i + 1];
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
  Eigen::Matrix<T, 2, 1> map(const T *direction) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3t;
    typedef Eigen::Matrix<T, 2, 1> Vector2t;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;

    Eigen::Map<const Vector3t> pt_(direction);
    Vector3t pt = pt_;
    VectorXt p = mMapPolyCoeffs.cast<T>();

    T angle = atan2(pt.template head<2>().norm(), pt[2]);

    T r = p[0];
    T angleN = angle;
    for (int i = 1; i < p.rows(); ++i) {
      r += p[i] * angleN;
      angleN *= angle;
    }

    Vector2t res = pt.template head<2>().normalized() * r;
    res[0] = T(mFx) * res[0] + T(mSkew) * res[1] + T(mPrincipalPoint[0]);
    res[1] = T(mFy) * res[1] + T(mPrincipalPoint[1]);
    return res;
  }

  /**
   * A specialization of unmapping to Eigen input
   */
  template <typename T>
  inline Eigen::Matrix<T, 3, 1> unmap(
      const Eigen::Matrix<T, 2, 1> &point) const {
    return unmap(point.data());
  }

  /**
   * A specialization of mapping to Eigen input
   */
  template <typename T>
  inline Eigen::Matrix<T, 2, 1> map(const Eigen::Matrix<T, 3, 1> &ray) const {
    return map(ray.data());
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
    return angle < mMaxAngle;
  }

  /**
   * Undistort cv::Mat image. Can work both with colored and grayscale images.
   * @tparam T cv::Vec3b if colored, unsigned char if grayscale
   * @param img the image to be undistorted
   * @param cameraMatrix the target camera matrix, in which we want the image to
   * be drawn. Usually takes the form f 0 cx 0 f cy 0 0  1
   * @return the image as if it was shot with the `cameraMatrix`
   */
  template <typename T>
  cv::Mat undistort(const cv::Mat &img, const Matrix33 &cameraMatrix) const {
    Matrix33 Kinv = cameraMatrix.inverse();
    cv::Mat result = cv::Mat::zeros(img.rows, img.cols, img.type());

    for (int y = 0; y < result.rows; ++y)
      for (int x = 0; x < result.cols; ++x) {
        Vector3 pnt(double(x), double(y), 1.);
        Vector2 origPix = map((Kinv * pnt).eval());
        int origX = origPix[0], origY = origPix[1];
        if (origX >= 0 && origX < result.cols && origY >= 0 &&
            origY < result.rows)
          result.at<T>(y, x) = img.at<T>(origY, origX);
      }
    return result;
  }

  // With the help of ceres we could implement these derivatives virtually for
  // free
  //  std::pair<Vector2, Mat23> diffMap(const Vector3 &ray) const;
  //  std::pair<Vector2f, Mat23f> diffMap(const Vector3f &ray) const;

  // It is one of the core classes, so we want to stick to a more
  // object-oriented approach
  inline int width() const { return mWidth; }

  inline int height() const { return mHeight; }

  inline Vector2 principalPoint() const { return mPrincipalPoint; }

  inline double minZ() const { return mMinZ; }

  inline double maxAngle() const { return mMaxAngle; }

  inline VectorX mapPolyCoeffs() const { return mMapPolyCoeffs; }

  inline VectorX unmapPolyCoeffs() const { return mUnmapPolyCoeffs; }

  inline double fx() const { return mFx; }

  inline double fy() const { return mFy; }

  inline double skew() const { return mSkew; }

  // A mask can be used to cancel out pixels that are not providing meaningful
  // information. For example, many wide-angle cameras mounted on a vehicle
  // capture parts of the vehicle itself which negatively affects quality of
  // SLAM providing spurious matches. We have several hand-drawn masks for the
  // RobotCar dataset.
  void setMask(const cv::Mat1b &mask);

  const cv::Mat1b &mask() const { return mMask; };

  /**
   * Checks if a point is on image, takes the mask into account if it was
   * provided.
   * @param p a point
   * @param border Additional border inside the image, useful in SLAM
   */
  bool isOnImage(const Vector2 &p, int border = 0) const;

  /**
   * Returns a pyramid of camera models for coarser images. Each image on the
   * nex level is 2x more narrow in each dimension.
   */
  CamPyr camPyr(int pyrLevels) const;

 private:
  double getImgRadiusByAngle(double observeAngle) const;

  void getRectByAngle(double observeAngle, int &width, int &height) const;

  void setImageCenter(const Vector2 &imcenter);

  void setMapPolyCoeffs();

  void setUnmapPolyCoeffs();

  void readUnmap(std::istream &is);

  void readMap(std::istream &is);

  inline double calcUnmapPoly(double r) const {
    double rN = r * r;
    double res = mUnmapPolyCoeffs[0];
    for (int i = 1; i < mUnmapPolyDeg; ++i) {
      res += mUnmapPolyCoeffs[i] * rN;
      rN *= r;
    }
    return res;
  }

  inline double calcMapPoly(double funcVal) const {
    double funcValN = funcVal;
    double res = mMapPolyCoeffs[0];
    for (int i = 1; i < mMapPolyCoeffs.rows(); ++i) {
      res += mMapPolyCoeffs[i] * funcValN;
      funcValN *= funcVal;
    }
    return res;
  }

  void recalcMaxRadius();

  void recalcBoundaries();

  int mWidth, mHeight;
  int mUnmapPolyDeg;
  VectorX mUnmapPolyCoeffs;
  double mFx, mFy;
  Vector2 mPrincipalPoint;
  double mSkew;
  double maxRadius;
  double mMinZ;
  double mMaxAngle;
  VectorX mMapPolyCoeffs;
  CameraModelSettings settings;
  cv::Mat1b mMask;
};

}  // namespace grpose

#endif
