#include "camera/CameraModelScaramuzza.h"
#include "defs.h"
#include "util.h"

#include <glog/logging.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

namespace grpose {

void adjustPrincipal(Vector2 &principal,
                     CameraModelScaramuzzaSettings::CenterShift centerMode) {
  switch (centerMode) {
    case CameraModelScaramuzzaSettings::NO_SHIFT:
      break;
    case CameraModelScaramuzzaSettings::MINUS_05:
      principal -= Vector2(0.5, 0.5);
      break;
    case CameraModelScaramuzzaSettings::PLUS_05:
      principal += Vector2(0.5, 0.5);
      break;
  }
}

CameraModelScaramuzza::CameraModelScaramuzza(
    int width, int height, double scale, const Vector2 &center,
    const VectorX &unmapPolyCoeffs,
    const CameraModelScaramuzzaSettings &settings)
    : width_(width),
      height_(height),
      unmapPolyDeg_(unmapPolyCoeffs.rows()),
      unmapPolyCoeffs_(unmapPolyCoeffs),
      fx_(scale),
      fy_(scale),
      principalPoint_(fx_ * center[0], fy_ * center[1]),
      skew_(0),
      settings_(settings) {
  adjustPrincipal(principalPoint_, settings.centerShift);
  recalcBoundaries();
  setMapPolyCoeffs();
}

CameraModelScaramuzza::CameraModelScaramuzza(
    int width, int height, const std::string &calibFileName, InputType type,
    const CameraModelScaramuzzaSettings &settings)
    : width_(width), height_(height), skew_(0), settings_(settings) {
  std::ifstream ifs(calibFileName, std::ifstream::in);
  CHECK(ifs.is_open()) << ("camera model file could not be open!");

  switch (type) {
    case POLY_UNMAP:
      readUnmap(ifs);
      adjustPrincipal(principalPoint_, settings.centerShift);
      recalcBoundaries();
      setMapPolyCoeffs();
      break;
    case POLY_MAP:
      readMap(ifs);
      adjustPrincipal(principalPoint_, settings.centerShift);
      maxAngle_ = settings.magicMaxAngle;
      setUnmapPolyCoeffs();
      recalcBoundaries();
      break;
  }
}

CameraModelScaramuzza::CameraModelScaramuzza(
    int width, int height, double f, double cx, double cy,
    const CameraModelScaramuzzaSettings &settings)
    : width_(width),
      height_(height),
      unmapPolyDeg_(0),
      fx_(f),
      fy_(f),
      principalPoint_(f * cx, f * cy),
      settings_(settings) {
  adjustPrincipal(principalPoint_, settings.centerShift);
  unmapPolyCoeffs_.resize(1, 1);
  unmapPolyCoeffs_[0] = 1;
  recalcBoundaries();

  //    setMapPolyCoeffs();
  CHECK_LT(settings.mapPolyDegree, 14);
  VectorX mapPolyCoeffsFull(14);
  mapPolyCoeffsFull << 0., 1., 0, 1. / 3., 0, 2. / 15., 0, 17. / 315., 0,
      62. / 2835., 0, 1382. / 155925., 0,
      21844. / 6081075.;  // Taylor expansion of tan(x)
  mapPolyCoeffs_ = mapPolyCoeffsFull.head(settings.mapPolyDegree + 1);

  LOG(INFO) << "\n\n CAMERA MODEL:\n"
            << "unmap coeffs  = " << unmapPolyCoeffs_.transpose()
            << "\nmap poly coeffs = " << mapPolyCoeffs_.transpose() << "\n\n";
}

double CameraModelScaramuzza::getImgRadiusByAngle(double observeAngle) const {
  double mapped = calcMapPoly(observeAngle);
  return std::min(fx_ * mapped, fy_ * mapped);
}

void CameraModelScaramuzza::setImageCenter(const Vector2 &imcenter) {
  principalPoint_ = imcenter;
}

void CameraModelScaramuzza::readUnmap(std::istream &is) {
  std::string tag;
  is >> tag;
  if (tag != "omnidirectional") throw std::domain_error("Invalid camera type");

  double scale, cx, cy;

  is >> scale >> cx >> cy >> unmapPolyDeg_;
  fx_ = fy_ = scale;
  principalPoint_ = Vector2(fx_ * cx, fy_ * cy);
  unmapPolyCoeffs_.resize(unmapPolyDeg_, 1);

  for (int i = 0; i < unmapPolyDeg_; ++i) is >> unmapPolyCoeffs_[i];
}

void CameraModelScaramuzza::readMap(std::istream &is) {
  int mapPolyDeg = 0;
  is >> mapPolyDeg >> fx_ >> fy_ >> principalPoint_[0] >> principalPoint_[1] >>
      skew_;
  // CHECK(std::abs(mSkew) < 0.05) << "Our CameraModel expects zero mSkew";
  mapPolyCoeffs_.resize(mapPolyDeg + 2);
  mapPolyCoeffs_[0] = 0;
  mapPolyCoeffs_[1] = 1;
  for (int i = 0; i < mapPolyDeg; ++i) is >> mapPolyCoeffs_[i + 2];
}

void CameraModelScaramuzza::setMapPolyCoeffs() {
  // points of type (r, \theta), where r stands for scaled radius of a point
  // in the image and \theta stands for angle to z-axis of the unprojected ray
  int nPnts = settings_.mapPolyPoints;
  int deg = settings_.mapPolyDegree;
  StdVectorA<Vector2> funcGraph;
  funcGraph.reserve(nPnts);
  std::mt19937 gen(settings_.isDeterministic ? 42 : std::random_device()());
  std::uniform_real_distribution<> distr(0, radius_);

  for (int it = 0; it < nPnts; ++it) {
    double r = distr(gen);
    // double r = maxRadius;
    double z = calcUnmapPoly(r);
    double angle = std::atan2(r, z);
    funcGraph.push_back(Vector2(r, angle));
  }

  std::sort(funcGraph.begin(), funcGraph.end(),
            [](Vector2 a, Vector2 b) { return a[0] < b[0]; });

  for (int i = 0; i < int(funcGraph.size()) - 1; ++i)
    if (funcGraph[i][1] > funcGraph[i + 1][1])
      throw std::runtime_error(
          "the camera unmapping polynomial (r -> angle) is not increasing!");

  // solving linear least-squares to approximate func^(-1) with a polynomial
  MatrixXX A(nPnts, deg + 1);
  VectorX b(nPnts);
  for (int i = 0; i < nPnts; ++i) {
    const double &funcVal = funcGraph[i][1];
    double funcValN = 1;
    for (int j = 0; j <= deg; ++j) {
      A(i, j) = funcValN;
      funcValN *= funcVal;
    }
    b(i) = funcGraph[i][0];
  }

  mapPolyCoeffs_ = A.fullPivHouseholderQr().solve(b);
}

void CameraModelScaramuzza::setUnmapPolyCoeffs() {
  recalcMaxRadius();

  unmapPolyDeg_ = settings_.unmapPolyDegree;

  MatrixX2 funcGraphFull(settings_.unmapPolyPoints, 2);
  int numTaken = 0;

  std::mt19937 gen(settings_.isDeterministic ? 42 : std::random_device()());
  const double eps = 1e-3;
  std::uniform_real_distribution<double> distr(eps, maxAngle_);
  LOG(INFO) << "Fitting the unmap polynomial in [" << eps << ", " << maxAngle_
            << "]" << std::endl;
  for (int i = 0; i < settings_.unmapPolyPoints; ++i) {
    double theta = distr(gen);
    double r = calcMapPoly(theta);
    double z = r * std::tan(M_PI_2 - theta);
    if (r <= radius_) {
      funcGraphFull(numTaken, 0) = r;
      funcGraphFull(numTaken, 1) = z;
      numTaken++;
    }
  }
  MatrixX2 funcGraph = funcGraphFull.topRows(numTaken);
  funcGraph.col(0) /=
      radius_;  // Normalizing the polynomial to take results from [0, 1]

  MatrixXX A(funcGraph.rows(), settings_.unmapPolyDegree);
  A.col(0).setOnes();
  A.col(1) = funcGraph.col(0).cwiseProduct(funcGraph.col(0));
  for (int j = 2; j < settings_.unmapPolyDegree; ++j)
    A.col(j) = A.col(j - 1).cwiseProduct(funcGraph.col(0));
  VectorX b = funcGraph.col(1);

  unmapPolyCoeffs_ = A.fullPivHouseholderQr().solve(b);

  double mult = radius_ * radius_;
  for (int i = 1; i < settings_.unmapPolyDegree; ++i) {
    unmapPolyCoeffs_(i) /= mult;
    mult *= radius_;
  }

  LOG(INFO) << "unmap poly coeffs: " << unmapPolyCoeffs_.transpose() << '\n';
}

cv::Mat1b downsampleMask(const cv::Mat1b &mask, int lvl) {
  int scale = (1 << lvl);
  cv::Mat1b result(mask.rows / scale, mask.cols / scale, 1);
  for (int y = 0; y < mask.rows; ++y)
    for (int x = 0; x < mask.cols; ++x)
      if (mask(y, x) == 0) result(y / scale, x / scale) = false;
  return result;
}

void CameraModelScaramuzza::recalcMaxRadius() {
  Vector2 normPp(principalPoint_[0] / fx_, principalPoint_[1] / fy_);
  Vector2 normSz(width_ / fx_, height_ / fy_);
  double dist[] = {normPp.norm(), (normPp - Vector2(0, normSz[1])).norm(),
                   (normPp - Vector2(normSz[0], 0)).norm(),
                   (normPp - normSz).norm()};
  radius_ = *std::max_element(dist, dist + 4);
}

void CameraModelScaramuzza::recalcBoundaries() {
  recalcMaxRadius();
  LOG(INFO) << "unmapping max radius = " << radius_;
  double minZUnnorm = calcUnmapPoly(radius_);
  minZ_ = minZUnnorm / std::hypot(minZUnnorm, radius_);
  maxAngle_ = std::atan2(radius_, minZUnnorm);

  LOG(INFO) << "mapping max angle = " << maxAngle_;
}

}  // namespace grpose
