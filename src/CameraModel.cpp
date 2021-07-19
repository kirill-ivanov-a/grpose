#include "CameraModel.h"
#include "defs.h"
#include "util.h"

#include <glog/logging.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

namespace mcam {

void adjustPrincipal(Vector2 &principal,
                     CameraModelSettings::CenterShift centerMode) {
  switch (centerMode) {
    case CameraModelSettings::NO_SHIFT:
      break;
    case CameraModelSettings::MINUS_05:
      principal -= Vector2(0.5, 0.5);
      break;
    case CameraModelSettings::PLUS_05:
      principal += Vector2(0.5, 0.5);
      break;
  }
}

CameraModel::CameraModel(int width, int height, double scale,
                         const Vector2 &center, const VectorX &unmapPolyCoeffs,
                         const CameraModelSettings &settings)
    : mWidth(width),
      mHeight(height),
      mUnmapPolyDeg(unmapPolyCoeffs.rows()),
      mUnmapPolyCoeffs(unmapPolyCoeffs),
      mFx(scale),
      mFy(scale),
      mPrincipalPoint(mFx * center[0], mFy * center[1]),
      mSkew(0),
      settings(settings),
      mMask(height, width, CV_WHITE_BYTE) {
  adjustPrincipal(mPrincipalPoint, settings.centerShift);
  recalcBoundaries();
  setMapPolyCoeffs();
}

CameraModel::CameraModel(int width, int height,
                         const std::string &calibFileName, InputType type,
                         const CameraModelSettings &settings)
    : mWidth(width),
      mHeight(height),
      mSkew(0),
      settings(settings),
      mMask(height, width, CV_WHITE_BYTE) {
  std::ifstream ifs(calibFileName, std::ifstream::in);
  CHECK(ifs.is_open()) << ("camera model file could not be open!");

  switch (type) {
    case POLY_UNMAP:
      readUnmap(ifs);
      adjustPrincipal(mPrincipalPoint, settings.centerShift);
      recalcBoundaries();
      setMapPolyCoeffs();
      break;
    case POLY_MAP:
      readMap(ifs);
      adjustPrincipal(mPrincipalPoint, settings.centerShift);
      mMaxAngle = settings.magicMaxAngle;
      setUnmapPolyCoeffs();
      recalcBoundaries();
      break;
  }
}

CameraModel::CameraModel(int width, int height, double f, double cx, double cy,
                         const CameraModelSettings &settings)
    : mWidth(width),
      mHeight(height),
      mUnmapPolyDeg(0),
      mFx(f),
      mFy(f),
      mPrincipalPoint(f * cx, f * cy),
      settings(settings),
      mMask(height, width, CV_WHITE_BYTE) {
  adjustPrincipal(mPrincipalPoint, settings.centerShift);
  mUnmapPolyCoeffs.resize(1, 1);
  mUnmapPolyCoeffs[0] = 1;
  recalcBoundaries();

  //    setMapPolyCoeffs();
  CHECK_LT(settings.mapPolyDegree, 14);
  VectorX mapPolyCoeffsFull(14);
  mapPolyCoeffsFull << 0., 1., 0, 1. / 3., 0, 2. / 15., 0, 17. / 315., 0,
      62. / 2835., 0, 1382. / 155925., 0,
      21844. / 6081075.;  // Taylor expansion of tan(x)
  mMapPolyCoeffs = mapPolyCoeffsFull.head(settings.mapPolyDegree + 1);

  LOG(INFO) << "\n\n CAMERA MODEL:\n"
            << "unmap coeffs  = " << mUnmapPolyCoeffs.transpose()
            << "\nmap poly coeffs = " << mMapPolyCoeffs.transpose() << "\n\n";
}

bool CameraModel::isOnImage(const Vector2 &p, int border) const {
  bool inBorder =
      Eigen::AlignedBox2d(Vector2(border, border),
                          Vector2(mWidth - border, mHeight - border))
          .contains(p);
  return inBorder && mMask(toCvPoint(p));
}

double CameraModel::getImgRadiusByAngle(double observeAngle) const {
  double mapped = calcMapPoly(observeAngle);
  return std::min(mFx * mapped, mFy * mapped);
}

void CameraModel::getRectByAngle(double observeAngle, int &retWidth,
                                 int &retHeight) const {
  double r = calcMapPoly(observeAngle);
  retWidth = int(r * mWidth / maxRadius);
  retHeight = int(r * mHeight / maxRadius);
}

void CameraModel::setImageCenter(const Vector2 &imcenter) {
  mPrincipalPoint = imcenter;
}

void CameraModel::readUnmap(std::istream &is) {
  std::string tag;
  is >> tag;
  if (tag != "omnidirectional") throw std::runtime_error("Invalid camera type");

  double scale, cx, cy;

  is >> scale >> cx >> cy >> mUnmapPolyDeg;
  mFx = mFy = scale;
  mPrincipalPoint = Vector2(mFx * cx, mFy * cy);
  mUnmapPolyCoeffs.resize(mUnmapPolyDeg, 1);

  for (int i = 0; i < mUnmapPolyDeg; ++i) is >> mUnmapPolyCoeffs[i];
}

void CameraModel::readMap(std::istream &is) {
  int mapPolyDeg = 0;
  is >> mapPolyDeg >> mFx >> mFy >> mPrincipalPoint[0] >> mPrincipalPoint[1] >>
      mSkew;
  // CHECK(std::abs(mSkew) < 0.05) << "Our CameraModel expects zero mSkew";
  mMapPolyCoeffs.resize(mapPolyDeg + 2);
  mMapPolyCoeffs[0] = 0;
  mMapPolyCoeffs[1] = 1;
  for (int i = 0; i < mapPolyDeg; ++i) is >> mMapPolyCoeffs[i + 2];
}

void CameraModel::setMapPolyCoeffs() {
  // points of type (r, \theta), where r stands for scaled radius of a point
  // in the image and \theta stands for angle to z-axis of the unprojected ray
  int nPnts = settings.mapPolyPoints;
  int deg = settings.mapPolyDegree;
  StdVectorA<Vector2> funcGraph;
  funcGraph.reserve(nPnts);
  std::mt19937 gen(settings.isDeterministic ? 42 : std::random_device()());
  std::uniform_real_distribution<> distr(0, maxRadius);

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

  mMapPolyCoeffs = A.fullPivHouseholderQr().solve(b);
}

void CameraModel::setUnmapPolyCoeffs() {
  recalcMaxRadius();

  mUnmapPolyDeg = settings.unmapPolyDegree;

  MatrixX2 funcGraphFull(settings.unmapPolyPoints, 2);
  int numTaken = 0;

  std::mt19937 gen(settings.isDeterministic ? 42 : std::random_device()());
  const double eps = 1e-3;
  std::uniform_real_distribution<double> distr(eps, mMaxAngle);
  LOG(INFO) << "Fitting the unmap polynomial in [" << eps << ", " << mMaxAngle
            << "]" << std::endl;
  for (int i = 0; i < settings.unmapPolyPoints; ++i) {
    double theta = distr(gen);
    double r = calcMapPoly(theta);
    double z = r * std::tan(M_PI_2 - theta);
    if (r <= maxRadius) {
      funcGraphFull(numTaken, 0) = r;
      funcGraphFull(numTaken, 1) = z;
      numTaken++;
    }
  }
  MatrixX2 funcGraph = funcGraphFull.topRows(numTaken);
  funcGraph.col(0) /=
      maxRadius;  // Normalizing the polynomial to take results from [0, 1]

  MatrixXX A(funcGraph.rows(), settings.unmapPolyDegree);
  A.col(0).setOnes();
  A.col(1) = funcGraph.col(0).cwiseProduct(funcGraph.col(0));
  for (int j = 2; j < settings.unmapPolyDegree; ++j)
    A.col(j) = A.col(j - 1).cwiseProduct(funcGraph.col(0));
  VectorX b = funcGraph.col(1);

  mUnmapPolyCoeffs = A.fullPivHouseholderQr().solve(b);

  double mult = maxRadius * maxRadius;
  for (int i = 1; i < settings.unmapPolyDegree; ++i) {
    mUnmapPolyCoeffs(i) /= mult;
    mult *= maxRadius;
  }

  LOG(INFO) << "unmap poly coeffs: " << mUnmapPolyCoeffs.transpose() << '\n';
}

cv::Mat1b downsampleMask(const cv::Mat1b &mask, int lvl) {
  int scale = (1 << lvl);
  cv::Mat1b result(mask.rows / scale, mask.cols / scale, 1);
  for (int y = 0; y < mask.rows; ++y)
    for (int x = 0; x < mask.cols; ++x)
      if (mask(y, x) == 0) result(y / scale, x / scale) = false;
  return result;
}

CameraModel::CamPyr CameraModel::camPyr(int pyrLevels) const {
  CHECK(pyrLevels > 0);
  CamPyr result;
  for (int i = 0; i < pyrLevels; ++i) {
    result.emplace_back(*this);
    result.back().mWidth /= (1 << i);
    result.back().mHeight /= (1 << i);
    result.back().mPrincipalPoint /= (1 << i);
    result.back().mFx /= (1 << i);
    result.back().mFy /= (1 << i);
    result.back().setMask(downsampleMask(mMask, i));
  }

  return result;
}

void CameraModel::recalcMaxRadius() {
  Vector2 normPp(mPrincipalPoint[0] / mFx, mPrincipalPoint[1] / mFy);
  Vector2 normSz(mWidth / mFx, mHeight / mFy);
  double dist[] = {normPp.norm(), (normPp - Vector2(0, normSz[1])).norm(),
                   (normPp - Vector2(normSz[0], 0)).norm(),
                   (normPp - normSz).norm()};
  maxRadius = *std::max_element(dist, dist + 4);
}

void CameraModel::recalcBoundaries() {
  recalcMaxRadius();
  LOG(INFO) << "unmapping max radius = " << maxRadius;
  double minZUnnorm = calcUnmapPoly(maxRadius);
  mMinZ = minZUnnorm / std::hypot(minZUnnorm, maxRadius);
  mMaxAngle = std::atan2(maxRadius, minZUnnorm);

  LOG(INFO) << "mapping max angle = " << mMaxAngle;
}

void CameraModel::setMask(const cv::Mat1b &mask) { this->mMask = mask; }

}  // namespace mcam
