#include "util.h"

#include <glog/logging.h>
#include <iostream>
#include <vector>

#include "types.h"

namespace grpose {

void printInPly(std::ostream &out, const std::vector<Vector3> &points,
                const std::vector<cv::Vec3b> &colors) {
  std::vector<Vector3> pf;
  std::vector<cv::Vec3b> cf;

  for (int i = 0; i < points.size(); ++i) {
    const Vector3 &p = points[i];
    const cv::Vec3b &color = colors[i];
    bool ok = true;
    for (int it = 0; it < 3; ++it)
      if (!std::isfinite(p[it])) ok = false;
    if (ok) {
      pf.push_back(p);
      cf.push_back(color);
    }
  }

  out.precision(15);
  out << R"__(ply
format ascii 1.0
element vertex )__"
      << pf.size() << R"__(
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
)__";

  for (int i = 0; i < pf.size(); ++i) {
    const Vector3 &p = pf[i];
    const cv::Vec3b &color = cf[i];

    out << p[0] << ' ' << p[1] << ' ' << p[2] << ' ';
    out << int(color[2]) << ' ' << int(color[1]) << ' ' << int(color[0])
        << '\n';
  }
}

std::vector<double> readDoublesFromBin(const fs::path &filename) {
  CHECK(fs::is_regular_file(filename));
  std::ifstream ifs(filename, std::ios::binary);
  ifs.seekg(0, std::ios::end);
  int size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::vector<double> result(size / sizeof(double));
  ifs.read(reinterpret_cast<char *>(result.data()),
           result.size() * sizeof(double));
  return result;
}

void putInMatrixForm(std::ostream &out, const SE3 &motion) {
  Eigen::Matrix<double, 3, 4, Eigen::RowMajor> pose = motion.matrix3x4();
  for (int i = 0; i < 12; ++i) out << pose.data()[i] << ' ';
  out << '\n';
}

void drawSquare(cv::Mat &img, const cv::Point &pos, int size2,
                const cv::Scalar &col, int thickness) {
  cv::Point from = pos - cv::Point(size2, size2);
  cv::Point to = pos + cv::Point(size2, size2);

  if (size2 > 0)
    cv::rectangle(img, from, to, col, thickness);
  else if (size2 == 0)
    img.at<cv::Vec3b>(from) = toCvVec3b(col);
}

std::string curTimeBrief() {
  constexpr int maxTimedOutputStringSize = 22;
  char curTime[maxTimedOutputStringSize + 2];
  std::time_t t = std::time(nullptr);
  std::strftime(curTime, maxTimedOutputStringSize + 1, "%Y%m%d_%H%M%S",
                std::localtime(&t));
  return std::string(curTime);
}

Vector2 toVector2(cv::Point p) { return Vector2(double(p.x), double(p.y)); }

Vector2 toVector2(cv::Point2f p) { return Vector2(double(p.x), double(p.y)); }

cv::Point toCvPoint(const Vector2 &vec) {
  return cv::Point(int(vec[0]), int(vec[1]));
}

cv::Vec3b toCvVec3b(cv::Scalar scalar) {
  return cv::Vec3b(scalar[0], scalar[1], scalar[2]);
}

cv::Mat1b cvtBgrToGray(const cv::Mat3b &coloredImg) {
  cv::Mat1b result;
  cv::cvtColor(coloredImg, result, cv::COLOR_BGR2GRAY);
  return result;
}

cv::Mat3b cvtBgrToGray3(const cv::Mat3b &coloredImg) {
  cv::Mat1b result1C;
  cv::cvtColor(coloredImg, result1C, cv::COLOR_BGR2GRAY);
  cv::Mat3b result3C;
  cv::cvtColor(result1C, result3C, cv::COLOR_GRAY2BGR);
  return result3C;
}

cv::Mat3b cvtGrayToBgr(const cv::Mat1b &grayImg) {
  cv::Mat3b result;
  cv::cvtColor(grayImg, result, cv::COLOR_GRAY2BGR);
  return result;
}

ChronoTimePoint toChronoTimePoint(Timestamp timestamp) {
  return std::chrono::time_point<std::chrono::high_resolution_clock,
                                 std::chrono::microseconds>(
      std::chrono::microseconds(timestamp));
}

std::string timeOfDay(ChronoTimePoint timePoint) {
  using namespace std::chrono;
  using days = duration<int64_t, std::ratio<86400>>;

  ChronoTimePoint day = floor<days>(timePoint);
  ChronoTimePoint hour = floor<hours>(timePoint);
  ChronoTimePoint minute = floor<minutes>(timePoint);
  ChronoTimePoint second = floor<seconds>(timePoint);

  int h = duration_cast<hours>(hour - day).count();
  int m = duration_cast<minutes>(minute - hour).count();
  double s = duration_cast<seconds>(second - minute).count();
  int mcs = duration_cast<microseconds>(timePoint - second).count();
  return std::to_string(h) + ":" + std::to_string(m) + ":" +
         std::to_string(s + mcs * 1e-6);
}

}  // namespace grpose
