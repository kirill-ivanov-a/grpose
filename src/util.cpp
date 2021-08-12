#include "util/util.h"

#include <iostream>
#include <vector>

#include <glog/logging.h>

#include "util/types.h"

namespace grpose {

void PrintInPly(std::ostream &out_stream, const std::vector<Vector3> &points,
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

  out_stream.precision(15);
  out_stream << R"__(ply
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

    out_stream << p[0] << ' ' << p[1] << ' ' << p[2] << ' ';
    out_stream << int(color[2]) << ' ' << int(color[1]) << ' ' << int(color[0])
               << '\n';
  }
}

std::vector<double> ReadDoublesFromBin(const fs::path &filename) {
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

void PutInMatrixForm(std::ostream &out_stream, const SE3 &motion) {
  Eigen::Matrix<double, 3, 4, Eigen::RowMajor> pose = motion.matrix3x4();
  for (int i = 0; i < 12; ++i) out_stream << pose.data()[i] << ' ';
  out_stream << '\n';
}

void DrawSquare(cv::Mat &image, const cv::Point &position, int size2,
                const cv::Scalar &color, int thickness) {
  cv::Point from = position - cv::Point(size2, size2);
  cv::Point to = position + cv::Point(size2, size2);

  if (size2 > 0)
    cv::rectangle(image, from, to, color, thickness);
  else if (size2 == 0)
    image.at<cv::Vec3b>(from) = ToCvVec3b(color);
}

std::vector<cv::Vec3b> GetColors(std::vector<double> values, double min_value,
                                 double max_value,
                                 const cv::ColormapTypes &colormap) {
  cv::Mat1b values_mat(values.size(), 1);
  for (int i = 0; i < values.size(); ++i)
    values_mat(i, 0) = (unsigned char)(std::clamp(
        255 * (values[i] - min_value) / (max_value - min_value), 0.1, 254.0));
  cv::Mat3b colors_mat;
  cv::applyColorMap(values_mat, colors_mat, colormap);
  std::vector<cv::Vec3b> colors(values.size());
  for (int i = 0; i < values.size(); ++i) colors[i] = colors_mat(i, 0);
  return colors;
}

std::string CurrentTimeBrief() {
  constexpr int kMaxTimedOutputStringSize = 22;
  // TODO cleanup fmt
  char current_time[kMaxTimedOutputStringSize + 2];
  std::time_t t = std::time(nullptr);
  std::strftime(current_time, kMaxTimedOutputStringSize + 1, "%Y%m%d_%H%M%S",
                std::localtime(&t));
  return std::string(current_time);
}

Vector2 ToVector2(cv::Point p) {
  return Vector2(static_cast<double>(p.x), static_cast<double>(p.y));
}

Vector2 ToVector2(cv::Point2f p) {
  return Vector2(static_cast<double>(p.x), static_cast<double>(p.y));
}

cv::Point ToCvPoint(const Vector2 &vec) {
  return cv::Point(static_cast<int>(vec[0]), static_cast<int>(vec[1]));
}

cv::Vec3b ToCvVec3b(const cv::Scalar &scalar) {
  return cv::Vec3b(scalar[0], scalar[1], scalar[2]);
}

cv::Mat1b ConvertBgrToGray(const cv::Mat3b &colored_image) {
  cv::Mat1b result;
  cv::cvtColor(colored_image, result, cv::COLOR_BGR2GRAY);
  return result;
}

cv::Mat3b ConvertBgrToGray3(const cv::Mat3b &colored_image) {
  cv::Mat1b result_1b;
  cv::cvtColor(colored_image, result_1b, cv::COLOR_BGR2GRAY);
  cv::Mat3b result_3b;
  cv::cvtColor(result_1b, result_3b, cv::COLOR_GRAY2BGR);
  return result_3b;
}

cv::Mat3b ConvertGrayToBgr(const cv::Mat1b &grayscale_image) {
  cv::Mat3b result;
  cv::cvtColor(grayscale_image, result, cv::COLOR_GRAY2BGR);
  return result;
}

ChronoTimePoint ToChronoTimePoint(Timestamp timestamp) {
  return std::chrono::time_point<std::chrono::high_resolution_clock,
                                 std::chrono::microseconds>(
      std::chrono::microseconds(timestamp));
}

std::string TimeOfDay(ChronoTimePoint time_point) {
  using namespace std::chrono;
  using days = duration<int64_t, std::ratio<86400>>;

  ChronoTimePoint day = floor<days>(time_point);
  ChronoTimePoint hour = floor<hours>(time_point);
  ChronoTimePoint minute = floor<minutes>(time_point);
  ChronoTimePoint second = floor<seconds>(time_point);

  long h = duration_cast<hours>(hour - day).count();
  long m = duration_cast<minutes>(minute - hour).count();
  long s = duration_cast<seconds>(second - minute).count();
  long mcs = duration_cast<microseconds>(time_point - second).count();
  // TODO cleanup fmt
  return std::to_string(h) + ":" + std::to_string(m) + ":" +
         std::to_string(static_cast<double>(s + mcs * 1e-6));
}

}  // namespace grpose
