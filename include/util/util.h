#ifndef GRPOSE_UTIL_UTIL_
#define GRPOSE_UTIL_UTIL_

#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "types.h"

namespace grpose {

/**
 * A helper function to output a point cloud into a PLY file
 * @param out_stream the stream to output into
 * @param points the vector of points
 * @param colors the corresponding colors
 */
void PrintInPly(std::ostream &out_stream, const std::vector<Vector3> &points,
                const std::vector<cv::Vec3b> &colors);

/**
 * Read a vector of doubles encoded continuously into a binary file.
 */
std::vector<double> ReadDoublesFromBin(const fs::path &filename);

/**
 * Write the \p motion into \p out_stream in the 3x4 matrix form in row-major
 * order
 */
void PutInMatrixForm(std::ostream &out_stream, const SE3 &motion);

/**
 * Draw a square on \p image with its center at position \p position. Pass
 * `cv::FILLED` as \p thickness for it to be fiilled.
 * @param size2 half of the size of the side of the square in pixels
 */
void DrawSquare(cv::Mat &image, const cv::Point &position, int size2,
                const cv::Scalar &color, int thickness);

class CentralPoint2dCorrespondences;
cv::Mat3b DrawMatches(const CentralPoint2dCorrespondences &correspondences,
                      const cv::Mat3b &frame1, const cv::Mat3b &frame2,
                      int point_radius = 5);

/**
 * Absolute distance between timestamps in microseconds
 */
Timestamp TimestampDistance(Timestamp a, Timestamp b);

/**
 * Randomly subsample a set of vectors in-place such that the total number of
 * points becomes \p needed_total, while retaining the approximate proportion of
 * original sizes of vectors.
 *
 * @param v an array of pointers to vectors that we want to sparsify
 */
template <typename T>
void SparsifyVectors(std::vector<T> *v[], int number_of_vectors,
                     int needed_total) {
  int total =
      std::accumulate(v, v + number_of_vectors, 0,
                      [](int x, std::vector<T> *v) { return x + v->size(); });
  if (total < needed_total) return;

  std::mt19937 mt;
  for (int i = 0; i < number_of_vectors; ++i)
    std::shuffle(v[i]->begin(), v[i]->end(), mt);

  int cur = 0;
  for (int i = 0; i < number_of_vectors - 1; ++i) {
    int remain = double(v[i]->size()) / total * needed_total;
    v[i]->resize(remain);
    cur += remain;
  }
  v[number_of_vectors - 1]->resize(needed_total - cur);
}

template <typename T>
T Average(const std::vector<T> &vector) {
  if (vector.empty())
    throw std::out_of_range("Computing Average for an empty vector!");
  T sum = std::accumulate(vector.begin() + 1, vector.end(), vector[0]);
  return sum / static_cast<T>(vector.size());
}

/**
 * Linearly map values from \p values to colors according to \p colormap
 */
std::vector<cv::Vec3b> GetColors(std::vector<double> values, double min_value,
                                 double max_value,
                                 const cv::ColormapTypes &colormap);

std::vector<std::string> SplitByComma(const std::string &comma_list);

/**
 * @return Current date and time in a format suitable for creating files
 * with a name that includes it.
 */
std::string CurrentTimeBrief();

Vector2 ToVector2(cv::Point p);
Vector2 ToVector2(cv::Point2f p);
cv::Point ToCvPoint(const Vector2 &vec);
cv::Vec3b ToCvVec3b(const cv::Scalar &scalar);
cv::Mat1b ConvertBgrToGray(const cv::Mat3b &colored_image);
cv::Mat3b ConvertBgrToGray3(const cv::Mat3b &colored_image);
cv::Mat3b ConvertGrayToBgr(const cv::Mat1b &grayscale_image);
ChronoTimePoint ToChronoTimePoint(Timestamp timestamp);

/**
 * Translates the \p time_point into a string representation with hours, minutes
 * and seconds. This is most certainly shifted and does not take the time zone
 * into account, but it is still useful for logging and debugging purposes.
 */
std::string TimeOfDay(ChronoTimePoint time_point);
inline std::string TimeOfDay(Timestamp timestamp) {
  return TimeOfDay(ToChronoTimePoint(timestamp));
}

void SaveArgv(const fs::path &filename, int argc, char *argv[]);

bool GetRelativePath(const fs::path &directory, const fs::path &absolute_path,
                     fs::path &relative_path);

template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
  return !((x.array() == x.array())).all();
}

}  // namespace grpose

#endif
