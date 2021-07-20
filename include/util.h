#ifndef INCLUDE_UTIL
#define INCLUDE_UTIL

#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>
#include "types.h"

namespace grpose {

/**
 * A helper function to output a point cloud into a PLY file
 * @param out the stream to output into
 * @param points the vector of points
 * @param colors the corresponding colors
 */
void printInPly(std::ostream &out, const std::vector<Vector3> &points,
                const std::vector<cv::Vec3b> &colors);

/**
 * Read a vector of doubles encoded continuously into a binary file.
 */
std::vector<double> readDoublesFromBin(const fs::path &filename);

/**
 * Write the \p motion into \p out in the 3x4 matrix form in row-major order
 */
void putInMatrixForm(std::ostream &out, const SE3 &motion);

/**
 * Draw a square on \p img with its center at position \p pos. Pass `cv::FILLED`
 * as \p thickness for it to be fiilled.
 * @param size2 half of the size of the side of the square in pixels
 */
void drawSquare(cv::Mat &img, const cv::Point &pos, int size2,
                const cv::Scalar &col, int thickness);

/**
 * Absolute distance between timestamps in microseconds
 */
Timestamp timestampDist(Timestamp a, Timestamp b);

/**
 * Randomly subsample a set of vectors in-place such that the total number of
 * points becomes \p neededTotal, while retaining the approximate proportion of
 * original sizes of vectors.
 *
 * @param v an array of pointers to vectors that we want to sparsify
 */
template <typename T>
void sparsifyVectors(std::vector<T> *v[], int numVectors, int neededTotal) {
  int total =
      std::accumulate(v, v + numVectors, 0,
                      [](int x, std::vector<T> *v) { return x + v->size(); });
  if (total < neededTotal) return;

  std::mt19937 mt;
  for (int i = 0; i < numVectors; ++i)
    std::shuffle(v[i]->begin(), v[i]->end(), mt);

  int cur = 0;
  for (int i = 0; i < numVectors - 1; ++i) {
    int remain = double(v[i]->size()) / total * neededTotal;
    v[i]->resize(remain);
    cur += remain;
  }
  v[numVectors - 1]->resize(neededTotal - cur);
}

/**
 * @return Current date and time in a format suitable for creating files with a
 * name that includes it.
 */
std::string curTimeBrief();

// Some handy conversions
Vector2 toVector2(cv::Point p);

Vector2 toVector2(cv::Point2f p);

cv::Point toCvPoint(const Vector2 &vec);

cv::Vec3b toCvVec3b(cv::Scalar scalar);

cv::Mat1b cvtBgrToGray(const cv::Mat3b &coloredImg);

cv::Mat3b cvtBgrToGray3(const cv::Mat3b &coloredImg);

cv::Mat3b cvtGrayToBgr(const cv::Mat1b &grayImg);

ChronoTimePoint toChronoTimePoint(Timestamp timestamp);

/**
 * Translates the \p timePoint into a string representation with hours, minutes
 * and seconds. This is most certainly shifted and does not take the time zone
 * into account, but it is still useful for logging and debugging purposes.
 */
std::string timeOfDay(ChronoTimePoint timePoint);

}  // namespace grpose

#endif
