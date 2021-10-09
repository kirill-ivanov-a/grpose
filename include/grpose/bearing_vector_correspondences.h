#ifndef GRPOSE_GRPOSE_BEARING_VECTOR_CORRESPONDENCES_
#define GRPOSE_GRPOSE_BEARING_VECTOR_CORRESPONDENCES_

#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>

#include "util/sampling.h"
#include "util/types.h"

namespace grpose {

class BearingVectorCorrespondences {
 public:
  BearingVectorCorrespondences() = default;

  void Add(const Vector3 &bearing_vector1, const Vector3 &bearing_vector2,
           int camera_index1, int camera_index2);

  inline int Size() const;
  inline int NumberOfCameras() const;
  inline const Vector3 &bearing_vector(int frame_index,
                                       int correspondence_index) const;
  inline int camera_index(int frame_index, int correspondence_index) const;
  inline const StdVectorA<Vector3> &bearing_vectors(int frame_index) const;
  inline const std::vector<int> &camera_indices(int frame_index) const;

  template <typename RandomBitsGenerator>
  void AddGaussianDirectionNoise(RandomBitsGenerator &generator,
                                 double angle_std);

  /**
   * Mixes \p other randomly into this.
   * @return a vector v of bool. If on the i-th position there is an old
   * correspondence, v[i] is true, otherwise it is false.
   */
  template <typename RandomBitsGenerator>
  std::vector<bool> MixIn(RandomBitsGenerator &generator,
                          const BearingVectorCorrespondences &other);

 private:
  StdVectorA<Vector3> bearing_vectors_[2];
  std::vector<int> camera_indices_[2];
  int number_of_cameras_ = 0;
};

// Implementation

inline int BearingVectorCorrespondences::Size() const {
  return static_cast<int>(bearing_vectors_[0].size());
}

inline int BearingVectorCorrespondences::NumberOfCameras() const {
  return number_of_cameras_;
}

inline const Vector3 &BearingVectorCorrespondences::bearing_vector(
    int frame_index, int correspondence_index) const {
  // TODO proper assert
  if (frame_index < 0 || frame_index > 1)
    throw std::out_of_range(
        fmt::format("BearingVectorCorrespondences::bearing_vector: Index {:d} "
                    "out of range [0, 2)",
                    frame_index));
  if (correspondence_index < 0 ||
      correspondence_index >= bearing_vectors_[frame_index].size())
    throw std::out_of_range(fmt::format(
        "BearingVectorCorrespondences::bearing_vector: Index {:d} out of range "
        "[0, {:d})",
        correspondence_index, bearing_vectors_[frame_index].size()));

  return bearing_vectors_[frame_index][correspondence_index];
}

inline int BearingVectorCorrespondences::camera_index(
    int frame_index, int correspondence_index) const {
  // TODO proper assert
  if (frame_index < 0 || frame_index > 1)
    throw std::out_of_range(
        fmt::format("BearingVectorCorrespondences::bearing_vector: Index {:d} "
                    "out of range [0, 2)",
                    frame_index));
  if (correspondence_index < 0 ||
      correspondence_index >= bearing_vectors_[frame_index].size())
    throw std::out_of_range(fmt::format(
        "BearingVectorCorrespondences::bearing_vector: Index {:d} out of range "
        "[0, {:d})",
        correspondence_index, bearing_vectors_[frame_index].size()));

  return camera_indices_[frame_index][correspondence_index];
}

inline const StdVectorA<Vector3> &BearingVectorCorrespondences::bearing_vectors(
    int frame_index) const {
  // TODO proper assert
  if (frame_index < 0 || frame_index > 1)
    throw std::out_of_range(
        fmt::format("BearingVectorCorrespondences::bearing_vector: Index {:d} "
                    "out of range [0, 2)",
                    frame_index));
  return bearing_vectors_[frame_index];
}

inline const std::vector<int> &BearingVectorCorrespondences::camera_indices(
    int frame_index) const {
  // TODO proper assert
  if (frame_index < 0 || frame_index > 1)
    throw std::out_of_range(
        fmt::format("BearingVectorCorrespondences::bearing_vector: Index {:d} "
                    "out of range [0, 2)",
                    frame_index));
  return camera_indices_[frame_index];
}

template <typename RandomBitsGenerator>
void BearingVectorCorrespondences::AddGaussianDirectionNoise(
    RandomBitsGenerator &generator, double angle_std) {
  for (int fi = 0; fi < 2; ++fi)
    for (Vector3 &vector : bearing_vectors_[fi])
      vector = grpose::AddGaussianDirectionNoise(generator, vector, angle_std);
}

template <typename RandomBitsGenerator>
std::vector<bool> BearingVectorCorrespondences::MixIn(
    RandomBitsGenerator &generator, const BearingVectorCorrespondences &other) {
  std::vector<bool> is_old(Size() + other.Size(), false);
  for (int i = 0; i < Size(); ++i) is_old[i] = true;
  std::shuffle(is_old.begin(), is_old.end(), generator);

  BearingVectorCorrespondences new_correspondences;
  int this_index = 0, other_index = 0;
  for (int v : is_old) {
    if (v) {
      new_correspondences.Add(
          bearing_vector(0, this_index), bearing_vector(1, this_index),
          camera_index(0, this_index), camera_index(1, this_index));
      this_index++;
    } else {
      new_correspondences.Add(other.bearing_vector(0, other_index),
                              other.bearing_vector(1, other_index),
                              other.camera_index(0, other_index),
                              other.camera_index(1, other_index));
      other_index++;
    }
  }
  CHECK_EQ(this_index, Size());
  CHECK_EQ(other_index, other.Size());

  *this = std::move(new_correspondences);

  return is_old;
}

}  // namespace grpose

#endif
