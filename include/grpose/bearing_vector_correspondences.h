#ifndef GRPOSE_GRPOSE_BEARING_VECTOR_CORRESPONDENCES_
#define GRPOSE_GRPOSE_BEARING_VECTOR_CORRESPONDENCES_

#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>

#include "util/sampling.h"
#include "util/types.h"

namespace grpose {

class BearingVectorCorrespondences {
 public:
  using OpengvAdapter = opengv::relative_pose::NoncentralRelativeAdapter;

  BearingVectorCorrespondences() = default;

  void AddCorrespondence(const Vector3 &bearing_vector1,
                         const Vector3 &bearing_vector2, int camera_index1,
                         int camera_index2);

  inline int NumberOfCorrespondences() const;
  inline int NumberOfCameras() const;
  inline const Vector3 &bearing_vector(int frame_index,
                                       int correspondence_index) const;
  inline int camera_index(int frame_index, int correspondence_index) const;
  inline const StdVectorA<Vector3> &bearing_vectors(int frame_index) const;
  inline const std::vector<int> &camera_indices(int frame_index) const;

  template <typename RandomBitsGenerator>
  void AddGaussianDirectionNoise(RandomBitsGenerator &generator,
                                 double angle_std);

 private:
  StdVectorA<Vector3> bearing_vectors_[2];
  std::vector<int> camera_indices_[2];
  int number_of_cameras_ = 0;
};

// Implementation

inline int BearingVectorCorrespondences::NumberOfCorrespondences() const {
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

}  // namespace grpose

#endif