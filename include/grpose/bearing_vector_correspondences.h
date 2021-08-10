#ifndef GRPOSE_GRPOSE_BEARING_VECTOR_CORRESPONDENCES_
#define GRPOSE_GRPOSE_BEARING_VECTOR_CORRESPONDENCES_

#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>

#include "util/types.h"

namespace grpose {

class BearingVectorCorrespondences {
 public:
  using OpengvAdapter = opengv::relative_pose::NoncentralRelativeAdapter;

  BearingVectorCorrespondences() = default;

  void AddCorrespondence(const Vector3 &bearing_vector1,
                         const Vector3 &bearing_vector2, int camera_index1,
                         int camera_index2);

  inline const Vector3 &bearing_vector(int frame_index,
                                       int correspondence_index) const;
  inline int camera_index(int frame_index, int correspondence_index) const;

  // TODO In principle, it should be possible to run OpenGV algorithms for
  // relative pose between different rigs, but then this conversion becomes more
  // cumbersome, since the adapter's class accepts a single set of camera
  // intrinsics.
  OpengvAdapter ToOpengvAdapter(const StdVectorA<SE3> &body_from_cameras) const;

 private:
  StdVectorA<Vector3> bearing_vectors_[2];
  std::vector<int> camera_indices_[2];
};

// Implementation

inline const Vector3 &BearingVectorCorrespondences::bearing_vector(
    int frame_index, int correspondence_index) const {
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

}  // namespace grpose

#endif
