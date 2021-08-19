#include "util/geometry.h"

namespace grpose {

void Triangulate(const SE3 &frame2_from_frame1, const Vector3 &ray1,
                 const Vector3 &ray2, Vector3 &point, double *error) {
  Matrix32 A;
  A.col(0) = frame2_from_frame1.so3() * ray1;
  A.col(1) = -ray2;

  const Vector2 multipliers =
      A.fullPivHouseholderQr().solve(-frame2_from_frame1.translation());
  const Vector3 p1 = multipliers[0] * ray1;
  const Vector3 p2 = frame2_from_frame1.inverse() * (multipliers[1] * ray2);
  point = (p1 + p2) / 2;
  if (error) *error = (point - p1).norm();
}

double Angle(const Vector3 &direction1, const Vector3 &direction2) {
  const double cos_angle = direction1.normalized().dot(direction2.normalized());
  return std::acos(std::clamp(cos_angle, -1.0, 1.0));
}

}  // namespace grpose
