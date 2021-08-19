#include "util/types.h"

namespace grpose {

/**
 * Triangulate two directions in different frames.
 * @param frame2_from_frame1 transformation from the first coordinate system
 * into the second.
 * @param ray1 a direction in the first coordinate system.
 * @param ray2 a direction in the second coordinate system.
 * @param point the resulting triangulated point in the first coordinate system.
 * @param error if not nullptr, the value is set to the distance from the point
 * to either of the lines defined by \p ray1 and \p ray2 (distances are equal).
 */
void Triangulate(const SE3 &frame2_from_frame1, const Vector3 &ray1,
                 const Vector3 &ray2, Vector3 &point, double *error = nullptr);

double Angle(const Vector3 &direction1, const Vector3 &direction2);

}  // namespace grpose