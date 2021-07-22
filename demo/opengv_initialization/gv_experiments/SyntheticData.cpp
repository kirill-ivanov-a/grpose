#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sophus/se3.hpp>

#include "init_opengv/NonCentralRelativePoseSolver.h"

// THIS IS A FILE with working Sophus initialization for some poses, and then
// uses OpenGV to solve for the translation and rotation between poses

// Eigen::Vector3d generateRandomTranslation(double maximumParallax);
// Eigen::Matrix3d generateRandomRotation();
// Eigen::Vector3d generateRandomPoint( double maximumDepth, double minimumDepth
// );

// maximum parallax is basically the maximum possible translation
Eigen::Vector3d generateRandomTranslation(double maximumParallax) {
  Eigen::Vector3d translation;
  translation[0] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  translation[1] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  translation[2] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  return maximumParallax * translation;
}

Eigen::Matrix3d generateRandomRotation() {
  Eigen::Vector3d rpy;
  rpy[0] = ((double)rand()) / ((double)RAND_MAX);
  rpy[1] = ((double)rand()) / ((double)RAND_MAX);
  rpy[2] = ((double)rand()) / ((double)RAND_MAX);

  rpy[0] = 2 * M_PI * (rpy[0] - 0.5);
  rpy[1] = M_PI * (rpy[1] - 0.5);
  rpy[2] = 2 * M_PI * (rpy[2] - 0.5);

  Eigen::Matrix3d R1;
  R1(0, 0) = 1.0;
  R1(0, 1) = 0.0;
  R1(0, 2) = 0.0;
  R1(1, 0) = 0.0;
  R1(1, 1) = cos(rpy[0]);
  R1(1, 2) = -sin(rpy[0]);
  R1(2, 0) = 0.0;
  R1(2, 1) = -R1(1, 2);
  R1(2, 2) = R1(1, 1);

  Eigen::Matrix3d R2;
  R2(0, 0) = cos(rpy[1]);
  R2(0, 1) = 0.0;
  R2(0, 2) = sin(rpy[1]);
  R2(1, 0) = 0.0;
  R2(1, 1) = 1.0;
  R2(1, 2) = 0.0;
  R2(2, 0) = -R2(0, 2);
  R2(2, 1) = 0.0;
  R2(2, 2) = R2(0, 0);

  Eigen::Matrix3d R3;
  R3(0, 0) = cos(rpy[2]);
  R3(0, 1) = -sin(rpy[2]);
  R3(0, 2) = 0.0;
  R3(1, 0) = -R3(0, 1);
  R3(1, 1) = R3(0, 0);
  R3(1, 2) = 0.0;
  R3(2, 0) = 0.0;
  R3(2, 1) = 0.0;
  R3(2, 2) = 1.0;

  Eigen::Matrix3d rotation = R3 * R2 * R1;

  rotation.col(0) = rotation.col(0) / rotation.col(0).norm();
  rotation.col(2) = rotation.col(0).cross(rotation.col(1));
  rotation.col(2) = rotation.col(2) / rotation.col(2).norm();
  rotation.col(1) = rotation.col(2).cross(rotation.col(0));
  rotation.col(1) = rotation.col(1) / rotation.col(1).norm();

  return rotation;
}

Eigen::Vector3d generateRandomPoint(double maximumDepth, double minimumDepth) {
  Eigen::Vector3d cleanPoint;
  cleanPoint[0] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  cleanPoint[1] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  cleanPoint[2] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  Eigen::Vector3d direction = cleanPoint / cleanPoint.norm();

  double depth =
      (((double)rand()) / ((double)RAND_MAX)) * (maximumDepth - minimumDepth) +
      minimumDepth;

  return depth * direction;
}

std::vector<int> getNindices(int n) {
  std::vector<int> indices;
  for (int i = 0; i < n; i++) indices.push_back(i);
  return indices;
}

int main() {
  // init random seed
  srand(10);

  // initialize viewpoint 1 as the world frame
  opengv::translation_t position1 = Eigen::Vector3d::Zero();
  opengv::rotation_t rotation1 = Eigen::Matrix3d::Identity();

  // init viewpoint 2 at random pose from the origin
  opengv::translation_t position2 = generateRandomTranslation(2.0);
  opengv::rotation_t rotation2 = generateRandomRotation();

  // create the central camera system of the viewpoints
  int num_cam = 3;
  double max_offset = 0.9;
  opengv::translations_t cam_offsets;
  opengv::rotations_t cam_rotations;
  for (int i = 0; i < num_cam; i++) {
    cam_offsets.push_back(generateRandomTranslation(max_offset));
    cam_rotations.push_back(generateRandomRotation());
  }

  // get 3D point correspondences
  int num_points = 300;
  double inlier_ratio = 0.8;
  int num_inliers = (int)floor(inlier_ratio * num_points);
  int num_outliers = num_points - num_inliers;

  std::cout << "num_inliers: " << num_inliers
            << "   num_outliers: " << num_outliers << std::endl
            << std::endl;

  double mindepth = 4;
  double maxdepth = 8;
  opengv::translations_t points_world;  // 3d points expressed in world frame
  for (int i = 0; i < num_inliers; i++) {
    points_world.push_back(generateRandomPoint(maxdepth, mindepth));
    // std::cout << points_world.back() << std::endl << std::endl;
  }

  opengv::bearingVectors_t bearing_vectors1;
  opengv::bearingVectors_t bearing_vectors2;
  std::vector<int> cam_correspondences1;
  std::vector<int> cam_correspondences2;

  int correspondence = 0;
  for (int i = 0; i < num_inliers; i++) {
    // pick the camera we're doing this correspondence in
    auto cam_offset = cam_offsets[correspondence];
    auto cam_rotation = cam_rotations[correspondence];

    // points are in world frame and thus already in viewpoint 1 frame
    auto point_v1 = points_world[i];

    // get point in viewpoint 2
    auto point_v2 = rotation2.transpose() * (point_v1 - position2);

    // get the point in the camera in viewpoint 1
    bearing_vectors1.push_back(cam_rotation.transpose() *
                               (point_v1 - cam_offset));

    // get the point in the camera in viewpoint 2
    bearing_vectors2.push_back(cam_rotation.transpose() *
                               (point_v2 - cam_offset));

    // IMPORTANT !!! normalize the bearing vectors
    bearing_vectors1.back().normalize();
    bearing_vectors2.back().normalize();

    // std::cout << bearingVectors1.back().norm() << std::endl;
    // std::cout << bearingVectors1.back().norm() << std::endl;

    // push back the camera correspondences
    cam_correspondences1.push_back(correspondence);
    cam_correspondences2.push_back(correspondence++);

    // spread the correspondences between the cameras
    if (correspondence > (num_cam - 1)) correspondence = 0;
  }

  // Add some outlier bearing correspondences
  correspondence = 0;
  for (int i = 0; i < num_outliers; i++) {
    // generate totally incorrect bearing and say that they're a correspondence
    bearing_vectors1.push_back(generateRandomPoint(1.0, 1.0).normalized());
    bearing_vectors2.push_back(generateRandomPoint(1.0, 1.0).normalized());

    // push back the camera correspondences
    cam_correspondences1.push_back(correspondence);
    cam_correspondences2.push_back(correspondence++);

    // spread the correspondences between the cameras
    if (correspondence > (num_cam - 1)) correspondence = 0;
  }

  // create the non-central relative adapter
  opengv::relative_pose::NoncentralRelativeAdapter adapter(
      bearing_vectors1, bearing_vectors2, cam_correspondences1,
      cam_correspondences2, cam_offsets, cam_rotations);

  // 17-point algorithm
  opengv::transformation_t seventeenpt_transformation =
      opengv::relative_pose::seventeenpt(adapter);

  std::cout << seventeenpt_transformation << std::endl << std::endl;
  std::cout << rotation2 << std::endl << std::endl;
  std::cout << position2 << std::endl << std::endl;

  // 6-point algorithm
  opengv::rotations_t sixpt_rotations =
      opengv::relative_pose::sixpt(adapter, getNindices(6));
  std::cout << sixpt_rotations.size() << std::endl << std::endl;

  // create like a virtual focal length to initialize the ransac threshold
  double focal_length = 500;
  grpose::NonCentralRelativePoseSolverSettings solver_settings(focal_length);

  grpose::StdVectorA<Sophus::SE3d> cam_extrinsics;
  for (int i = 0; i < num_cam; i++) {
    cam_extrinsics.push_back(Sophus::SE3d(cam_rotations[i], cam_offsets[i]));
  }
  grpose::NonCentralRelativePoseSolver solver(solver_settings, cam_extrinsics);

  auto solution = solver.solve(bearing_vectors1, bearing_vectors2,
                               cam_correspondences1, cam_correspondences2);

  auto triangulated_points_world = solution.triangulated_points;

  // Ordering of points is lost, so for each triangulated point, find it's
  // nearest neighbour in the actual points
  for (const auto &tp : triangulated_points_world) {
    opengv::point_t nn;
    float best_error = std::numeric_limits<float>::max();

    for (const auto &p : points_world) {
      if ((tp - p).norm() < best_error) {
        best_error = (tp - p).norm();
        nn = p;
      }
    }
    std::cout << "triangulated point" << std::endl;
    std::cout << tp << std::endl;
    std::cout << "nearest actual point" << std::endl;
    std::cout << nn << std::endl;
    std::cout << "error" << std::endl;
    std::cout << best_error << std::endl << std::endl;
  }
}
