#include "types.h"
#include <iostream>
#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>
#include <stdio.h>
#include <stdlib.h>

// THIS IS A FILE that initializes using Sophus, and generatesa a cube of points in the
// world and also some cameras in random locations and then uses OpenGV to solve for the
// translation between two frames

// Eigen::Vector3d generateRandomTranslation(double maximumParallax);
// Eigen::Matrix3d generateRandomRotation();
// Eigen::Vector3d generateRandomPoint( double maximumDepth, double minimumDepth );

opengv::translations_t
generate_cube(int cubeDim) { // bearingVectors should have good angular distance
  double spacing = 1;
  opengv::translations_t pts_in_world;
  double offsetFromOrigin = 3; // JULIA: tune this parameter to break the parallax of the
                               // bearing vectors, result should become degenerate
  for (int i = 0; i < cubeDim; i++) {
    for (int j = 0; j < cubeDim; j++) {
      for (int k = 0; k < cubeDim; k++) {
        Eigen::Vector3d cubePoint((double)(offsetFromOrigin + i * spacing),
                                  (double)(offsetFromOrigin + j * spacing),
                                  (double)(k * spacing));
        pts_in_world.push_back(cubePoint);
        // std::cout << "\n" << pts_in_world.back() << std::endl;
      }
    }
  }
  return pts_in_world;
}

Eigen::Vector3d
generateRandomTranslation(double maximumParallax) // maximum parallax is basically the
                                                  // maximum possible translation
{
  Eigen::Vector3d translation;
  translation[0] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  translation[1] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  translation[2] = (((double)rand()) / ((double)RAND_MAX) - 0.5) * 2.0;
  return maximumParallax * translation;
}

Eigen::Matrix3d generateRandomRotation() // TODO sophus implementation
{
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

  double depth = (((double)rand()) / ((double)RAND_MAX)) * (maximumDepth - minimumDepth) +
                 minimumDepth;

  return depth * direction;
}

std::vector<int> getNindices(int n) {
  std::vector<int> indices;
  for (int i = 0; i < n; i++)
    indices.push_back(i);
  return indices;
}

int main() {

  //   opengv::translation_t vec1(1, 0, 0);
  //   Eigen::Matrix3d rot = generateRandomRotation();
  //   Eigen::Vector3d trans(2, 0, 0);
  //   Sophus::SE3 transform(rot, trans);
  //   opengv::translation_t vec2(0, 1, 0);
  //   vec2 = transform * vec1;
  //   std::cout << "vec1\n" << vec1 << std::endl;
  //   std::cout << "vec2\n" << vec2 << std::endl;
  //   std::cout << "rot\n" << rot << std::endl;
  //   std::cout << "trans\n" << trans << std::endl;

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
  double max_offset = 0.5;
  opengv::translations_t cam_offsets;
  opengv::rotations_t cam_rotations;
  for (int i = 0; i < num_cam; i++) {
    cam_offsets.push_back(generateRandomTranslation(max_offset));
    cam_rotations.push_back(generateRandomRotation());
  }

  // get 3D point correspondences
  int num_points = 300;
  double inlier_ratio = 1;
  int num_inliers = (int)floor(inlier_ratio * num_points);
  int num_outliers = num_points - num_inliers;

  std::cout << "num_inliers: " << num_inliers << "   num_outliers: " << num_outliers
            << std::endl
            << std::endl;

  double mindepth = 4;
  double maxdepth = 8;
  opengv::translations_t points_world; // 3d points expressed in world frame
                                       //   for (int i = 0; i < num_inliers; i++) {
  //     points_world.push_back(generateRandomPoint(maxdepth, mindepth));
  //     // std::cout << points_world.back() << std::endl << std::endl;
  //   }

  points_world =
      generate_cube(3); // 27 points defined in the world, cube of dimension 3x3x3
  std::cout << "length " << points_world.size() << std::endl;

  opengv::bearingVectors_t bearing_vectors1;
  opengv::bearingVectors_t bearing_vectors2;
  std::vector<int> cam_correspondences1;
  std::vector<int> cam_correspondences2;

  int correspondence = 0;
  for (int i = 0; i < points_world.size(); i++) {

    // pick the camera we're doing this correspondence in
    auto cam_offset = cam_offsets[correspondence];
    auto cam_rotation = cam_rotations[correspondence];

    // points are in world frame and thus already in viewpoint 1 frame
    Eigen::Vector3d point_v1 = points_world[i];
    // std::cout << "\n point_v1 \n" << point_v1 << std::endl;
    // get point in viewpoint 2
    Sophus::SE3 vp1_vp2(rotation2, position2);
    Eigen::Vector3d point_v2 = vp1_vp2.inverse() * point_v1;
    Eigen::Vector3d point_v2_test = rotation2.transpose() * (point_v1 - position2);

    // std::cout << "\n point_v2 \n " << point_v2 << std::endl;

    // get the point in the camera in viewpoint 1

    Sophus::SE3 camSE3(cam_rotation, cam_offset);
    bearing_vectors1.push_back(camSE3.inverse() * point_v1);
    // std::cout << "\nONE " << camSE3.inverse() * point_v1 << std::endl;
    // std::cout << "\nTWO " << cam_rotation.transpose() * (point_v1 - cam_offset)
    //   << std::endl;
    // bearing_vectors1.push_back(cam_rotation.transpose() * (point_v1 -
    //   cam_offset));

    // get the point in the camera in viewpoint 2
    bearing_vectors2.push_back(camSE3.inverse() * point_v2);
    // std::cout << "\nTHREE " << camSE3.inverse() * point_v2 << std::endl;
    // std::cout << "\nFOUR " << cam_rotation.transpose() * (point_v2 - cam_offset)
    //   << std::endl;
    // bearing_vectors2.push_back(cam_rotation.transpose() * (point_v2 -
    //   cam_offset));

    // IMPORTANT !!! normalize the bearing vectors

    bearing_vectors1.back().normalize();
    bearing_vectors2.back().normalize();
    std::cout << "bearing vec 1 \n"
              << (camSE3.rotationMatrix() * bearing_vectors1.back()) << std::endl;
    std::cout << "bearing vec 2 \n"
              << (vp1_vp2.rotationMatrix() * camSE3.rotationMatrix() *
                  bearing_vectors2.back())
              << std::endl;

    // std::cout << bearingVectors1.back().norm() << std::endl;
    // std::cout << bearingVectors1.back().norm() << std::endl;

    // push back the camera correspondences
    cam_correspondences1.push_back(correspondence);
    cam_correspondences2.push_back(correspondence++);

    // spread the correspondences between the cameras
    if (correspondence > (num_cam - 1))
      correspondence = 0;
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
    if (correspondence > (num_cam - 1))
      correspondence = 0;
  }

  // create the non-central relative adapter
  opengv::relative_pose::NoncentralRelativeAdapter adapter(
      bearing_vectors1, bearing_vectors2, cam_correspondences1, cam_correspondences2,
      cam_offsets, cam_rotations);

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

  // with RANSAC
  opengv::sac::Ransac<
      opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem>
      ransac;
  // create a NoncentralRelativePoseSacProblem
  std::shared_ptr<opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem>
      relposeproblem_ptr(
          new opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem(
              adapter, opengv::sac_problems::relative_pose::
                           NoncentralRelativePoseSacProblem::SEVENTEENPT));
  // run ransac
  ransac.sac_model_ = relposeproblem_ptr;
  ransac.threshold_ = 0.9;
  ransac.max_iterations_ = 100;
  ransac.computeModel();

  // get the result
  opengv::transformation_t best_transformation = ransac.model_coefficients_;

  std::cout << best_transformation << std::endl;
}
