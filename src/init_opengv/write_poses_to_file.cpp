#include "init_opengv/write_poses_to_file.h"
#include "util/util.h"

namespace grpose {

void WritePosesToFile(const std::string &path_to_segment,
                      const grpose::NonCentralRelativePoseSolution &solution,
                      int first_frame_index, int second_frame_index,
                      const std::string &folder_parameters) {
  // Need to output timestamp, pose (px, py, pz, qx, qy, qz, qw)
  // Also a file of 3D points for the triangulated points
  std::string folder =
      path_to_segment + "_initialization_output" + folder_parameters;
  folder = folder + "/" + std::to_string(solution.timestamps.first) + "_" +
           std::to_string(solution.timestamps.second);

  grpose::fs::create_directories("/" + folder);

  /* Writing found relative poses */
  std::ofstream relative_pose_file;
  std::string rpf_file = folder + "/relative_pose.txt";
  std::cout << "Writing to " << rpf_file << std::endl;
  relative_pose_file.open(rpf_file);
  relative_pose_file << solution.timestamps.second << " ";
  relative_pose_file << solution.first_frame_from_second_frame.translation().x()
                     << " ";
  relative_pose_file << solution.first_frame_from_second_frame.translation().y()
                     << " ";
  relative_pose_file << solution.first_frame_from_second_frame.translation().z()
                     << " ";
  relative_pose_file
      << solution.first_frame_from_second_frame.unit_quaternion().x() << " ";
  relative_pose_file
      << solution.first_frame_from_second_frame.unit_quaternion().y() << " ";
  relative_pose_file
      << solution.first_frame_from_second_frame.unit_quaternion().z() << " ";
  relative_pose_file
      << solution.first_frame_from_second_frame.unit_quaternion().w() << " ";
  relative_pose_file.close();

  /* Writing ground truth relative poses */
  std::ofstream gt_relative_pose_file;
  std::string gt_rpf_path = folder + "/relative_pose_gt.txt";
  std::cout << "Writing to " << gt_rpf_path << std::endl;
  gt_relative_pose_file.open(gt_rpf_path);
  gt_relative_pose_file << solution.timestamps.second << " ";
  gt_relative_pose_file << solution.ground_truth_relative_pose.translation().x()
                        << " ";
  gt_relative_pose_file << solution.ground_truth_relative_pose.translation().y()
                        << " ";
  gt_relative_pose_file << solution.ground_truth_relative_pose.translation().z()
                        << " ";
  gt_relative_pose_file
      << solution.ground_truth_relative_pose.unit_quaternion().x() << " ";
  gt_relative_pose_file
      << solution.ground_truth_relative_pose.unit_quaternion().y() << " ";
  gt_relative_pose_file
      << solution.ground_truth_relative_pose.unit_quaternion().z() << " ";
  gt_relative_pose_file
      << solution.ground_truth_relative_pose.unit_quaternion().w() << " ";
  gt_relative_pose_file.close();

  /* Writing .ply of triangulated points between two frames */
  std::filebuf tri_pts_fb;
  std::string tri_pts_path = folder + "/triangulated_points.txt";
  tri_pts_fb.open(tri_pts_path, std::ios::out);
  std::ostream os(&tri_pts_fb);
  std::vector<grpose::Vector3>
      tri_pts;  // Reformat triangulated points and make colors
  std::vector<cv::Vec3b> colors;
  for (int i = 0; i < solution.triangulated_points.size(); i++) {
    tri_pts.push_back(solution.triangulated_points[i]);
    colors.push_back(cv::Vec3b(0, 0, 0));
  }
  PrintInPly(os, tri_pts, colors);
  tri_pts_fb.close();

  /* Writing a status.txt file */
  std::ofstream status_file;
  std::string status_path = folder + "/status.txt";
  std::cout << "Writing to " << status_path << std::endl;
  status_file.open(status_path);
  status_file << solution.status << "\n";
  status_file << path_to_segment << "\n";
  status_file << first_frame_index << "\n";
  status_file << second_frame_index;
  status_file.close();
}

}  // namespace grpose