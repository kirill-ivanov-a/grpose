#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "grpose/bearing_vector_correspondences.h"
#include "grpose/opengv_adapter.h"
#include "grpose/opengv_solver.h"
#include "synthetic/car_like_scene.h"

DEFINE_string(method_name, "8pt",
              "Solver to be used. Expected to be one of [6pt, 8pt, 17pt]");
DEFINE_int32(num_corresps, 1000, "Number of correspondences to generate");

using namespace grpose;

OpengvSolver::Algorithm NameToAlgorithm(const std::string &method_name) {
  if (method_name == "6pt")
    return OpengvSolver::Algorithm::kSixPoint;
  else if (method_name == "8pt")
    return OpengvSolver::Algorithm::kGeneralizedEigensolver;
  else if (method_name == "17pt")
    return OpengvSolver::Algorithm::kSeventeenPoint;
  else
    throw std::domain_error(fmt::format("Unknown method \"{}\"", method_name));
}

int main(int argc, char *argv[]) {
  std::string usage = R"abacaba(Usage:   ./numeric_stability)abacaba";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  auto *car_like_scene = new synthetic::CarLikeScene();
  car_like_scene->SetLengthwiseMotion(15.0);
  car_like_scene->SetTurnAngle(M_PI / 180.0 * 30.0);
  std::unique_ptr<synthetic::Scene> scene(car_like_scene);
  std::shared_ptr correspondences =
      std::make_shared<BearingVectorCorrespondences>(
          scene->GetBearingVectorCorrespondences(FLAGS_num_corresps));
  std::shared_ptr opengv_adapter = std::make_shared<OpengvAdapter>(
      correspondences, scene->GetBodyFromCameras());

  std::unique_ptr<NonCentralRelativePoseSolver> solver(
      new OpengvSolver(opengv_adapter, NameToAlgorithm(FLAGS_method_name)));

  const SE3 true_frame1_from_frame2 =
      scene->GetWorldFromBody(0).inverse() * scene->GetWorldFromBody(1);
  std::cout << "GT:\n" << true_frame1_from_frame2.matrix3x4() << std::endl;
  StdVectorA<SE3> frame1_from_frame2;

  std::uniform_int_distribution ind(0, FLAGS_num_corresps);
  std::mt19937 mt{std::random_device()()};
  std::vector<int> correspondence_indices;
  for (int i = 0; i < 17; ++i) correspondence_indices.push_back(ind(mt));
  std::sort(correspondence_indices.begin(), correspondence_indices.end());
  std::cout << fmt::format("indices: {}\n",
                           fmt::join(correspondence_indices, ", "))
            << std::endl;
  solver->Solve(correspondence_indices, frame1_from_frame2);
  std::cout << "est:\n" << frame1_from_frame2[0].matrix3x4() << std::endl;
  return 0;
}
