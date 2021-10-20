#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "features/colmap_database.h"

using namespace grpose;

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./draw_extracted_matches autovision_root autovision_config database database_root
where autovision_root is the path to SP20 directory and autovision_config is the path to the configuration JSON file, database is the path to the COLMAP database containing matches, and database_root is the directory that was used to produce the database.
      )abacaba";

  fs::path output_directory =
      fs::path("output") /
      ("central_refinement_benchmark_" + CurrentTimeBrief());
  fs::create_directories(output_directory);
  std::cout << "output dir: " << output_directory.string() << std::endl;
  SaveArgv(output_directory / "argv.txt", argc, argv);

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);
  CHECK_EQ(argc, 3) << usage;


  return 0;
}
