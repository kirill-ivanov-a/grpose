#include "dataset/multicam_reader.h"

#include <fstream>

#include <fmt/format.h>

#include "util/util.h"

namespace grpose {

/**
 * Reading the camera model from an intrinsics file as provided by the MultiFoV
 * datasets
 */
Camera GetMfovCam(const fs::path &intrinsics_filename) {
  // Our CameraModel is partially compatible with the provided one (affine
  // transformation used in omni_cam is just scaling in our case, but no problem
  // raises since in this dataset no affine transformation is happening). We
  // also compute the inverse polynomial ourselves instead of using the provided
  // one.

  int width, height;
  double unmap_poly_coeffs[5];
  Vector2 center;
  std::ifstream camera_ifsteram(intrinsics_filename);
  // TODO cleanup throw
  CHECK(camera_ifsteram.is_open()) << "could not open camera intrinsics file \""
                                   << intrinsics_filename.native() << "\"";
  camera_ifsteram >> width >> height;
  for (int i = 0; i < 5; ++i) camera_ifsteram >> unmap_poly_coeffs[i];
  VectorX our_coefficients(4);
  our_coefficients << unmap_poly_coeffs[0], unmap_poly_coeffs[2],
      unmap_poly_coeffs[3], unmap_poly_coeffs[4];
  our_coefficients *= -1;
  camera_ifsteram >> center[0] >> center[1];
  // TODO fix Camera!
  return Camera(width, height, CameraModelId::kInvalid, {});
}

cv::Mat1f ReadBinMat(const fs::path &filename, int image_width,
                     int image_height) {
  std::ifstream depths_ifstream(filename, std::ios::binary);
  CHECK(depths_ifstream.is_open()) << "failed to open file: " << filename;
  cv::Mat1f depths(image_height, image_width);
  for (int y = 0; y < image_height; ++y)
    for (int x = 0; x < image_width; ++x)
      depths_ifstream.read(reinterpret_cast<char *>(&depths(y, x)),
                           sizeof(float));
  return depths;
}

SE3 ReadFromMatrix3x4(std::istream &istream) {
  Matrix34 mat;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c) istream >> mat(r, c);

  Matrix33 R = mat.leftCols<3>();
  double Rdet = R.determinant();
  CHECK_NEAR(Rdet, 1, 1e-6);
  Eigen::JacobiSVD Rsvd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix33 U = Rsvd.matrixU(), V = Rsvd.matrixV();
  Matrix33 Rfixed = U * V.transpose();

  return SE3(Rfixed, mat.col(3));
}

const std::vector<std::string> MultiCamReaderSettings::default_camera_names = {
    "front", "right", "rear", "left"};

const Eigen::AlignedBox2d MultiCamReader::Depths::bounding_box_(
    Vector2::Zero(),
    Vector2(MultiCamReader::kImageWidth - 1, MultiCamReader::kImageHeight - 1));

int MultiCamReaderSettings::NumberOfCameras() const {
  return camera_names.size();
}

MultiCamReader::Depths::Depths(const fs::path &dataset_directory,
                               int frame_index,
                               const MultiCamReaderSettings &settings)
    : settings_(settings) {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, kNumberOfFrames);

  depths_.reserve(settings_.NumberOfCameras());
  for (int ci = 0; ci < settings_.NumberOfCameras(); ++ci) {
    std::string inner_name =
        fmt::format("{:s}_{:04d}.bin", settings_.camera_names[ci], frame_index);
    fs::path depthPath = dataset_directory / "data" / "depth" /
                         settings_.camera_names[ci] / inner_name;
    depths_.push_back(ReadBinMat(depthPath, kImageWidth, kImageHeight));
  }
}

std::optional<double> MultiCamReader::Depths::Depth(
    int camera_index, const Vector2 &point) const {
  if (!bounding_box_.contains(point)) return std::nullopt;
  // TODO cleanup throw
  CHECK_GE(camera_index, 0);
  CHECK_LT(camera_index, settings_.NumberOfCameras());
  return std::make_optional(
      static_cast<double>(depths_[camera_index](ToCvPoint(point))));
}

bool MultiCamReader::IsMultiCam(const fs::path &dataset_directory) {
  fs::path info_dir = dataset_directory / "info";
  fs::path intrinsics_dir = info_dir / "intrinsics";
  fs::path extrinsics_dir = info_dir / "extrinsics";
  fs::path data_dir = dataset_directory / "data";
  fs::path img_dir = data_dir / "img";
  fs::path depth_dir = data_dir / "depth";
  bool isMcam = fs::exists(info_dir / "body_to_world.txt");
  for (const auto &camera_name : MultiCamReaderSettings::default_camera_names) {
    isMcam = isMcam && fs::exists(intrinsics_dir / (camera_name + ".txt"));
    isMcam =
        isMcam && fs::exists(extrinsics_dir / (camera_name + "_to_body.txt"));
    isMcam = isMcam && fs::exists(img_dir / camera_name);
    isMcam = isMcam && fs::exists(depth_dir / camera_name);
  }
  return isMcam;
}

CameraBundle MultiCamReader::CreateCameraBundle(
    const fs::path &dataset_directory,
    const std::vector<std::string> &camera_names) {
  fs::path extrinsics_dir(dataset_directory / "info" / "extrinsics");
  fs::path intrinsics_dir(dataset_directory / "info" / "intrinsics");

  StdVectorA<Camera> cameras;
  StdVectorA<SE3> camera_from_body;
  cameras.reserve(camera_names.size());
  camera_from_body.reserve(camera_names.size());
  for (const auto &camera_name : camera_names) {
    std::ifstream extrinsics_file(extrinsics_dir /
                                  (camera_name + "_to_body.txt"));
    // TODO cleanup throw
    CHECK(extrinsics_file.is_open());
    camera_from_body.push_back(ReadFromMatrix3x4(extrinsics_file).inverse());
    cameras.push_back(GetMfovCam(intrinsics_dir / (camera_name + ".txt")));
  }

  return CameraBundle(camera_from_body.data(), cameras.data(),
                      camera_names.size());
}

StdVectorA<SE3> MultiCamReader::ReadWorldFromBody(
    const fs::path &dataset_directory) {
  std::ifstream trajectory_file(dataset_directory / "info" /
                                "body_to_world.txt");
  StdVectorA<SE3> world_from_body;
  world_from_body.reserve(kNumberOfFrames);
  for (int fi = 0; fi < kNumberOfFrames; ++fi)
    world_from_body.push_back(ReadFromMatrix3x4(trajectory_file));
  return world_from_body;
}

MultiCamReader::MultiCamReader(const fs::path &dataset_directory,
                               const MultiCamReaderSettings &settings)
    : settings_(settings),
      dataset_directory_(dataset_directory),
      camera_bundle_(
          CreateCameraBundle(dataset_directory, settings.camera_names)),
      world_from_body_(ReadWorldFromBody(dataset_directory)) {
  CHECK(IsMultiCam(dataset_directory));
}

int MultiCamReader::NumberOfFrames() const { return kNumberOfFrames; }

int MultiCamReader::FirstTimestampToIndex(Timestamp timestamp) const {
  int ind = int(std::round(double(timestamp) / kFrameTimestampStep));
  CHECK_GE(ind, 0);
  CHECK_LT(ind, kNumberOfFrames);
  return ind;
}

std::vector<Timestamp> MultiCamReader::TimestampsFromIndex(
    int frame_index) const {
  if (frame_index < 0)
    return std::vector<Timestamp>(settings_.NumberOfCameras(), 0);
  if (frame_index >= kNumberOfFrames)
    return std::vector<Timestamp>(settings_.NumberOfCameras(),
                                  kFrameTimestampStep * (kNumberOfFrames - 1));
  return std::vector<Timestamp>(settings_.NumberOfCameras(),
                                kFrameTimestampStep * frame_index);
}

std::vector<fs::path> MultiCamReader::FrameFiles(int frame_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, kNumberOfFrames);

  std::vector<fs::path> fnames;
  fnames.reserve(settings_.NumberOfCameras());
  for (int ci = 0; ci < settings_.NumberOfCameras(); ++ci) {
    std::string inner_name =
        fmt::format("{:s}_{:04d}.jpg", settings_.camera_names[ci], frame_index);
    fs::path imagePath = dataset_directory_ / "data" / "img" /
                         settings_.camera_names[ci] / inner_name;
    fnames.push_back(imagePath);
  }
  return fnames;
}

std::vector<DatasetReader::FrameEntry> MultiCamReader::Frame(
    int frame_index) const {
  // TODO cleanup throw
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, kNumberOfFrames);
  std::vector<DatasetReader::FrameEntry> frame;
  frame.reserve(settings_.NumberOfCameras());
  std::vector<fs::path> fnames = FrameFiles(frame_index);
  for (int ci = 0; ci < settings_.NumberOfCameras(); ++ci) {
    cv::Mat3b image = cv::imread(fnames[ci].string());
    CHECK_NE(image.data, (unsigned char *)nullptr)
        << "path: " << fnames[ci].string();
    frame.push_back({image, Timestamp(frame_index)});
  }
  return frame;
}

CameraBundle MultiCamReader::GetCameraBundle() const { return camera_bundle_; }

std::unique_ptr<FrameDepths> MultiCamReader::GetDepths(int frame_index) const {
  return std::unique_ptr<FrameDepths>(
      new Depths(dataset_directory_, frame_index, settings_));
}

bool MultiCamReader::HasWorldFromFrame(int frame_index) const {
  return frame_index >= 0 && frame_index < kNumberOfFrames;
}

SE3 MultiCamReader::WorldFromFrame(int frame_index) const {
  // TODO cleanup throw
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, world_from_body_.size());
  return world_from_body_[frame_index];
}

Trajectory MultiCamReader::GroundTruthTrajectory() const {
  StdMapA<Timestamp, SE3> timestamped_world_from_frame;
  for (int fi = 0; fi < NumberOfFrames(); ++fi) {
    timestamped_world_from_frame[TimestampsFromIndex(fi)[0]] =
        world_from_body_[fi];
  }
  return Trajectory{timestamped_world_from_frame};
}

}  // namespace grpose
