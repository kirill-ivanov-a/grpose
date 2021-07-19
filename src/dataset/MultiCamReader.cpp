#include "dataset/MultiCamReader.h"
#include <fstream>
#include "util.h"

namespace mcam {

/**
 * Reading the camera model from an intrinsics file as provided by the MultiFoV
 * datasets
 */
CameraModel getMfovCam(const fs::path &intrinsicsFname) {
  // Our CameraModel is partially compatible with the provided one (affine
  // transformation used in omni_cam is just scaling in our case, but no problem
  // raises since in this dataset no affine transformation is happening). We
  // also compute the inverse polynomial ourselves instead of using the provided
  // one.

  int width, height;
  double unmapPolyCoeffs[5];
  Vector2 center;
  std::ifstream camIfs(intrinsicsFname);
  CHECK(camIfs.is_open()) << "could not open camera intrinsics file \""
                          << intrinsicsFname.native() << "\"";
  camIfs >> width >> height;
  for (int i = 0; i < 5; ++i) camIfs >> unmapPolyCoeffs[i];
  VectorX ourCoeffs(4);
  ourCoeffs << unmapPolyCoeffs[0], unmapPolyCoeffs[2], unmapPolyCoeffs[3],
      unmapPolyCoeffs[4];
  ourCoeffs *= -1;
  camIfs >> center[0] >> center[1];
  return CameraModel(width, height, 1.0, center, ourCoeffs);
}

cv::Mat1f readBinMat(const fs::path &fname, int imgWidth, int imgHeight) {
  std::ifstream depthsIfs(fname, std::ios::binary);
  CHECK(depthsIfs.is_open()) << "failed to open file: " << fname;
  cv::Mat1f depths(imgHeight, imgWidth);
  for (int y = 0; y < imgHeight; ++y)
    for (int x = 0; x < imgWidth; ++x)
      depthsIfs.read(reinterpret_cast<char *>(&depths(y, x)), sizeof(float));
  return depths;
}

SE3 readFromMatrix3x4(std::istream &istream) {
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

const std::vector<std::string> MultiCamReaderSettings::default_camNames = {
    "front", "right", "rear", "left"};

const Eigen::AlignedBox2d MultiCamReader::Depths::boundingBox(
    Vector2::Zero(),
    Vector2(MultiCamReader::imgWidth - 1, MultiCamReader::imgHeight - 1));

int MultiCamReaderSettings::numCams() const { return camNames.size(); }

MultiCamReader::Depths::Depths(const fs::path &datasetDir, int frameInd,
                               const MultiCamReaderSettings &newSettings)
    : settings(newSettings) {
  CHECK_GE(frameInd, 0);
  CHECK_LT(frameInd, mNumFrames);

  depths.reserve(settings.numCams());
  for (int ci = 0; ci < settings.numCams(); ++ci) {
    constexpr int maxlen = 100;
    char innerName[maxlen];
    snprintf(innerName, maxlen, "%s_%04d.bin", settings.camNames[ci].c_str(),
             frameInd);
    fs::path depthPath =
        datasetDir / "data" / "depth" / settings.camNames[ci] / innerName;
    depths.push_back(readBinMat(depthPath, imgWidth, imgHeight));
  }
}

std::optional<double> MultiCamReader::Depths::depth(
    int camInd, const Vector2 &point) const {
  if (!boundingBox.contains(point)) return std::nullopt;
  CHECK_GE(camInd, 0);
  CHECK_LT(camInd, settings.numCams());
  return std::optional<double>(double(depths[camInd](toCvPoint(point))));
}

bool MultiCamReader::isMultiCam(const fs::path &datasetDir) {
  fs::path infoDir = datasetDir / "info";
  fs::path intrinsicsDir = infoDir / "intrinsics";
  fs::path extrinsicsDir = infoDir / "extrinsics";
  fs::path dataDir = datasetDir / "data";
  fs::path imgDir = dataDir / "img";
  fs::path depthDir = dataDir / "depth";
  bool isMcam = fs::exists(infoDir / "body_to_world.txt");
  for (const auto &camName : MultiCamReaderSettings::default_camNames) {
    isMcam = isMcam && fs::exists(intrinsicsDir / (camName + ".txt"));
    isMcam = isMcam && fs::exists(extrinsicsDir / (camName + "_to_body.txt"));
    isMcam = isMcam && fs::exists(imgDir / camName);
    isMcam = isMcam && fs::exists(depthDir / camName);
  }
  return isMcam;
}

CameraBundle MultiCamReader::createCameraBundle(
    const fs::path &datasetDir, const std::vector<std::string> &camNames) {
  fs::path extrinsicsDir(datasetDir / "info" / "extrinsics");
  fs::path intrinsicsDir(datasetDir / "info" / "intrinsics");

  StdVectorA<CameraModel> cams;
  StdVectorA<SE3> bodyToCam;
  cams.reserve(camNames.size());
  bodyToCam.reserve(camNames.size());
  for (const auto &camName : camNames) {
    std::ifstream extrinsicsFile(extrinsicsDir / (camName + "_to_body.txt"));
    CHECK(extrinsicsFile.is_open());
    bodyToCam.push_back(readFromMatrix3x4(extrinsicsFile).inverse());
    cams.push_back(getMfovCam(intrinsicsDir / (camName + ".txt")));
  }

  return CameraBundle(bodyToCam.data(), cams.data(), camNames.size());
}

StdVectorA<SE3> MultiCamReader::readBodyToWorld(const fs::path &datasetDir) {
  std::ifstream trajIfs(datasetDir / "info" / "body_to_world.txt");
  StdVectorA<SE3> bodyToWorld;
  bodyToWorld.reserve(mNumFrames);
  for (int fi = 0; fi < mNumFrames; ++fi)
    bodyToWorld.push_back(readFromMatrix3x4(trajIfs));
  return bodyToWorld;
}

MultiCamReader::MultiCamReader(const fs::path &_datasetDir,
                               const MultiCamReaderSettings &settings)
    : settings(settings),
      datasetDir(_datasetDir),
      mCam(createCameraBundle(_datasetDir, settings.camNames)),
      bodyToWorld(readBodyToWorld(_datasetDir)) {
  CHECK(isMultiCam(datasetDir));
}

int MultiCamReader::numFrames() const { return mNumFrames; }

int MultiCamReader::firstTimestampToInd(Timestamp timestamp) const {
  int ind = int(std::round(double(timestamp) / tsPerFrame));
  CHECK_GE(ind, 0);
  CHECK_LT(ind, mNumFrames);
  return ind;
}

std::vector<Timestamp> MultiCamReader::timestampsFromInd(int frameInd) const {
  if (frameInd < 0) return std::vector<Timestamp>(settings.numCams(), 0);
  if (frameInd >= mNumFrames)
    return std::vector<Timestamp>(settings.numCams(),
                                  tsPerFrame * (mNumFrames - 1));
  return std::vector<Timestamp>(settings.numCams(), tsPerFrame * frameInd);
}

std::vector<fs::path> MultiCamReader::frameFiles(int frameInd) const {
  CHECK_GE(frameInd, 0);
  CHECK_LT(frameInd, mNumFrames);

  std::vector<fs::path> fnames;
  fnames.reserve(settings.numCams());
  for (int ci = 0; ci < settings.numCams(); ++ci) {
    constexpr int maxlen = 100;
    char innerName[maxlen];
    snprintf(innerName, maxlen, "%s_%04d.jpg", settings.camNames[ci].c_str(),
             frameInd);
    fs::path imagePath =
        datasetDir / "data" / "img" / settings.camNames[ci] / innerName;
    fnames.push_back(imagePath);
  }
  return fnames;
}

std::vector<DatasetReader::FrameEntry> MultiCamReader::frame(
    int frameInd) const {
  CHECK_GE(frameInd, 0);
  CHECK_LT(frameInd, mNumFrames);
  std::vector<DatasetReader::FrameEntry> frame;
  frame.reserve(settings.numCams());
  std::vector<fs::path> fnames = frameFiles(frameInd);
  for (int ci = 0; ci < settings.numCams(); ++ci) {
    cv::Mat3b image = cv::imread(fnames[ci].string());
    CHECK_NE(image.data, (unsigned char *)nullptr)
        << "path: " << fnames[ci].string();
    frame.push_back({image, Timestamp(frameInd)});
  }
  return frame;
}

CameraBundle MultiCamReader::cam() const { return mCam; }

std::unique_ptr<FrameDepths> MultiCamReader::depths(int frameInd) const {
  return std::unique_ptr<FrameDepths>(
      new Depths(datasetDir, frameInd, settings));
}

bool MultiCamReader::hasFrameToWorld(int frameInd) const {
  return frameInd >= 0 && frameInd < mNumFrames;
}

SE3 MultiCamReader::frameToWorld(int frameInd) const {
  CHECK_GE(frameInd, 0);
  CHECK_LT(frameInd, bodyToWorld.size());
  return bodyToWorld[frameInd];
}

Trajectory MultiCamReader::gtTrajectory() const {
  StdMapA<Timestamp, SE3> timestampedWorldFromFrame;
  for (int fi = 0; fi < numFrames(); ++fi) {
    timestampedWorldFromFrame[timestampsFromInd(fi)[0]] = bodyToWorld[fi];
  }
  return Trajectory{timestampedWorldFromFrame};
}

}  // namespace mcam
