#include "dataset/RobotcarReader.h"
#include "util.h"

namespace mcam {

RobotcarReaderSettings::RobotcarReaderSettings() {
  cam.unmapPolyDegree = 9;  // hand-tuned parameters
  cam.unmapPolyPoints = 60000;
}

bool RobotcarReader::isRobotcar(const fs::path &chunkDir) {
  return fs::exists(chunkDir / "mono_left.timestamps") &&
         fs::exists(chunkDir / "mono_left") &&
         fs::exists(chunkDir / "mono_rear.timestamps") &&
         fs::exists(chunkDir / "mono_rear") &&
         fs::exists(chunkDir / "mono_right.timestamps") &&
         fs::exists(chunkDir / "mono_right") &&
         fs::exists(chunkDir / "lms_front.timestamps") &&
         fs::exists(chunkDir / "lms_front") &&
         fs::exists(chunkDir / "lms_rear.timestamps") &&
         fs::exists(chunkDir / "lms_rear") &&
         fs::exists(chunkDir / "ldmrs.timestamps") &&
         fs::exists(chunkDir / "ldmrs") && fs::exists(chunkDir / "vo");
}

// clang-format off
const SE3 RobotcarReader::camToImage =
        SE3((Matrix44() <<
                        0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 0, 1).finished())
                .inverse();
// clang-format on

SE3 fromXyzrpy(double x, double y, double z, double roll, double pitch,
               double yaw) {
  SO3 rot = SO3::rotZ(yaw) * SO3::rotY(pitch) * SO3::rotX(roll);
  return SE3(rot, Vector3(x, y, z));
}

SE3 readFromExtrin(const fs::path &extrinsicsFile) {
  std::ifstream ifs(extrinsicsFile);
  CHECK(ifs.is_open()) << "Could not open extrinsics file " << extrinsicsFile;
  double x, y, z, roll, pitch, yaw;
  ifs >> x >> y >> z >> roll >> pitch >> yaw;
  SE3 result = fromXyzrpy(x, y, z, roll, pitch, yaw);
  return result;
}

SE3 readBodyToCam(const fs::path &extrinsicsFile) {
  SE3 result = readFromExtrin(extrinsicsFile);
  LOG(INFO) << "file: " << extrinsicsFile << "\nbody -> this:\n"
            << result.matrix();
  return result;
}

SE3 readBodyToLidar(const fs::path &extrinsicsFile) {
  SE3 lidarToBody = readFromExtrin(extrinsicsFile);
  LOG(INFO) << "file: " << extrinsicsFile << "\nthis -> body:\n"
            << lidarToBody.matrix();
  return lidarToBody.inverse();
}

SE3 readBodyToIns(const fs::path &extrinsicsFile) {
  SE3 insToBody = readFromExtrin(extrinsicsFile);
  return insToBody.inverse();
}

CameraBundle RobotcarReader::createFromData(
    const fs::path &modelsDir, const SE3 &bodyToLeft, const SE3 &bodyToRear,
    const SE3 &bodyToRight, int w, int h,
    const CameraModelSettings &camSettings) {
  fs::path leftModel = modelsDir / "mono_left.txt";
  fs::path rearModel = modelsDir / "mono_rear.txt";
  fs::path rightModel = modelsDir / "mono_right.txt";
  CameraModel models[RobotcarReader::numCams] = {
      CameraModel(w, h, leftModel, CameraModel::POLY_MAP, camSettings),
      CameraModel(w, h, rearModel, CameraModel::POLY_MAP, camSettings),
      CameraModel(w, h, rightModel, CameraModel::POLY_MAP, camSettings)};

  SE3 bodyToCam[RobotcarReader::numCams] = {bodyToLeft, bodyToRear,
                                            bodyToRight};
  for (int i = 0; i < RobotcarReader::numCams; ++i)
    bodyToCam[i] = RobotcarReader::camToImage * bodyToCam[i];

  return CameraBundle(bodyToCam, models, RobotcarReader::numCams);
}

void readTs(const fs::path &tsFile, std::vector<Timestamp> &timestamps) {
  CHECK(fs::exists(tsFile)) << tsFile.native() + " does not exist";
  std::ifstream ifs(tsFile);
  Timestamp ts;
  int one;
  while (ifs >> ts >> one) {
    CHECK(timestamps.empty() || timestamps.back() < ts);
    timestamps.push_back(ts);
  }
}

void readVo(const fs::path &voFile, std::vector<Timestamp> &timestamps,
            StdVectorA<SE3> &voBodyToFirst, bool fillVoGaps) {
  CHECK(fs::exists(voFile)) << voFile.native() + " does not exist";

  Timestamp maxSkip = -1;
  int skipCount = 0, backSkipCount = 0;
  SE3 lboToLast;

  std::ifstream ifs(voFile);
  std::string header;
  std::getline(ifs, header);
  char comma;
  Timestamp srcTs, dstTs;
  double x, y, z, roll, pitch, yaw;
  while (ifs >> srcTs >> comma >> dstTs >> comma >> x >> comma >> y >> comma >>
         z >> comma >> roll >> comma >> pitch >> comma >> yaw) {
    if (timestamps.empty()) {
      timestamps.push_back(dstTs);
      voBodyToFirst.push_back(SE3());
    }

    SE3 srcToLast = fromXyzrpy(x, y, z, roll, pitch, yaw);
    if (dstTs != timestamps.back()) {
      Timestamp skip = (dstTs - timestamps.back());
      if (skip < 0)
        backSkipCount++;
      else
        maxSkip = std::max(skip, maxSkip);
      skipCount++;

      if (fillVoGaps && timestamps.size() >= 2) {
        double tsFrac = double(skip) /
                        (timestamps.back() - timestamps[timestamps.size() - 2]);
        SE3 dstToSrcSkip = SE3::exp(tsFrac * lboToLast.log());
        srcToLast = dstToSrcSkip * srcToLast;
      }
    }
    lboToLast = srcToLast;
    SE3 srcToFirst = voBodyToFirst.back() * srcToLast;
    timestamps.push_back(srcTs);
    voBodyToFirst.push_back(srcToFirst);
  }

  LOG(INFO) << "the biggest skip in VO is " << maxSkip * 1e-6 << " seconds";
  LOG(INFO) << "there are " << skipCount << " skips in total";
}

void readRtk(const fs::path &rtkFile, const SE3 &bodyToIns, bool correctRtk,
             std::vector<Timestamp> &timestamps,
             StdVectorA<SE3> &rtkBodyToWorld) {
  std::ifstream rtkIfs(rtkFile);
  std::string curLine;
  std::getline(rtkIfs, curLine);
  std::optional<SE3> worldToFirst;
  while (std::getline(rtkIfs, curLine)) {
    Timestamp ts;
    double latitude, longitude, altitude;
    double northing, easting, down;
    double vNorth, vEast, vDown;
    double roll, pitch, yaw;
    constexpr int needToRead = 13;
    int numRead =
        sscanf(curLine.c_str(),
               "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%*3c,%lf,%lf,%lf,%lf,%lf,%lf\n",
               &ts, &latitude, &longitude, &altitude, &northing, &easting,
               &down, &vNorth, &vEast, &vDown, &roll, &pitch, &yaw);
    if (numRead != needToRead) {
      LOG(WARNING) << "Read " << numRead << " instead of " << needToRead
                   << " elements in RTK ground truth";
      break;
    }
    SE3 curBodyToWorld =
        correctRtk
            ? fromXyzrpy(-easting, northing, down, roll, pitch, yaw) * bodyToIns
            : fromXyzrpy(northing, easting, down, roll, pitch, yaw) * bodyToIns;

    if (!worldToFirst) {
      worldToFirst.emplace(curBodyToWorld.inverse());
      curBodyToWorld = SE3();
    } else
      curBodyToWorld = worldToFirst.value() * curBodyToWorld;

    timestamps.push_back(ts);
    rtkBodyToWorld.push_back(curBodyToWorld);
  }

  double timeCovered = double(timestamps.back() - timestamps[0]) / 1e6;
  LOG(INFO) << "RTK data time covered = " << timeCovered << " sec";
  LOG(INFO) << "RTK data num positions = " << timestamps.size()
            << "; avg time between positions = "
            << timeCovered / timestamps.size();
}

void logTimeInterval(const std::vector<Timestamp> &timestamps,
                     const std::string &name) {
  LOG(INFO) << name << ": [" << timeOfDay(toChronoTimePoint(timestamps[0]))
            << " -- " << timeOfDay(toChronoTimePoint(timestamps.back())) << "]"
            << " total items: " << timestamps.size();
}

RobotcarReader::RobotcarReader(const fs::path &_chunkDir,
                               const fs::path &modelsDir,
                               const fs::path &extrinsicsDir,
                               const std::optional<fs::path> &rtkDir,
                               const RobotcarReaderSettings &_settings)
    : bodyToLeft(readBodyToCam(extrinsicsDir / "mono_left.txt")),
      bodyToRear(readBodyToCam(extrinsicsDir / "mono_rear.txt")),
      bodyToRight(readBodyToCam(extrinsicsDir / "mono_right.txt")),
      bodyToLmsFront(readBodyToLidar(extrinsicsDir / "lms_front.txt")),
      bodyToLmsRear(readBodyToLidar(extrinsicsDir / "lms_rear.txt")),
      bodyToLdmrs(readBodyToLidar(extrinsicsDir / "ldmrs.txt")),
      bodyToIns(readBodyToIns(extrinsicsDir / "ins.txt")),
      mCam(createFromData(modelsDir, bodyToLeft, bodyToRear, bodyToRight,
                          imageWidth, imageHeight, _settings.cam)),
      chunkDir(_chunkDir),
      leftDir(chunkDir / fs::path("mono_left")),
      rearDir(chunkDir / fs::path("mono_rear")),
      rightDir(chunkDir / fs::path("mono_right")),
      lmsFrontDir(chunkDir / fs::path("lms_front")),
      lmsRearDir(chunkDir / fs::path("lms_rear")),
      ldmrsDir(chunkDir / fs::path("ldmrs")),
      mMasksProvided(false),
      settings(_settings) {
  CHECK(fs::is_directory(leftDir));
  CHECK(fs::is_directory(rearDir));
  CHECK(fs::is_directory(rightDir));
  CHECK(fs::is_directory(lmsFrontDir));
  CHECK(fs::is_directory(lmsRearDir));
  CHECK(fs::is_directory(ldmrsDir));

  readTs(chunkDir / "mono_left.timestamps", mLeftTs);
  readTs(chunkDir / "mono_rear.timestamps", mRearTs);
  readTs(chunkDir / "mono_right.timestamps", mRightTs);
  readTs(chunkDir / "lms_front.timestamps", mLmsFrontTs);
  readTs(chunkDir / "lms_rear.timestamps", mLmsRearTs);
  readTs(chunkDir / "ldmrs.timestamps", mLdmrsTs);

  bool isRtkFound = false;
  if (rtkDir) {
    fs::path rtkFile = rtkDir.value() / chunkDir.filename() / "rtk.csv";
    if (fs::is_regular_file(rtkFile)) {
      readRtk(rtkFile, bodyToIns, settings.correctRtk, mGroundTruthTs,
              gtBodyToWorld);
      isRtkFound = true;
    }
  }

  readVo(chunkDir / fs::path("vo") / fs::path("vo.csv"), mVoTs, voBodyToWorld,
         settings.fillVoGaps);
  if (!isRtkFound) {
    LOG(WARNING) << "No RTK ground truth for chunk \'" << chunkDir
                 << "\' found";
    readVo(chunkDir / fs::path("vo") / fs::path("vo.csv"), mGroundTruthTs,
           gtBodyToWorld, settings.fillVoGaps);
  }

  LOG(INFO) << "Before timestamp synchronization:";
  logTimeInterval(mLeftTs, "left cam ");
  logTimeInterval(mRearTs, "rear cam ");
  logTimeInterval(mRightTs, "right cam");

  syncTimestamps();

  LOG(INFO) << "After timestamp synchronization:";
  logTimeInterval(mLeftTs, "left cam ");
  logTimeInterval(mRearTs, "rear cam ");
  logTimeInterval(mRightTs, "right cam");
  logTimeInterval(mLmsRearTs, "LMS rear ");
  logTimeInterval(mLmsFrontTs, "LMS front");
  logTimeInterval(mLdmrsTs, "LDMRS    ");
  if (isRtkFound) {
    logTimeInterval(mGroundTruthTs, "rtk      ");
    logTimeInterval(mVoTs, "vo       ");
  } else {
    logTimeInterval(mGroundTruthTs, "vo       ");
  }

  printVoAndGT(settings.outputDirectory);
}

void RobotcarReader::printVoAndGT(const fs::path &outputDirectory) const {
  std::ofstream outVo(outputDirectory / "vo_traj.txt");
  std::ofstream outGt(outputDirectory / "gt_traj.txt");
  for (int i = 0; i < mGroundTruthTs.size(); ++i) {
    if (mGroundTruthTs[i] < mVoTs[0] || mGroundTruthTs[i] >= mVoTs.back())
      continue;
    putInMatrixForm(outGt, gtBodyToWorld[i]);
    putInMatrixForm(outVo, tsToWorld(mGroundTruthTs[i], true));
  }
}

void RobotcarReader::provideMasks(const fs::path &masksDir) {
  CHECK(fs::is_directory(masksDir));
  fs::path leftMaskPath = masksDir / "mono_left.png";
  fs::path rearMaskPath = masksDir / "mono_rear.png";
  fs::path rightMaskPath = masksDir / "mono_right.png";
  CHECK(fs::is_regular_file(leftMaskPath));
  CHECK(fs::is_regular_file(rearMaskPath));
  CHECK(fs::is_regular_file(rightMaskPath));
  cv::Mat3b leftMask = cv::imread(std::string(leftMaskPath));
  cv::Mat3b rearMask = cv::imread(std::string(rearMaskPath));
  cv::Mat3b rightMask = cv::imread(std::string(rightMaskPath));
  mCam.cam(0).setMask(cvtBgrToGray(leftMask));
  mCam.cam(1).setMask(cvtBgrToGray(rearMask));
  mCam.cam(2).setMask(cvtBgrToGray(rightMask));
  mMasksProvided = true;
}

int RobotcarReader::numFrames() const { return mLeftTs.size(); }

int RobotcarReader::firstTimestampToInd(Timestamp timestamp) const {
  if (timestamp < mLeftTs[0]) return 0;
  if (timestamp > mLeftTs.back()) return numFrames();

  auto it = std::lower_bound(mLeftTs.begin(), mLeftTs.end(), timestamp);
  if (it == mLeftTs.end()) return numFrames();
  return it - mLeftTs.begin();
}

std::vector<Timestamp> RobotcarReader::timestampsFromInd(int frameInd) const {
  CHECK_GE(frameInd, 0);
  CHECK_LT(frameInd, numFrames());

  return std::vector{mLeftTs[frameInd], mRearTs[frameInd], mRightTs[frameInd]};
}

std::vector<fs::path> RobotcarReader::frameFiles(int frameInd) const {
  CHECK_GE(frameInd, 0);
  CHECK_LT(frameInd, numFrames());

  fs::path leftPath = chunkDir / fs::path("mono_left") /
                      fs::path(std::to_string(mLeftTs[frameInd]) + ".png");
  fs::path rearPath = chunkDir / fs::path("mono_rear") /
                      fs::path(std::to_string(mRearTs[frameInd]) + ".png");
  fs::path rightPath = chunkDir / fs::path("mono_right") /
                       fs::path(std::to_string(mRightTs[frameInd]) + ".png");

  return {leftPath, rearPath, rightPath};
}

std::vector<RobotcarReader::FrameEntry> RobotcarReader::frame(
    int frameInd) const {
  CHECK_GE(frameInd, 0);
  CHECK_LT(frameInd, numFrames());
  std::vector<fs::path> fnames = frameFiles(frameInd);
  fs::path leftPath = fnames[0];
  fs::path rearPath = fnames[1];
  fs::path rightPath = fnames[2];

  if (fs::is_regular_file(leftPath) && fs::is_regular_file(rearPath) &&
      fs::is_regular_file(rightPath)) {
    // Can fail if image is corrupted or not read correctly for whatever reason
    try {
      cv::Mat1b leftOrig = cv::imread(leftPath.native(), cv::IMREAD_GRAYSCALE);
      cv::Mat1b rearOrig = cv::imread(rearPath.native(), cv::IMREAD_GRAYSCALE);
      cv::Mat1b rightOrig =
          cv::imread(rightPath.native(), cv::IMREAD_GRAYSCALE);

      std::vector<FrameEntry> result(numCams);
      cv::cvtColor(leftOrig, result[0].frame, cv::COLOR_BayerBG2BGR);
      result[0].timestamp = mLeftTs[frameInd];
      cv::cvtColor(rearOrig, result[1].frame, cv::COLOR_BayerBG2BGR);
      result[1].timestamp = mRearTs[frameInd];
      cv::cvtColor(rightOrig, result[2].frame, cv::COLOR_BayerBG2BGR);
      result[2].timestamp = mRightTs[frameInd];

      return result;
    } catch (const std::exception &e) {
      throw e;
    }
  } else {
    throw std::runtime_error("Image paths for frame contain invalid paths");
  }
}

std::unique_ptr<FrameDepths> RobotcarReader::depths(int frameInd) const {
  LOG(WARNING) << "RobotcarReader::depths not implemented yet!";
  return std::unique_ptr<FrameDepths>();
}

StdVectorA<Vector2> getPoints(
    const StdVectorA<std::pair<Vector2, double>> &projected) {
  StdVectorA<Vector2> result(projected.size());
  for (int i = 0; i < projected.size(); ++i) result[i] = projected[i].first;
  return result;
}

std::vector<double> getDepths(
    const StdVectorA<std::pair<Vector2, double>> &projected) {
  std::vector<double> result(projected.size());
  for (int i = 0; i < projected.size(); ++i) result[i] = projected[i].second;
  return result;
}

bool RobotcarReader::hasFrameToWorld(int frameInd) const {
  if (frameInd < 0 || frameInd >= numFrames()) return false;
  Timestamp ts = timestampsFromInd(frameInd)[0];
  return ts >= mGroundTruthTs[0] && ts <= mGroundTruthTs.back();
}

SE3 RobotcarReader::frameToWorld(int frameInd) const {
  CHECK_GE(frameInd, 0);
  CHECK_LT(frameInd, numFrames());

  Timestamp ts = timestampsFromInd(frameInd)[0];
  CHECK_GE(ts, mGroundTruthTs[0]);
  CHECK_LT(ts, mGroundTruthTs.back());
  return tsToWorld(ts);
}

Trajectory RobotcarReader::gtTrajectory() const {
  StdMapA<Timestamp, SE3> timestampedWorldFromFrame;
  for (int i = 0; i < mGroundTruthTs.size(); ++i)
    timestampedWorldFromFrame[mGroundTruthTs[i]] = gtBodyToWorld[i];
  return Trajectory{timestampedWorldFromFrame};
}

std::string logTimestamp(Timestamp ts) {
  return std::to_string(ts) + " (" + timeOfDay(toChronoTimePoint(ts)) + ")";
}

SE3 tsToWorldHelper(Timestamp ts, const StdVectorA<SE3> &bodyToWorld,
                    const std::vector<Timestamp> &timestamps) {
  CHECK(ts >= timestamps[0] && ts <= timestamps.back())
      << "Interpolating outside of ground truth! "
      << "ts = " << logTimestamp(ts) << ", bounds = ["
      << logTimestamp(timestamps[0]) << ", " << logTimestamp(timestamps.back())
      << "]";
  CHECK(timestamps.size() >= 2);
  if (ts == timestamps[0]) return bodyToWorld[0];

  int ind = std::lower_bound(timestamps.begin(), timestamps.end(), ts) -
            timestamps.begin();
  CHECK(ind > 0 && ind < bodyToWorld.size());
  SE3 highToLow = bodyToWorld[ind - 1].inverse() * bodyToWorld[ind];
  double tsFrac = double(ts - timestamps[ind - 1]) /
                  (timestamps[ind] - timestamps[ind - 1]);
  SE3 tsToLow = SE3::exp(tsFrac * highToLow.log());
  return bodyToWorld[ind - 1] * tsToLow;
}

SE3 RobotcarReader::tsToWorld(Timestamp ts, bool useVo) const {
  if (useVo)
    return tsToWorldHelper(ts, voBodyToWorld, mVoTs);
  else
    return tsToWorldHelper(ts, gtBodyToWorld, mGroundTruthTs);
}

void RobotcarReader::getPointCloudHelper(
    std::vector<Vector3> &cloud, const fs::path &scanDir,
    const SE3 &sensorToBody, Timestamp base,
    const std::vector<Timestamp> &timestamps, Timestamp from, Timestamp to,
    bool isLdmrs) const {
  int indFrom = std::lower_bound(timestamps.begin(), timestamps.end(), from) -
                timestamps.begin();
  int indTo = std::upper_bound(timestamps.begin(), timestamps.end(), to) -
              timestamps.begin();

  LOG(INFO) << "index bounds of the cloud: [" << indFrom << ", " << indTo
            << "]";

  int curPerc = 0;
  int totalScans = indTo - indFrom;
  std::cout << "scanning the cloud from " << scanDir << ": 0% ...";
  std::cout.flush();
  for (int i = indFrom; i < indTo; ++i) {
    SE3 sensorToBase =
        tsToWorld(base).inverse() * tsToWorld(timestamps[i]) * sensorToBody;
    fs::path scanFile =
        scanDir / fs::path(std::to_string(timestamps[i]) + ".bin");
    std::vector<double> data = readDoublesFromBin(scanFile);
    double x, y, z, reflectance;
    for (int i = 0; i + 2 < data.size(); i += 3) {
      if (isLdmrs) {
        x = data[i];
        y = data[i + 1];
        z = data[i + 2];
      } else {
        x = data[i];
        y = data[i + 1];
        z = 0;
        reflectance = data[i + 2];
      }
      cloud.push_back(sensorToBase * Vector3(x, y, z));
    }

    int newPerc = (i + 1 - indFrom) * 100 / totalScans;
    if (newPerc % 20 == 0 && newPerc != curPerc) {
      curPerc = newPerc;
      std::cout << " " << newPerc << "%";
      std::cout.flush();
      if (newPerc != 100)
        std::cout << " ...";
      else
        std::cout << std::endl;
    }
  }
}

std::vector<Vector3> RobotcarReader::getLmsFrontCloud(Timestamp from,
                                                      Timestamp to,
                                                      Timestamp base) const {
  std::vector<Vector3> cloud;
  getPointCloudHelper(cloud, lmsFrontDir, bodyToLmsFront.inverse(), base,
                      mLmsFrontTs, from, to, false);
  return cloud;
}

std::vector<Vector3> RobotcarReader::getLmsRearCloud(Timestamp from,
                                                     Timestamp to,
                                                     Timestamp base) const {
  std::vector<Vector3> cloud;
  getPointCloudHelper(cloud, lmsRearDir, bodyToLmsRear.inverse(), base,
                      mLmsRearTs, from, to, false);
  return cloud;
}

std::vector<Vector3> RobotcarReader::getLdmrsCloud(Timestamp from, Timestamp to,
                                                   Timestamp base) const {
  std::vector<Vector3> cloud;
  getPointCloudHelper(cloud, ldmrsDir, bodyToLdmrs.inverse(), base, mLdmrsTs,
                      from, to, true);
  return cloud;
}

std::array<StdVectorA<std::pair<Vector2, double>>, RobotcarReader::numCams>
RobotcarReader::project(Timestamp from, Timestamp to, Timestamp base,
                        bool useLmsFront, bool useLmsRear,
                        bool useLdmrs) const {
  LOG(INFO) << "projection time window: [" << timeOfDay(toChronoTimePoint(from))
            << ", " << timeOfDay(toChronoTimePoint(to)) << "]";
  std::vector<Vector3> lmsFrontCloud;
  if (useLmsFront) lmsFrontCloud = getLmsFrontCloud(from, to, base);
  std::vector<Vector3> lmsRearCloud;
  if (useLmsRear) lmsRearCloud = getLmsRearCloud(from, to, base);
  std::vector<Vector3> ldmrsCloud;
  if (useLdmrs) ldmrsCloud = getLdmrsCloud(from, to, base);
  std::vector<Vector3> cloud;
  cloud.reserve(lmsFrontCloud.size() + lmsRearCloud.size() + ldmrsCloud.size());
  cloud.insert(cloud.end(), lmsFrontCloud.begin(), lmsFrontCloud.end());
  cloud.insert(cloud.end(), lmsRearCloud.begin(), lmsRearCloud.end());
  cloud.insert(cloud.end(), ldmrsCloud.begin(), ldmrsCloud.end());
  return project(cloud);
}

std::array<StdVectorA<std::pair<Vector2, double>>, RobotcarReader::numCams>
RobotcarReader::project(const std::vector<Vector3> &cloud) const {
  std::array<StdVectorA<std::pair<Vector2, double>>, RobotcarReader::numCams>
      result;
  for (int camInd = 0; camInd < RobotcarReader::numCams; ++camInd) {
    SE3 bodyToCam = mCam.bodyToCam(camInd);
    for (const Vector3 &p : cloud) {
      Vector3 moved = bodyToCam * p;
      if (!mCam.cam(camInd).isMappable(moved)) continue;
      double depth = moved.norm();
      Vector2 projected = mCam.cam(camInd).map(moved);
      result[camInd].push_back({projected, depth});
    }
  }
  return result;
}

Timestamp RobotcarReader::minTs() const {
  return std::max({mLeftTs[0], mRearTs[0], mRightTs[0], mGroundTruthTs[0],
                   mLmsFrontTs[0], mLmsRearTs[0], mLdmrsTs[0]});
}

Timestamp RobotcarReader::maxTs() const {
  return std::min({mLeftTs.back(), mRearTs.back(), mRightTs.back(),
                   mGroundTruthTs.back(), mLmsFrontTs.back(), mLmsRearTs.back(),
                   mLdmrsTs.back()});
}

// Here goes a large piece of code for timestamp synchronization between
// different cameras. The auxiliary functions might be moved to another place,
// but I don't know where yet.
constexpr int npos = -1;

Timestamp timestampDist(Timestamp a, Timestamp b) {
  return a < b ? b - a : a - b;
}

Timestamp triDist(Timestamp x1, Timestamp x2, Timestamp x3) {
  return timestampDist(x1, x2) + timestampDist(x1, x3) + timestampDist(x2, x3);
}

std::vector<int> closestNotHigherInds(const std::vector<Timestamp> &a,
                                      const std::vector<Timestamp> &b) {
  CHECK(!b.empty());
  std::vector<int> inds(a.size(), npos);
  int indB = 0, indA = 0;
  while (indB < b.size() && b[indB] > a[indA]) indA++;
  for (; indA < a.size(); ++indA) {
    if (indB + 1 == b.size() && a[indA] < b[indB]) break;
    while (indB + 1 < b.size() && b[indB + 1] <= a[indA]) ++indB;
    inds[indA] = indB;
  }
  return inds;
}

std::vector<int> closestNotLowerInds(const std::vector<Timestamp> &a,
                                     const std::vector<Timestamp> &b) {
  CHECK(!b.empty());
  std::vector<int> inds(a.size(), npos);
  int indA = 0, indB = 0;
  while (indB < b.size() && b[indB] < a[indA]) indB++;
  for (; indA < a.size(); ++indA) {
    while (indB < b.size() && b[indB] < a[indA]) ++indB;
    if (indB == b.size()) break;
    inds[indA] = indB;
  }
  return inds;
}

std::vector<int> closestIndsInB(const std::vector<Timestamp> &a,
                                const std::vector<Timestamp> &b) {
  CHECK(!b.empty());
  std::vector<int> closestToB(a.size());
  int indB = 0;
  for (int indA = 0; indA < a.size(); ++indA) {
    while (indB + 1 < b.size() && b[indB + 1] < a[indA]) ++indB;
    if (indB + 1 == b.size()) {
      closestToB[indA] = indB;
      continue;
    }
    if (timestampDist(a[indA], b[indB]) < timestampDist(a[indA], b[indB + 1]))
      closestToB[indA] = indB;
    else
      closestToB[indA] = ++indB;
  }
  return closestToB;
}

void filterBest(std::vector<Timestamp> &a, const std::vector<Timestamp> &b,
                const std::vector<Timestamp> &c, std::vector<int> &aToB,
                std::vector<int> &aToC) {
  constexpr int npos = -1;
  std::vector<int> bestIndAInB(b.size(), npos);
  std::vector<Timestamp> bestAInB(b.size());
  for (int indA = 0; indA < a.size(); ++indA) {
    Timestamp closeB = b[aToB[indA]], closeC = c[aToC[indA]];
    Timestamp curDist = triDist(a[indA], closeB, closeC);
    if (bestIndAInB[aToB[indA]] == npos || curDist < bestAInB[aToB[indA]]) {
      bestAInB[aToB[indA]] = curDist;
      bestIndAInB[aToB[indA]] = indA;
    }
  }

  std::vector<char> removed(a.size(), false);
  for (int indA = 0; indA < a.size(); ++indA) {
    if (bestIndAInB[aToB[indA]] != indA) removed[indA] = true;
  }
  int newIndA = 0;
  for (int oldIndA = 0; oldIndA < a.size(); ++oldIndA)
    if (!removed[oldIndA]) {
      a[newIndA] = a[oldIndA];
      aToB[newIndA] = aToB[oldIndA];
      aToC[newIndA] = aToC[oldIndA];
      ++newIndA;
    }
  a.resize(newIndA);
  aToB.resize(newIndA);
  aToC.resize(newIndA);
}

void filterOutNoRef(std::vector<Timestamp> &v, std::vector<int> &inds) {
  auto newIndsEnd = std::unique(inds.begin(), inds.end());
  CHECK_EQ(newIndsEnd - inds.begin(), inds.end() - inds.begin());
  int newInd = 0;
  for (int i : inds) v[newInd++] = v[i];
  v.resize(newInd);
}

void checkInds(const std::vector<Timestamp> &a, const std::vector<Timestamp> &b,
               const std::vector<Timestamp> &c, int indA, int indB, int indC,
               Timestamp &bestDist, int &bestIndB, int &bestIndC) {
  if (indB == npos || indC == npos) return;
  Timestamp dist = triDist(a[indA], b[indB], c[indC]);
  if (dist < bestDist) {
    bestDist = dist;
    bestIndB = indB;
    bestIndC = indC;
  }
}

void mostConsistentTriples(std::vector<Timestamp> &a, std::vector<Timestamp> &b,
                           std::vector<Timestamp> &c) {
  std::vector<int> lowerInB = closestNotHigherInds(a, b);
  std::vector<int> higherInB = closestNotLowerInds(a, b);
  std::vector<int> lowerInC = closestNotHigherInds(a, c);
  std::vector<int> higherInC = closestNotLowerInds(a, c);

  std::vector<int> aToB(a.size()), aToC(a.size());
  for (int indA = 0; indA < a.size(); ++indA) {
    Timestamp bestDist = std::numeric_limits<Timestamp>::max();
    int bestInB = npos, bestInC = npos;
    checkInds(a, b, c, indA, lowerInB[indA], lowerInC[indA], bestDist, bestInB,
              bestInC);
    checkInds(a, b, c, indA, lowerInB[indA], higherInC[indA], bestDist, bestInB,
              bestInC);
    checkInds(a, b, c, indA, higherInB[indA], lowerInC[indA], bestDist, bestInB,
              bestInC);
    checkInds(a, b, c, indA, higherInB[indA], higherInC[indA], bestDist,
              bestInB, bestInC);
    aToB[indA] = bestInB;
    aToC[indA] = bestInC;
  }

  filterBest(a, b, c, aToB, aToC);
  filterBest(a, c, b, aToC, aToB);
  filterOutNoRef(b, aToB);
  filterOutNoRef(c, aToC);
  CHECK_EQ(a.size(), b.size());
  CHECK_EQ(a.size(), c.size());
}

double avgTriDist(const std::vector<Timestamp> &a,
                  const std::vector<Timestamp> &b,
                  const std::vector<Timestamp> &c) {
  double sum = 0;
  for (int i = 0; i < a.size(); ++i) sum += triDist(a[i], b[i], c[i]);
  return sum / a.size();
}

void RobotcarReader::syncTimestamps() {
  int oldSize = mLeftTs.size();
  double oldAvgTriDist = avgTriDist(mLeftTs, mRearTs, mRightTs);
  mostConsistentTriples(mLeftTs, mRearTs, mRightTs);
  CHECK_EQ(mLeftTs.size(), mRightTs.size());
  CHECK_EQ(mLeftTs.size(), mRearTs.size());
  int newSize = mLeftTs.size();
  double newAvgTriDist = avgTriDist(mLeftTs, mRearTs, mRightTs);
  LOG(INFO) << "triples filtered out for syncing timestamps: "
            << oldSize - newSize;
  LOG(INFO) << "old avg tridist (s) = " << oldAvgTriDist / 1e6;
  LOG(INFO) << "new avg tridist (s) = " << newAvgTriDist / 1e6;
}

}  // namespace mcam