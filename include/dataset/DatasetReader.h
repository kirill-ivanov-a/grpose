#ifndef GRPOSE_DATASET_DATASETREADER_
#define GRPOSE_DATASET_DATASETREADER_

#include <memory>
#include <optional>

#include "camera/CameraBundle.h"
#include "trajectory.h"
#include "types.h"

namespace grpose {

/**
 * An interface for providing ground-truth depths for a multi-frame.
 */
class FrameDepths {
 public:
  virtual ~FrameDepths() = 0;

  /**
   * Depth at a specified image position on the camera with index \p camInd
   * Note that as it is not always available, we return `std::optional`.
   */
  virtual std::optional<double> Depth(int camera_index,
                                      const Vector2 &point) const = 0;
};

/**
 * An interface that represents a continuous segment of a multi-camera stream.
 * A valid implementation should provide:
 * 1. intrinsic & extrinsic calibration (in the form of CameraBundle)
 * 2. Continuous indexed multi-frames from synchronized cameras
 *      - An image with a timestamp from each camera for each multi-frame
 * Optional:
 * 1. Ground-truth poses of the body frame for some multi-frames
 * 2. Ground-truth depths for some pixels on the multi-frame
 */
class DatasetReader {
 public:
  struct FrameEntry {
    cv::Mat3b frame;
    Timestamp timestamp;
  };

  virtual ~DatasetReader() = 0;

  /**
   * Number of multi-frames that this object provides.
   */
  virtual int NumberOfFrames() const = 0;

  /**
   * An index of the multi-frame that has the biggest timestamp still lower than
   * \p timestamp. Note that we use the timestamp from the first camera in the
   * bundle.
   */
  virtual int FirstTimestampToIndex(Timestamp timestamp) const = 0;

  /**
   * The vector of timestamps of frames in a multi-frame with the specified
   * index. Note that we allow for deviations in timestamps inside one
   * multi-frame. It happens even if the cameras are synchronized, as, for
   * example, in the RobotCar dataset.
   */
  virtual std::vector<Timestamp> TimestampsFromIndex(int frame_index) const = 0;

  /**
   * Paths to image files that correspond to multi-frame with index
   * \p frame_index.
   */
  virtual std::vector<fs::path> FrameFiles(int frame_index) const = 0;

  /**
   * Multi-frame with index \p frame_index.
   */
  virtual std::vector<FrameEntry> Frame(int frame_index) const = 0;

  /**
   * The CameraBundle (extrinsic and intrinsic calibration) provided by the
   * dataset.
   */
  virtual CameraBundle GetCameraBundle() const = 0;

  /**
   * Ground-truth depths for the multi-frame with the specified index. May
   * return `nullptr` if depths are not available.
   */
  virtual std::unique_ptr<FrameDepths> GetDepths(int frame_index) const = 0;

  /**
   * Checks if ground-truth pose is available for a frame with the provided
   * index.
   */
  virtual bool HasWorldFromFrame(int frame_index) const = 0;

  /**
   * Provides a pose if its available. Terminates with an error if not. Not
   * implemented via `std::optional` since in most use cases it is known whether
   * all poses are available or not.
   */
  virtual SE3 WorldFromFrame(int frame_index) const = 0;

  /**
   * @returns - Raw ground-truth trajectory for the dataset, not
   * timestamp-aligned with the frames.
   */
  virtual Trajectory GroundTruthTrajectory() const = 0;
};

}  // namespace grpose

#endif