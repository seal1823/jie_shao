#ifndef CROP_ROW_DETECTOR_HPP
#define CROP_ROW_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>
#include <std_msgs/msg/header.hpp>

struct CropRowResult {
  // Detected row lines in image coordinates (point1, point2 for each line)
  std::vector<std::pair<cv::Point2f, cv::Point2f>> row_lines;
  // Centerline of the path (desired trajectory)
  std::pair<cv::Point2f, cv::Point2f> center_line;
  // Whether detection was successful
  bool detected;
  // Raw vegetation mask
  cv::Mat mask;
  // Error: horizontal offset of center from image center (normalized [-1, 1])
  float lateral_error;
  // Error: angle of centerline relative to vertical (radians)
  float angular_error;
  // Optional header for tracking the source image
  std_msgs::msg::Header header;
  // Indices of the two lane-boundary rows selected by computeCenterLine
  int left_lane_idx = -1;
  int right_lane_idx = -1;
  // Quality of the fit (0..1, 1 = very confident)
  float confidence = 0.0f;
  // Reason this frame was rejected ("" if accepted)
  std::string reject_reason;
  // True if the centerline is a prediction from the EMA tracker (no fresh
  // detection this frame).  Lets the renderer distinguish real vs predicted.
  bool predicted = false;
};

class CropRowDetector {
public:
  CropRowDetector();

  /**
   * @brief Detect crop rows from an RGB image
   * @param rgb_input Input RGB image
   * @return CropRowResult containing detected rows and control errors
   */
  CropRowResult detect(const cv::Mat &rgb_input);

  /**
   * @brief Draw detection results on an image for visualization
   * @param image Image to draw on (will be modified in-place)
   * @param result Detection result to visualize
   */
  void drawResults(cv::Mat &image, const CropRowResult &result);

  // Setters for parameters
  void setGreenLower(int h, int s, int v);
  void setGreenUpper(int h, int s, int v);
  void setMinContourArea(double area);
  void setMorphKernelSize(int size);
  void setMinLaneWidthPx(float px);
  void setMaxLaneWidthPx(float px);
  void setMaxCenterJitter(float fraction);
  void setMinConfidence(float c);

private:
  // HSV thresholds for green vegetation
  cv::Scalar green_lower_;
  cv::Scalar green_upper_;

  // Morphological kernel size
  int morph_kernel_size_;

  // Minimum contour area to consider
  double min_contour_area_;

  // Lane geometry constraints (pixels) - used to reject bad detections
  float min_lane_width_px_ = 40.0f;
  float max_lane_width_px_ = 200.0f;

  // If centerline moves more than this fraction of image width per frame, reject
  float max_center_jitter_ = 0.20f;

  // Minimum confidence to publish a detection
  float min_confidence_ = 0.3f;

  // Previous frame's accepted center (bottom x, normalized to image center).
  // 0.0 means "image center" — used as the default prediction when no
  // detection has happened yet, so the first frame can still produce a
  // centerline for the controller.
  float prev_center_bottom_norm_ = 0.0f;
  float prev_center_top_norm_ = 0.0f;
  float prev_angular_error_ = 0.0f;
  // True after the first real detection.  The EMA blend only kicks in
  // once we have at least one real measurement, so the first frame isn't
  // pulled toward zero.
  bool prev_initialized_ = false;
  // How many consecutive frames we are allowed to keep predicting after
  // losing detection, before giving up and reporting detected=false.
  int max_predicted_frames_ = 30;
  int predicted_frame_count_ = 0;

  /**
   * @brief Create vegetation mask using HSV thresholding
   */
  cv::Mat createVegetationMask(const cv::Mat &hsv_image);

  /**
   * @brief Reset EMA history and prediction counter.  Call this whenever
   * the camera switches which lane it is looking at (e.g. at the start
   * of a new aisle, or when the active camera changes from front to back)
   * so the first few frames don't carry over a stale centerline from the
   * previous lane.
   */
  void reset();

  /**
   * @brief Clean up mask with morphological operations
   */
  cv::Mat cleanMask(const cv::Mat &mask);

  /**
   * @brief Find row-like contours and fit lines
   */
  std::vector<std::pair<cv::Point2f, cv::Point2f>>
  findRowLines(const cv::Mat &mask, const cv::Mat &rgb_image);

  /**
   * @brief Compute the centerline between the two rows closest to image center
   *        and return the indices of the selected rows.  Also returns a
   *        confidence score and (if rejected) a reason string.
   */
  std::pair<cv::Point2f, cv::Point2f>
  computeCenterLine(const std::vector<std::pair<cv::Point2f, cv::Point2f>> &row_lines,
                    float image_width,
                    float image_height,
                    int &left_lane_idx,
                    int &right_lane_idx,
                    float &confidence,
                    std::string &reject_reason);
};

#endif // CROP_ROW_DETECTOR_HPP