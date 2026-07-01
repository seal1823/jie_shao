#include "agribot_visual_servoing/crop_row_detector.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

CropRowDetector::CropRowDetector()
  : green_lower_(25, 30, 30)
  , green_upper_(95, 255, 255)
  , morph_kernel_size_(7)
  , min_contour_area_(80.0)
{}

void CropRowDetector::setGreenLower(int h, int s, int v) {
  green_lower_ = cv::Scalar(h, s, v);
}

void CropRowDetector::setGreenUpper(int h, int s, int v) {
  green_upper_ = cv::Scalar(h, s, v);
}

void CropRowDetector::setMinContourArea(double area) {
  min_contour_area_ = area;
}

void CropRowDetector::setMorphKernelSize(int size) {
  morph_kernel_size_ = size;
}

void CropRowDetector::setMinLaneWidthPx(float px) {
  min_lane_width_px_ = px;
}

void CropRowDetector::setMaxLaneWidthPx(float px) {
  max_lane_width_px_ = px;
}

void CropRowDetector::setMaxCenterJitter(float fraction) {
  max_center_jitter_ = fraction;
}

void CropRowDetector::setMinConfidence(float c) {
  min_confidence_ = c;
}

void CropRowDetector::reset() {
  prev_center_top_norm_ = 0.0f;
  prev_center_bottom_norm_ = 0.0f;
  prev_angular_error_ = 0.0f;
  prev_initialized_ = false;
  predicted_frame_count_ = 0;
}

cv::Mat CropRowDetector::createVegetationMask(const cv::Mat &hsv_image) {
  cv::Mat mask;
  cv::inRange(hsv_image, green_lower_, green_upper_, mask);
  return mask;
}

cv::Mat CropRowDetector::cleanMask(const cv::Mat &mask) {
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(morph_kernel_size_, morph_kernel_size_));

  cv::Mat cleaned;
  // Close then open to fill small holes and remove noise
  cv::morphologyEx(mask, cleaned, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(cleaned, cleaned, cv::MORPH_OPEN, kernel);
  return cleaned;
}

// Distance from a point to a line given as (vx, vy, x0, y0) in the
// cv::fitLine parameterisation (unit direction (vx,vy) passing through
// (x0,y0)).
static float pointToLineDistance(const cv::Point &p,
                                 float vx, float vy,
                                 float x0, float y0) {
  // d = |(p - x0,y0) x (vx, vy)|  (magnitude of the 2D cross product,
  // which equals the perpendicular distance since (vx,vy) is unit).
  float dx = static_cast<float>(p.x) - x0;
  float dy = static_cast<float>(p.y) - y0;
  return std::abs(dx * vy - dy * vx);
}

// Fit a line to a set of 2D points using RANSAC.  The returned line
// parameters are in the cv::fitLine format.  The inlier_points vector is
// filled with the points that were counted as inliers in the best
// iteration, suitable for a final least-squares refit.  This is more
// robust than a plain cv::fitLine on the full set, because it ignores
// outlier points (e.g. a heavy near-camera crop on one row, or a stray
// blob that slipped into the cluster) that would otherwise pull the fit
// off the actual row.
static cv::Vec4f fitLineRansac(const std::vector<cv::Point> &points,
                               std::vector<cv::Point> &inlier_points,
                               int iterations = 80,
                               float inlier_threshold = 6.0f) {
  inlier_points.clear();
  if (points.size() < 2) return cv::Vec4f(0, 1, 0, 0);

  cv::Vec4f best_line(0, 1, 0, 0);
  std::size_t best_inlier_count = 0;
  std::mt19937 rng(0xC0FFEEu);  // deterministic for reproducibility

  const std::size_t n = points.size();
  for (int it = 0; it < iterations; ++it) {
    // Sample two distinct points.
    std::uniform_int_distribution<std::size_t> uni(0, n - 1);
    std::size_t i1 = uni(rng);
    std::size_t i2 = uni(rng);
    if (i1 == i2) continue;

    const cv::Point &p1 = points[i1];
    const cv::Point &p2 = points[i2];
    float dx = static_cast<float>(p2.x - p1.x);
    float dy = static_cast<float>(p2.y - p1.y);
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-3f) continue;  // degenerate sample
    dx /= len;
    dy /= len;

    // Count inliers.
    std::vector<cv::Point> inliers;
    inliers.reserve(n);
    for (const auto &p : points) {
      if (pointToLineDistance(p, dx, dy,
                              static_cast<float>(p1.x),
                              static_cast<float>(p1.y)) < inlier_threshold) {
        inliers.push_back(p);
      }
    }
    if (inliers.size() > best_inlier_count) {
      best_inlier_count = inliers.size();
      best_line = cv::Vec4f(dx, dy, static_cast<float>(p1.x), static_cast<float>(p1.y));
      inlier_points = std::move(inliers);
      // Early exit if a clear majority of points agree (saves iterations
      // when the cluster is mostly one row).
      if (best_inlier_count * 2 > n) break;
    }
  }

  // If RANSAC didn't find anything, fall back to a vertical line at the
  // mean x of all points.
  if (best_inlier_count < 2) {
    float sum_x = 0.0f, sum_y = 0.0f;
    for (const auto &p : points) { sum_x += p.x; sum_y += p.y; }
    sum_x /= n;
    sum_y /= n;
    return cv::Vec4f(0, 1, sum_x, sum_y);
  }
  return best_line;
}

std::vector<std::pair<cv::Point2f, cv::Point2f>>
CropRowDetector::findRowLines(const cv::Mat &mask, const cv::Mat &rgb_image) {
  std::vector<std::pair<cv::Point2f, cv::Point2f>> row_lines;

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(mask.clone(), contours, hierarchy,
                    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Collect significant contour blobs with their center x
  struct Blob {
    std::vector<cv::Point> points;
    float center_x;
    float center_y;
  };
  std::vector<Blob> blobs;
  for (const auto &contour : contours) {
    double area = cv::contourArea(contour);
    if (area < min_contour_area_) continue;

    cv::Moments m = cv::moments(contour);
    if (m.m00 == 0) continue;
    float cx = static_cast<float>(m.m10 / m.m00);
    float cy = static_cast<float>(m.m01 / m.m00);

    blobs.push_back({contour, cx, cy});
  }

  if (blobs.empty()) return row_lines;

  // Sort blobs by center x
  std::sort(blobs.begin(), blobs.end(),
            [](const Blob &a, const Blob &b) { return a.center_x < b.center_x; });

  // Cluster blobs into rows based on horizontal proximity of contour centers.
  // This is more robust than a histogram when rows appear near image borders
  // or when a row is split into several disjoint contours.
  std::vector<std::vector<Blob>> clusters;
  float min_row_separation = static_cast<float>(mask.cols) / 8.0f;

  for (const auto &blob : blobs) {
    bool merged = false;
    for (auto &cluster : clusters) {
      float cluster_center = 0.0f;
      for (const auto &b : cluster) cluster_center += b.center_x;
      cluster_center /= static_cast<float>(cluster.size());
      if (std::abs(blob.center_x - cluster_center) < min_row_separation) {
        cluster.push_back(blob);
        merged = true;
        break;
      }
    }
    if (!merged) {
      clusters.push_back({blob});
    }
  }

  // Limit to max 4 rows, keep the largest clusters if there are too many
  if (clusters.size() > 4) {
    std::sort(clusters.begin(), clusters.end(),
              [](const std::vector<Blob> &a, const std::vector<Blob> &b) {
                float area_a = 0.0f, area_b = 0.0f;
                for (const auto &blob : a) {
                  area_a += static_cast<float>(cv::contourArea(blob.points));
                }
                for (const auto &blob : b) {
                  area_b += static_cast<float>(cv::contourArea(blob.points));
                }
                return area_a > area_b;
              });
    clusters.resize(4);
  }

  if (clusters.empty()) return row_lines;

  float image_height = static_cast<float>(mask.rows);
  float image_width = static_cast<float>(mask.cols);

  // Generate a row line for each cluster.
  // Crop rows are roughly vertical in the image, so we use the mean x of the
  // cluster points as the row center and draw a vertical line.
  for (const auto &cluster : clusters) {
    if (cluster.empty()) continue;

    std::vector<cv::Point> all_points;
    float sum_cx = 0.0f;
    for (const auto &b : cluster) {
      all_points.insert(all_points.end(), b.points.begin(), b.points.end());
      sum_cx += b.center_x * static_cast<float>(b.points.size());
    }

    if (all_points.size() < 10) continue;

    float mean_x = sum_cx / static_cast<float>(all_points.size());

    // Robust line fit: a least-squares fit through ALL cluster points is
    // easily skewed by a single heavy outlier (e.g. a large near-camera
    // crop on one row, or a stray blob that slipped into the cluster).
    // The result is a line that misses the back-of-row crops.  RANSAC
    // ignores outliers and produces a line that actually follows the row.
    std::vector<cv::Point> inlier_points;
    cv::Vec4f line_params = fitLineRansac(all_points, inlier_points);

    // Refit on the inliers for a cleaner slope.  Skip the refit if RANSAC
    // found very few inliers (e.g. cluster was mostly noise) — fall back
    // to the mean-x vertical line.
    if (inlier_points.size() >= 10) {
      cv::fitLine(inlier_points, line_params, cv::DIST_L2, 0, 0.01, 0.01);
    }

    float vx = line_params[0];
    float vy = line_params[1];
    float x0 = line_params[2];
    float y0 = line_params[3];

    float top_x, bottom_x;
    if (std::abs(vy) < 0.01f) {
      // Fit is nearly horizontal, fall back to vertical line at mean x
      top_x = mean_x;
      bottom_x = mean_x;
    } else {
      top_x = x0 + (0.0f - y0) * vx / vy;
      bottom_x = x0 + (image_height - y0) * vx / vy;
    }

    top_x = std::max(0.0f, std::min(image_width, top_x));
    bottom_x = std::max(0.0f, std::min(image_width, bottom_x));

    row_lines.emplace_back(
        cv::Point2f(top_x, 0.0f),
        cv::Point2f(bottom_x, image_height));
  }

  // Sort by x
  std::sort(row_lines.begin(), row_lines.end(),
            [](const std::pair<cv::Point2f, cv::Point2f> &a,
               const std::pair<cv::Point2f, cv::Point2f> &b) {
              return a.first.x < b.first.x;
            });

  return row_lines;
}

std::pair<cv::Point2f, cv::Point2f>
CropRowDetector::computeCenterLine(
    const std::vector<std::pair<cv::Point2f, cv::Point2f>> &row_lines,
    float image_width,
    float image_height,
    int &left_lane_idx,
    int &right_lane_idx,
    float &confidence,
    std::string &reject_reason) {
  // Default center line (vertical through image center)
  float image_center_x = image_width / 2.0f;
  confidence = 0.0f;
  reject_reason.clear();

  left_lane_idx = -1;
  right_lane_idx = -1;

  if (row_lines.size() < 2) {
    // Can't compute centerline, return default
    reject_reason = "not_enough_rows";
    return {cv::Point2f(image_center_x, 0.0f),
            cv::Point2f(image_center_x, image_height)};
  }

  // Pick the pair of *adjacent* rows whose center is closest to image center.
  // Adjacent rows in the sorted list are most likely to be the two boundaries
  // of the same lane, which prevents the robot from driving between distant
  // rows that actually belong to different lanes.
  float min_distance = std::numeric_limits<float>::max();
  size_t best_left = 0, best_right = 1;
  for (size_t i = 0; i + 1 < row_lines.size(); ++i) {
    size_t j = i + 1;
    float avg_top = (row_lines[i].first.x + row_lines[j].first.x) / 2.0f;
    float avg_bottom = (row_lines[i].second.x + row_lines[j].second.x) / 2.0f;
    float dist = std::abs(avg_top - image_center_x)
               + std::abs(avg_bottom - image_center_x);
    if (dist < min_distance) {
      min_distance = dist;
      best_left = i;
      best_right = j;
    }
  }

  left_lane_idx = static_cast<int>(best_left);
  right_lane_idx = static_cast<int>(best_right);

  const auto &left_line = row_lines[best_left];
  const auto &right_line = row_lines[best_right];

  // Reject impossible lane geometries early to keep the centerline stable.
  // The lane width is measured at both top and bottom of the image.
  float lane_width_top = std::abs(right_line.first.x - left_line.first.x);
  float lane_width_bottom = std::abs(right_line.second.x - left_line.second.x);
  float lane_width = std::max(lane_width_top, lane_width_bottom);

  if (lane_width < min_lane_width_px_) {
    reject_reason = "lane_too_narrow";
    return {cv::Point2f(image_center_x, 0.0f),
            cv::Point2f(image_center_x, image_height)};
  }
  if (lane_width > max_lane_width_px_) {
    reject_reason = "lane_too_wide";
    return {cv::Point2f(image_center_x, 0.0f),
            cv::Point2f(image_center_x, image_height)};
  }

  // Compute centerline at top and bottom
  float center_top_x = (left_line.first.x + right_line.first.x) / 2.0f;
  float center_bottom_x = (left_line.second.x + right_line.second.x) / 2.0f;

  // Confidence = how close the center is to image center, scaled to 0..1.
  // Centerline at image center -> 1.0; far from center -> 0.0.  Also
  // penalized if the centerline has jumped relative to the previous frame.
  float offset_top = std::abs(center_top_x - image_center_x) / image_center_x;
  float offset_bottom = std::abs(center_bottom_x - image_center_x) / image_center_x;
  float offset = std::max(offset_top, offset_bottom);
  confidence = std::max(0.0f, std::min(1.0f, 1.0f - offset));

  float center_top_norm = (center_top_x - image_center_x) / image_center_x;
  float center_bottom_norm = (center_bottom_x - image_center_x) / image_center_x;
  if (prev_center_bottom_norm_ > -1.5f && prev_center_top_norm_ > -1.5f) {
    float d_top = std::abs(center_top_norm - prev_center_top_norm_);
    float d_bottom = std::abs(center_bottom_norm - prev_center_bottom_norm_);
    float jitter = std::max(d_top, d_bottom);
    // Penalize confidence if the centerline jumped a lot this frame.
    confidence = std::max(0.0f, confidence - jitter);
  }

  return {cv::Point2f(center_top_x, 0.0f),
          cv::Point2f(center_bottom_x, image_height)};
}

CropRowResult CropRowDetector::detect(const cv::Mat &rgb_input) {
  CropRowResult result;
  result.detected = false;
  result.lateral_error = 0.0f;
  result.angular_error = 0.0f;
  result.left_lane_idx = -1;
  result.right_lane_idx = -1;
  result.confidence = 0.0f;
  result.predicted = false;

  if (rgb_input.empty()) return result;

  cv::Mat hsv;
  cv::cvtColor(rgb_input, hsv, cv::COLOR_BGR2HSV);

  // Step 1: Create vegetation mask
  cv::Mat mask = createVegetationMask(hsv);
  result.mask = mask;

  // Step 2: Clean the mask
  cv::Mat cleaned = cleanMask(mask);

  // Step 3: Find row lines
  result.row_lines = findRowLines(cleaned, rgb_input);

  if (result.row_lines.size() < 2) {
    // No fresh detection.  Use the EMA history to predict a centerline so
    // the controller always has a reference.  On the very first frame,
    // prev_* is 0.0, so the predicted line is the image-center vertical —
    // a sensible default for "I don't know where the lane is yet".
    if (predicted_frame_count_ < max_predicted_frames_) {
      float image_width = static_cast<float>(rgb_input.cols);
      float image_height = static_cast<float>(rgb_input.rows);
      float image_center_x = image_width / 2.0f;
      float top_x = image_center_x + prev_center_top_norm_ * image_center_x;
      float bottom_x = image_center_x + prev_center_bottom_norm_ * image_center_x;
      result.center_line = {cv::Point2f(top_x, 0.0f),
                            cv::Point2f(bottom_x, image_height)};
      result.lateral_error = prev_center_bottom_norm_;
      result.angular_error = prev_angular_error_;
      // Confidence decays linearly with the number of predicted frames.
      float decay = 1.0f - static_cast<float>(predicted_frame_count_)
                            / static_cast<float>(max_predicted_frames_);
      result.confidence = std::max(0.05f, 0.4f * decay);
      result.predicted = true;
      result.reject_reason = "not_enough_rows (predicted, n=" +
                              std::to_string(result.row_lines.size()) + ")";
      predicted_frame_count_++;
      result.detected = true;
    } else {
      result.reject_reason = "not_enough_rows";
    }
    return result;
  }

  // Step 4: Compute centerline — also selects which two rows form the lane.
  float image_width = static_cast<float>(rgb_input.cols);
  float image_height = static_cast<float>(rgb_input.rows);
  result.center_line = computeCenterLine(result.row_lines,
                                         image_width,
                                         image_height,
                                         result.left_lane_idx,
                                         result.right_lane_idx,
                                         result.confidence,
                                         result.reject_reason);

  // If the centerline was rejected by geometry, predict from EMA history.
  if (!result.reject_reason.empty()) {
    if (predicted_frame_count_ < max_predicted_frames_) {
      float image_center_x = image_width / 2.0f;
      float top_x = image_center_x + prev_center_top_norm_ * image_center_x;
      float bottom_x = image_center_x + prev_center_bottom_norm_ * image_center_x;
      result.center_line = {cv::Point2f(top_x, 0.0f),
                            cv::Point2f(bottom_x, image_height)};
      result.lateral_error = prev_center_bottom_norm_;
      result.angular_error = prev_angular_error_;
      result.confidence = std::max(0.05f, 0.3f - 0.05f * predicted_frame_count_);
      result.predicted = true;
      result.reject_reason += " (predicted)";
      predicted_frame_count_++;
      result.detected = true;
    }
    return result;
  }

  // Step 5: Compute errors for visual servoing
  float image_center_x = image_width / 2.0f;
  float center_top_x = result.center_line.first.x;
  float center_bottom_x = result.center_line.second.x;

  // EMA smoothing.  Heavier smoothing (alpha=0.2) keeps the output stable
  // against single-frame outliers while still tracking real drift.
  const float alpha = 0.2f;
  float raw_lateral = (center_bottom_x - image_center_x) / image_center_x;
  float raw_angular = std::atan2(center_bottom_x - center_top_x,
                                 image_height);
  float raw_top_norm = (center_top_x - image_center_x) / image_center_x;
  float raw_bottom_norm = (center_bottom_x - image_center_x) / image_center_x;

  if (prev_initialized_) {
    result.lateral_error = (1.0f - alpha) * prev_center_bottom_norm_
                         + alpha * raw_lateral;
    result.angular_error = (1.0f - alpha) * prev_angular_error_
                         + alpha * raw_angular;
  } else {
    result.lateral_error = raw_lateral;
    result.angular_error = raw_angular;
    prev_initialized_ = true;
  }

  prev_center_top_norm_ = (1.0f - alpha) * prev_center_top_norm_ + alpha * raw_top_norm;
  prev_center_bottom_norm_ = (1.0f - alpha) * prev_center_bottom_norm_ + alpha * raw_bottom_norm;
  prev_angular_error_ = result.angular_error;
  predicted_frame_count_ = 0;  // reset predictor on fresh detection

  result.detected = true;
  return result;
}

void CropRowDetector::drawResults(cv::Mat &image, const CropRowResult &result) {
  if (image.empty()) return;

  // Draw only the two lane-boundary rows selected by computeCenterLine.
  // All other detected rows are suppressed to keep the display clean.
  if (result.left_lane_idx >= 0 && result.left_lane_idx < static_cast<int>(result.row_lines.size()) &&
      result.right_lane_idx >= 0 && result.right_lane_idx < static_cast<int>(result.row_lines.size())) {
    // Left lane boundary in yellow
    const auto &left = result.row_lines[result.left_lane_idx];
    cv::line(image, left.first, left.second, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    // Right lane boundary in yellow
    const auto &right = result.row_lines[result.right_lane_idx];
    cv::line(image, right.first, right.second, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  }

  // Draw centerline in red (the desired path).  Always draw it when we
  // have a centerline — predicted lines use a dashed style so the user
  // can tell them apart from real detections.
  if (result.detected) {
    cv::Scalar color = result.predicted ? cv::Scalar(180, 130, 255)
                                        : cv::Scalar(0, 0, 255);
    int thickness = result.predicted ? 2 : 3;
    int line_type = cv::LINE_AA;
    if (result.predicted) {
      // Draw a dashed line by hand
      cv::Point2f a = result.center_line.first;
      cv::Point2f b = result.center_line.second;
      float dx = b.x - a.x, dy = b.y - a.y;
      float total = std::sqrt(dx * dx + dy * dy);
      if (total > 1.0f) {
        float ux = dx / total, uy = dy / total;
        const float dash = 12.0f, gap = 8.0f;
        for (float s = 0.0f; s < total; s += dash + gap) {
          float s1 = s;
          float s2 = std::min(s + dash, total);
          cv::Point2f p1(a.x + ux * s1, a.y + uy * s1);
          cv::Point2f p2(a.x + ux * s2, a.y + uy * s2);
          cv::line(image, p1, p2, color, thickness, line_type);
        }
      }
    } else {
      cv::line(image, result.center_line.first, result.center_line.second,
               color, thickness, line_type);
    }

    // Draw direction arrow on centerline
    cv::Point2f mid(
        (result.center_line.first.x + result.center_line.second.x) / 2.0f,
        (result.center_line.first.y + result.center_line.second.y) / 2.0f);
    float dx = result.center_line.second.x - result.center_line.first.x;
    float dy = result.center_line.second.y - result.center_line.first.y;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len > 0) {
      dx /= len;
      dy /= len;
      cv::Point2f arrow_end(mid.x + dx * 30, mid.y + dy * 30);
      cv::arrowedLine(image, mid, arrow_end, color, 2, cv::LINE_AA, 0, 0.3);
    }

    // Draw image center vertical line for reference (white, dashed style)
    float cx = static_cast<float>(image.cols) / 2.0f;
    cv::line(image, cv::Point2f(cx, 0), cv::Point2f(cx, static_cast<float>(image.rows)),
             cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // Show error values as text
    char lateral_text[64], angular_text[64];
    std::snprintf(lateral_text, sizeof(lateral_text), "Lateral Error: %.3f", result.lateral_error);
    std::snprintf(angular_text, sizeof(angular_text), "Angular Error: %.3f rad", result.angular_error);
    cv::putText(image, lateral_text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(image, angular_text, cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

    // Detection status
    cv::Scalar status_color = result.detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    std::string status = result.detected ? "ROWS DETECTED" : "NO ROWS";
    if (!result.reject_reason.empty()) {
      status += " (" + result.reject_reason + ")";
    }
    cv::putText(image, status, cv::Point(10, image.rows - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2);

    // Confidence
    char conf_text[64];
    std::snprintf(conf_text, sizeof(conf_text), "Conf: %.2f", result.confidence);
    cv::putText(image, conf_text, cv::Point(10, image.rows - 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
  }

  // Draw vegetation mask overlay (semi-transparent) in bottom-right corner
  if (!result.mask.empty()) {
    cv::Mat small_mask;
    cv::resize(result.mask, small_mask, cv::Size(160, 120));
    cv::Mat mask_color;
    cv::cvtColor(small_mask, mask_color, cv::COLOR_GRAY2BGR);
    cv::Mat roi = image(cv::Rect(image.cols - 170, image.rows - 130, 160, 120));
    cv::addWeighted(roi, 0.6, mask_color, 0.4, 0, roi);
    cv::rectangle(image, cv::Rect(image.cols - 170, image.rows - 130, 160, 120),
                  cv::Scalar(200, 200, 200), 1);
    cv::putText(image, "Vegetation Mask", cv::Point(image.cols - 165, image.rows - 135),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
  }
}