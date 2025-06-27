#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "CaliperTool.h"

class EdgeWidthMeasure {
public:
    EdgeWidthMeasure();
    ~EdgeWidthMeasure();

    int runToolPro(const cv::Mat& srcImage, const CaliperTool& caliper,
                   double threshold_delta1, int direction1,
                   double threshold_delta2, int direction2,
                   int segment_num, int cull_distance,
                   cv::Mat& dstImage, double& distance);

    const std::vector<cv::Point2f>& getEdgePoints1() const { return _last_edge_points1; }
    const std::vector<cv::Point2f>& getEdgePoints2() const { return _last_edge_points2; }

private:
    cv::Point2f _findCrosspointimprove(const std::vector<float>& lineTiDu, const std::vector<cv::Point2f>& ijRecord,
                                 int direction, int threshold_delta);

    int _getEdgeWidth(const cv::Mat& src_mat, const std::vector<std::pair<cv::Point2f, cv::Point2f>>& segments,
                 std::vector<cv::Point2f>& edge_points1, std::vector<cv::Point2f>& edge_points2,
                 int threshold_delta1, int direction1, int threshold_delta2, int direction2, int segment_num);

    double _getDistP2L(const cv::Point2f pointP,
                      const cv::Point2f pointA,
                      const cv::Point2f pointB);

    double _average(const std::vector<double>& arrays, int n);

    cv::Point2d _affineTransformPoint(const cv::Point2d match_origin_point,
                                     double match_origin_angle,
                                     const cv::Point2d match_current_point,
                                     double match_current_angle,
                                     const cv::Point2d input_point);

    static void _saveToFile(const std::vector<std::pair<cv::Point2f, cv::Point2f>>& segments,
                          const std::string& filename);

    std::vector<cv::Point2f> _last_edge_points1;
    std::vector<cv::Point2f> _last_edge_points2;
}; 