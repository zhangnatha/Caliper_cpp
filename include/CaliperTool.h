#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class CaliperTool {
public:
    CaliperTool(cv::Point2f p1, cv::Point2f p2, float height, int segment_num);
    void calculate();
    void draw(cv::Mat& img, bool draw_rect = true, bool draw_arrow = true, bool draw_segments = true, float alpha = 0.2f);
    std::vector<std::pair<cv::Point2f, cv::Point2f>> getSegments() const;
    std::vector<cv::Point2f> getRectPoints() const;
    cv::Point2f getArrowStart() const;
    cv::Point2f getArrowEnd() const;
private:
    cv::Point2f _p1, _p2;
    float _height;
    int _segment_num;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> _segments;
    std::vector<cv::Point2f> _rect_points;
    cv::Point2f _arrow_start, _arrow_end;
}; 