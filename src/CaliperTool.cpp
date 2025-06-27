#include "CaliperTool.h"
#include <opencv2/imgproc.hpp>

CaliperTool::CaliperTool(cv::Point2f p1, cv::Point2f p2, float height, int segment_num)
    : _p1(p1), _p2(p2), _height(height), _segment_num(segment_num) {
    calculate();
}

void CaliperTool::calculate() {
    _segments.clear();
    _rect_points.clear();
    // 主方向
    cv::Point2f dir = _p2 - _p1;
    float len = cv::norm(dir);
    if (len == 0) return;
    dir /= len;
    // 法向
    cv::Point2f normal(-dir.y, dir.x);
    // 四角
    cv::Point2f q1 = _p1 + normal * (_height / 2);
    cv::Point2f q2 = _p2 + normal * (_height / 2);
    cv::Point2f q3 = _p2 - normal * (_height / 2);
    cv::Point2f q4 = _p1 - normal * (_height / 2);
    _rect_points = { q1, q2, q3, q4 };

    // 分割线（与主方向平行，等距分布）
    for (int i = 0; i < _segment_num; ++i) {
        float offset = -_height / 2 + i * (_height / (_segment_num - 1));
        cv::Point2f center = (_p1 + _p2) * 0.5f + normal * offset;
        cv::Point2f start = center - dir * (len / 2);
        cv::Point2f end   = center + dir * (len / 2);
        _segments.push_back({ start, end });
    }
    // 箭头
    _arrow_start = (_p1 + _p2) * 0.5f;
    _arrow_end = _arrow_start + dir * (len * 0.3f);
}

void CaliperTool::draw(cv::Mat& img, bool draw_rect, bool draw_arrow, bool draw_segments, float alpha) {
    // 先将rect_points转为int型点
    std::vector<cv::Point> int_rect_points;
    for (const auto& pt : _rect_points) {
        int_rect_points.emplace_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
    }
    // 半透明填充
    if (draw_rect && alpha > 0) {
        std::vector<std::vector<cv::Point>> pts = { int_rect_points };
        cv::Mat overlay;
        img.copyTo(overlay);
        cv::fillPoly(overlay, pts, cv::Scalar(255, 200, 0, 60));
        cv::addWeighted(overlay, alpha, img, 1 - alpha, 0, img);
    }
    // 边框
    if (draw_rect) {
        const cv::Point* pts[1] = { int_rect_points.data() };
        int npts = 4;
        cv::polylines(img, pts, &npts, 1, true, cv::Scalar(255, 0, 0), 1);
    }
    // 分割线
    if (draw_segments) {
        for (const auto& seg : _segments) {
            cv::line(img, seg.first, seg.second, cv::Scalar(255, 200, 0), 1);
        }
    }
    // 主方向箭头
    if (draw_arrow) {
        cv::arrowedLine(img, _arrow_start, _arrow_end, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.2);
        // 箭头中心加方块
        cv::rectangle(img, _arrow_start - cv::Point2f(5, 5), _arrow_start + cv::Point2f(5, 5), cv::Scalar(255, 0, 0), cv::FILLED);
    }
}

std::vector<std::pair<cv::Point2f, cv::Point2f>> CaliperTool::getSegments() const {
    return _segments;
}
std::vector<cv::Point2f> CaliperTool::getRectPoints() const {
    return _rect_points;
}
cv::Point2f CaliperTool::getArrowStart() const {
    return _arrow_start;
}
cv::Point2f CaliperTool::getArrowEnd() const {
    return _arrow_end;
} 