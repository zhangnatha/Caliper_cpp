#include "edge_width_measure.h"
#include <cmath>
#include <fstream>

#define M_PI 3.14159265358979323846

EdgeWidthMeasure::EdgeWidthMeasure() {}
EdgeWidthMeasure::~EdgeWidthMeasure() {}

void EdgeWidthMeasure::_saveToFile(const std::vector<std::pair<cv::Point2f, cv::Point2f>>& segments,
                          const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 保存 segments.first (line_small_points)
    for (const auto& segment : segments) {
        outFile << "p1: " << segment.first.x << "," << segment.first.y << std::endl;
    }

    // 保存分隔线
    outFile << "************************************" << std::endl;

    // 保存 segments.second (line_big_points)
    for (const auto& segment : segments) {
        outFile << "p2: " << segment.second.x << "," << segment.second.y << std::endl;
    }

    outFile.close();
    std::cout << "内容已成功保存到: " << filename << std::endl;
}

int EdgeWidthMeasure::runToolPro(const cv::Mat& srcImage, const CaliperTool& caliper,
                                 double threshold_delta1, int direction1,
                                 double threshold_delta2, int direction2,
                                 int segment_num, int cull_distance,
                                 cv::Mat& dstImage, double& distance) {
    try {
        srcImage.copyTo(dstImage);
        std::vector<std::pair<cv::Point2f, cv::Point2f>> segments = caliper.getSegments();
        std::vector<cv::Point2f> edge_points1, edge_points2;
        // _saveToFile(segments,"../output.txt");
        int result = _getEdgeWidth(dstImage, segments, edge_points1, edge_points2,
                                  threshold_delta1, direction1, threshold_delta2, direction2, segment_num);
        if (result != 0) return -1;
        // 保存到成员变量
        _last_edge_points1 = edge_points1;
        _last_edge_points2 = edge_points2;

        // 拟合直线1
        if (edge_points1.size() < 2) return -1;
        std::vector<cv::Point2f> new_xy1(edge_points1.size());
        for (size_t i = 0; i < edge_points1.size(); i++) {
            new_xy1[i] = cv::Point2f(edge_points1[i].x, edge_points1[i].y);
        }
        int w = srcImage.cols;
        cv::Vec4f fitline1;
        cv::fitLine(new_xy1, fitline1, cv::DIST_L2, 0, 0.01, 0.01);
        float vx1 = fitline1[0];
        float vy1 = fitline1[1];
        float x1 = fitline1[2];
        float y1 = fitline1[3];
        double x1_1 = w - 1;
        double y1_1 = (w - x1) * vy1 / vx1 + y1;
        double x1_2 = 0;
        double y1_2 = (-x1 * vy1 / vx1) + y1;

        // 剔除点1
        std::vector<cv::Point2f> select_xy1;
        std::vector<cv::Point2f> cull_xy1;
        for (size_t i = 0; i < new_xy1.size(); i++) {
            // 计算点到直线的距离
            double distance = _getDistP2L(new_xy1[i], cv::Point2f(x1_1, y1_1), cv::Point2f(x1_2, y1_2));
            if (distance > cull_distance) {
                cull_xy1.push_back(new_xy1[i]);
            } else {
                select_xy1.push_back(new_xy1[i]);
            }
        }

        // 剔除点后再次拟合直线1
        cv::fitLine(select_xy1, fitline1, cv::DIST_L2, 0, 0.01, 0.01);
        vx1 = fitline1[0];
        vy1 = fitline1[1];
        x1 = fitline1[2];
        y1 = fitline1[3];
        x1_1 = w - 1;
        y1_1 = (w - x1) * vy1 / vx1 + y1;
        x1_2 = 0;
        y1_2 = (-x1 * vy1 / vx1) + y1;

        // 拟合直线2
        std::vector<cv::Point2f> new_xy2(edge_points2.size());
        for (size_t i = 0; i < edge_points2.size(); i++) {
            new_xy2[i] = cv::Point2f(edge_points2[i].x, edge_points2[i].y);
        }
        cv::Vec4f fitline2;
        cv::fitLine(new_xy2, fitline2, cv::DIST_L2, 0, 0.01, 0.01);
        float vx2 = fitline2[0];
        float vy2 = fitline2[1];
        float x2 = fitline2[2];
        float y2 = fitline2[3];
        double x2_1 = w - 1;
        double y2_1 = (w - x2) * vy2 / vx2 + y2;
        double x2_2 = 0;
        double y2_2 = (-x2 * vy2 / vx2) + y2;

        // 剔除点2
        std::vector<cv::Point2f> select_xy2;
        std::vector<cv::Point2f> cull_xy2;
        for (size_t i = 0; i < new_xy2.size(); i++) {
            // 计算点到直线的距离
            double distance = _getDistP2L(new_xy2[i], cv::Point2f(x2_1, y2_1), cv::Point2f(x2_2, y2_2));
            if (distance > cull_distance) {
                cull_xy2.push_back(new_xy2[i]);
            } else {
                select_xy2.push_back(new_xy2[i]);
            }
        }

        // 输出结果
        std::vector<double> out_distances(select_xy2.size());
        for (size_t i = 0; i < select_xy2.size(); i++) {
            // 计算点到直线的距离
            double distance = _getDistP2L(select_xy2[i], cv::Point2f(x1_1, y1_1), cv::Point2f(x1_2, y1_2));
            out_distances[i] = distance;
        }

        if (out_distances.empty()) {
            return -1;
        }

        // 去除最大最小值求平均值
        distance = _average(out_distances, out_distances.size());

        // 可选：绘制边缘点
        for (const auto& point : select_xy1) {
            cv::circle(dstImage, point, 3, cv::Scalar(0, 0, 255), -1);
        }
        return 0;
    } catch (...) {
        return -1;
    }
}

// 获取边缘宽度
// threshold_delta为阈值；direction的值为0时代表“从白到黑”寻找边界点，direction的值为1时代表“从黑到白”寻找边界点
// segment_num为分割线数量
int EdgeWidthMeasure::_getEdgeWidth(const cv::Mat& src_mat, const std::vector<std::pair<cv::Point2f, cv::Point2f>>& segments,
                 std::vector<cv::Point2f>& edge_points1, std::vector<cv::Point2f>& edge_points2,
                 int threshold_delta1, int direction1, int threshold_delta2, int direction2, int segment_num) {
    try {
        cv::Mat gray;
        if (src_mat.channels() == 3) {
            cv::cvtColor(src_mat, gray, cv::COLOR_BGR2GRAY);
        } else if (src_mat.channels() == 4) {
            cv::cvtColor(src_mat, gray, cv::COLOR_RGBA2GRAY);
        } else {
            src_mat.copyTo(gray);
        }

        int ww = gray.cols;
        int hh = gray.rows;

        // 边界点
        std::vector<cv::Point2f> backPoint1(segment_num);
        std::vector<cv::Point2f> backOutPoint1(segment_num);
        std::vector<cv::Point2f> backPoint2(segment_num);
        std::vector<cv::Point2f> backOutPoint2(segment_num);

        int m1 = 0, m2 = 0;

        // 计算两点间的距离
        double distance = std::sqrt(std::pow(segments[0].first.x - segments[0].second.x, 2) +
                                   std::pow(segments[0].first.y - segments[0].second.y, 2));

        std::vector<float> temparrclor1;
        temparrclor1.reserve(5000);
        std::vector<cv::Point2f> position1;
        position1.reserve(5000);
        std::vector<float> temparrclor2;
        temparrclor2.reserve(5000);
        std::vector<cv::Point2f> position2;
        position2.reserve(5000);

        for (int k = 0; k < segment_num; k++) {
            temparrclor1.clear();
            position1.clear();

            // 从segments.first往segments.second扫描
            for (int n = 0; n < (int)distance; n++) {
                float x_point1 = (n * (segments[k].second.x - segments[k].first.x)) / distance + segments[k].first.x;
                float y_point1 = (n * (segments[k].second.y - segments[k].first.y)) / distance + segments[k].first.y;
                float tempf = std::abs(y_point1 - (int)y_point1);
                cv::Point2f tempPt(x_point1, y_point1);

                float avgGrey = (float)gray.at<uchar>((int)y_point1, (int)x_point1);
                float avgGrey1 = (float)gray.at<uchar>((int)(y_point1 + 1), (int)x_point1);
                float grey = avgGrey * (1 - tempf) + avgGrey1 * tempf;

                temparrclor1.push_back(grey);
                position1.push_back(tempPt);
            }

            // 找穿越点
            cv::Point2f return_point1 = _findCrosspointimprove(temparrclor1, position1, direction1, threshold_delta1);
            if (return_point1.x != 0 || return_point1.y != 0) {
                backPoint1[m1] = return_point1;
                m1++;
            }

            temparrclor2.clear();
            position2.clear();

            // 从segments.second往segments.first扫描
            for (int n = 0; n < (int)distance; n++) {
                float x_point2 = (n * (segments[k].first.x - segments[k].second.x)) / distance + segments[k].second.x;
                float y_point2 = (n * (segments[k].first.y - segments[k].second.y)) / distance + segments[k].second.y;
                float tempf = std::abs(y_point2 - (int)y_point2);
                cv::Point2f tempPt(x_point2, y_point2);

                float avgGrey = (float)gray.at<uchar>((int)y_point2, (int)x_point2);
                float avgGrey1 = (float)gray.at<uchar>((int)(y_point2 + 1), (int)x_point2);
                float grey = avgGrey * (1 - tempf) + avgGrey1 * tempf;

                temparrclor2.push_back(grey);
                position2.push_back(tempPt);
            }

            // 找穿越点
            cv::Point2f return_point2 = _findCrosspointimprove(temparrclor2, position2, direction2, threshold_delta2);
            if (return_point2.x != 0 || return_point2.y != 0) {
                backPoint2[m2] = return_point2;
                m2++;
            }
        }

        backOutPoint1.resize(m1);
        for (int n = 0; n < m1; n++) {
            backOutPoint1[n] = backPoint1[n];
        }
        edge_points1 = backOutPoint1;

        backOutPoint2.resize(m2);
        for (int n = 0; n < m2; n++) {
            backOutPoint2[n] = backPoint2[n];
        }
        edge_points2 = backOutPoint2;

        return 0;
    } catch (...) {
        return -1;
    }
}

// 梯度和求穿越点
cv::Point2f EdgeWidthMeasure::_findCrosspointimprove(const std::vector<float>& lineTiDu, const std::vector<cv::Point2f>& ijRecord,
                                 int direction, int threshold_delta) {
    try {
        std::vector<float> deltatemp;
        deltatemp.reserve(5000);
        std::vector<float> fenzuaverage;
        fenzuaverage.reserve(5000);

        for (size_t i = 0; i < lineTiDu.size() - 1; i++) {
            float grey1 = lineTiDu[i + 1];
            float delta = (lineTiDu[i] - grey1);
            if (std::abs(delta) < 5)
                delta = 0;
            deltatemp.push_back(delta);
        }

        int fenzuC = deltatemp.size() / 5;
        for (int i = 0; i < fenzuC * 5; i += 5) {
            float sum5 = deltatemp[i] + deltatemp[i + 1] + deltatemp[i + 2] + deltatemp[i + 3] + deltatemp[i + 4];
            float aver = sum5 / 5;
            if (std::abs(aver) < 5)
                aver = 0;
            fenzuaverage.push_back(aver);
        }

        if (fenzuaverage.empty()) return cv::Point2f();

        int crosspoint = 0;
        int crosspoint1 = 0;

        // 从白到黑寻找
        if (direction == 0) {
            for (int iii = fenzuaverage.size() - 1; iii >= 0; iii--) {
                if (fenzuaverage[iii] > 0) {
                    crosspoint = 5 * iii;
                    break;
                }
            }
            float maxVar = deltatemp[crosspoint];
            for (int m = -5; m < 5; m++) {
                int tempint = crosspoint + m;
                if (crosspoint == 0)
                    tempint = 0;
                if (deltatemp[tempint] >= maxVar) {
                    maxVar = deltatemp[tempint];
                    crosspoint1 = tempint;
                }
            }
        }
        // 从黑到白寻找
        else if (direction == 1) {
            for (size_t iii = 0; iii < fenzuaverage.size(); iii++) {
                if (fenzuaverage[iii] < 0) {
                    crosspoint = 5 * iii;
                    break;
                }
            }
            float minVar = deltatemp[crosspoint];
            for (int m = -5; m < 5; m++) {
                int tempint = crosspoint + m;
                if (crosspoint == 0)
                    tempint = 0;
                if (deltatemp[tempint] <= minVar) {
                    minVar = deltatemp[tempint];
                    crosspoint1 = tempint;
                }
            }
        }

        // 阈值判别
        if (fenzuaverage[crosspoint / 5] * 5 >= threshold_delta ||
            fenzuaverage[crosspoint / 5] * 5 <= -threshold_delta) {
            return ijRecord[crosspoint1];
        } else {
            return cv::Point2f(0, 0);
        }
    } catch (...) {
        return cv::Point2f(0, 0);
    }
}

double EdgeWidthMeasure::_getDistP2L(const cv::Point2f pointP,
                                    const cv::Point2f pointA,
                                    const cv::Point2f pointB) {
    double A = pointA.y - pointB.y;
    double B = pointB.x - pointA.x;
    double C = pointA.x * pointB.y - pointA.y * pointB.x;
    return std::abs(A * pointP.x + B * pointP.y + C) / std::sqrt(A * A + B * B);
}

double EdgeWidthMeasure::_average(const std::vector<double>& arrays, int n) {
    if (n <= 2) return 0.0;
    double max = arrays[0], min = arrays[0], sum = 0;
    for (int i = 0; i < n; i++) {
        sum += arrays[i];
        max = std::max(max, arrays[i]);
        min = std::min(min, arrays[i]);
    }
    return (sum - max - min) / (n - 2);
}

cv::Point2d EdgeWidthMeasure::_affineTransformPoint(const cv::Point2d match_origin_point,
                                                 double match_origin_angle,
                                                 const cv::Point2d match_current_point,
                                                 double match_current_angle,
                                                 const cv::Point2d input_point) {
    try {
        // 获取旋转矩阵
        cv::Mat rotMat = cv::getRotationMatrix2D(match_origin_point,
                                               (match_current_angle - match_origin_angle) * 180 / M_PI,
                                               1.0);
        
        // 转换成3*3矩阵
        cv::Mat rotMat2 = cv::Mat::zeros(3, 3, CV_32FC1);
        rotMat2.at<float>(0, 0) = static_cast<float>(rotMat.at<double>(0, 0));
        rotMat2.at<float>(0, 1) = static_cast<float>(rotMat.at<double>(0, 1));
        rotMat2.at<float>(0, 2) = static_cast<float>(rotMat.at<double>(0, 2));
        rotMat2.at<float>(1, 0) = static_cast<float>(rotMat.at<double>(1, 0));
        rotMat2.at<float>(1, 1) = static_cast<float>(rotMat.at<double>(1, 1));
        rotMat2.at<float>(1, 2) = static_cast<float>(rotMat.at<double>(1, 2));
        rotMat2.at<float>(2, 2) = 1;
        
        // 平移矩阵
        cv::Mat t_mat = cv::Mat::zeros(3, 3, CV_32FC1);
        t_mat.at<float>(0, 0) = 1;
        t_mat.at<float>(0, 2) = static_cast<float>(match_current_point.x - match_origin_point.x);
        t_mat.at<float>(1, 1) = 1;
        t_mat.at<float>(1, 2) = static_cast<float>(match_current_point.y - match_origin_point.y);
        t_mat.at<float>(2, 2) = 1;
        
        // 平移矩阵*旋转矩阵，得到仿射变换阵
        cv::Mat resMat = t_mat * rotMat2;
        
        double a11 = resMat.at<float>(0, 0);
        double a12 = resMat.at<float>(0, 1);
        double a13 = resMat.at<float>(0, 2);
        double a21 = resMat.at<float>(1, 0);
        double a22 = resMat.at<float>(1, 1);
        double a23 = resMat.at<float>(1, 2);
        
        return cv::Point2d(
            input_point.x * a11 + input_point.y * a12 + a13,
            input_point.x * a21 + input_point.y * a22 + a23
        );
    }
    catch (...) {
        return cv::Point2d(0, 0);
    }
} 