#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "edge_width_measure.h"
#include "CaliperTool.h"

// 绘制紫色叉叉
void draw_cross(cv::Mat& img, const cv::Point2f& pt, int size = 6, cv::Scalar color = cv::Scalar(211, 0, 141), int thickness = 1) {
    cv::line(img, cv::Point2f(pt.x - size, pt.y - size), cv::Point2f(pt.x + size, pt.y + size), color, thickness);
    cv::line(img, cv::Point2f(pt.x - size, pt.y + size), cv::Point2f(pt.x + size, pt.y - size), color, thickness);
}

int main() {
    try {
        // 读取测试图像
        // cv::Mat testImage = cv::imread("../assert/pic_8.png");
        cv::Mat testImage = cv::imread("../assert/fish_sticks_raw_01.png");
        if (testImage.empty()) {
            std::cout << "错误：无法读取图像文件！" << std::endl;
            return -1;
        }
        std::cout << "成功读取图像，尺寸：" << testImage.size() << std::endl;

        // ********************* 设置卡尺参数
        // cv::Point2f p1(164.113f, 2.96746f);// 卡尺初始点
        // cv::Point2f p2(164.113f, 80.8373);// 卡尺终止点
        cv::Point2f p1(200, 261);// 卡尺初始点
        cv::Point2f p2(450, 261);// 卡尺终止点
        float height = 168.299;// 卡尺宽度
        int segment_num = 30;// 卡尺内分割数量[0-360]
        CaliperTool caliper(p1, p2, height, segment_num);
        caliper.calculate();

        // 绘制卡尺工具
        cv::Mat showImg = testImage.clone();
        if (showImg.channels() == 1)
            cv::cvtColor(showImg, showImg, cv::COLOR_GRAY2BGR);
        caliper.draw(showImg, true, true, true, 0.2f);
        // cv::imshow("showImg",showImg);
        // cv::waitKey(0);

        // ********************* 计算边缘点
        int cull_distance = 100;// 剔除距离[1-100]
        double threshold_delta1 = 50;// 边缘强度[0-255]
        int direction1 = 0;// 灰度方向: 0-白到黑 1-黑到白
        double threshold_delta2 = 50;
        int direction2 = 0;

        EdgeWidthMeasure measure;
        double distance;
        cv::Mat dstImage;
        measure.runToolPro(testImage, caliper, threshold_delta1, direction1, threshold_delta2, direction2, segment_num, cull_distance, dstImage, distance);

        // 绘制两组边缘点为紫色叉叉
        int size = 2;
        int thickness = 1;
        cv::Scalar color1 = cv::Scalar(211, 0, 141);
        cv::Scalar color2 = cv::Scalar(211, 0, 255);
        for (const auto& pt : measure.getEdgePoints1()) {
            draw_cross(showImg, pt, size, color1, thickness);
        }
        for (const auto& pt : measure.getEdgePoints2()) {
            draw_cross(showImg, pt, size, color2, thickness);
        }

        // 在左上角绘制耗时（白色背景，黑色字体）
        std::string resultText = "Distance is[image coordinate]: " + std::to_string(distance) + " pixels";
        cv::Size textSize = cv::getTextSize(resultText, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, nullptr);
        cv::rectangle(showImg, cv::Point(10, 10), cv::Point(20 + textSize.width, 40 + textSize.height),
                     cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(showImg, resultText, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 0, 0), 2);

        cv::imshow("CaliperTool", showImg);
        cv::imwrite("caliper_result.png", showImg);
        std::cout << "Distance: " << distance << std::endl;
        cv::waitKey(0);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "发生错误: " << e.what() << std::endl;
        return -1;
    }
} 