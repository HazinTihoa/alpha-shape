
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <iostream>

// Helper function to project 3D points to 2D
void projectTo2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<cv::Point2f>& points2D) {
    for (const auto& point : cloud->points) {
        points2D.emplace_back(point.x, point.y);
    }
}

// Helper function to draw polygons
void drawPolygon(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    if (points.size() > 1) {
        for (size_t i = 0; i < points.size(); ++i) {
            cv::line(image, points[i], points[(i + 1) % points.size()], color, 2);
        }
    }
}

int main(int argc, char** argv) {
    // 加载点云数据
    std::string file_name = "/home/tzh/stair_detection/horizontal_planes.pcd"; // 替换为你的文件路径
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    // 投影到二维平面
    std::vector<cv::Point2f> points2D;
    projectTo2D(cloud, points2D);

    // 创建一个空白图像用于显示角点
    cv::Mat image = cv::Mat::zeros(800, 800, CV_8UC3);
    for (const auto& point : points2D) {
        int x = static_cast<int>((point.x + 10) * 40); // 根据需要调整比例因子和偏移
        int y = static_cast<int>((point.y + 10) * 40);
        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
        }
    }


    cv::imshow("origin", image);
    
 
    // 转换为灰度图像
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // // 对图像进行腐蚀操作
    // cv::Mat eroded;
    // cv::erode(gray, eroded, cv::Mat(), cv::Point(-1, -1), 2); // 腐蚀次数可调

    // 对图像进行膨胀操作
    cv::Mat dilated;
    cv::dilate(gray, dilated, cv::Mat(), cv::Point(-1, -1), 2); // 膨胀次数可调


    cv::imshow("dilated", dilated);
    

    // 使用OpenCV检测角点
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(dilated, corners, 50, 0.01, 10);

    // 确保角点数据不为空
    if (corners.empty()) {
        std::cerr << "No corners detected!" << std::endl;
        return -1;
    }

    // 确保角点是CV_32F类型
    cv::Mat cornersMat(corners.size(), 1, CV_32FC2, corners.data());

    // 使用凸包算法识别外部角点
    std::vector<int> hull;
    cv::convexHull(cornersMat, hull, false, false);

    // 生成外部多边形
    std::vector<cv::Point2f> outer_polygon;
    for (const auto& index : hull) {
        outer_polygon.push_back(corners[index]);
    }

    // 在图像上绘制外部多边形
    drawPolygon(image, outer_polygon, cv::Scalar(0, 255, 0));

    // 在图像上绘制角点
    for (const auto& corner : corners) {
        cv::circle(image, corner, 5, cv::Scalar(0, 0, 255), 2);
    }

    // 显示结果
    cv::imshow("Corners and Polygons", image);
    cv::waitKey(0);

    return 0;
}

