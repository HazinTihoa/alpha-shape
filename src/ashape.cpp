#include <pcl/common/common.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <omp.h>
#include <chrono>
#include <thread>

double get2DDist(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    double dist = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    return dist;
}

void print_progress(float progress) {
    int bar_width = 70;
    std::cout << "[";
    int pos = bar_width * progress;
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

int main() {
    std::string file_name("/home/tzh/stair_detection/horizontal_planes.pcd");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud_in);

    // 投影到二维平面
    for (auto &point : *cloud_in) {
        point.z = 0.0;
    }
    pcl::io::savePCDFile<pcl::PointXYZ>("二维平面.pcd", *cloud_in);

    std::vector<int> boundary_bool(cloud_in->size(), 0);
    double r = 1.8;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_in);

    int total_points = cloud_in->size();

    #pragma omp parallel for
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        pcl::PointXYZ p = cloud_in->points[i];
        std::vector<int> indices;
        std::vector<float> dist;
        kdtree.radiusSearch(p, 2 * r, indices, dist, 100);

        for (size_t j = 1; j < indices.size(); ++j) {
            pcl::PointXYZ p1 = cloud_in->points[indices[j]];
            double s_2 = std::pow((p.x - p1.x), 2) + std::pow((p.y - p1.y), 2);
            double h = std::sqrt((r * r / s_2 - 0.25));
            double x2 = p.x + 0.5 * (p1.x - p.x) - h * (p1.y - p.y);
            double y2 = p.y + 0.5 * (p1.y - p.y) - h * (p.x - p1.x);
            double x3 = p.x + 0.5 * (p1.x - p.x) + h * (p1.y - p.y);
            double y3 = p.y + 0.5 * (p1.y - p.y) + h * (p.x - p1.x);
            pcl::PointXYZ p2(x2, y2, 0.0);
            pcl::PointXYZ p3(x3, y3, 0.0);

            int count = 0;
            int distp2_bool = 0, distp3_bool = 0;

            for (size_t k = 1; k < indices.size(); ++k) {
                if (k == j) continue;
                pcl::PointXYZ p_other = cloud_in->points[indices[k]];
                ++count;
                if (get2DDist(p_other, p2) > r) ++distp2_bool;
                if (get2DDist(p_other, p3) > r) ++distp3_bool;
            }

            if (count == distp2_bool || count == distp3_bool) {
                #pragma omp critical
                boundary_bool[i] = 1;
                break;
            }
        }

        // 更新进度
        if (i % 1000 == 0) {
            float progress = static_cast<float>(i) / total_points;
            print_progress(progress);
        }
    }

    pcl::PointCloud<pcl::PointXYZ> boundary_cloud;
    for (size_t it = 0; it < boundary_bool.size(); ++it) {
        if (boundary_bool[it] == 1) {
            boundary_cloud.push_back(cloud_in->points[it]);
        }
    }
    boundary_cloud.height = boundary_cloud.size();
    boundary_cloud.width = 1;
    boundary_cloud.resize(boundary_cloud.height * boundary_cloud.width);
    if (!boundary_cloud.empty()) {
        pcl::io::savePCDFile<pcl::PointXYZ>("边界点云.pcd", boundary_cloud);
    }

    std::cout << "\nProcessing complete!" << std::endl;
    return 0;
}
