#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
typedef CGAL::Alpha_shape_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;
typedef K::Point_2 Point;


int main(int argc, char **argv) {
    // 读取PCD文件
    std::string file_name = "/home/tzh/stair_detection/horizontal_planes.pcd"; // 替换为你的文件路径
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    std::vector<Point> points;
    for (const auto &point : cloud->points) {
        points.emplace_back(point.x, point.y);
    }

    // 计算Alpha Shape
    Alpha_shape_2 A(points.begin(), points.end(), 0, Alpha_shape_2::GENERAL);
    A.set_alpha(0.005);

    // 保存边缘点到文件
    std::ofstream ofs("alpha_shape_edges.txt");
    size_t num_edges = std::distance(A.alpha_shape_edges_begin(), A.alpha_shape_edges_end());
    size_t count = 0;

    pcl::PointCloud<pcl::PointXYZ> boundary_cloud;
    for (auto it = A.alpha_shape_edges_begin(); it != A.alpha_shape_edges_end(); ++it) {
        auto segment = A.segment(*it);
        pcl::PointXYZ p1(segment.source().x(), segment.source().y(), 0.0);
        pcl::PointXYZ p2(segment.target().x(), segment.target().y(), 0.0);
        boundary_cloud.push_back(p1);
        boundary_cloud.push_back(p2);
        ofs << segment << "\n";

        // 更新进度
        count += 2;
        float progress = static_cast<float>(count) / static_cast<float>(num_edges * 2);

    }
    ofs.close();

    // 保存边界点云到PCD文件
    boundary_cloud.width = boundary_cloud.size();
    boundary_cloud.height = 1;
    boundary_cloud.is_dense = true;
    pcl::io::savePCDFileASCII("boundary_cloud.pcd", boundary_cloud);

    std::cout << "\nAlpha shape edges saved to alpha_shape_edges.txt and boundary_cloud.pcd" << std::endl;
    return 0;
}
