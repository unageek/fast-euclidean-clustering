#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fast_euclidean_clustering.h>

#include <chrono>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace chrono = std::chrono;

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = PointCloud::Ptr;
using KdTree = pcl::search::KdTree<pcl::PointXYZI>;
using KdTreePtr = KdTree::Ptr;

int
main(int argc, char** argv)
{
  try {
    std::vector<std::string> args(argv, argv + argc);
    if (args.size() != 4) {
      std::cerr << "usage: " << args.at(0)
                << " <pcd-file> <cluster-tolerance> <quality>" << std::endl;
      return 1;
    }

    fs::path pcd_path(args.at(1));
    const auto cluster_tolerance = std::strtod(args.at(2).c_str(), nullptr);
    const auto quality = std::strtod(args.at(3).c_str(), nullptr);

    PointCloudPtr cloud(new PointCloud);
    pcl::io::loadPCDFile(pcd_path.string(), *cloud);

    KdTreePtr tree(new KdTree);
    std::vector<pcl::PointIndices> clusters;

    FastEuclideanClustering<pcl::PointXYZI> fec;
    fec.setInputCloud(cloud);
    fec.setSearchMethod(tree);
    fec.setClusterTolerance(cluster_tolerance);
    fec.setQuality(quality);

    auto t1 = chrono::high_resolution_clock::now();
    fec.segment(clusters);
    auto t2 = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = t2 - t1;

    std::cout << "#clusters: " << clusters.size() << std::endl;
    std::cout << "Elapsed: " << elapsed.count() << "s" << std::endl;

    auto label = 0;
    for (const auto& cluster : clusters) {
      for (auto index : cluster.indices) {
        auto& point = cloud->at(index);
        point.intensity = static_cast<float>(label);
      }
      label++;
    }

    pcl::io::savePCDFileBinary(
        fs::path(pcd_path)
            .replace_filename(pcd_path.stem().string() + "_clustered.pcd")
            .string(),
        *cloud);

    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (...) {
    return 1;
  }
}
