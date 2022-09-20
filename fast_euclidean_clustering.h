/*
 * Copyright (c) 2022 Masashi Mizuno
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/pcl_base.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <queue>

#include <cmath>
#include <vector>

template <typename PointT>
class FastEuclideanClustering : public pcl::PCLBase<PointT> {
  using Base = pcl::PCLBase<PointT>;
  using Base::deinitCompute;
  using Base::indices_;
  using Base::initCompute;
  using Base::input_;

public:
  using KdTree = pcl::search::Search<PointT>;
  using KdTreePtr = typename KdTree::Ptr;
  using Graph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>;

  double
  getClusterTolerance() const
  {
    return cluster_tolerance_;
  }

  void
  setClusterTolerance(double tolerance)
  {
    cluster_tolerance_ = tolerance;
  }

  double
  getQuality() const
  {
    return quality_;
  }

  void
  setQuality(double quality)
  {
    quality_ = quality;
  }

  KdTreePtr
  getSearchMethod() const
  {
    return (tree_);
  }

  void
  setSearchMethod(const KdTreePtr& tree)
  {
    tree_ = tree;
  }

  void
  segment(std::vector<pcl::PointIndices>& clusters)
  {
    initCompute();

    const pcl::index_t invalid_label = -1;
    std::vector<pcl::index_t> labels(input_->size(), invalid_label);
    std::vector<bool> removed(input_->size(), false);

    tree_->setInputCloud(input_, indices_);

    pcl::Indices nn_indices;
    std::vector<float> nn_distances;
    auto nn_distance_threshold = std::pow((1.0 - quality_) * cluster_tolerance_, 2.0);

    Graph g;
    std::queue<pcl::index_t> queue;

    {
      pcl::index_t next_p = 0;
      pcl::index_t label = 0;
      while (next_p < static_cast<pcl::index_t>(input_->size())) {
        boost::add_edge(label, label, g);
        queue.push(next_p);
        while (!queue.empty()) {
          auto p = queue.front();
          queue.pop();
          if (removed.at(p)) {
            continue;
          }

          tree_->radiusSearch(p, cluster_tolerance_, nn_indices, nn_distances);

          for (std::size_t i = 0; i < nn_indices.size(); ++i) {
            auto q = nn_indices.at(i);
            auto q_label = labels.at(q);

            if (q_label != invalid_label && q_label != label) {
              boost::add_edge(label, q_label, g);
            }

            if (removed.at(q)) {
              continue;
            }

            labels.at(q) = label;

            // Must be <= to remove self (p).
            if (nn_distances.at(i) <= nn_distance_threshold) {
              removed.at(q) = true;
            }
            else {
              queue.push(q);
            }
          }
        }

        while (next_p < static_cast<pcl::index_t>(input_->size()) &&
               removed.at(next_p)) {
          next_p++;
        }
        label++;
      }
    }

    // Merge labels.

    std::vector<pcl::index_t> label_map(boost::num_vertices(g));
    auto num_components = boost::connected_components(g, label_map.data());
    clusters.resize(num_components);

    for (auto index : *indices_) {
      auto label = labels.at(index);
      auto new_label = label_map.at(label);
      clusters.at(new_label).indices.push_back(index);
    }

    deinitCompute();
  }

private:
  double cluster_tolerance_;
  double quality_;
  KdTreePtr tree_;
};
