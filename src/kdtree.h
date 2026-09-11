#ifndef OBSTACLE_KDTREE_H
#define OBSTACLE_KDTREE_H
#include <algorithm>
#include <array>
#include <numeric>
#include <vector>

// Median splits keep the tree balanced even for scan-ordered lidar points.
class KdTree3D {
    struct Node { int id, left, right, axis; };
    std::vector<std::array<float, 3>> points_;
    std::vector<Node> nodes_;
    int root_;
    int build(std::vector<int>& ids, int first, int last, int depth) {
        if (first == last) return -1;
        const int axis = depth % 3, middle = first + (last-first)/2;
        std::nth_element(ids.begin()+first, ids.begin()+middle, ids.begin()+last,
            [&](int a, int b) { return points_[a][axis] < points_[b][axis]; });
        const int node = static_cast<int>(nodes_.size());
        nodes_.push_back({ids[middle], -1, -1, axis});
        const int left = build(ids, first, middle, depth+1);
        const int right = build(ids, middle+1, last, depth+1);
        nodes_[node].left = left; nodes_[node].right = right;
        return node;
    }
    void searchNode(int index, const std::array<float,3>& target, float radius,
                    float squaredRadius, std::vector<int>& result) const {
        if (index < 0) return;
        const Node& node = nodes_[index];
        const auto& point = points_[node.id];
        const float dx=point[0]-target[0], dy=point[1]-target[1], dz=point[2]-target[2];
        if (dx*dx+dy*dy+dz*dz <= squaredRadius) result.push_back(node.id);
        const float delta = target[node.axis]-point[node.axis];
        if (delta <= radius) searchNode(node.left,target,radius,squaredRadius,result);
        if (delta >= -radius) searchNode(node.right,target,radius,squaredRadius,result);
    }
public:
    explicit KdTree3D(const std::vector<std::array<float,3>>& points) : points_(points) {
        std::vector<int> ids(points.size());
        std::iota(ids.begin(),ids.end(),0);
        nodes_.reserve(points.size());
        root_ = build(ids,0,static_cast<int>(ids.size()),0);
    }
    std::vector<int> search(const std::array<float,3>& target,float radius) const {
        std::vector<int> result;
        if (radius >= 0) searchNode(root_,target,radius,radius*radius,result);
        return result;
    }
};
#endif
