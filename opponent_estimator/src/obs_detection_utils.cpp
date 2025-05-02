#include "opponent_estimator/opponent_estimator.hpp"

std::vector<std::vector<int>> OpponentEstimator::cluster(const std::vector<std::vector<double>> &points, double tol) {
    int n = (int) points.size();
    std::vector<int> parents(n);
    for (int i = 0; i < n; ++i) {
        parents[i] = i;
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < i; ++j) {
            double dx = points[i][0] - points[j][0];
            double dy = points[i][1] - points[j][1];
            double dist = sqrt(dx * dx + dy * dy);
            if (dist > tol) {
                continue;
            }
            connect(parents, i, j);
        }
    }

    std::map<int, std::vector<int>> clusters;
    for (int i = 0; i < n; ++i) {
        int root = parents[i];
        if (clusters.find(root) != clusters.end()) {
            clusters[root].push_back(i);
        } else {
            clusters[root] = {i};
        }
    }

    std::vector<std::vector<int>> res;
    for (const auto& cluster: clusters) {
        res.push_back(cluster.second);
    }

    return res;
}

void OpponentEstimator::connect(std::vector<int>& parents, int i, int j) {
    int root_i = find(parents, i);
    int root_j = find(parents, j);

    if (root_i != root_j) {
        parents[root_i] = root_j;
    }
}

int OpponentEstimator::find(std::vector<int>& parents, int i) {
    if (i == parents[i]) {
        return i;
    }
    return parents[i] = find(parents, parents[i]);
}
