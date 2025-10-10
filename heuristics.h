#pragma once
#include <vector>
#include <algorithm>
#include <limits>
#include "node.h"
#include <random>

using namespace std;

vector<int> selectNodes(int totalNodes) {
    int k = (totalNodes + 1) / 2;
    vector<int> indices(totalNodes);
    std::iota(indices.begin(), indices.end(), 0);
    shuffle(indices.begin(), indices.end(), std::mt19937{random_device{}()});
    indices.resize(k);
    return indices;
}

int computeObjective(const vector<int>& path,
                     const vector<vector<int>>& dist,
                     const vector<Node>& nodes) {
    int totalDist = 0, totalCost = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        totalCost += nodes[path[i]].cost;
        totalDist += dist[path[i]][path[(i + 1) % path.size()]];
    }
    return totalDist + totalCost;
}

vector<int> randomSolution(const vector<int>& selectedNodes) {
    vector<int> path = selectedNodes;
    shuffle(path.begin(), path.end(), std::mt19937{random_device{}()});
    return path;
}

vector<int> nearestNeighborEnd(const vector<int>& selectedNodes,
                                    const vector<vector<int>>& dist,
                                    const vector<Node>& nodes,
                                    int start) {
    vector<int> path = {start};
    vector<bool> visited(nodes.size(), false);
    visited[start] = true;

    while (path.size() < selectedNodes.size()) {
        int bestNode = -1;
        int bestScore = numeric_limits<int>::max();
        for (int node : selectedNodes) {
            if (visited[node]) continue;
            int score = dist[path.back()][node] + nodes[node].cost;
            if (score < bestScore) {
                bestScore = score;
                bestNode = node;
            }
        }
        path.push_back(bestNode);
        visited[bestNode] = true;
    }
    return path;
}

vector<int> nearestNeighborFlexible(const vector<int>& selectedNodes,
                                         const vector<vector<int>>& dist,
                                         const vector<Node>& nodes,
                                         int start) {
    vector<int> path = {start};
    vector<bool> visited(nodes.size(), false);
    visited[start] = true;

    while (path.size() < selectedNodes.size()) {
        int bestNode = -1, bestPos = -1;
        int bestScore = numeric_limits<int>::max();
        for (int node : selectedNodes) {
            if (visited[node]) continue;
            for (size_t i = 0; i <= path.size(); ++i) {
                vector<int> tempPath = path;
                tempPath.insert(tempPath.begin() + i, node);
                int score = computeObjective(tempPath, dist, nodes);
                if (score < bestScore) {
                    bestScore = score;
                    bestNode = node;
                    bestPos = i;
                }
            }
        }
        path.insert(path.begin() + bestPos, bestNode);
        visited[bestNode] = true;
    }
    return path;
}

vector<int> greedyCycle(const vector<int>& selectedNodes,
                             const vector<vector<int>>& dist,
                             const vector<Node>& nodes,
                             int start) {
    vector<int> path = {start};
    vector<bool> visited(nodes.size(), false);
    visited[start] = true;

    while (path.size() < selectedNodes.size()) {
        int bestNode = -1, bestPos = -1;
        int bestScore = numeric_limits<int>::max();
        for (int node : selectedNodes) {
            if (visited[node]) continue;
            for (size_t i = 0; i < path.size(); ++i) {
                int prev = path[i];
                int next = path[(i + 1) % path.size()];
                int score = dist[prev][node] + dist[node][next] - dist[prev][next] + nodes[node].cost;
                if (score < bestScore) {
                    bestScore = score;
                    bestNode = node;
                    bestPos = i + 1;
                }
            }
        }
        path.insert(path.begin() + bestPos, bestNode);
        visited[bestNode] = true;
    }
    return path;
}
