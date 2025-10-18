#pragma once

#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
#include <random>
#include "node.h"

using namespace std;

// Node Selection
vector<int> selectNodes(int totalNodes) {
    int k = (totalNodes + 1) / 2;
    vector<int> indices(totalNodes);
    iota(indices.begin(), indices.end(), 0);

    shuffle(indices.begin(), indices.end(), mt19937{random_device{}()});
    indices.resize(k);
    return indices;
}

// Objective Function
// Objective Function
int computeObjective(const vector<int>& path,
                     const vector<vector<int>>& dist,
                     const vector<Node>& nodes) {
    int totalDist = 0;
    int totalCost = 0;

    for (size_t i = 0; i < path.size()-1; i++) {
        totalCost += nodes[path[i]].cost;
        totalDist += dist[path[i]][path[(i + 1)]];
    }

    return totalDist + totalCost;
}


// Random Solution
vector<int> randomSolution(const vector<int>& selectedNodes) {
    vector<int> path = selectedNodes;
    shuffle(path.begin(), path.end(), mt19937{random_device{}()});
    path.push_back(path[0]);
    return path;
}

// Nearest Neighbor Heuristics (To the End)
vector<int> nearestNeighborEnd(const vector<vector<int>>& dist,
                               const vector<Node>& nodes,
                               int startNodeId) {
    vector<int> path = { startNodeId };
    int maxSize = nodes.size() / 2;
    vector<bool> visited(nodes.size(), false);
    visited[startNodeId] = true;

    while (path.size() < static_cast<size_t>(maxSize)) {
        int bestNode = -1;
        int bestScore = numeric_limits<int>::max();

        for (const Node& node : nodes) {
            if (visited[node.id]) continue;

            int score = dist[path.back()][node.id] + node.cost;

            if (score < bestScore) {
                bestScore = score;
                bestNode = node.id;
            }
        }

        path.push_back(bestNode);
        visited[bestNode] = true;
    }
    path.push_back(startNodeId);
    return path;
}

// Nearest Neighbor Heuristics (At any place)
vector<int> nearestNeighborFlexible(const vector<vector<int>>& dist,
                                    const vector<Node>& nodes,
                                    int startNodeId) {
    vector<int> path = { startNodeId };
    int maxSize = nodes.size() / 2;
    vector<bool> visited(nodes.size(), false);
    visited[startNodeId] = true;

    while (path.size() < static_cast<size_t>(maxSize)) {
        int bestNode = -1;
        int bestPos = -1;
        int bestScore = numeric_limits<int>::max();

        for (const Node& node : nodes) {
            if (visited[node.id]) continue;

            for (size_t i = 0; i <= path.size(); ++i) {
                vector<int> tempPath = path;
                tempPath.insert(tempPath.begin() + i, node.id);
                int score = computeObjective(tempPath, dist, nodes);

                if (score < bestScore) {
                    bestScore = score;
                    bestNode = node.id;
                    bestPos = i;
                }
            }
        }

        path.insert(path.begin() + bestPos, bestNode);
        visited[bestNode] = true;
    }
    path.push_back(path[0]);
    return path;
}

// Greedy Cycle Heuristic
vector<int> greedyCycle(const vector<vector<int>>& dist,
                        const vector<Node>& nodes,
                        int startNodeId) {
    vector<int> path = { startNodeId };
    int numToSelect = nodes.size() / 2;
    vector<bool> visited(nodes.size(), false);
    visited[startNodeId] = true;

    //select the best second node to form initial 2-node cycle
    int bestSecondNode = -1;
    int bestInitialScore = numeric_limits<int>::max();
    for (const Node& node : nodes) {
        if (visited[node.id]) continue;
        int score = dist[startNodeId][node.id] + nodes[startNodeId].cost + node.cost;
        if (score < bestInitialScore) {
            bestInitialScore = score;
            bestSecondNode = node.id;
        }
    }

    path.push_back(bestSecondNode);
    visited[bestSecondNode] = true;
    path.push_back(startNodeId);


    //iteratively insert remaining nodes
    while (path.size() < static_cast<size_t>(numToSelect+1)) {
        int bestNode = -1;
        int bestPos = -1;
        int bestScore = numeric_limits<int>::max();

        for (const Node& node : nodes) {
            if (visited[node.id]) continue;

            for (size_t i = 0; i < path.size() -1; ++i) {
                vector<int> tempPath = path;
                tempPath.insert(tempPath.begin() + i, node.id);
                int score = computeObjective(tempPath, dist, nodes);

                if (score < bestScore) {
                    bestScore = score;
                    bestNode = node.id;
                    bestPos = i;
                }
            }
        }

        path.insert(path.begin() + bestPos, bestNode);
        visited[bestNode] = true;
    }

    return path;
}
