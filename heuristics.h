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

    mt19937 rng(42); // fixed seed
    shuffle(indices.begin(), indices.end(), rng);
    indices.resize(k);
    return indices;
}

// Objective Function
int computeObjective(const vector<int>& path,
                     const vector<vector<int>>& dist,
                     const vector<Node>& nodes) {
    int totalDist = 0;
    int totalCost = 0;

    for (size_t i = 0; i < path.size(); ++i) {
        totalCost += nodes[path[i]].cost;
        totalDist += dist[path[i]][path[(i + 1) % path.size()]];
    }

    return totalDist + totalCost;
}

// Random Solution
vector<int> randomSolution(const vector<int>& selectedNodes) {
    vector<int> path = selectedNodes;
    shuffle(path.begin(), path.end(), mt19937{random_device{}()});
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

            // Add return-to-start cost near the end
            if (path.size() == static_cast<size_t>(maxSize) - 2) {
                score += dist[node.id][startNodeId];
            }

            if (score < bestScore) {
                bestScore = score;
                bestNode = node.id;
            }
        }

        path.push_back(bestNode);
        visited[bestNode] = true;
    }

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
        int score = 2 * dist[startNodeId][node.id] + nodes[startNodeId].cost + node.cost;
        if (score < bestInitialScore) {
            bestInitialScore = score;
            bestSecondNode = node.id;
        }
    }

    path.push_back(bestSecondNode);
    visited[bestSecondNode] = true;

    //iteratively insert remaining nodes
    while (path.size() < static_cast<size_t>(numToSelect)) {
        int bestNode = -1;
        int bestPos = -1;
        int bestScore = numeric_limits<int>::max();

        for (const Node& node : nodes) {
            if (visited[node.id]) continue;

            for (size_t i = 0; i < path.size(); ++i) {
                int prev = path[i];
                int next = path[(i + 1) % path.size()];
                int score = dist[prev][node.id] + dist[node.id][next] - dist[prev][next] + node.cost;

                if (score < bestScore) {
                    bestScore = score;
                    bestNode = node.id;
                    bestPos = i + 1;
                }
            }
        }

        path.insert(path.begin() + bestPos, bestNode);
        visited[bestNode] = true;
    }

    return path;
}
