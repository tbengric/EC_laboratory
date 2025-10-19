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

// Greedy Cycle
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
        int score = 2*dist[startNodeId][node.id] + nodes[startNodeId].cost + node.cost;
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

            for (size_t i = 1; i < path.size() ; ++i) { // a-> b -> c -> d -> a
                
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


vector<int> greedyCycleKRegretWeighted(
    const vector<vector<int>>& dist,
    const vector<Node>& nodes,
    int startNodeId,
    int kRegret = 2,
    double weight_regret = 0.5
) {
    double weight_objective = 1.0 - weight_regret;
    vector<int> path = { startNodeId };
    int numToSelect = nodes.size() / 2;
    vector<bool> visited(nodes.size(), false);
    visited[startNodeId] = true;

    // Select best second
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
    path.push_back(startNodeId); // close cycle

    // Iteratively insert remaining nodes
    while (path.size() < static_cast<size_t>(numToSelect + 1)) {
        int bestNode = -1;
        int bestPos = -1;
        double bestWeightedScore = -numeric_limits<int>::max();

        for (const Node& node : nodes) {
            if (visited[node.id]) continue;

            // List<total_cost, position_of_node>
            vector<pair<int,int>> insertionCosts;

            for (size_t i = 0; i < path.size() - 1; ++i) {
                int u = path[i];
                int v = path[i + 1];
                int cost = dist[u][node.id] + dist[node.id][v] - dist[u][v] + node.cost;
                insertionCosts.push_back({cost, static_cast<int>(i + 1)});
            }

            // Sort by cost in the list
            sort(insertionCosts.begin(), insertionCosts.end(), [](auto &a, auto &b){
                return a.first < b.first;
            });


            // k-regret 
            double regret = 0.0;
            int count = kRegret - 1;
            for (int j = 1; j <= count; ++j) {
                regret += insertionCosts[j].first - insertionCosts[0].first;
            }

            // Weighted sum
            double weightedScore = weight_regret * regret - weight_objective * insertionCosts[0].first;

            if (weightedScore > bestWeightedScore) {
                bestWeightedScore = weightedScore;
                bestNode = node.id;
                bestPos = insertionCosts[0].second; // use position from the list
            }
        }

        if (bestNode == -1){cout<<"HELLp"<<endl; break;}

        path.insert(path.begin() + bestPos, bestNode);
        visited[bestNode] = true;
    }

    return path;
}


vector<int> nearestNeighborKRegretWeighted(
    const vector<vector<int>>& dist,
    const vector<Node>& nodes,
    int startNodeId,
    int kRegret = 2,
    double weight_regret = 0.5
) {
    double weight_objective = 1.0 - weight_regret;
    vector<int> path = { startNodeId };
    int maxSize = nodes.size() / 2;
    vector<bool> visited(nodes.size(), false);
    visited[startNodeId] = true;

    // Iteratively insert remaining nodes
    while (path.size() < static_cast<size_t>(maxSize)) {
        int bestNode = -1;
        int bestPos = -1;
        double bestWeightedScore = -numeric_limits<int>::max();

        for (const Node& node : nodes) {
            if (visited[node.id]) continue;

            // List<total_cost, position_of_node>
            vector<pair<int,int>> insertionCosts;

            for (size_t i = 0; i < path.size(); ++i) {
                int u = path[i];
                int v = path[(i + 1) % path.size()]; 
                int cost = dist[u][node.id] + dist[node.id][v] - dist[u][v] + node.cost;
                insertionCosts.push_back({cost, static_cast<int>(i + 1)});
            }

            // Sort by cost in the list
            sort(insertionCosts.begin(), insertionCosts.end(), [](auto &a, auto &b){
                return a.first > b.first;
            });

            // K-regret
            double regret = 0.0;
            int count = kRegret - 1;
            for (int j = 1; j <= count; ++j) {
                regret += insertionCosts[j].first - insertionCosts[0].first;
            }

            // Weighted sum
            double weightedScore = weight_regret * regret + weight_objective * insertionCosts[0].first;

            if (weightedScore > bestWeightedScore) {
                bestWeightedScore = weightedScore;
                bestNode = node.id;
                bestPos = insertionCosts[0].second; // use position from the list
            }
        }

        if (bestNode == -1) break;

        path.insert(path.begin() + bestPos, bestNode);
        visited[bestNode] = true;
    }

    // Close cycle
    path.push_back(path[0]);
    return path;
}