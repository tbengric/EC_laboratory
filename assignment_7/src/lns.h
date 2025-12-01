#pragma once
#include <vector>
#include <set>
#include <algorithm>
#include <numeric>
#include <random>
#include <chrono>
#include <iostream>
#include <limits>
#include <cmath>

#include "node.h"
#include "distance_matrix.h"
#include "timer.h"
#include "heuristics.h"

using namespace std;

// Remove duplicate closing element -> create linear path
inline vector<int> normalize_cycle(const vector<int>& path) {
    if (path.empty()) return path;
    vector<int> p = path;
    if (p.size() > 1 && p.front() == p.back()) p.pop_back();
    return p;
}

// Close cycle 
inline vector<int> close_cycle(const vector<int>& path) {
    vector<int> p = path;
    if (!p.empty() && p.front() != p.back()) p.push_back(p.front());
    return p;
}

const double EPSILON = 1e-5;

// Swap two nodes in a linear path
double delta_intra_node(
    const vector<int>& path,
    size_t first_idx,
    size_t second_idx,
    const vector<vector<int>>& dist_matrix
) {
    size_t n = path.size();
    if (n < 2 || first_idx == second_idx) return 0.0;

    int first = path[first_idx];
    int second = path[second_idx];

    // Determine neighbors for first node
    int prev_first = (first_idx > 0) ? path[first_idx - 1] : -1;
    int next_first = (first_idx + 1 < n) ? path[first_idx + 1] : -1;

    // Determine neighbors for second node
    int prev_second = (second_idx > 0) ? path[second_idx - 1] : -1;
    int next_second = (second_idx + 1 < n) ? path[second_idx + 1] : -1;

    double old_cost = 0.0, new_cost = 0.0;

    // Remove old edges
    if (prev_first != -1) old_cost += dist_matrix[prev_first][first];
    if (next_first != -1 && next_first != second) old_cost += dist_matrix[first][next_first];

    if (prev_second != -1 && prev_second != first) old_cost += dist_matrix[prev_second][second];
    if (next_second != -1) old_cost += dist_matrix[second][next_second];

    // Add new edges after swap
    if (prev_first != -1) new_cost += dist_matrix[prev_first][second];
    if (next_first != -1 && next_first != second) new_cost += dist_matrix[second][next_first];

    if (prev_second != -1 && prev_second != first) new_cost += dist_matrix[prev_second][first];
    if (next_second != -1) new_cost += dist_matrix[first][next_second];

    return new_cost - old_cost;
}

// 2-opt (reverse segment) for linear path
double delta_intra_edge(
    const vector<int>& path,
    size_t first_edge_idx,
    size_t second_edge_idx,
    const vector<vector<int>>& dist_matrix
) {
    size_t n = path.size();
    if (n < 4 || first_edge_idx >= second_edge_idx) return 0.0;

    int a = path[first_edge_idx];
    int b = path[first_edge_idx + 1];
    int c = path[second_edge_idx];
    int d = (second_edge_idx + 1 < n) ? path[second_edge_idx + 1] : -1;

    if (b == c) return 0.0; // adjacent edges, skip

    double old_cost = dist_matrix[a][b];
    if (d != -1) old_cost += dist_matrix[c][d];

    double new_cost = dist_matrix[a][c];
    if (d != -1) new_cost += dist_matrix[b][d];

    return new_cost - old_cost;
}

// Swap a node in path with a node outside the solution
double delta_inter_node(
    const vector<int>& path,
    size_t selected_idx,
    int new_node,
    const vector<vector<int>>& dist_matrix,
    const vector<Node>& nodes
) {
    size_t n = path.size();
    int old_node = path[selected_idx];

    int prev_node = (selected_idx > 0) ? path[selected_idx - 1] : -1;
    int next_node = (selected_idx + 1 < n) ? path[selected_idx + 1] : -1;

    double old_cost = nodes[old_node].cost;
    if (prev_node != -1) old_cost += dist_matrix[prev_node][old_node];
    if (next_node != -1) old_cost += dist_matrix[old_node][next_node];

    double new_cost = nodes[new_node].cost;
    if (prev_node != -1) new_cost += dist_matrix[prev_node][new_node];
    if (next_node != -1) new_cost += dist_matrix[new_node][next_node];

    return new_cost - old_cost;
}

// -------------------- Steepest Local Search (Linear Path) --------------------

vector<int> steepest_local_search_linear(
    const vector<int>& initial_path,
    const vector<vector<int>>& dist_matrix,
    const string& intra_method,
    const vector<Node>& nodes
) {
    vector<int> path = normalize_cycle(initial_path);
    size_t n = path.size();
    if (n < 2) return path;

    // Nodes not in solution
    set<int> selected(path.begin(), path.end());
    vector<int> not_in_solution;
    for (const auto& node : nodes) {
        if (selected.find(node.id) == selected.end()) not_in_solution.push_back(node.id);
    }

    while (true) {
        double best_delta = EPSILON;
        int best_move_type = -1; // 0=node,1=edge,2=inter
        size_t idx1 = 0, idx2 = 0;
        int inter_node_id = -1;

        // Intra moves
        if (intra_method == "node") {
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = i + 1; j < n; ++j) {
                    double delta = delta_intra_node(path, i, j, dist_matrix);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 0;
                        idx1 = i;
                        idx2 = j;
                    }
                }
            }
        } else if (intra_method == "edge") {
            for (size_t i = 0; i + 1 < n; ++i) {
                for (size_t j = i + 2; j < n; ++j) {
                    double delta = delta_intra_edge(path, i, j, dist_matrix);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 1;
                        idx1 = i;
                        idx2 = j;
                    }
                }
            }
        }

        // Inter-node moves
        for (size_t i = 0; i < n; ++i) {
            for (int node_id : not_in_solution) {
                double delta = delta_inter_node(path, i, node_id, dist_matrix, nodes);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 2;
                    idx1 = i;
                    inter_node_id = node_id;
                }
            }
        }

        if (best_move_type == -1 || best_delta >= -EPSILON) break;

        // Apply move
        if (best_move_type == 0) {
            swap(path[idx1], path[idx2]);
        } else if (best_move_type == 1) {
            reverse(path.begin() + idx1 + 1, path.begin() + idx2 + 1);
        } else if (best_move_type == 2) {
            int old_node = path[idx1];
            path[idx1] = inter_node_id;
            auto it = find(not_in_solution.begin(), not_in_solution.end(), inter_node_id);
            if (it != not_in_solution.end()) *it = old_node;
        }
    }

    return close_cycle(path);
}

// NEW FUNCTIONS
struct Item { 
    int idx;
    double weight; 
};

vector<int> destroy_solution(
    const vector<int>& solution,
    const vector<vector<int>>& dist_matrix,
    const vector<Node>& nodes,
    mt19937& rng,
    double fraction = 0.30 // zawsze 30%
) {
    vector<int> current = normalize_cycle(solution);
    int n = current.size();
    if (n <= 3) return current;

    // ---- Build weighted items (nodes + edges) ----
    
    vector<Item> items;
    items.reserve(n);

    for (int i = 0; i < n; i++) {
        int u = current[i];
        int v = current[(i + 1) % n];

        double edge_cost = dist_matrix[u][v];
        double node_cost = nodes[u].cost;

        items.push_back({i, node_cost + edge_cost}); // node weight
    }

    // ---- Target removal count = 30% ----
    int target = max(1, (int)round(n * fraction));

    set<int> removed;

    uniform_int_distribution<int> seg_len_dist(2, 6);
    uniform_int_distribution<int> mode_dist(1, 2);
    uniform_int_distribution<int> offset_dist(0, n/2);

    // ---- Remove until exactly ~30% removed ----
    while ((int)removed.size() < target) {

        // --- Weighted roulette-wheel selection ---
        double total_weight = 0.0;
        for (auto& it : items)
            total_weight += it.weight;

        double r = std::uniform_real_distribution<>(0.0, total_weight)(rng);
        double cum = 0.0;
        int start_idx = 0;

        for (auto& it : items) {
            cum += it.weight;
            if (cum >= r && removed.count(it.idx) == 0) {
                start_idx = it.idx;
                break;
            }
        }


        // --- Choose a destruction mode ---
        int mode = mode_dist(rng);

        if (mode == 1) {
            // 1) SINGLE SUBPATH
            int len = seg_len_dist(rng);
            for (int i = 0; i < len && removed.size() < target; i++)
                removed.insert((start_idx + i) % n);

        } else if (mode == 2) {
            // 2) MULTIPLE SUBPATHS
            int num_sp = uniform_int_distribution<>(2, 3)(rng);
            for (int s = 0; s < num_sp && removed.size() < target; s++) {
                int len = seg_len_dist(rng);
                int off = offset_dist(rng);
                for (int j = 0; j < len && removed.size() < target; j++)
                    removed.insert((start_idx + off + j) % n);
            }
        }
    }

    // ---- Build resulting solution ----
    std::vector<int> new_sol;
    new_sol.reserve(n);
    for (int i = 0; i < n; i++)
        if (!removed.count(i))
            new_sol.push_back(current[i]);

    return close_cycle(new_sol);
}


vector<int> repair_solution(
    const vector<int>& partial_solution,
    const vector<vector<int>>& dist,
    const vector<Node>& nodes,
    int kRegret = 2,
    double weightRegret = 0.5
) {
    double weightObjective = 1.0 - weightRegret;
    int total_nodes = nodes.size();
    int numToSelect = (int)ceil(total_nodes / 2.0);

    vector<int> path;
    vector<bool> visited(total_nodes, false);

    // Initialize path with partial_solution
    if (!partial_solution.empty()) {
        path = partial_solution;
        for (int node : path) visited[node] = true;
    }

    // Iteratively insert remaining nodes based on k-regret
    while ((int)path.size() < numToSelect + 1) {
        int bestNode = -1;
        int bestPos = -1;
        double bestWeightedScore = -numeric_limits<double>::infinity();

        for (const Node& node : nodes) {
            if (visited[node.id]) continue;

            vector<pair<int,int>> insertionCosts;

            for (size_t i = 0; i < path.size() - 1; ++i) { // position between u->v
                int u = path[i];
                int v = path[i + 1];
                int cost = dist[u][node.id] + dist[node.id][v] - dist[u][v] + node.cost;
                insertionCosts.push_back({cost, static_cast<int>(i + 1)});
            }

            sort(insertionCosts.begin(), insertionCosts.end(), [](auto &a, auto &b){
                return a.first < b.first;
            });

            // k-regret computation
            double regret = 0.0;
            for (int j = 1; j < min(kRegret, (int)insertionCosts.size()); ++j) {
                regret += insertionCosts[j].first - insertionCosts[0].first;
            }

            double weightedScore = weightRegret * regret - weightObjective * insertionCosts[0].first;

            if (weightedScore > bestWeightedScore) {
                bestWeightedScore = weightedScore;
                bestNode = node.id;
                bestPos = insertionCosts[0].second;
            }
        }

        if (bestNode == -1) break;

        path.insert(path.begin() + bestPos, bestNode);
        visited[bestNode] = true;
    }

    return path;
}

vector<int> large_neighborhood_search(
    const vector<int>& initial,
    const vector<vector<int>>& dist,
    const vector<Node>& nodes,
    double remove_fraction,
    double avg_msls_time_ms,
    Timer& timer,
    bool useLocalSearchAfterRepair,
    int& iterations_out
) {
    vector<int> current_solution = initial;

    current_solution = steepest_local_search_linear(current_solution, dist, "edge", nodes);
    iterations_out = 0;

    vector<int> best_solution = current_solution;
    double best_score = computeObjective(best_solution, dist, nodes);
    double current_score = best_score;

    mt19937 rng(random_device{}());

    while (timer.toc_ms() < avg_msls_time_ms) {
        iterations_out++;

        // Destroy
        vector<int> partial_solution = destroy_solution(current_solution, dist, nodes, rng);

        // Repair
        vector<int> repaired_solution = repair_solution(partial_solution, dist, nodes);

        // Optional Local Search
        if (useLocalSearchAfterRepair) {
            repaired_solution = steepest_local_search_linear(repaired_solution, dist, "edge", nodes);
        }

        double repaired_score = computeObjective(repaired_solution, dist, nodes);

        // Check locally
        if (repaired_score < current_score) {
            current_solution = repaired_solution;
            current_score = repaired_score;

            //check in total
            if (current_score < best_score) {
                best_solution = current_solution;
                best_score = current_score;
            }
        }
    }

    return best_solution;
}
