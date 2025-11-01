#pragma once
#include <vector>
#include <cmath>
#include <set>
#include <algorithm>
#include "node.h"
#include "distance_matrix.h"

using namespace std;

vector<vector<int>> build_candidate_neighbors(const vector<Node>& nodes,
                                              const vector<vector<int>>& dist_matrix,
                                              int k = 10) {
    vector<vector<int>> candidates(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        vector<pair<double, int>> distances;
        for (size_t j = 0; j < nodes.size(); ++j) {
            if (i == j) continue;
            double cost = dist_matrix[i][j] + nodes[j].cost;
            distances.emplace_back(cost, j);
        }
        sort(distances.begin(), distances.end());
        for (int n = 0; n < min(k, (int)distances.size()); ++n)
            candidates[i].push_back(distances[n].second);
    }
    return candidates;
}

vector<int> steepest_local_search_candidates(const vector<int>& initial_path,
                                             const vector<vector<int>>& dist_matrix,
                                             const string& intra_method,
                                             const vector<Node>& nodes,
                                             const vector<vector<int>>& candidate_neighbors) {
    vector<int> current_path = initial_path;

    while (true) {
        double best_delta = 0.0;
        int best_move_type = -1; // 0: intra-node, 1: intra-edge, 2: inter-node
        int best_first = -1, best_second = -1, best_not_selected = -1;

        set<int> selected_nodes(current_path.begin(), current_path.end());
        set<int> not_selected;
        for (const auto& n : nodes)
            if (!selected_nodes.count(n.id))
                not_selected.insert(n.id);

        int n = current_path.size();
        if (n < 2) break;

        // --- Fast position lookup ---
        vector<int> pos_in_path(nodes.size(), -1);
        for (int i = 0; i < n; ++i)
            pos_in_path[current_path[i]] = i;

        // --- Intra-node moves ---
        if (intra_method == "node") {
            for (int i = 0; i < n; ++i) {
                for (int neighbor : candidate_neighbors[current_path[i]]) {
                    int j_index = pos_in_path[neighbor];
                    if (j_index == -1 || j_index == i) continue;

                    double delta = delta_intra_node(current_path, i, j_index, dist_matrix);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 0;
                        best_first = i;
                        best_second = j_index;
                    }
                }
            }
        }
        // --- Intra-edge moves ---
        else if (intra_method == "edge") {
            for (int i = 0; i < n - 1; ++i) {
                for (int neighbor : candidate_neighbors[current_path[i]]) {
                    int j_index = pos_in_path[neighbor];
                    if (j_index == -1 || abs(i - j_index) < 2) continue;

                    double delta = delta_intra_edge(current_path, i, j_index, dist_matrix);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 1;
                        best_first = i;
                        best_second = j_index;
                    }
                }
            }
        }

        // --- Inter-node moves ---
        for (int i = 0; i < n; ++i) {
            for (int neighbor : candidate_neighbors[current_path[i]]) {
                if (!not_selected.count(neighbor)) continue;

                double delta = delta_inter_node(current_path, i, neighbor, dist_matrix, nodes);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 2;
                    best_first = i;
                    best_not_selected = neighbor;
                }
            }
        }

        // --- Stop if no better move found ---
        if (best_move_type == -1 || best_delta > -1e-9) break;

        // --- Apply best move ---
        if (best_move_type == 0) {
            swap(current_path[best_first], current_path[best_second]);
        } else if (best_move_type == 1) {
            if (best_first > best_second) swap(best_first, best_second);
            reverse(current_path.begin() + best_first + 1, current_path.begin() + best_second + 1);
        } else if (best_move_type == 2) {
            int old_node = current_path[best_first];
            current_path[best_first] = best_not_selected;
            not_selected.erase(best_not_selected);
            not_selected.insert(old_node);
        }
    }

    return current_path;
}

