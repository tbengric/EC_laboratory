#include <vector>
#include <algorithm>
#include <limits>
#include <tuple>
#include <random>
#include "node.h"
#include "distance_matrix.h"
#include <stdexcept> 

using namespace std;

// Function to calculate candidate moves
vector<vector<int>> candidate_moves(
    const vector<Node>& nodes, 
    const vector<vector<int>>& dist_matrix,
    int K = 10
) {
    int n = nodes.size();
    int neighbors_size = n - 1;
    vector<vector<int>> candidate_neighbors(n);

    for (int i = 0; i < n; ++i) {
        vector<pair<int,int>> neighbor_costs;
        neighbor_costs.reserve(neighbors_size);

        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            int cost = dist_matrix[i][j] + nodes[j].cost;
            neighbor_costs.emplace_back(cost, j);
        }

        sort(neighbor_costs.begin(), neighbor_costs.end());

        int actual_k = min(K, neighbors_size);
        candidate_neighbors[i].reserve(actual_k);

        for (int k = 0; k < actual_k; ++k)
            candidate_neighbors[i].push_back(neighbor_costs[k].second);
    }

    return candidate_neighbors;
}

// Function to calculate candidate moves
vector<int> steepest_local_search_candidates(
    const vector<int>& initial_path,
    const vector<vector<int>>& dist_matrix,
    const vector<Node>& nodes,
    int K = 10
) {
    // --- 1. Check that path is cyclic ---
    if (initial_path.empty() || initial_path.front() != initial_path.back()) {
        throw std::runtime_error("Error: The first and last nodes of the path must be the same (cyclic path required).");
    }

    vector<int> current_path = initial_path;
    int current_path_size = current_path.size();
    if (current_path_size < 2) return current_path;

    int n = nodes.size();
    const auto& D = dist_matrix;
    const auto& N = nodes;

    // Precompute candidate neighbors for each node
    vector<vector<int>> candidate_neighbors = candidate_moves(nodes, dist_matrix, K);

    vector<int> pos(n, -1); // position of each node in current_path

    while (true) {
        double best_delta = 0.0;
        int best_move_type = -1; // 1: intra-edge, 2: inter-node
        int best_first_node = -1, best_second_node = -1, best_not_selected_node = -1;

        // Build pos array
        fill(pos.begin(), pos.end(), -1);
        for (int i = 0; i < current_path_size; ++i)
            pos[current_path[i]] = i;

        // === 1. Intra-route "edge" moves (2-opt) ===
        for (int idx = 1; idx < current_path_size - 1; ++idx) { // start from 1 to keep first node fixed
            int node_i = current_path[idx];
            const auto& neighs = candidate_neighbors[node_i];

            for (int neigh : neighs) {
                int jdx = pos[neigh];
                if (jdx == -1 || idx == jdx || std::abs(idx - jdx) <= 1) continue;
                if (jdx == 0 || jdx == current_path_size - 1) continue; // never move first/last node

                double delta = delta_intra_edge(current_path, idx, jdx, D);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 1;
                    best_first_node = idx;
                    best_second_node = jdx;
                }
            }
        }

        // === 2. Inter-route moves (swap selected with not selected) ===
        for (int idx = 1; idx < current_path_size - 1; ++idx) { // skip first/last node
            int node_i = current_path[idx];
            const auto& neighs = candidate_neighbors[node_i];

            for (int neigh : neighs) {
                if (pos[neigh] != -1) continue; // already in path

                // Only consider moves that introduce a candidate edge (node_i -> neigh)
                double delta = delta_inter_node(current_path, idx, neigh, D, N);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 2;
                    best_first_node = idx;
                    best_not_selected_node = neigh;
                }
            }
        }

        // === 3. Apply best move ===
        if (best_move_type != -1 && best_delta < 0.0) {
            if (best_move_type == 1) { // intra-edge
                int a = min(best_first_node, best_second_node);
                int b = max(best_first_node, best_second_node);
                reverse(current_path.begin() + a + 1, current_path.begin() + b + 1);
                for (int t = a + 1; t <= b; ++t)
                    pos[current_path[t]] = t;
            } else if (best_move_type == 2) { // inter-node
                int i = best_first_node;
                int oldnode = current_path[i];
                current_path[i] = best_not_selected_node;
                pos[oldnode] = -1;
                pos[best_not_selected_node] = i;
            }

            // Ensure the cycle is maintained
            // current_path.back() = current_path.front();
        } else {
            break; // no improving move found
        }
    }
    if (initial_path.empty() || initial_path.front() != initial_path.back()) {
        throw std::runtime_error("The first node is not the same as last");
    }

    return current_path;
}
