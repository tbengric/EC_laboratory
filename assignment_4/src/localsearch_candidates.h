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
    vector<int> current_path = initial_path;
    int path_size = current_path.size();
    int n = nodes.size();

    vector<vector<int>> candidate_neighbors = candidate_moves(nodes, dist_matrix, K);

    vector<int> pos(n, -1); 

    while (true) {
        double best_delta = 0.0;
        int best_move_type = -1; // 1: intra-edge, 2: inter-node
        int best_first = -1, best_second = -1, best_new_node = -1;

        // Update positions of nodes in current_path
        fill(pos.begin(), pos.end(), -1);
        for (int i = 0; i < path_size; ++i)
            pos[current_path[i]] = i;

        // 1. Intra-route moves (2-opt) 
        for (int i = 0; i < path_size; ++i) {
            int node_i = current_path[i];
            for (int neigh : candidate_neighbors[node_i]) {
                int j = pos[neigh];
                if (j <= 0 || j >= path_size - 1 || std::abs(i - j) <= 1)
                    continue;

                double delta = delta_intra_edge(current_path, i, j, dist_matrix);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 1;
                    best_first = i;
                    best_second = j;
                }
            }
        }

        // 2. Inter-route moves (swap with not selected node) 
        for (int i = 0; i < path_size - 1; ++i) {
            int node_i = current_path[i];
            for (int neigh : candidate_neighbors[node_i]) {
                if (pos[neigh] != -1) 
                    continue;

                double delta = delta_inter_node(current_path, i, neigh, dist_matrix, nodes);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 2;
                    best_first = i;
                    best_new_node = neigh;
                }
            }
        }

        // 3. Apply best move 
        if (best_move_type == 1) {
            int a = min(best_first, best_second);
            int b = max(best_first, best_second);
            reverse(current_path.begin() + a + 1, current_path.begin() + b + 1);

            for (int t = a + 1; t <= b; ++t)
                pos[current_path[t]] = t;

        } else if (best_move_type == 2) {
            int old_node = current_path[best_first];
            current_path[best_first] = best_new_node;
            pos[old_node] = -1;
            pos[best_new_node] = best_first;

        } else {
            break; // no improving move found
        }
    }

    if (current_path.front() != current_path.back()) {
        throw std::runtime_error("The first node is not the same as the last node.");
    }

    return current_path;
}
