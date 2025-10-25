#pragma once
#include <vector>
#include <cmath>
#include "node.h"
#include <random>
#include "heuristics.h"
#include "distance_matrix.h"
#include <set>
#include <tuple>

using namespace std;

double delta_intra_node(const vector<int>& path, int first_node_index, int second_node_index, const vector<vector<int>>& dist_matrix) {
    int path_size = path.size();
    if (path_size < 2) return 0.0;

    int prev_first = path[(first_node_index - 1 + path_size) % path_size];
    int first = path[first_node_index];
    int next_first = path[(first_node_index + 1) % path_size];

    int prev_second = path[(first_node_index - 1 + path_size) % path_size];
    int second = path[second_node_index];
    int next_second = path[(second_node_index + 1) % path_size];

    double old_cost = 0.0;
    double new_cost = 0.0;

    if ((first_node_index + 1) % path_size == second_node_index) {
        // adjacent nodes
        old_cost += dist_matrix[prev_first][first] + dist_matrix[first][second] + dist_matrix[second][next_second];
        new_cost += dist_matrix[prev_first][second] + dist_matrix[second][first] + dist_matrix[first][next_second];
    } else if ((second_node_index + 1) % path_size == first_node_index) {
        // adjacent nodes
        old_cost += dist_matrix[prev_second][second] + dist_matrix[second][first] + dist_matrix[first][next_first];
        new_cost += dist_matrix[prev_second][first] + dist_matrix[first][second] + dist_matrix[second][next_first];
    } else {
        // non-adjacent nodes
        old_cost += dist_matrix[prev_first][first] + dist_matrix[first][next_first];
        old_cost += dist_matrix[prev_second][second] + dist_matrix[second][next_second];

        new_cost += dist_matrix[prev_first][second] + dist_matrix[second][next_first];
        new_cost += dist_matrix[prev_second][first] + dist_matrix[first][next_second];
    }   
    double delta = new_cost - old_cost;
    return delta;
}

double delta_intra_edge(const vector<int>& path, int first_node_of_first_edge, int first_node_of_second_edge, const vector<vector<int>>& dist_matrix) {
    int path_size = path.size();
    if (path_size < 4) return 0.0;
    int first_node_first_edge = path[first_node_of_first_edge];
    int second_node_first_edge = path[(first_node_of_first_edge + 1) % path_size];
    int first_node_second_edge = path[first_node_of_second_edge];
    int second_node_second_edge = path[(first_node_of_second_edge + 1) % path_size];

    if (second_node_first_edge == first_node_second_edge) {
        // edges are adjacent
        return 0.0;
    }

    double old_cost = dist_matrix[first_node_first_edge][second_node_first_edge] +
                      dist_matrix[first_node_second_edge][second_node_second_edge];
    double new_cost = dist_matrix[first_node_first_edge][first_node_second_edge] +
                      dist_matrix[second_node_first_edge][second_node_second_edge];
    double delta = new_cost - old_cost;
    return delta;
}

double delta_inter_node(const vector<int>& path, int selected_node, int not_selected_node, const vector<vector<int>>& dist_matrix, const vector<Node>& nodes) {
    int path_size = path.size();
    if (path_size < 2) return 0.0;

    int node_selected = path[selected_node];
    int prev_node = path[(selected_node - 1 + path_size) % path_size];
    int next_node = path[(selected_node + 1) % path_size];

    double old_cost = dist_matrix[prev_node][node_selected] + dist_matrix[node_selected][next_node] + nodes[node_selected].cost;
    double new_cost = dist_matrix[prev_node][not_selected_node] + dist_matrix[not_selected_node][next_node] + nodes[not_selected_node].cost;

    double delta = new_cost - old_cost;
    return delta;
}

//Not sure if it works due to stop condition of the loop
vector<int> steepest_local_search(const vector<int>& initial_path, const vector<vector<int>>& dist_matrix, const string& intra_method, const vector<Node>& nodes) {
    int dist_size = dist_matrix.size();
    vector<int> current_path = initial_path;
    //double current_cost = initial_cost;

    while(true){
        double best_delta = 0.0;
        int best_move_type = -1; // 0: intra-node, 1: intra-edge, 2: inter-node
        int best_first_node = 0, best_second_node = 0;
        int best_not_selected_node = -1;

        int current_path_size = current_path.size();
        if (current_path_size < 2) break;

        set<int> selected_nodes(current_path.begin(), current_path.end());
        vector<int> not_selected_nodes;
        for (int i = 0; i < dist_size; ++i) {
            if (selected_nodes.find(i) == selected_nodes.end()) {
                not_selected_nodes.push_back(i);
            }
        }

        if (intra_method == "node"){
            for (int i = 0; i < current_path_size; ++i) {
                for (int j = i + 1; j < current_path_size; ++j) {
                    double delta = delta_intra_node(current_path, i, j, dist_matrix);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 0;
                        best_first_node = i;
                        best_second_node = j;
                    }
                }
            }
        } else if (intra_method == "edge"){
            for (int i = 0; i < current_path_size; ++i) {
                for (int j = i + 2; j < current_path_size; ++j) {
                    double delta = delta_intra_edge(current_path, i, j, dist_matrix);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 1;
                        best_first_node = i;
                        best_second_node = j;
                    }
                }
            }
        }

        for (int i = 0; i < current_path_size; ++i) {
            for (int not_selected_node : not_selected_nodes) {
                double delta = delta_inter_node(current_path, i, not_selected_node, dist_matrix, nodes);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 2;
                    best_first_node = i;
                    best_not_selected_node = not_selected_node;
                }
            }
        }

    

        if (best_move_type != -1 && best_delta < 0.0) {
            //current_cost += best_delta;
            if (best_move_type == 0) {
                // intra-node swap
                swap(current_path[best_first_node], current_path[best_second_node]);
            } else if (best_move_type == 1) {
                // intra-edge swap
                reverse(current_path.begin() + best_first_node + 1, current_path.begin() + best_second_node + 1);
            } else if (best_move_type == 2) {
                // inter-node swap
                current_path[best_first_node] = best_not_selected_node;
            } 
        } else {
                break; // no improving move found
            }
        

    }
    return current_path;
}

vector<int> greedy_local_search(const vector<int>& initial_path, const vector<vector<int>>& dist_matrix, const string& intra_method, const vector<Node>& nodes) {
    int dist_size = dist_matrix.size();
    vector<int> current_path = initial_path;
    //double current_cost = initial_cost;

    while (true){
        int current_path_size = current_path.size();
        if (current_path_size < 2) break;

        set<int> selected_nodes(current_path.begin(), current_path.end());
        vector<int> not_selected_nodes;
        for (int i = 0; i < dist_size; ++i) {
            if (selected_nodes.find(i) == selected_nodes.end()) {
                not_selected_nodes.push_back(i);
            }
        }

        vector<tuple<int, int, int, int>> possible_moves; // move_type, first_node, second_node, not_selected_node (in case of inter_node)

        if (intra_method == "node"){
            for (int i = 0; i < current_path_size; ++i) {
                for (int j = i + 1; j < current_path_size; ++j) {
                    possible_moves.emplace_back(0, i, j, -1);
                }
            }
        } else if (intra_method == "edge"){
            for (int i = 0; i < current_path_size; ++i) {
                for (int j = i + 2; j < current_path_size; ++j) {
                    possible_moves.emplace_back(1, i, j, -1);
                }
            }
        }

        for (int i = 0; i < current_path_size; ++i) {
            for (int not_selected_node : not_selected_nodes) {
                possible_moves.emplace_back(2, i, -1, not_selected_node);
            }
        }

        if (possible_moves.empty()) break;

        shuffle(possible_moves.begin(), possible_moves.end(), std::mt19937{std::random_device{}()});
        bool found_better_move = false;

        for (const auto& move : possible_moves) {
            int move_type = get<0>(move);
            int first_node = get<1>(move);
            int second_node = get<2>(move);
            int not_selected_node = get<3>(move);

            double delta = 0.0;
            if (move_type == 0) {
                delta = delta_intra_node(current_path, first_node, second_node, dist_matrix);
            } else if (move_type == 1) {
                delta = delta_intra_edge(current_path, first_node, second_node, dist_matrix);
            } else if (move_type == 2) {
                delta = delta_inter_node(current_path, first_node, not_selected_node, dist_matrix, nodes);
            }

            
            if (delta < 0.0) {
                //current_cost += delta;
                if (move_type == 0) {
                    swap(current_path[first_node], current_path[second_node]);
                } else if (move_type == 1) {
                    reverse(current_path.begin() + first_node + 1, current_path.begin() + second_node + 1);
                } else if (move_type == 2) {
                    current_path[first_node] = not_selected_node;
                }
                found_better_move = true;
                break; // exit after the first improving move
            }
        }

        if (!found_better_move) {
            break; // no improving move found
        }
    }
    
    return current_path;
}
