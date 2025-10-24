#pragma once
#include <vector>
#include <cmath>
#include "node.h"
#include <random>
#include "heuristics.h"
#include "distance_matrix.h"

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
    
}

double delta_inter_node(const vector<int>& path, int i, int j, const vector<vector<int>>& dist_matrix, const vector<Node>& nodes) {
    
}

pair<vector<int>, int> steepest_local_search(const vector<int>& initial_path, double initial_cost, const vector<vector<int>>& dist_matrix, const string& intra_method) {
    
}

pair<vector<int>, int> greedy_local_search(const vector<int>& initial_path, double initial_cost, const vector<vector<int>>& dist_matrix, const string& intra_method) {
    
}
