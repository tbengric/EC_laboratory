#pragma once
#include <vector>
#include <cmath>
#include "node.h"
#include <random>
#include "heuristics.h"
#include "distance_matrix.h"

using namespace std;

double delta_intra_node(const vector<int>& path, int i, int j, const vector<vector<int>>& dist_matrix, const vector<Node>& nodes) {
    
}

double delta_intra_edge(const vector<int>& path, int i, int j, const vector<vector<int>>& dist_matrix, const vector<Node>& nodes) {
    
}

double delta_inter_node(const vector<int>& path, int i, int j, const vector<vector<int>>& dist_matrix, const vector<Node>& nodes) {
    
}

pair<vector<int>, int> steepest_local_search(const vector<int>& initial_path, double initial_cost, const vector<vector<int>>& dist_matrix, const string& intra_method) {
    
}

pair<vector<int>, int> greedy_local_search(const vector<int>& initial_path, double initial_cost, const vector<vector<int>>& dist_matrix, const string& intra_method) {
    
}
