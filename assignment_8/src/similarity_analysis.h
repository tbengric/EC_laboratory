#pragma once
#include <vector>
#include <set>
#include <cmath>
#include <numeric>
#include <algorithm>

using namespace std;

// Structure to store a local optimum with its objective value
struct LocalOptimum {
    vector<int> path;
    int objective;
};

// Calculate the number of common edges between two paths
int commonEdges(const vector<int>& path1, const vector<int>& path2) {
    set<pair<int, int>> edges1, edges2;
    
    // Extract edges from path1 (undirected)
    for (size_t i = 0; i < path1.size() - 1; ++i) {
        int a = path1[i];
        int b = path1[i + 1];
        if (a > b) swap(a, b);
        edges1.insert({a, b});
    }
    
    // Extract edges from path2 (undirected)
    for (size_t i = 0; i < path2.size() - 1; ++i) {
        int a = path2[i];
        int b = path2[i + 1];
        if (a > b) swap(a, b);
        edges2.insert({a, b});
    }
    
    // Count common edges
    int common = 0;
    for (const auto& edge : edges1) {
        if (edges2.count(edge) > 0) {
            common++;
        }
    }
    
    return common;
}

// Calculate the number of common selected nodes between two paths
int commonNodes(const vector<int>& path1, const vector<int>& path2) {
    set<int> nodes1(path1.begin(), path1.end());
    set<int> nodes2(path2.begin(), path2.end());
    
    int common = 0;
    for (int node : nodes1) {
        if (nodes2.count(node) > 0) {
            common++;
        }
    }
    
    return common;
}

// Calculate correlation coefficient
double correlationCoefficient(const vector<int>& x, const vector<double>& y) {
    if (x.size() != y.size() || x.size() == 0) return 0.0;
    
    double meanX = accumulate(x.begin(), x.end(), 0.0) / x.size();
    double meanY = accumulate(y.begin(), y.end(), 0.0) / y.size();
    
    double numerator = 0.0;
    double denomX = 0.0;
    double denomY = 0.0;
    
    for (size_t i = 0; i < x.size(); ++i) {
        double dx = x[i] - meanX;
        double dy = y[i] - meanY;
        numerator += dx * dy;
        denomX += dx * dx;
        denomY += dy * dy;
    }
    
    if (denomX == 0.0 || denomY == 0.0) return 0.0;
    
    return numerator / sqrt(denomX * denomY);
}
