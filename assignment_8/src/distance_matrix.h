#pragma once
#include <vector>
#include <cmath>
#include "node.h"

using namespace std;

vector<vector<int>> DistanceMatrix(const vector<Node>& nodes) {
    int n = nodes.size();
    vector<vector<int>> dist(n, vector<int>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            double dx = nodes[i].x - nodes[j].x;
            double dy = nodes[i].y - nodes[j].y;
            dist[i][j] = static_cast<int>(round(sqrt(dx * dx + dy * dy)));
        }
    return dist;
}
