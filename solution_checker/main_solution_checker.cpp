#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip> 
#include <algorithm>
#include <numeric>
#include <limits>
#include <random>
#include <cmath>

using namespace std;


// Structure of the node
struct Node {
    int id, x, y, cost;
};

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

//Distance Matrix
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

int main() {
    vector<string> instances = {"A", "B"}; 

    for (const string& id : instances) {
        cout << "\n=== Processing TSP" << id << " ===\n";

        string csvFile = "data/TSP" + id + ".csv";
        string nodeFilePath = "solution_checker/selected_nodes/TSP" + id + ".txt";

        // Open CSV
        ifstream file(csvFile);
        if (!file.is_open()) {
            cerr << "Error: Could not open file " << csvFile << endl;
            continue;
        }

        char delimiter = ';';
        vector<int> x, y, costs;
        string line;
        int a, b, c;

        while (getline(file, line)) {
            stringstream ss(line);
            if (ss >> a >> delimiter >> b >> delimiter >> c) {
                x.push_back(a);
                y.push_back(b);
                costs.push_back(c);
            } else {
                cerr << "Warning: Skipping malformed line: " << line << endl;
            }
        }
        file.close();

        // Create nodes
        vector<Node> nodes;
        for (size_t i = 0; i < x.size(); ++i) {
            Node node;
            node.x = x[i];
            node.y = y[i];
            node.cost = costs[i];
            nodes.push_back(node);
        }

        vector<vector<int>> distanceMatrix = DistanceMatrix(nodes);

        ifstream nodeFile(nodeFilePath);
        if (!nodeFile.is_open()) {
            cerr << "Error: Could not open " << nodeFilePath << endl;
            continue;
        }

        vector<int> selectedNodes;
        int node;
        while (nodeFile >> node) {
            selectedNodes.push_back(node);
        }
        nodeFile.close();

        int score = computeObjective(selectedNodes, distanceMatrix, nodes);

        int expectedScore = 0;
        if (id == "A") expectedScore = 265366;
        else if (id == "B") expectedScore = 208785;

        if (score == expectedScore) {
            cout << "Test passed. Objective matches expected value: " << expectedScore << "\n";
        } else {
            cout << "Test failed! Expected: " << expectedScore 
                << ", but got: " << score << "\n";
        }

    }

    cout << "\nAll TSP instances processed successfully.\n";
    return 0;
}
