#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "node.h"
#include "distance_matrix.h"
#include "heuristics.h"


using namespace std;

int main() {
    string filename = "TSPA.csv"; // your CSV file
    char delimiter = ';';         // assuming semicolon-separated CSV

    // Vectors to store each column
    vector<int> x;
    vector<int> y;
    vector<int> costs;

    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

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

    vector<Node> nodes;
    for (size_t i = 0; i < x.size(); ++i) {
        Node node;
        node.x = x[i];
        node.y = y[i];
        node.cost = costs[i];
        nodes.push_back(node);
    }

    vector<vector<int>> distanceMatrix = DistanceMatrix(nodes);
    vector<int> selectedNodes = selectNodes(nodes.size());

    for (int i = 0; i < 200; ++i) {
        auto randPath = randomSolution(selectedNodes);
        auto score = computeObjective(randPath, distanceMatrix, nodes);
        std::cout << "Random " << i << ": " << score << "\n";
    }

    for (int start : selectedNodes) {
        for (int i = 0; i < 200; ++i) {
            auto path1 = nearestNeighborEnd(selectedNodes, distanceMatrix, nodes, start);
            auto path2 = nearestNeighborFlexible(selectedNodes, distanceMatrix, nodes, start);
            auto path3 = greedyCycle(selectedNodes, distanceMatrix, nodes, start);
            std::cout << "NN-End " << i << ": " << computeObjective(path1, distanceMatrix, nodes) << "\n";
            std::cout << "NN-Flex " << i << ": " << computeObjective(path2, distanceMatrix, nodes) << "\n";
            std::cout << "Greedy " << i << ": " << computeObjective(path3, distanceMatrix, nodes) << "\n";
        }
    }

    return 0;
}
