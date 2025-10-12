#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "node.h"
#include "distance_matrix.h"
#include "heuristics.h"
#include <iomanip> 

using namespace std;

void saveResults(const string& filename,
                 const vector<Node>& nodes,
                 const vector<int>& path,
                 const string& label) {
    ofstream out(filename, ios::app); // append mode
    if (!out.is_open()) {
        cerr << "Error: could not open " << filename << endl;
        return;
    }
    out << label << "\n";
    out << "x,y,cost\n"; // header for cost
    for (int idx : path) {
        out << nodes[idx].x << "," << nodes[idx].y << "," << nodes[idx].cost << "\n";
    }
    out << "\n";
    out.close();
}

void printPath(const vector<int>& path) {
    for (int node : path) {
        cout << node << " ";
    }
    cout << endl;
}

int main() {
    vector<string> instances = {"A", "B"}; 

    for (const string& id : instances) {
        cout << "\n=== Processing TSP" << id << " ===\n";

        string csvFile = "TSP" + id + ".csv";
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

        cout << "Selected nodes: ";
        printPath(selectedNodes);

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
