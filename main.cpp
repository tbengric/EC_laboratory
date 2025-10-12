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
    string filename = "TSPB.csv"; // your CSV file
    string tsp_type = "TSPB";
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

    cout << "Selected nodes: ";
    printPath(selectedNodes);

    vector<int> bestRandPath;
    int bestRandScore = -1;

    for (int i = 0; i < 200; ++i) {
        auto randPath = randomSolution(selectedNodes);
        int score = computeObjective(randPath, distanceMatrix, nodes);
        cout << "Random " << i << ": " << score << "\n";

        if (bestRandScore == -1 || score < bestRandScore) {
            bestRandScore = score;
            bestRandPath = randPath;
        }
    }

    saveResults("visualization/"+ tsp_type +"_paths.csv", nodes, bestRandPath, "Random Search");

    int i = 1;
    int bestScoreNNend = -1;
    int bestScoreNNflex = -1;
    int bestScoreGreedy = -1;
    vector<int> bestPath1, bestPath2, bestPath3;
    for (int start : selectedNodes) {
        auto path1 = nearestNeighborEnd(selectedNodes, distanceMatrix, nodes, start);
        auto path2 = nearestNeighborFlexible(selectedNodes, distanceMatrix, nodes, start);
        auto path3 = greedyCycle(selectedNodes, distanceMatrix, nodes, start);
        int costNNend = computeObjective(path1, distanceMatrix, nodes);
        int costNNflex = computeObjective(path2, distanceMatrix, nodes);
        int costGreedy = computeObjective(path3, distanceMatrix, nodes);
        cout << "NN-End " << i << ": " << costNNend << "\n";
        //printPath(path1);
        cout << "NN-Flex " << i << ": " << costNNflex << "\n";
        //printPath(path2);
        cout << "Greedy " << i << ": " << costGreedy << "\n";
        //printPath(path3);
        cout << "------------\n";
        i++;
        if (bestScoreNNend == -1) {bestScoreNNend = costNNend; bestPath1 = path1;}
        else if (costNNend < bestScoreNNend && bestScoreNNend != -1) {bestScoreNNend = costNNend; bestPath1 = path1;}

        if (bestScoreNNflex == -1) {bestScoreNNflex = costNNflex; bestPath2 = path2;}
        else if (costNNflex < bestScoreNNflex && bestScoreNNflex != -1) {bestScoreNNflex = costNNflex; bestPath2 = path2;}

        if (bestScoreGreedy == -1) {bestScoreGreedy = costGreedy; bestPath3 = path3;}
        else if (costGreedy < bestScoreGreedy && bestScoreGreedy != -1) {bestScoreGreedy = costGreedy; bestPath3 = path3;}
    }
    cout << "Best NN-End: " << bestScoreNNend << "\n";
    cout << "Best Path: " << "\n";
    printPath(bestPath1);
    cout << "Best NN-Flex: " << bestScoreNNflex << "\n";
    cout << "Best Path: " << "\n";
    printPath(bestPath2);
    cout << "Best Greedy: " << bestScoreGreedy << "\n";
    cout << "Best Path: " << "\n";
    printPath(bestPath3);

    saveResults("visualization/"+ tsp_type +"_paths.csv", nodes, bestPath1, "Nearest Neighbor");
    saveResults("visualization/"+ tsp_type +"_paths.csv", nodes, bestPath2, "Nearest Neighbor Flexible");
    saveResults("visualization/"+ tsp_type +"_paths.csv", nodes, bestPath3, "Greedy Cycle");

    return 0;
}
