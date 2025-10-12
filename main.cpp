#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "node.h"
#include "distance_matrix.h"
#include "heuristics.h"
#include <iomanip>
#include <numeric>
#include <algorithm>

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

double average(const vector<int>& values) {
    if (values.empty()) return 0.0;
    return accumulate(values.begin(), values.end(), 0.0) / values.size();
}

void printPath(const vector<int>& path) {
    for (int node : path) cout << node << " ";
    cout << endl;
}

auto printStats = [](const string& name, const vector<int>& values) {
    if (values.empty()) return;
    cout << fixed << setprecision(2);
    cout << name << ":\n";
    cout << "  Min: " << *min_element(values.begin(), values.end()) << "\n";
    cout << "  Max: " << *max_element(values.begin(), values.end()) << "\n";
    cout << "  Avg: " << average(values) << "\n";
};

int main() {
    string filename = "TSPB.csv"; // your CSV file
    string tsp_type = "TSPB";
    char delimiter = ';';         // assuming semicolon-separated CSV

    vector<int> x, y, costs;

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

    // Random Search
    vector<int> bestRandPath;
    int bestRandScore = -1;
    vector<int> randScores;

    for (int i = 0; i < 200; ++i) {
        auto randPath = randomSolution(selectedNodes);
        int score = computeObjective(randPath, distanceMatrix, nodes);
        randScores.push_back(score);
        cout << "Random " << i << ": " << score << "\n";

        if (bestRandScore == -1 || score < bestRandScore) {
            bestRandScore = score;
            bestRandPath = randPath;
        }
    }
    saveResults("visualization/" + tsp_type + "_paths.csv", nodes, bestRandPath, "Random Search");

    // Heuristic Searches
    int i = 1;
    int bestScoreNNend = -1;
    int bestScoreNNflex = -1;
    int bestScoreGreedy = -1;
    vector<int> bestPath1, bestPath2, bestPath3;
    vector<int> nnEndScores, nnFlexScores, greedyScores;

    for (int start : selectedNodes) {
        auto path1 = nearestNeighborEnd(selectedNodes, distanceMatrix, nodes, start);
        auto path2 = nearestNeighborFlexible(selectedNodes, distanceMatrix, nodes, start);
        auto path3 = greedyCycle(selectedNodes, distanceMatrix, nodes, start);
        int costNNend = computeObjective(path1, distanceMatrix, nodes);
        int costNNflex = computeObjective(path2, distanceMatrix, nodes);
        int costGreedy = computeObjective(path3, distanceMatrix, nodes);

        nnEndScores.push_back(costNNend);
        nnFlexScores.push_back(costNNflex);
        greedyScores.push_back(costGreedy);

        cout << "NN-End " << i << ": " << costNNend << "\n";
        cout << "NN-Flex " << i << ": " << costNNflex << "\n";
        cout << "Greedy " << i << ": " << costGreedy << "\n";
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

    printStats("Random Search", randScores);
    printStats("Nearest Neighbor End", nnEndScores);
    printStats("Nearest Neighbor Flexible", nnFlexScores);
    printStats("Greedy Cycle", greedyScores);

    // Save data for visualisation
    saveResults("visualization/" + tsp_type + "_paths.csv", nodes, bestPath1, "Nearest Neighbor");
    saveResults("visualization/" + tsp_type + "_paths.csv", nodes, bestPath2, "Nearest Neighbor Flexible");
    saveResults("visualization/" + tsp_type + "_paths.csv", nodes, bestPath3, "Greedy Cycle");

    // Save the avg(min,max) values int .tex format
    string texFile = "visualization/" + tsp_type + "_results_table.tex";
    ofstream texOut(texFile);
    if (!texOut.is_open()) {
        cerr << "Error: could not create LaTeX file: " << texFile << endl;
        return 1;
    }

    texOut << "\\begin{table}[h!]\n"
           << "\\centering\n"
           << "\\begin{tabular}{lc}\n"
           << "\\hline\n"
           << "Method & Avg (Min, Max) \\\\\n"
           << "\\hline\n";

    auto writeRowCompact = [&](const string& name, const vector<int>& values) {
        if (values.empty()) return;
        int minVal = *min_element(values.begin(), values.end());
        int maxVal = *max_element(values.begin(), values.end());
        double avgVal = accumulate(values.begin(), values.end(), 0.0) / values.size();
        texOut << fixed << setprecision(2);
        texOut << name << " & " << avgVal << " (" << minVal << ", " << maxVal << ") \\\\\n";
    };

    writeRowCompact("Random Search", randScores);
    writeRowCompact("Nearest Neighbor End", nnEndScores);
    writeRowCompact("Nearest Neighbor Flexible", nnFlexScores);
    writeRowCompact("Greedy Cycle", greedyScores);

    texOut << "\\hline\n"
           << "\\end{tabular}\n"
           << "\\caption{Average, minimum, and maximum objective values for " << tsp_type << "}\n"
           << "\\label{tab:" << tsp_type << "_results}\n"
           << "\\end{table}\n";
    texOut.close();

    cout << "\nLaTeX table saved to: " << texFile << endl;

    return 0;
}
