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
    out << "id,x,y,cost\n"; 
    for (int idx : path) {
        out <<nodes[idx].id << ","<< nodes[idx].x << "," << nodes[idx].y << "," << nodes[idx].cost << "\n";
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

int main() {
    vector<string> tsp_types = {"TSPA", "TSPB"};

    for (const auto& tsp_type : tsp_types) {
        cout << "\n==============================" << endl;
        cout << "Processing " << tsp_type << endl;
        cout << "==============================" << endl;

        // Read the nodes from .CSV file
        string filename = "data/" + tsp_type + ".csv";
        char delimiter = ';';

        vector<int> x, y, costs;

        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error: Could not open file " << filename << endl;
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
            node.id = i;
            node.x = x[i];
            node.y = y[i];
            node.cost = costs[i];
            nodes.push_back(node);
        }

        // Create distance matrix
        vector<vector<int>> distanceMatrix = DistanceMatrix(nodes);

        // 1. Random Search
        vector<int> bestRandPath;
        int bestRandScore = -1;
        vector<int> randScores;

        for (int i = 0; i < 200; i++) {
            vector<int> selectedNodes = selectNodes(nodes.size());
            auto randPath = randomSolution(selectedNodes);
            int score = computeObjective(randPath, distanceMatrix, nodes);
            randScores.push_back(score);

            if (bestRandScore == -1 || score < bestRandScore) {
                bestRandScore = score;
                bestRandPath = randPath;
            }
        }

        // --- Heuristic Searches ---
        int bestScoreNNend = -1;
        int bestScoreNNflex = -1;
        int bestScoreGreedy = -1;
        vector<int> bestPath1, bestPath2, bestPath3;
        vector<int> nnEndScores, nnFlexScores, greedyScores;

        for (int id_starting_node = 0; id_starting_node < 200; id_starting_node++) {
            cout <<"Starting from node: " << id_starting_node << endl;

            auto path1 = nearestNeighborEnd(distanceMatrix, nodes, id_starting_node);
            auto path2 = nearestNeighborFlexible(distanceMatrix, nodes, id_starting_node);
            auto path3 = greedyCycle(distanceMatrix, nodes, id_starting_node);

            int costNNend = computeObjective(path1, distanceMatrix, nodes);
            int costNNflex = computeObjective(path2, distanceMatrix, nodes);
            int costGreedy = computeObjective(path3, distanceMatrix, nodes);

            nnEndScores.push_back(costNNend);
            nnFlexScores.push_back(costNNflex);
            greedyScores.push_back(costGreedy);

            if (bestScoreNNend == -1) {bestScoreNNend = costNNend; bestPath1 = path1;}
            else if (costNNend < bestScoreNNend && bestScoreNNend != -1) {bestScoreNNend = costNNend; bestPath1 = path1;}

            if (bestScoreNNflex == -1) {bestScoreNNflex = costNNflex; bestPath2 = path2;}
            else if (costNNflex < bestScoreNNflex && bestScoreNNflex != -1) {bestScoreNNflex = costNNflex; bestPath2 = path2;}

            if (bestScoreGreedy == -1) {bestScoreGreedy = costGreedy; bestPath3 = path3;}
            else if (costGreedy < bestScoreGreedy && bestScoreGreedy != -1) {bestScoreGreedy = costGreedy; bestPath3 = path3;}

        }

        // --- Save results for visualization ---
        saveResults("assignment_1/visualization/" + tsp_type + "_paths.csv", nodes, bestRandPath, "Random Search");
        saveResults("assignment_1/visualization/" + tsp_type + "_paths.csv", nodes, bestPath1, "Nearest Neighbor");
        saveResults("assignment_1/visualization/" + tsp_type + "_paths.csv", nodes, bestPath2, "Nearest Neighbor Flexible");
        saveResults("assignment_1/visualization/" + tsp_type + "_paths.csv", nodes, bestPath3, "Greedy Cycle");

        // --- Save LaTeX table with results ---
        string texFile = "assignment_1/results/" + tsp_type + "_results_table.tex";
        ofstream texOut(texFile);
        if (!texOut.is_open()) {
            cerr << "Error: could not create LaTeX file: " << texFile << endl;
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
    }

    cout << "\nFinished processing all datasets.\n";
    return 0;
}
