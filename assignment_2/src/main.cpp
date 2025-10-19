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
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

// --- Utility functions ---
void delete_content_file(const string& filename) {
    fs::path filepath = filename;
    try {
        if (fs::remove(filepath)) {
            cout << "File has been deleted\n";
        } else {
            cout << "File does not exist.\n";
        }
    } catch (const fs::filesystem_error& e) {
        cerr << "Error: " << e.what() << '\n';
    }
}

void saveResults(const string& filename,
                 const vector<Node>& nodes,
                 const vector<int>& path,
                 const string& label) {
    ofstream out(filename, ios::app);
    if (!out.is_open()) {
        cerr << "Error: could not open " << filename << endl;
        return;
    }
    out << label << "\n";
    out << "id,x,y,cost\n";
    for (int idx : path) {
        out << nodes[idx].id << "," << nodes[idx].x << "," << nodes[idx].y << "," << nodes[idx].cost << "\n";
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

// --- Main program ---
int main() {
    vector<string> tsp_types = {"TSPA", "TSPB"};

    for (const auto& tsp_type : tsp_types) {
        cout << "\n==============================" << endl;
        cout << "Processing " << tsp_type << endl;
        cout << "==============================" << endl;

        // --- Read nodes from CSV ---
        string filename = "../data/" + tsp_type + ".csv";
        char delimiter = ';';

        vector<int> x, y, costs;
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error: Could not open file " << filename << endl;
            continue;
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

        // --- Distance matrix ---
        vector<vector<int>> distanceMatrix = DistanceMatrix(nodes);

        // --- Random Search ---
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

        // --- Heuristic searches ---
        int bestScoreNNend = -1, bestScoreNNflex = -1, bestScoreGreedy = -1;

        int bestScoreGreedy2Regret = -1, bestScoreGreedy3Regret = -1;
        int bestScoreGreedy2RegretWeighted = -1, bestScoreGreedy2RegretWeighted1 = -1, bestScoreGreedy2RegretWeighted2 = -1;

        int bestScoreNNflex2Regret = -1, bestScoreNNflex3Regret = -1;
        int bestScoreNNflex2RegretWeighted = -1, bestScoreNNflex2RegretWeighted1 = -1, bestScoreNNflex2RegretWeighted2 = -1;

        vector<int> bestPath1, bestPath2, bestPathGreedy;
        vector<int> bestPathGreedy2Regret, bestPathGreedy3Regret, bestPathGreedy2RegretWSum, bestPathGreedy2RegretWSum1, bestPathGreedy2RegretWSum2;
        vector<int> bestPathNNflex2Regret, bestPathNNflex3Regret, bestPathNNflex2RegretWSum, bestPathNNflex2RegretWSum1, bestPathNNflex2RegretWSum2;

        vector<int> nnEndScores, nnFlexScores, greedyScores;
        vector<int> greedy2Regret, greedy3Regret, greedy2RegretWSum, greedy2RegretWSum1, greedy2RegretWSum2;
        vector<int> nnFlex2Regret, nnFlex3Regret, nnFlex2RegretWSum, nnFlex2RegretWSum1, nnFlex2RegretWSum2;

        auto updateBest = [](int& bestScore, vector<int>& bestPath, const vector<int>& path, int cost) {
            if (bestScore == -1 || cost < bestScore) {
                bestScore = cost;
                bestPath = path;
            }
        };

        for (int id_starting_node = 0; id_starting_node < 200; id_starting_node++) {
            cout << "Starting from node: " << id_starting_node << endl;

            // --- Run heuristics ---
            auto path1 = nearestNeighborEnd(distanceMatrix, nodes, id_starting_node);
            auto path2 = nearestNeighborFlexible(distanceMatrix, nodes, id_starting_node);
            auto path3 = greedyCycle(distanceMatrix, nodes, id_starting_node);

            auto path_Greedy1 = greedyCycleKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 1.0);
            auto path_Greedy2 = greedyCycleKRegretWeighted(distanceMatrix, nodes, id_starting_node, 3, 1.0);
            auto path_Greedy3 = greedyCycleKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 0.5);
            auto path_Greedy4 = greedyCycleKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 0.2);
            auto path_Greedy5 = greedyCycleKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 0.7);

            auto path_NNflex1 = nearestNeighborKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 1.0);
            auto path_NNflex2 = nearestNeighborKRegretWeighted(distanceMatrix, nodes, id_starting_node, 3, 1.0);
            auto path_NNflex3 = nearestNeighborKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 0.5);
            auto path_NNflex4 = nearestNeighborKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 0.2);
            auto path_NNflex5 = nearestNeighborKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 0.7);

            // --- Compute objectives ---
            if (!path1.empty()) { int cost = computeObjective(path1, distanceMatrix, nodes); nnEndScores.push_back(cost); updateBest(bestScoreNNend, bestPath1, path1, cost); }
            if (!path2.empty()) { int cost = computeObjective(path2, distanceMatrix, nodes); nnFlexScores.push_back(cost); updateBest(bestScoreNNflex, bestPath2, path2, cost); }
            if (!path3.empty()) { int cost = computeObjective(path3, distanceMatrix, nodes); greedyScores.push_back(cost); updateBest(bestScoreGreedy, bestPathGreedy, path3, cost); }

            if (!path_Greedy1.empty()) { int cost = computeObjective(path_Greedy1, distanceMatrix, nodes); greedy2Regret.push_back(cost); updateBest(bestScoreGreedy2Regret, bestPathGreedy2Regret, path_Greedy1, cost); }
            if (!path_Greedy2.empty()) { int cost = computeObjective(path_Greedy2, distanceMatrix, nodes); greedy3Regret.push_back(cost); updateBest(bestScoreGreedy3Regret, bestPathGreedy3Regret, path_Greedy2, cost); }
            if (!path_Greedy3.empty()) { int cost = computeObjective(path_Greedy3, distanceMatrix, nodes); greedy2RegretWSum.push_back(cost); updateBest(bestScoreGreedy2RegretWeighted, bestPathGreedy2RegretWSum, path_Greedy3, cost); }
            if (!path_Greedy4.empty()) { int cost = computeObjective(path_Greedy4, distanceMatrix, nodes); greedy2RegretWSum1.push_back(cost); updateBest(bestScoreGreedy2RegretWeighted1, bestPathGreedy2RegretWSum1, path_Greedy4, cost); }
            if (!path_Greedy5.empty()) { int cost = computeObjective(path_Greedy5, distanceMatrix, nodes); greedy2RegretWSum2.push_back(cost); updateBest(bestScoreGreedy2RegretWeighted2, bestPathGreedy2RegretWSum2, path_Greedy5, cost); }

            if (!path_NNflex1.empty()) { int cost = computeObjective(path_NNflex1, distanceMatrix, nodes); nnFlex2Regret.push_back(cost); updateBest(bestScoreNNflex2Regret, bestPathNNflex2Regret, path_NNflex1, cost); }
            if (!path_NNflex2.empty()) { int cost = computeObjective(path_NNflex2, distanceMatrix, nodes); nnFlex3Regret.push_back(cost); updateBest(bestScoreNNflex3Regret, bestPathNNflex3Regret, path_NNflex2, cost); }
            if (!path_NNflex3.empty()) { int cost = computeObjective(path_NNflex3, distanceMatrix, nodes); nnFlex2RegretWSum.push_back(cost); updateBest(bestScoreNNflex2RegretWeighted, bestPathNNflex2RegretWSum, path_NNflex3, cost); }
            if (!path_NNflex4.empty()) { int cost = computeObjective(path_NNflex4, distanceMatrix, nodes); nnFlex2RegretWSum1.push_back(cost); updateBest(bestScoreNNflex2RegretWeighted1, bestPathNNflex2RegretWSum1, path_NNflex4, cost); }
            if (!path_NNflex5.empty()) { int cost = computeObjective(path_NNflex5, distanceMatrix, nodes); nnFlex2RegretWSum2.push_back(cost); updateBest(bestScoreNNflex2RegretWeighted2, bestPathNNflex2RegretWSum2, path_NNflex5, cost); }
        }

        // --- Save results for visualization ---
        string visFile = "visualization/" + tsp_type + "_paths.csv";
        delete_content_file(visFile);

        saveResults(visFile, nodes, bestRandPath, "Random Search");
        saveResults(visFile, nodes, bestPath1, "Nearest Neighbor (End)");
        saveResults(visFile, nodes, bestPath2, "Nearest Neighbor (Flexible)");
        saveResults(visFile, nodes, bestPathGreedy, "Greedy Cycle");

        saveResults(visFile, nodes, bestPathGreedy2Regret, "Greedy Cycle (2-Regret)");
        saveResults(visFile, nodes, bestPathGreedy3Regret, "Greedy Cycle (3-Regret)");
        saveResults(visFile, nodes, bestPathGreedy2RegretWSum, "Greedy Cycle (2-Regret Weighted 0.5)");
        saveResults(visFile, nodes, bestPathGreedy2RegretWSum1, "Greedy Cycle (2-Regret Weighted 0.2)");
        saveResults(visFile, nodes, bestPathGreedy2RegretWSum2, "Greedy Cycle (2-Regret Weighted 0.7)");

        saveResults(visFile, nodes, bestPathNNflex2Regret, "NN-Flexible (2-Regret)");
        saveResults(visFile, nodes, bestPathNNflex3Regret, "NN-Flexible (3-Regret)");
        saveResults(visFile, nodes, bestPathNNflex2RegretWSum, "NN-Flexible (2-Regret Weighted 0.5)");
        saveResults(visFile, nodes, bestPathNNflex2RegretWSum1, "NN-Flexible (2-Regret Weighted 0.2)");
        saveResults(visFile, nodes, bestPathNNflex2RegretWSum2, "NN-Flexible (2-Regret Weighted 0.7)");

        // --- Save LaTeX table ---
        string texFile = "results/" + tsp_type + "_results_table.tex";
        ofstream texOut(texFile);
        if (!texOut.is_open()) {
            cerr << "Error: could not create LaTeX file: " << texFile << endl;
            continue;
        }

        texOut << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\n";
        texOut << "Method & Avg (Min, Max) \\\\\n\\hline\n";

        auto writeRowCompact = [&](const string& name, const vector<int>& values) {
            if (values.empty()) return;
            int minVal = *min_element(values.begin(), values.end());
            int maxVal = *max_element(values.begin(), values.end());
            double avgVal = accumulate(values.begin(), values.end(), 0.0) / values.size();
            texOut << fixed << setprecision(2);
            texOut << name << " & " << avgVal << " (" << minVal << ", " << maxVal << ") \\\\\n";
        };

        writeRowCompact("Random Search", randScores);
        writeRowCompact("Nearest Neighbor (End)", nnEndScores);
        writeRowCompact("Nearest Neighbor (Flexible)", nnFlexScores);
        writeRowCompact("Greedy Cycle", greedyScores);
        writeRowCompact("Greedy Cycle (2-Regret)", greedy2Regret);
        writeRowCompact("Greedy Cycle (3-Regret)", greedy3Regret);
        writeRowCompact("Greedy Cycle (2-Regret Weighted 0.5)", greedy2RegretWSum);
        writeRowCompact("Greedy Cycle (2-Regret Weighted 0.2)", greedy2RegretWSum1);
        writeRowCompact("Greedy Cycle (2-Regret Weighted 0.7)", greedy2RegretWSum2);
        writeRowCompact("NN-Flexible (2-Regret)", nnFlex2Regret);
        writeRowCompact("NN-Flexible (3-Regret)", nnFlex3Regret);
        writeRowCompact("NN-Flexible (2-Regret Weighted 0.5)", nnFlex2RegretWSum);
        writeRowCompact("NN-Flexible (2-Regret Weighted 0.2)", nnFlex2RegretWSum1);
        writeRowCompact("NN-Flexible (2-Regret Weighted 0.7)", nnFlex2RegretWSum2);

        texOut << "\\hline\n\\end{tabular}\n";
        texOut << "\\caption{Average, minimum, and maximum objective values for " << tsp_type << "}\n";
        texOut << "\\label{tab:" << tsp_type << "_results}\n\\end{table}\n";
        texOut.close();

        cout << "\nLaTeX table saved to: " << texFile << endl;
        cout << "\nFinished processing all datasets.\n";
    }

    return 0;
}
