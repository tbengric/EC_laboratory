#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include "node.h"
#include "distance_matrix.h"
#include "heuristics.h"
#include "localsearch.h"

using namespace std;
namespace fs = std::filesystem;

// --- Timer ---
struct Timer {
    chrono::high_resolution_clock::time_point start;

    void tic() { start = chrono::high_resolution_clock::now(); }

    double toc_ms() {
        auto end = chrono::high_resolution_clock::now();
        return chrono::duration<double, std::milli>(end - start).count();
    }
};

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

double average_ints(const vector<int>& values) {
    if (values.empty()) return 0.0;
    return accumulate(values.begin(), values.end(), 0.0) / values.size();
}

double average_doubles(const vector<double>& values) {
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
        string filename = "../../data/" + tsp_type + ".csv";
        char delimiter = ';';

        vector<int> x, y, costs;
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error: Could not open file " << filename << endl;
            continue;
        }

        string line;
        while (getline(file, line)) {
            stringstream ss(line);
            string token;
            int a, b, c;
            if (getline(ss, token, delimiter)) a = stoi(token);
            else { cerr << "Skipping malformed line: " << line << endl; continue; }

            if (getline(ss, token, delimiter)) b = stoi(token);
            else { cerr << "Skipping malformed line: " << line << endl; continue; }

            if (getline(ss, token, delimiter)) c = stoi(token);
            else { cerr << "Skipping malformed line: " << line << endl; continue; }

            x.push_back(a);
            y.push_back(b);
            costs.push_back(c);
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
        auto distanceMatrix = DistanceMatrix(nodes);

        // --- Initialize scores and times ---
        vector<int> randScores;
        vector<int> nnEndScores, nnFlexScores, greedyScores;
        vector<int> greedy2Regret, greedy2RegretWSum;
        vector<int> nnFlex2Regret, nnFlex2RegretWSum;

        vector<double> randTimes_ms;
        vector<double> nnEndTimes_ms, nnFlexTimes_ms, greedyTimes_ms;
        vector<double> greedy2RegretTimes_ms, greedy2RegretWSumTimes_ms;
        vector<double> nnFlex2RegretTimes_ms, nnFlex2RegretWSumTimes_ms;

        vector<int> localGreedyNodeScores_Greedy, localGreedyEdgeScores_Greedy;
        vector<int> localSteepestNodeScores_Greedy, localSteepestEdgeScores_Greedy;
        vector<int> localGreedyNodeScores_Rand, localGreedyEdgeScores_Rand;
        vector<int> localSteepestNodeScores_Rand, localSteepestEdgeScores_Rand;

        vector<double> localGreedyNodeTimes_Greedy_ms, localGreedyEdgeTimes_Greedy_ms;
        vector<double> localSteepestNodeTimes_Greedy_ms, localSteepestEdgeTimes_Greedy_ms;
        vector<double> localGreedyNodeTimes_Rand_ms, localGreedyEdgeTimes_Rand_ms;
        vector<double> localSteepestNodeTimes_Rand_ms, localSteepestEdgeTimes_Rand_ms;

        vector<int> bestRandPath, bestNNEnd, bestNNFlex, bestGreedy, bestG2, bestG05, bestNNF2, bestNNF05;
        vector<int> bestLocalGreedyNode_Greedy, bestLocalGreedyEdge_Greedy, bestLocalSteepestNode_Greedy, bestLocalSteepestEdge_Greedy;
        vector<int> bestLocalGreedyNode_Rand, bestLocalGreedyEdge_Rand, bestLocalSteepestNode_Rand, bestLocalSteepestEdge_Rand;

        int bestRandScore = -1;
        int bestScoreNNend = -1, bestScoreNNflex = -1, bestScoreGreedy = -1;
        int bestScoreGreedy2Regret = -1, bestScoreGreedy2RegretWeighted = -1;
        int bestScoreNNflex2Regret = -1, bestScoreNNflex2RegretWeighted = -1;
        int bestLocalGreedyNodeScore_Greedy = -1, bestLocalGreedyEdgeScore_Greedy = -1;
        int bestLocalSteepestNodeScore_Greedy = -1, bestLocalSteepestEdgeScore_Greedy = -1;
        int bestLocalGreedyNodeScore_Rand = -1, bestLocalGreedyEdgeScore_Rand = -1;
        int bestLocalSteepestNodeScore_Rand = -1, bestLocalSteepestEdgeScore_Rand = -1;

        auto updateBest = [](int& bestScore, vector<int>& bestPath, const vector<int>& path, int score) {
            if (path.empty()) return;
            if (bestScore == -1 || score < bestScore) {
                bestScore = score;
                bestPath = path;
            }
        };

        // --- Random Search ---
        for (int i = 0; i < 200; ++i) {
            Timer timer;
            timer.tic();
            vector<int> selectedNodes = selectNodes(nodes.size()); 
            auto randPath = randomSolution(selectedNodes);        
            double t_ms = timer.toc_ms();
            int score = computeObjective(randPath, distanceMatrix, nodes); 

            randScores.push_back(score);
            randTimes_ms.push_back(t_ms);
            updateBest(bestRandScore, bestRandPath, randPath, score);
        }

        // --- Heuristics and local searches ---
        int max_start_nodes = min<int>(200, static_cast<int>(nodes.size()));
        for (int id_starting_node = 0; id_starting_node < max_start_nodes; ++id_starting_node) {
            cout << "Starting from node: " << id_starting_node << endl;

            // --- Assignment 1 algorithms ---
            Timer t1; t1.tic();
            auto pathNNend = nearestNeighborEnd(distanceMatrix, nodes, id_starting_node);
            double dt1 = t1.toc_ms();

            Timer t2; t2.tic();
            auto pathNNflex = nearestNeighborFlexible(distanceMatrix, nodes, id_starting_node);
            double dt2 = t2.toc_ms();

            Timer t3; t3.tic();
            auto pathGreedy = greedyCycle(distanceMatrix, nodes, id_starting_node);
            double dt3 = t3.toc_ms();

            // --- Assignment 2 algorithms ---
            Timer t_g2; t_g2.tic();
            auto path_Greedy_w1 = greedyCycleKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 1.0);
            double dt_g2 = t_g2.toc_ms();

            Timer t_g05; t_g05.tic();
            auto path_Greedy_w05 = greedyCycleKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 0.5);
            double dt_g05 = t_g05.toc_ms();

            Timer t_nf2; t_nf2.tic();
            auto path_NNflex_w1 = nearestNeighborKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 1.0);
            double dt_nf2 = t_nf2.toc_ms();

            Timer t_nf05; t_nf05.tic();
            auto path_NNflex_w05 = nearestNeighborKRegretWeighted(distanceMatrix, nodes, id_starting_node, 2, 0.5);
            double dt_nf05 = t_nf05.toc_ms();

            auto process = [&](const vector<int>& path, vector<int>& scoresVec, vector<double>& timesVec_ms, vector<int>& bestPathVec, int& bestScoreRef, double time_ms) {
                if (path.empty()) return;
                int cost = computeObjective(path, distanceMatrix, nodes);
                scoresVec.push_back(cost);
                timesVec_ms.push_back(time_ms);
                updateBest(bestScoreRef, bestPathVec, path, cost);
            };

            process(pathNNend, nnEndScores, nnEndTimes_ms, bestNNEnd, bestScoreNNend, dt1);
            process(pathNNflex, nnFlexScores, nnFlexTimes_ms, bestNNFlex, bestScoreNNflex, dt2);
            process(pathGreedy, greedyScores, greedyTimes_ms, bestGreedy, bestScoreGreedy, dt3);

            process(path_Greedy_w1, greedy2Regret, greedy2RegretTimes_ms, bestG2, bestScoreGreedy2Regret, dt_g2);
            process(path_Greedy_w05, greedy2RegretWSum, greedy2RegretWSumTimes_ms, bestG05, bestScoreGreedy2RegretWeighted, dt_g05);

            process(path_NNflex_w1, nnFlex2Regret, nnFlex2RegretTimes_ms, bestNNF2, bestScoreNNflex2Regret, dt_nf2);
            process(path_NNflex_w05, nnFlex2RegretWSum, nnFlex2RegretWSumTimes_ms, bestNNF05, bestScoreNNflex2RegretWeighted, dt_nf05);

            // --- Assignment 3: Local search ---
            auto applyLocal = [&](const vector<int>& basePath,
                                  const string& method,
                                  const string& moveType,
                                  vector<int>& scoresVec,
                                  vector<double>& timesVec_ms,
                                  vector<int>& bestPathVec,
                                  int& bestScoreRef) {
                if (basePath.empty()) return;
                Timer tloc; tloc.tic();
                vector<int> newPath;
                if (method == "greedy") {
                    newPath = greedy_local_search(basePath, distanceMatrix, moveType, nodes);
                } else {
                    newPath = steepest_local_search(basePath, distanceMatrix, moveType, nodes);
                }
                double dtloc = tloc.toc_ms();
                if (!newPath.empty()) {
                    int cost = computeObjective(newPath, distanceMatrix, nodes);
                    scoresVec.push_back(cost);
                    timesVec_ms.push_back(dtloc);
                    updateBest(bestScoreRef, bestPathVec, newPath, cost);
                }
            };

            // Local searches on Greedy
            applyLocal(pathGreedy, "greedy", "node", localGreedyNodeScores_Greedy, localGreedyNodeTimes_Greedy_ms, bestLocalGreedyNode_Greedy, bestLocalGreedyNodeScore_Greedy);
            applyLocal(pathGreedy, "greedy", "edge", localGreedyEdgeScores_Greedy, localGreedyEdgeTimes_Greedy_ms, bestLocalGreedyEdge_Greedy, bestLocalGreedyEdgeScore_Greedy);
            applyLocal(pathGreedy, "steepest", "node", localSteepestNodeScores_Greedy, localSteepestNodeTimes_Greedy_ms, bestLocalSteepestNode_Greedy, bestLocalSteepestNodeScore_Greedy);
            applyLocal(pathGreedy, "steepest", "edge", localSteepestEdgeScores_Greedy, localSteepestEdgeTimes_Greedy_ms, bestLocalSteepestEdge_Greedy, bestLocalSteepestEdgeScore_Greedy);

            // Local searches on Random path
            applyLocal(bestRandPath, "greedy", "node", localGreedyNodeScores_Rand, localGreedyNodeTimes_Rand_ms, bestLocalGreedyNode_Rand, bestLocalGreedyNodeScore_Rand);
            applyLocal(bestRandPath, "greedy", "edge", localGreedyEdgeScores_Rand, localGreedyEdgeTimes_Rand_ms, bestLocalGreedyEdge_Rand, bestLocalGreedyEdgeScore_Rand);
            applyLocal(bestRandPath, "steepest", "node", localSteepestNodeScores_Rand, localSteepestNodeTimes_Rand_ms, bestLocalSteepestNode_Rand, bestLocalSteepestNodeScore_Rand);
            applyLocal(bestRandPath, "steepest", "edge", localSteepestEdgeScores_Rand, localSteepestEdgeTimes_Rand_ms, bestLocalSteepestEdge_Rand, bestLocalSteepestEdgeScore_Rand);
        }

        // --- Save best paths for visualization ---
        string visDir = "../visualization";
        fs::create_directories(visDir);
        string visFile = visDir + "/" + tsp_type + "_paths.csv";
        delete_content_file(visFile);

        saveResults(visFile, nodes, bestNNEnd, "NN End");
        saveResults(visFile, nodes, bestNNFlex, "NN Flexible");
        saveResults(visFile, nodes, bestGreedy, "Greedy Cycle");
        saveResults(visFile, nodes, bestG2, "Greedy 2-Regret");
        saveResults(visFile, nodes, bestG05, "Greedy 2-Regret (w=0.5)");
        saveResults(visFile, nodes, bestNNF2, "NN-Flex 2-Regret");
        saveResults(visFile, nodes, bestNNF05, "NN-Flex 2-Regret (w=0.5)");

        saveResults(visFile, nodes, bestLocalGreedyNode_Greedy, "Local Greedy Node (Greedy Cycle)");
        saveResults(visFile, nodes, bestLocalGreedyEdge_Greedy, "Local Greedy Edge (Greedy Cycle)");
        saveResults(visFile, nodes, bestLocalSteepestNode_Greedy, "Local Steepest Node (Greedy Cycle)");
        saveResults(visFile, nodes, bestLocalSteepestEdge_Greedy, "Local Steepest Edge (Greedy Cycle)");
        saveResults(visFile, nodes, bestLocalGreedyNode_Rand, "Local Greedy Node (Random Path)");
        saveResults(visFile, nodes, bestLocalGreedyEdge_Rand, "Local Greedy Edge (Random Path)");
        saveResults(visFile, nodes, bestLocalSteepestNode_Rand, "Local Steepest Node (Random Path)");
        saveResults(visFile, nodes, bestLocalSteepestEdge_Rand, "Local Steepest Edge (Random Path)");

        // --- LaTeX tables ---
        string resultsDir = "../results";
        fs::create_directories(resultsDir);
        string texFile = resultsDir + "/" + tsp_type + "_results_table.tex";
        string texTimeFile = resultsDir + "/" + tsp_type + "_timings_table.tex";

        ofstream texOut(texFile);
        ofstream texTimeOut(texTimeFile);

        auto writeRowCompact = [&](ofstream& out, const string& name, const vector<int>& vals) {
            if (vals.empty()) return;
            int minv = *min_element(vals.begin(), vals.end());
            int maxv = *max_element(vals.begin(), vals.end());
            double avgv = average_ints(vals);
            out << fixed << setprecision(2);
            out << name << " & " << avgv << " (" << minv << ", " << maxv << ") \\\\\n";
        };

        auto writeRowTime = [&](ofstream& out, const string& name, const vector<double>& vals_ms) {
            if (vals_ms.empty()) return;
            vector<double> vals_s;
            vals_s.reserve(vals_ms.size());
            for (double v : vals_ms) vals_s.push_back(v / 1000.0);
            double minv = *min_element(vals_s.begin(), vals_s.end());
            double maxv = *max_element(vals_s.begin(), vals_s.end());
            double avgv = average_doubles(vals_s);
            out << fixed << setprecision(4);
            out << name << " & " << avgv << " (" << minv << ", " << maxv << ") \\\\\n";
        };

        // Objective Table
        texOut << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMethod & Avg (Min, Max) \\\\\n\\hline\n";
        writeRowCompact(texOut, "Random Path", randScores);
        writeRowCompact(texOut, "Nearest Neighbor (End)", nnEndScores);
        writeRowCompact(texOut, "Nearest Neighbor (Flexible)", nnFlexScores);
        writeRowCompact(texOut, "Greedy Cycle", greedyScores);
        writeRowCompact(texOut, "Greedy 2-Regret", greedy2Regret);
        writeRowCompact(texOut, "Greedy 2-Regret (w=0.5)", greedy2RegretWSum);
        writeRowCompact(texOut, "NN-Flex 2-Regret", nnFlex2Regret);
        writeRowCompact(texOut, "NN-Flex 2-Regret (w=0.5)", nnFlex2RegretWSum);

        writeRowCompact(texOut, "Local Greedy Node (Greedy Cycle)", localGreedyNodeScores_Greedy);
        writeRowCompact(texOut, "Local Greedy Edge (Greedy Cycle)", localGreedyEdgeScores_Greedy);
        writeRowCompact(texOut, "Local Steepest Node (Greedy Cycle)", localSteepestNodeScores_Greedy);
        writeRowCompact(texOut, "Local Steepest Edge (Greedy Cycle)", localSteepestEdgeScores_Greedy);

        writeRowCompact(texOut, "Local Greedy Node (Random Path)", localGreedyNodeScores_Rand);
        writeRowCompact(texOut, "Local Greedy Edge (Random Path)", localGreedyEdgeScores_Rand);
        writeRowCompact(texOut, "Local Steepest Node (Random Path)", localSteepestNodeScores_Rand);
        writeRowCompact(texOut, "Local Steepest Edge (Random Path)", localSteepestEdgeScores_Rand);
        texOut << "\\hline\n\\end{tabular}\n\\caption{Average, min, and max objective values for " << tsp_type << "}\n\\label{tab:" << tsp_type << "_scores}\n\\end{table}\n";
        texOut.close();

        // Timing Table
        texTimeOut << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMethod & Time (avg, min, max) [s] \\\\\n\\hline\n";
        writeRowTime(texTimeOut, "Random Path", randTimes_ms);
        writeRowTime(texTimeOut, "Nearest Neighbor (End)", nnEndTimes_ms);
        writeRowTime(texTimeOut, "Nearest Neighbor (Flexible)", nnFlexTimes_ms);
        writeRowTime(texTimeOut, "Greedy Cycle", greedyTimes_ms);
        writeRowTime(texTimeOut, "Greedy 2-Regret", greedy2RegretTimes_ms);
        writeRowTime(texTimeOut, "Greedy 2-Regret (w=0.5)", greedy2RegretWSumTimes_ms);
        writeRowTime(texTimeOut, "NN-Flex 2-Regret", nnFlex2RegretTimes_ms);
        writeRowTime(texTimeOut, "NN-Flex 2-Regret (w=0.5)", nnFlex2RegretWSumTimes_ms);

        writeRowTime(texTimeOut, "Local Greedy Node (Greedy Cycle)", localGreedyNodeTimes_Greedy_ms);
        writeRowTime(texTimeOut, "Local Greedy Edge (Greedy Cycle)", localGreedyEdgeTimes_Greedy_ms);
        writeRowTime(texTimeOut, "Local Steepest Node (Greedy Cycle)", localSteepestNodeTimes_Greedy_ms);
        writeRowTime(texTimeOut, "Local Steepest Edge (Greedy Cycle)", localSteepestEdgeTimes_Greedy_ms);

        writeRowTime(texTimeOut, "Local Greedy Node (Random Path)", localGreedyNodeTimes_Rand_ms);
        writeRowTime(texTimeOut, "Local Greedy Edge (Random Path)", localGreedyEdgeTimes_Rand_ms);
        writeRowTime(texTimeOut, "Local Steepest Node (Random Path)", localSteepestNodeTimes_Rand_ms);
        writeRowTime(texTimeOut, "Local Steepest Edge (Random Path)", localSteepestEdgeTimes_Rand_ms);
        texTimeOut << "\\hline\n\\end{tabular}\n\\caption{Average, minimum, and maximum execution times for " << tsp_type << "}\n\\label{tab:" << tsp_type << "_timings}\n\\end{table}\n";
        texTimeOut.close();

        cout << "Results and timing tables generated for " << tsp_type << endl;
        cout << "-> " << texFile << endl;
        cout << "-> " << texTimeFile << endl;
    }

    cout << "\nAll TSP datasets processed!" << endl;
    return 0;
}
