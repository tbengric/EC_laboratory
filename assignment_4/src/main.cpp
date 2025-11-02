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
#include <map>
#include "node.h"
#include "distance_matrix.h"
#include "heuristics.h"
#include "localsearch.h"
#include "localsearch_candidates.h"

using namespace std;
namespace fs = std::filesystem;

// --- Timer ---
struct Timer {
    std::chrono::high_resolution_clock::time_point start;

    void tic() {
        start = std::chrono::high_resolution_clock::now();
    }

    double toc_ms() const {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::micro>(end - start);
        return duration.count(); // microseconds
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
        vector<double> randTimes_ms;

        vector<int> localSteepestEdgeScores_Rand;
        vector<double> localSteepestEdgeTimes_Rand_ms;

        vector<int> bestRandPath;
        vector<int> bestLocalSteepestEdge_Rand;

        int bestRandScore = -1;
        int bestLocalSteepestEdgeScore_Rand = -1;

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
        printPath(bestRandPath);

        // --- Local search function ---
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

        // --- Candidates local search maps ---
        vector<int> k_values = {5,10,15};
        map<int, vector<int>> candidatesScores_Rand;
        map<int, vector<double>> candidatesTimes_Rand_ms;
        map<int, vector<int>> bestCandidatesPaths_Rand;
        map<int, int> bestCandidatesScores_Rand;

        int max_start_nodes = min<int>(200, static_cast<int>(nodes.size()));
        for (int id_starting_node = 0; id_starting_node < max_start_nodes; ++id_starting_node) {
            cout << "Starting from node: " << id_starting_node << endl;

            // Local searches on Random path (steepest edge)
            applyLocal(bestRandPath, "steepest", "edge",
                       localSteepestEdgeScores_Rand, localSteepestEdgeTimes_Rand_ms,
                       bestLocalSteepestEdge_Rand, bestLocalSteepestEdgeScore_Rand);

            // Candidates local search k=5,10,15
            for (int k : k_values) {
                Timer tloc; tloc.tic();
                auto newPath = steepest_local_search_candidates(bestRandPath, distanceMatrix, nodes, k);
                double dtloc = tloc.toc_ms();

                if (!newPath.empty()) {
                    int cost = computeObjective(newPath, distanceMatrix, nodes);
                    candidatesScores_Rand[k].push_back(cost);
                    candidatesTimes_Rand_ms[k].push_back(dtloc);

                    if (bestCandidatesScores_Rand.find(k) == bestCandidatesScores_Rand.end() || cost < bestCandidatesScores_Rand[k]) {
                        bestCandidatesScores_Rand[k] = cost;
                        bestCandidatesPaths_Rand[k] = newPath;
                    }

                }
            }
        }

        // --- Save best paths for visualization ---
        string visDir = "../visualization";
        fs::create_directories(visDir);
        string visFile = visDir + "/" + tsp_type + "_paths.csv";
        delete_content_file(visFile);

        saveResults(visFile, nodes, bestRandPath, "Random Search");
        saveResults(visFile, nodes, bestLocalSteepestEdge_Rand, "Local Steepest Edge (Random Path)");

        for (int k : k_values) {
            string label = "Local Steepest Edge (Candidates, k=" + to_string(k) + ", Random Path)";
            saveResults(visFile, nodes, bestCandidatesPaths_Rand[k], label);
        }

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
        writeRowCompact(texOut, "Local Steepest Edge (Random Path)", localSteepestEdgeScores_Rand);
        for (int k : k_values) {
            string label = "Local Steepest Edge (Candidates, k=" + to_string(k) + ", Random Path)";
            writeRowCompact(texOut, label, candidatesScores_Rand[k]);
        }
        texOut << "\\hline\n\\end{tabular}\n\\caption{Average, min, and max objective values for " << tsp_type << "}\n\\label{tab:" << tsp_type << "_scores}\n\\end{table}\n";
        texOut.close();

        // Timing Table
        texTimeOut << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMethod & Time (avg, min, max) [s] \\\\\n\\hline\n";
        writeRowTime(texTimeOut, "Random Path", randTimes_ms);
        writeRowTime(texTimeOut, "Local Steepest Edge (Random Path)", localSteepestEdgeTimes_Rand_ms);
        for (int k : k_values) {
            string label = "Local Steepest Edge (Candidates, k=" + to_string(k) + ", Random Path)";
            writeRowTime(texTimeOut, label, candidatesTimes_Rand_ms[k]);
        }
        texTimeOut << "\\hline\n\\end{tabular}\n\\caption{Average, minimum, and maximum execution times for " << tsp_type << "}\n\\label{tab:" << tsp_type << "_timings}\n\\end{table}\n";
        texTimeOut.close();

        cout << "Results and timing tables generated for " << tsp_type << endl;
        cout << "-> " << texFile << endl;
        cout << "-> " << texTimeFile << endl;
    }

    cout << "\nAll TSP datasets processed!" << endl;
    return 0;
}
