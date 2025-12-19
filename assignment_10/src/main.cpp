#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include "distance_matrix.h"
#include "heuristics.h"
#include "localsearch.h"
#include "node.h"
#include "timer.h"

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

double average_ints(const vector<int>& values) {
    if (values.empty()) return 0.0;
    return accumulate(values.begin(), values.end(), 0.0) / values.size();
}

double average_doubles(const vector<double>& values) {
    if (values.empty()) return 0.0;
    return accumulate(values.begin(), values.end(), 0.0) / values.size();
}

void writeRowInt(ofstream& out, const string& name, const vector<int>& vals) {
    if (vals.empty()) return;
    int minv = *min_element(vals.begin(), vals.end());
    int maxv = *max_element(vals.begin(), vals.end());
    double avgv = average_ints(vals);
    out << fixed << setprecision(2);
    out << name << " & " << avgv << " (" << minv << ", " << maxv << ") \\\n";
}

void writeRowDouble(ofstream& out, const string& name, const vector<double>& vals_ms) {
    if (vals_ms.empty()) return;
    vector<double> vals_s;
    vals_s.reserve(vals_ms.size());
    for (double v : vals_ms) vals_s.push_back(v / 1000.0); // convert ms -> s
    double minv = *min_element(vals_s.begin(), vals_s.end());
    double maxv = *max_element(vals_s.begin(), vals_s.end());
    double avgv = average_doubles(vals_s);
    out << fixed << setprecision(4);
    out << name << " & " << avgv << " (" << minv << ", " << maxv << ") \\\n";
}


// --- Main program ---
int main() {
    const double ILS_TIME_LIMIT_MS = 20000000.0; 
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

        // --- ILS scores and times ---
        vector<int> ilsScores;
        vector<double> ilsTimes_ms;
        vector<int> ilsLocalSearchCounts;
        vector<int> bestILSPath;
        int bestILSScore = -1;

        auto updateBest = [](int& bestScore, vector<int>& bestPath, const vector<int>& path, int score) {
            if (path.empty()) return;
            if (bestScore == -1 || score < bestScore) {
                bestScore = score;
                bestPath = path;
            }
        };

        // Generate several initial solutions from different greedy variants and a random start.
        auto generateInitialSolutions = [&]() {
            vector<vector<int>> seeds;

            // deterministic spread of start nodes
            int n = static_cast<int>(nodes.size());
            vector<int> startNodes;
            if (n > 0) {
                startNodes.push_back(0);
                startNodes.push_back(n / 3);
                startNodes.push_back((2 * n) / 3);
                startNodes.push_back(n - 1);
                sort(startNodes.begin(), startNodes.end());
                startNodes.erase(unique(startNodes.begin(), startNodes.end()), startNodes.end());
            }

            // random baseline
            vector<int> selectedNodes = selectNodes(nodes.size());
            seeds.push_back(randomSolution(selectedNodes));

            for (int s : startNodes) {
                seeds.push_back(nearestNeighborEnd(distanceMatrix, nodes, s));
                seeds.push_back(nearestNeighborFlexible(distanceMatrix, nodes, s));
                seeds.push_back(greedyCycle(distanceMatrix, nodes, s));
                seeds.push_back(greedyCycleKRegretWeighted(distanceMatrix, nodes, s, 2, 0.6));
                seeds.push_back(nearestNeighborKRegretWeighted(distanceMatrix, nodes, s, 2, 0.6));
            }

            return seeds;
        };

        // --- ILS: perturbation (double-bridge style) ---
        auto perturbation = [&](const vector<int>& path) -> vector<int> {
            vector<int> perturbed = path;
            int n = static_cast<int>(perturbed.size());
            if (n < 8) return perturbed; // need enough nodes

            set<int> cut_set;
            while (static_cast<int>(cut_set.size()) < 4) {
                cut_set.insert(1 + rand() % (n - 2));
            }
            vector<int> cuts(cut_set.begin(), cut_set.end());
            sort(cuts.begin(), cuts.end());

            vector<int> result;
            for (int i = 0; i < cuts[0]; ++i) result.push_back(perturbed[i]);
            for (int i = cuts[1]; i < cuts[2]; ++i) result.push_back(perturbed[i]);
            for (int i = cuts[0]; i < cuts[1]; ++i) result.push_back(perturbed[i]);
            for (int i = cuts[2]; i < n; ++i) result.push_back(perturbed[i]);

            return result;
        };

        // --- ILS runs ---
        for (int ils_run = 0; ils_run < 20; ++ils_run) {
            cout << "ILS Run: " << ils_run + 1 << "/20" << endl;
            // multi-start: evaluate several seeds, keep the best after a local search pass
            vector<vector<int>> seeds = generateInitialSolutions();
            int best_run_score = -1;
            vector<int> best_run_path;
            int local_search_count = 0;

            for (auto& seed : seeds) {
                if (seed.empty()) continue;
                seed = steepest_local_search(seed, distanceMatrix, "edge", nodes);
                local_search_count++;
                if (seed.empty()) continue;
                int score = computeObjective(seed, distanceMatrix, nodes);
                if (best_run_score == -1 || score < best_run_score) {
                    best_run_score = score;
                    best_run_path = seed;
                }
            }

            // If nothing valid was generated, skip this run
            if (best_run_path.empty()) continue;

            // Start timing after seeding to measure only perturbation + local search phase
            Timer ils_timer;
            ils_timer.tic();
            vector<int> x = best_run_path;

            while (ils_timer.toc_ms() < ILS_TIME_LIMIT_MS) {
                vector<int> y = perturbation(x);
                if (!y.empty()) {
                    y = steepest_local_search(y, distanceMatrix, "edge", nodes);
                    local_search_count++;

                    if (!y.empty()) {
                        int y_score = computeObjective(y, distanceMatrix, nodes);
                        if (best_run_score == -1 || y_score < best_run_score) {
                            x = y;
                            best_run_score = y_score;
                            best_run_path = y;
                        }
                    }
                }
            }

            double ils_time = ils_timer.toc_ms();

            if (best_run_score != -1) {
                ilsScores.push_back(best_run_score);
                ilsTimes_ms.push_back(ils_time);
                ilsLocalSearchCounts.push_back(local_search_count);
                updateBest(bestILSScore, bestILSPath, best_run_path, best_run_score);
            }

            cout << "  Local searches performed: " << local_search_count << endl;
        }

        // --- Save best path for visualization ---
        string visDir = "../visualization";
        fs::create_directories(visDir);
        string visFile = visDir + "/" + tsp_type + "_paths.csv";
        delete_content_file(visFile);
        saveResults(visFile, nodes, bestILSPath, "ILS_best");

        // --- LaTeX tables for ILS ---
        string resultsDir = "../results";
        fs::create_directories(resultsDir);

        string texFileILS = resultsDir + "/" + tsp_type + "_ils_results_table.tex";
        ofstream texOutILS(texFileILS);
        texOutILS << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMetric & Avg (Min, Max) \\\\n\\hline\n";
        writeRowInt(texOutILS, "Objective Score", ilsScores);
        writeRowInt(texOutILS, "Local Search Count", ilsLocalSearchCounts);
        texOutILS << "\\hline\n\\end{tabular}\n\\caption{ILS results for " << tsp_type << "}\n";
        texOutILS << "\\label{tab:" << tsp_type << "_ils_scores}\n\\end{table}\n";
        texOutILS.close();

        string texTimeFileILS = resultsDir + "/" + tsp_type + "_ils_timings_table.tex";
        ofstream texTimeOutILS(texTimeFileILS);
        texTimeOutILS << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMetric & Time (avg, min, max) [s] \\\\n\\hline\n";
        writeRowDouble(texTimeOutILS, "Execution Time", ilsTimes_ms);
        texTimeOutILS << "\\hline\n\\end{tabular}\n\\caption{ILS execution times for " << tsp_type << "}\n";
        texTimeOutILS << "\\label{tab:" << tsp_type << "_ils_timings}\n\\end{table}\n";
        texTimeOutILS.close();
    }

    cout << "\nAll TSP datasets processed!" << endl;
    return 0;
}
