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
#include <list>
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

        vector<int> localSteepestEdgeLMScores_Rand;
        vector<double> localSteepestEdgeLMTimes_Rand_ms;

        // MSLS vectors
        vector<int> mslsScores;
        vector<double> mslsTimes_ms;

        // ILS vectors
        vector<int> ilsScores;
        vector<double> ilsTimes_ms;
        vector<int> ilsLocalSearchCounts;

        vector<int> bestRandPath;
        vector<int> bestLocalSteepestEdge_Rand;
        vector<int> bestLocalSteepestEdgeLM_Rand;
        vector<int> bestMSLSPath;
        vector<int> bestILSPath;

        int bestRandScore = -1;
        int bestLocalSteepestEdgeScore_Rand = -1;
        int bestLocalSteepestEdgeLMScore_Rand = -1;
        int bestMSLSScore = -1;
        int bestILSScore = -1;

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

        // --- List of moves for steepest_local_search_with_lm (persists across all iterations)
        list<SavedMove> list_of_moves_persistent;

        // --- MSLS: Run 20 times, each with 200 iterations ---
        for (int msls_run = 0; msls_run < 20; ++msls_run) {
            cout << "MSLS Run: " << msls_run + 1 << "/20" << endl;
            Timer msls_timer;
            msls_timer.tic();
            
            vector<int> best_msls_path;
            int best_msls_score = -1;
            
            // Perform 200 iterations of basic local search
            for (int iter = 0; iter < 200; ++iter) {
                // Generate random starting solution
                vector<int> selectedNodes = selectNodes(nodes.size());
                auto randPath = randomSolution(selectedNodes);
                
                // Apply steepest local search with edge exchange
                if (!randPath.empty()) {
                    auto localPath = steepest_local_search(randPath, distanceMatrix, "edge", nodes);
                    
                    if (!localPath.empty()) {
                        int localScore = computeObjective(localPath, distanceMatrix, nodes);
                        
                        // Update best solution for this MSLS run
                        if (best_msls_score == -1 || localScore < best_msls_score) {
                            best_msls_score = localScore;
                            best_msls_path = localPath;
                        }
                    }
                }
            }
            
            double msls_time = msls_timer.toc_ms();
            
            // Store results for this MSLS run
            if (best_msls_score != -1) {
                mslsScores.push_back(best_msls_score);
                mslsTimes_ms.push_back(msls_time);
                updateBest(bestMSLSScore, bestMSLSPath, best_msls_path, best_msls_score);
            }
        }

        // Calculate average MSLS time for ILS stopping condition
        double avg_msls_time_ms = 0.0;
        if (!mslsTimes_ms.empty()) {
            avg_msls_time_ms = average_doubles(mslsTimes_ms);
        }
        cout << "Average MSLS time: " << avg_msls_time_ms / 1000.0 << " ms" << endl;

        // --- ILS: Run 20 times with time-based stopping condition ---
        // Perturbation strategy: Double-bridge move (4-opt)
        // This removes 4 edges and reconnects the path in a different way
        // More disruptive than simple 2-opt, helping escape local optima
        auto perturbation = [&](const vector<int>& path) -> vector<int> {
            vector<int> perturbed = path;
            int n = perturbed.size();
            if (n < 8) return perturbed; // Need at least 8 nodes for double-bridge
            
            // Select 4 random cut points that divide the tour into 4 segments
            // Make sure they are in increasing order and sufficiently spaced
            vector<int> cuts;
            cuts.push_back(rand() % (n / 4));
            cuts.push_back(n / 4 + rand() % (n / 4));
            cuts.push_back(n / 2 + rand() % (n / 4));
            cuts.push_back(3 * n / 4 + rand() % (n / 4));
            
            // Double-bridge: reconnect segments in order A-C-B-D instead of A-B-C-D
            vector<int> result;
            // Segment A: [0, cuts[0])
            for (int i = 0; i < cuts[0]; ++i) {
                result.push_back(perturbed[i]);
            }
            // Segment C: [cuts[2], cuts[3])
            for (int i = cuts[2]; i < cuts[3]; ++i) {
                result.push_back(perturbed[i]);
            }
            // Segment B: [cuts[0], cuts[2])
            for (int i = cuts[0]; i < cuts[2]; ++i) {
                result.push_back(perturbed[i]);
            }
            // Segment D: [cuts[3], n)
            for (int i = cuts[3]; i < n; ++i) {
                result.push_back(perturbed[i]);
            }
            
            return result;
        };

        for (int ils_run = 0; ils_run < 20; ++ils_run) {
            cout << "ILS Run: " << ils_run + 1 << "/20" << endl;
            Timer ils_timer;
            ils_timer.tic();
            
            // Generate initial random solution
            vector<int> selectedNodes = selectNodes(nodes.size());
            vector<int> x = randomSolution(selectedNodes);
            
            // Apply initial local search
            if (!x.empty()) {
                x = steepest_local_search(x, distanceMatrix, "edge", nodes);
            }
            
            int best_ils_score = -1;
            vector<int> best_ils_path = x;
            if (!x.empty()) {
                best_ils_score = computeObjective(x, distanceMatrix, nodes);
            }
            
            int local_search_count = 1; // Count initial local search
            
            // ILS main loop: continue until time limit (avg MSLS time)
            while (ils_timer.toc_ms() < avg_msls_time_ms) {
                // Perturbation
                vector<int> y = perturbation(x);
                
                // Local search on perturbed solution
                if (!y.empty()) {
                    y = steepest_local_search(y, distanceMatrix, "edge", nodes);
                    local_search_count++;
                    
                    if (!y.empty()) {
                        int y_score = computeObjective(y, distanceMatrix, nodes);
                        
                        // Acceptance criterion: accept if better
                        if (best_ils_score == -1 || y_score < best_ils_score) {
                            x = y;
                            best_ils_score = y_score;
                            best_ils_path = y;
                        }
                    }
                }
            }
            
            double ils_time = ils_timer.toc_ms();
            
            // Store results for this ILS run
            if (best_ils_score != -1) {
                ilsScores.push_back(best_ils_score);
                ilsTimes_ms.push_back(ils_time);
                ilsLocalSearchCounts.push_back(local_search_count);
                updateBest(bestILSScore, bestILSPath, best_ils_path, best_ils_score);
            }
            
            cout << "  Local searches performed: " << local_search_count << endl;
        }

        int max_start_nodes = min<int>(200, static_cast<int>(nodes.size()));
        for (int id_starting_node = 0; id_starting_node < max_start_nodes; ++id_starting_node) {
            cout << "Starting from node: " << id_starting_node << endl;
            vector<int> selectedNodes = selectNodes(nodes.size());
            auto randPath = randomSolution(selectedNodes);

            // Local searches on Random path (steepest edge)
            applyLocal(randPath, "steepest", "edge",
                       localSteepestEdgeScores_Rand, localSteepestEdgeTimes_Rand_ms,
                       bestLocalSteepestEdge_Rand, bestLocalSteepestEdgeScore_Rand);

            // Local searches on Random path (steepest edge with list of moves)
            if (!randPath.empty()) {
                Timer tloc_lm; tloc_lm.tic();
                auto newPath_lm = steepest_local_search_with_lm(randPath, distanceMatrix, "edge", nodes, list_of_moves_persistent);
                double dtloc_lm = tloc_lm.toc_ms();
                if (!newPath_lm.empty()) {
                    int cost_lm = computeObjective(newPath_lm, distanceMatrix, nodes);
                    localSteepestEdgeLMScores_Rand.push_back(cost_lm);
                    localSteepestEdgeLMTimes_Rand_ms.push_back(dtloc_lm);
                    updateBest(bestLocalSteepestEdgeLMScore_Rand, bestLocalSteepestEdgeLM_Rand, newPath_lm, cost_lm);
                }
            }

            // Candidates local search k=5,10,15
            for (int k : k_values) {
                Timer tloc; tloc.tic();
                auto newPath = steepest_local_search_candidates(randPath, distanceMatrix, nodes, k);
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
        saveResults(visFile, nodes, bestLocalSteepestEdgeLM_Rand, "Local Steepest Edge with LM (Random Path)");
        saveResults(visFile, nodes, bestMSLSPath, "MSLS (20 runs x 200 iterations)");
        saveResults(visFile, nodes, bestILSPath, "ILS (20 runs, time-limited)");

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
        writeRowCompact(texOut, "Local Steepest Edge with LM (Random Path)", localSteepestEdgeLMScores_Rand);
        writeRowCompact(texOut, "MSLS (20 runs x 200 iterations)", mslsScores);
        writeRowCompact(texOut, "ILS (20 runs, time-limited)", ilsScores);
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
        writeRowTime(texTimeOut, "Local Steepest Edge with LM (Random Path)", localSteepestEdgeLMTimes_Rand_ms);
        writeRowTime(texTimeOut, "MSLS (20 runs x 200 iterations)", mslsTimes_ms);
        writeRowTime(texTimeOut, "ILS (20 runs, time-limited)", ilsTimes_ms);
        for (int k : k_values) {
            string label = "Local Steepest Edge (Candidates, k=" + to_string(k) + ", Random Path)";
            writeRowTime(texTimeOut, label, candidatesTimes_Rand_ms[k]);
        }
        texTimeOut << "\\hline\n\\end{tabular}\n\\caption{Average, minimum, and maximum execution times for " << tsp_type << "}\n\\label{tab:" << tsp_type << "_timings}\n\\end{table}\n";
        texTimeOut.close();

        // ILS Local Search Count Table
        if (!ilsLocalSearchCounts.empty()) {
            string texILSCountFile = resultsDir + "/" + tsp_type + "_ils_counts_table.tex";
            ofstream texILSCountOut(texILSCountFile);
            int minCount = *min_element(ilsLocalSearchCounts.begin(), ilsLocalSearchCounts.end());
            int maxCount = *max_element(ilsLocalSearchCounts.begin(), ilsLocalSearchCounts.end());
            double avgCount = average_ints(ilsLocalSearchCounts);
            
            texILSCountOut << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\n";
            texILSCountOut << "Metric & Value \\\\\n\\hline\n";
            texILSCountOut << fixed << setprecision(2);
            texILSCountOut << "Average Local Searches per ILS Run & " << avgCount << " \\\\\n";
            texILSCountOut << "Min Local Searches & " << minCount << " \\\\\n";
            texILSCountOut << "Max Local Searches & " << maxCount << " \\\\\n";
            texILSCountOut << "\\hline\n\\end{tabular}\n\\caption{ILS local search counts for " << tsp_type << "}\n";
            texILSCountOut << "\\label{tab:" << tsp_type << "_ils_counts}\n\\end{table}\n";
            texILSCountOut.close();
            cout << "-> " << texILSCountFile << endl;
        }

        cout << "Results and timing tables generated for " << tsp_type << endl;
        cout << "-> " << texFile << endl;
        cout << "-> " << texTimeFile << endl;
    }

    cout << "\nAll TSP datasets processed!" << endl;
    return 0;
}
