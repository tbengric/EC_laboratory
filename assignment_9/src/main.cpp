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
#include "lns.h"
#include "timer.h"
#include "hybrid.h"

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

void printPath(const vector<int>& path) {
    for (int node : path) cout << node << " ";
    cout << endl;
}

void writeRowInt(ofstream& out, const string& name, const vector<int>& vals) {
    if (vals.empty()) return;
    int minv = *min_element(vals.begin(), vals.end());
    int maxv = *max_element(vals.begin(), vals.end());
    double avgv = average_ints(vals);
    out << fixed << setprecision(2);
    out << name << " & " << avgv << " (" << minv << ", " << maxv << ") \\\\\n";
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
    out << name << " & " << avgv << " (" << minv << ", " << maxv << ") \\\\\n";
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

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------


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
                    auto localPath = steepest_local_search_new(randPath, distanceMatrix, "edge", nodes);
                    
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
        // Perturbation : Selects 4 random cut points and reconnects segments in a different order
        auto perturbation = [&](const vector<int>& path) -> vector<int> {
            vector<int> perturbed = path;
            int n = perturbed.size();
            if (n < 8) return perturbed; // Need at least 8 nodes
            
            // Select 4 random cut points, ensuring they are distinct and ordered
            set<int> cut_set;
            while (cut_set.size() < 4) {
                cut_set.insert(1 + rand() % (n - 2)); // Avoid first and last positions
            }
            vector<int> cuts(cut_set.begin(), cut_set.end());
            sort(cuts.begin(), cuts.end());
            
            // Four segments: A=[0,cuts[0]), B=[cuts[0],cuts[1]), C=[cuts[1],cuts[2]), D=[cuts[2],n)
            // Double-bridge: reconnect as A-C-B-D instead of A-B-C-D
            vector<int> result;
            
            // Segment A: [0, cuts[0])
            for (int i = 0; i < cuts[0]; ++i) {
                result.push_back(perturbed[i]);
            }
            // Segment C: [cuts[1], cuts[2])
            for (int i = cuts[1]; i < cuts[2]; ++i) {
                result.push_back(perturbed[i]);
            }
            // Segment B: [cuts[0], cuts[1])
            for (int i = cuts[0]; i < cuts[1]; ++i) {
                result.push_back(perturbed[i]);
            }
            // Segment D: [cuts[2], n)
            for (int i = cuts[2]; i < n; ++i) {
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
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------

        //Run Hybrid Evolutionary Algorithm

        // Best paths
        vector<int> bestHEAPath_LS;
        int bestHEAScore_LS = -1;

        vector<int> bestHEAPath_noLS;
        int bestHEAScore_noLS = -1;

        // Track all runs for statistics
        vector<int> heaScores_true, heaScores_false;
        vector<double> heaTimes_true, heaTimes_false;
        vector<int> localSearchCounts_true, localSearchCounts_false;
        vector<int> mainLoops_true, mainLoops_false;


        for (int run = 0; run < 20; ++run) {
            cout << "HEA Run " << run + 1 << "/20 (with local search)\n";
            Timer hea_timer; hea_timer.tic();

            auto result = hybrid_evolutionary(distanceMatrix, nodes, 20, avg_msls_time_ms, hea_timer, true);
            int score = computeObjective(result.best_path, distanceMatrix, nodes);

            // Store for statistics
            heaScores_true.push_back(score);
            heaTimes_true.push_back(hea_timer.toc_ms());
            localSearchCounts_true.push_back(result.local_search_count);
            mainLoops_true.push_back(result.main_loop);

            // Track the best path for visualization
            if (bestHEAScore_LS == -1 || score < bestHEAScore_LS) {
                bestHEAScore_LS = score;
                bestHEAPath_LS = result.best_path;
            }
        }

        // Repeat for HEA without local search
        for (int run = 0; run < 20; ++run) {
            cout << "HEA Run " << run + 1 << "/20 (without local search)\n";
            Timer hea_timer; hea_timer.tic();

            auto result = hybrid_evolutionary(distanceMatrix, nodes, 20, avg_msls_time_ms, hea_timer, false);
            int score = computeObjective(result.best_path, distanceMatrix, nodes);

            // Store for statistics
            heaScores_false.push_back(score);
            heaTimes_false.push_back(hea_timer.toc_ms());
            localSearchCounts_false.push_back(result.local_search_count);
            mainLoops_false.push_back(result.main_loop);

            // Track the best path for visualization
            if (bestHEAScore_noLS == -1 || score < bestHEAScore_noLS) {
                bestHEAScore_noLS = score;
                bestHEAPath_noLS = result.best_path;
            }
        }
        
        // --- Save best paths for visualization ---
        string visDir = "../visualization";
        fs::create_directories(visDir);
        string visFile = visDir + "/" + tsp_type + "_paths.csv";
        delete_content_file(visFile);
        saveResults(visFile, nodes, bestHEAPath_LS, "HEA_with_LS_Best");
        saveResults(visFile, nodes, bestHEAPath_noLS, "HEA_without_LS_Best");
        saveResults(visFile, nodes, bestILSPath, "ILS (20 runs, time-limited)");

        // --- LaTeX tables ---
        string resultsDir = "../results";

        // --- HEA with LS results ---
        string texFileHEA_LS = resultsDir + "/" + tsp_type + "_hea_with_ls_results_table.tex";
        ofstream texOutHEA_LS(texFileHEA_LS);
        texOutHEA_LS << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMetric & Avg (Min, Max) \\\\\n\\hline\n";
        writeRowInt(texOutHEA_LS, "Objective Score", heaScores_true);
        writeRowInt(texOutHEA_LS, "Objective Score for ILS", ilsScores);
        writeRowInt(texOutHEA_LS, "Local Search Count", localSearchCounts_true);
        writeRowInt(texOutHEA_LS, "Main Loop Iterations", mainLoops_true);
        texOutHEA_LS << "\\hline\n\\end{tabular}\n\\caption{HEA with local search for " << tsp_type << "}\n";
        texOutHEA_LS << "\\label{tab:" << tsp_type << "_hea_with_ls_scores}\n\\end{table}\n";
        texOutHEA_LS.close();

        // Timing table
        string texTimeFileHEA_LS = resultsDir + "/" + tsp_type + "_hea_with_ls_timings_table.tex";
        ofstream texTimeOutHEA_LS(texTimeFileHEA_LS);
        texTimeOutHEA_LS << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMetric & Time (avg, min, max) [s] \\\\\n\\hline\n";
        writeRowDouble(texTimeOutHEA_LS, "Execution Time", heaTimes_true);
        writeRowDouble(texTimeOutHEA_LS, "ILS (20 runs, time-limited)", ilsTimes_ms);
        texTimeOutHEA_LS << "\\hline\n\\end{tabular}\n\\caption{Execution times for HEA with local search for " << tsp_type << "}\n";
        texTimeOutHEA_LS << "\\label{tab:" << tsp_type << "_hea_with_ls_timings}\n\\end{table}\n";
        texTimeOutHEA_LS.close();

        // --- HEA without LS results ---
        string texFileHEA_noLS = resultsDir + "/" + tsp_type + "_hea_without_ls_results_table.tex";
        ofstream texOutHEA_noLS(texFileHEA_noLS);
        texOutHEA_noLS << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMetric & Avg (Min, Max) \\\\\n\\hline\n";
        writeRowInt(texOutHEA_noLS, "Objective Score", heaScores_false);
        writeRowInt(texOutHEA_noLS, "Local Search Count", localSearchCounts_false);
        writeRowInt(texOutHEA_noLS, "Main Loop Iterations", mainLoops_false);
        texOutHEA_noLS << "\\hline\n\\end{tabular}\n\\caption{HEA without local search for " << tsp_type << "}\n";
        texOutHEA_noLS << "\\label{tab:" << tsp_type << "_hea_without_ls_scores}\n\\end{table}\n";
        texOutHEA_noLS.close();

        // Timing table
        string texTimeFileHEA_noLS = resultsDir + "/" + tsp_type + "_hea_without_ls_timings_table.tex";
        ofstream texTimeOutHEA_noLS(texTimeFileHEA_noLS);
        texTimeOutHEA_noLS << "\\begin{table}[h!]\n\\centering\n\\begin{tabular}{lc}\n\\hline\nMetric & Time (avg, min, max) [s] \\\\\n\\hline\n";
        writeRowDouble(texTimeOutHEA_noLS, "Execution Time", heaTimes_false);
        texTimeOutHEA_noLS << "\\hline\n\\end{tabular}\n\\caption{Execution times for HEA without local search for " << tsp_type << "}\n";
        texTimeOutHEA_noLS << "\\label{tab:" << tsp_type << "_hea_without_ls_timings}\n\\end{table}\n";
        texTimeOutHEA_noLS.close();


    }


    cout << "\nAll TSP datasets processed!" << endl;
    return 0;
}
