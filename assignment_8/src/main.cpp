#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <filesystem>
#include <set>
#include "node.h"
#include "distance_matrix.h"
#include "heuristics.h"
#include "localsearch.h"
#include "similarity_analysis.h"

using namespace std;
namespace fs = std::filesystem;

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

        // --- Generate 1000 random local optima using greedy local search ---
        cout << "Generating 1000 random local optima..." << endl;
        vector<LocalOptimum> localOptima;
        
        for (int i = 0; i < 1000; ++i) {
            if ((i + 1) % 100 == 0) {
                cout << "Generated " << i + 1 << "/1000 local optima" << endl;
            }
            
            vector<int> selectedNodes = selectNodes(nodes.size());
            auto randPath = randomSolution(selectedNodes);
            
            if (!randPath.empty()) {
                // Apply greedy local search
                auto localPath = greedy_local_search(randPath, distanceMatrix, "edge", nodes);
                int objective = computeObjective(localPath, distanceMatrix, nodes);
                
                localOptima.push_back({localPath, objective});
            }
        }

        // Find the best local optimum
        int bestIdx = 0;
        for (size_t i = 1; i < localOptima.size(); ++i) {
            if (localOptima[i].objective < localOptima[bestIdx].objective) {
                bestIdx = i;
            }
        }
        
        cout << "Best local optimum has objective: " << localOptima[bestIdx].objective << endl;

        // Generate very good solution using ILS (best method so far)
        cout << "Generating very good solution using ILS..." << endl;
        vector<int> selectedNodes = selectNodes(nodes.size());
        vector<int> ilsSolution = randomSolution(selectedNodes);
        ilsSolution = steepest_local_search(ilsSolution, distanceMatrix, "edge", nodes);
        
        // Simple ILS with 100 iterations
        auto perturbation = [&](const vector<int>& path) -> vector<int> {
            vector<int> perturbed = path;
            int n = perturbed.size();
            if (n < 8) return perturbed;
            
            set<int> cut_set;
            while (cut_set.size() < 4) {
                cut_set.insert(1 + rand() % (n - 2));
            }
            vector<int> cuts(cut_set.begin(), cut_set.end());
            sort(cuts.begin(), cuts.end());
            
            vector<int> result;
            for (int idx = 0; idx < cuts[0]; ++idx) result.push_back(perturbed[idx]);
            for (int idx = cuts[1]; idx < cuts[2]; ++idx) result.push_back(perturbed[idx]);
            for (int idx = cuts[0]; idx < cuts[1]; ++idx) result.push_back(perturbed[idx]);
            for (int idx = cuts[2]; idx < n; ++idx) result.push_back(perturbed[idx]);
            
            return result;
        };
        
        int bestILSObjective = computeObjective(ilsSolution, distanceMatrix, nodes);
        for (int iter = 0; iter < 100; ++iter) {
            vector<int> perturbed = perturbation(ilsSolution);
            perturbed = steepest_local_search(perturbed, distanceMatrix, "edge", nodes);
            int obj = computeObjective(perturbed, distanceMatrix, nodes);
            
            if (obj < bestILSObjective) {
                bestILSObjective = obj;
                ilsSolution = perturbed;
            }
        }
        
        cout << "ILS solution has objective: " << bestILSObjective << endl;

        // Create output directories
        string plotsDir = "../visualization/similarity_plots";
        fs::create_directories(plotsDir);

        // Calculate similarities and generate CSV files for plotting
        // 1. Average similarity (edges)
        {
            ofstream out(plotsDir + "/" + tsp_type + "_avg_similarity_edges.csv");
            out << "objective,similarity,type\n";
            
            vector<int> objectives;
            vector<double> similarities;
            
            for (size_t i = 0; i < localOptima.size(); ++i) {
                double avgSim = 0.0;
                for (size_t j = 0; j < localOptima.size(); ++j) {
                    if (i != j) {
                        avgSim += commonEdges(localOptima[i].path, localOptima[j].path);
                    }
                }
                avgSim /= (localOptima.size() - 1);
                
                out << localOptima[i].objective << "," << avgSim << ",avg_edges\n";
                objectives.push_back(localOptima[i].objective);
                similarities.push_back(avgSim);
            }
            
            double corr = correlationCoefficient(objectives, similarities);
            cout << "Correlation (Avg Similarity - Edges): " << corr << endl;
            out.close();
        }

        // 2. Average similarity (nodes)
        {
            ofstream out(plotsDir + "/" + tsp_type + "_avg_similarity_nodes.csv");
            out << "objective,similarity,type\n";
            
            vector<int> objectives;
            vector<double> similarities;
            
            for (size_t i = 0; i < localOptima.size(); ++i) {
                double avgSim = 0.0;
                for (size_t j = 0; j < localOptima.size(); ++j) {
                    if (i != j) {
                        avgSim += commonNodes(localOptima[i].path, localOptima[j].path);
                    }
                }
                avgSim /= (localOptima.size() - 1);
                
                out << localOptima[i].objective << "," << avgSim << ",avg_nodes\n";
                objectives.push_back(localOptima[i].objective);
                similarities.push_back(avgSim);
            }
            
            double corr = correlationCoefficient(objectives, similarities);
            cout << "Correlation (Avg Similarity - Nodes): " << corr << endl;
            out.close();
        }

        // 3. Similarity to best local optimum (edges)
        {
            ofstream out(plotsDir + "/" + tsp_type + "_best_similarity_edges.csv");
            out << "objective,similarity,type\n";
            
            vector<int> objectives;
            vector<double> similarities;
            
            for (size_t i = 0; i < localOptima.size(); ++i) {
                if (i == (size_t)bestIdx) continue; // Skip the best solution itself
                
                int sim = commonEdges(localOptima[i].path, localOptima[bestIdx].path);
                out << localOptima[i].objective << "," << sim << ",best_edges\n";
                objectives.push_back(localOptima[i].objective);
                similarities.push_back(sim);
            }
            
            double corr = correlationCoefficient(objectives, similarities);
            cout << "Correlation (Best Similarity - Edges): " << corr << endl;
            out.close();
        }

        // 4. Similarity to best local optimum (nodes)
        {
            ofstream out(plotsDir + "/" + tsp_type + "_best_similarity_nodes.csv");
            out << "objective,similarity,type\n";
            
            vector<int> objectives;
            vector<double> similarities;
            
            for (size_t i = 0; i < localOptima.size(); ++i) {
                if (i == (size_t)bestIdx) continue;
                
                int sim = commonNodes(localOptima[i].path, localOptima[bestIdx].path);
                out << localOptima[i].objective << "," << sim << ",best_nodes\n";
                objectives.push_back(localOptima[i].objective);
                similarities.push_back(sim);
            }
            
            double corr = correlationCoefficient(objectives, similarities);
            cout << "Correlation (Best Similarity - Nodes): " << corr << endl;
            out.close();
        }

        // 5. Similarity to ILS solution (edges)
        {
            ofstream out(plotsDir + "/" + tsp_type + "_ils_similarity_edges.csv");
            out << "objective,similarity,type\n";
            
            vector<int> objectives;
            vector<double> similarities;
            
            for (size_t i = 0; i < localOptima.size(); ++i) {
                int sim = commonEdges(localOptima[i].path, ilsSolution);
                out << localOptima[i].objective << "," << sim << ",ils_edges\n";
                objectives.push_back(localOptima[i].objective);
                similarities.push_back(sim);
            }
            
            double corr = correlationCoefficient(objectives, similarities);
            cout << "Correlation (ILS Similarity - Edges): " << corr << endl;
            out.close();
        }

        // 6. Similarity to ILS solution (nodes)
        {
            ofstream out(plotsDir + "/" + tsp_type + "_ils_similarity_nodes.csv");
            out << "objective,similarity,type\n";
            
            vector<int> objectives;
            vector<double> similarities;
            
            for (size_t i = 0; i < localOptima.size(); ++i) {
                int sim = commonNodes(localOptima[i].path, ilsSolution);
                out << localOptima[i].objective << "," << sim << ",ils_nodes\n";
                objectives.push_back(localOptima[i].objective);
                similarities.push_back(sim);
            }
            
            double corr = correlationCoefficient(objectives, similarities);
            cout << "Correlation (ILS Similarity - Nodes): " << corr << endl;
            out.close();
        }

        cout << "CSV files generated in " << plotsDir << endl;
    }

    cout << "\nAll similarity analyses completed!" << endl;
    return 0;
}
