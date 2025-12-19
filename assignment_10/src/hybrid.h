#include <stdexcept>
#include <vector>
#include <random>
#include <utility>
#include <vector>
#include <random>
#include <iostream>
#include "localsearch.h"

using namespace std;


bool insert_to_population(
    vector<vector<int>>& population,
    const vector<int>& solution_to_insert,
    const vector<vector<int>>& dist_matrix,
    const vector<Node>& nodes,
    size_t MAX_POP_SIZE
) {
    int score_new = computeObjective(solution_to_insert, dist_matrix, nodes);

    // 1. Check for duplicate solution or duplicate score
    for (const auto& individual : population) {
        if (individual == solution_to_insert) {
            return false;
        }
        if (computeObjective(individual, dist_matrix, nodes) == score_new) {
            return false;
        }
    }

    // 2. If population is not full, just add
    if (population.size() < MAX_POP_SIZE) {
        population.push_back(solution_to_insert);
        return true;
    }

    // 3. Find the worst individual
    int worst_index = 0;
    int worst_score = computeObjective(population[0], dist_matrix, nodes);

    for (size_t i = 1; i < population.size(); ++i) {
        int score = computeObjective(population[i], dist_matrix, nodes);
        if (score > worst_score) {  
            worst_score = score;
            worst_index = i;
        }
    }

    // 4. Replace worst if the new solution is better
    if (score_new < worst_score) {
        population[worst_index] = solution_to_insert;
        return true;
    }

    return false;
}


pair<vector<int>, vector<int>> select_parents(
    const vector<vector<int>>& population,
    mt19937& rng
) {
    uniform_int_distribution<int> distribution(0, population.size() - 1);

    int parent1 = distribution(rng);
    int parent2;
    do {
        parent2 = distribution(rng);
    } while (parent2 == parent1);

    vector<int> path_parent_1 = population[parent1];
    vector<int> path_parent_2 = population[parent2];

    return {path_parent_1, path_parent_2};
}

struct Edge{
    int u, v;
};

#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
using namespace std;

vector<int> operator1(
    const vector<int>& parent1,
    const vector<int>& parent2,
    const vector<Node>& nodes
){
    vector<int> p1 = parent1;
    vector<int> p2 = parent2;

    // Step 0: Remove last element if same as first
    if (!p1.empty() && p1.front() == p1.back()) p1.pop_back();
    if (!p2.empty() && p2.front() == p2.back()) p2.pop_back();

    int n1 = p1.size();
    int n2 = p2.size();

    // Step 1: Identify common nodes and edges
    unordered_set<int> in_p1(p1.begin(), p1.end());
    unordered_set<int> in_p2(p2.begin(), p2.end());

    // Find common nodes
    vector<int> common_nodes;
    for (int v : in_p1) {
        if (in_p2.count(v)) common_nodes.push_back(v);
    }

    // Find common edges
    vector<pair<int,int>> common_edges;
    for (int i = 0; i < n1; i++) {
        int u = p1[i];
        int v = p1[(i + 1) % n1];

        for (int j = 0; j < n2; j++) {
            if (p2[j] == u) {
                int next2 = p2[(j + 1) % n2];
                int prev2 = p2[(j - 1 + n2) % n2];

                if (next2 == v || prev2 == v) {
                    common_edges.emplace_back(u, v);
                }
                break;
            }
        }
    }

    // Step 2: Create subpaths from common edges
    vector<vector<int>> subpaths;
    unordered_set<int> used;
    for (auto &[u, v] : common_edges) {
        if (!used.count(u) && !used.count(v)) {
            subpaths.push_back({u, v});
            used.insert(u);
            used.insert(v);
        } else if (!used.count(u)) {
            for (auto &sp : subpaths) {
                if (sp.back() == v) { sp.push_back(u); used.insert(u); break; }
                if (sp.front() == v) { sp.insert(sp.begin(), u); used.insert(u); break; }
            }
        } else if (!used.count(v)) {
            for (auto &sp : subpaths) {
                if (sp.back() == u) { sp.push_back(v); used.insert(v); break; }
                if (sp.front() == u) { sp.insert(sp.begin(), v); used.insert(v); break; }
            }
        }
    }

    // Step 2b: Add single-node subpaths for common nodes not in any edge
    for (int node : common_nodes) {
        if (!used.count(node)) {
            subpaths.push_back({node});
            used.insert(node);
        }
    }
    // Step 3: Add random nodes 
    vector<int> remaining_nodes;
    for (auto &node : nodes) {
        if (!used.count(node.id)) remaining_nodes.push_back(node.id);
    }

    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    shuffle(remaining_nodes.begin(), remaining_nodes.end(), default_random_engine(seed));

    int target_size = nodes.size() / 2;
    int count_to_add = max(0, target_size - (int)used.size());

    for (int i = 0; i < count_to_add && i < remaining_nodes.size(); i++) {
        subpaths.push_back({remaining_nodes[i]});
        used.insert(remaining_nodes[i]); 
    }

    // Step 4: Randomly connect subpaths
    shuffle(subpaths.begin(), subpaths.end(), default_random_engine(seed));
    vector<int> offspring;
    for (auto &sp : subpaths) {
        if (!offspring.empty() && rand() % 2) {
            offspring.insert(offspring.end(), sp.rbegin(), sp.rend());
        } else {
            offspring.insert(offspring.end(), sp.begin(), sp.end());
        }
    }
    // Step 5: Optional close loop
    if (!offspring.empty()) offspring.push_back(offspring.front());

    return offspring;
};

vector<int> operator2(
    const vector<int>& p1,
    const vector<int>& p2,
    const vector<Node>& nodes,
    const vector<vector<int>>& dist 
) {

    vector<int> parent1 = p1;
    vector<int> parent2 = p2;

    // Remove last element if same as first
    if (!parent1.empty() && parent1.front() == parent1.back()) parent1.pop_back();
    if (!parent2.empty() && parent2.front() == parent2.back()) parent2.pop_back();

    // Step 0: Choose randomly one of the parents as the starting solution
    const vector<int>& base_parent = (rand() % 2) ? parent1 : parent2;
    const vector<int>& other_parent = (&base_parent == &parent1) ? parent2 : parent1;

    // Step 1: Remove nodes not present in the other parent
    unordered_set<int> other_nodes(other_parent.begin(), other_parent.end());
    vector<int> offspring;
    for (int node : base_parent) {
        if (other_nodes.count(node)) {
            offspring.push_back(node);
        }
    }

    // Step 2: Repair solution using heuristic
    if (!offspring.empty()) {
        if (!offspring.empty()) offspring.push_back(offspring.front());
        offspring = repair_solution(offspring, dist, nodes); 
    }

    return offspring;
};

struct HEAResult {
    vector<int> best_path;
    int local_search_count;
    int main_loop;
};

int solution_distance(
    const vector<int>& a,
    const vector<int>& b
) {
    unordered_set<int> set_a(a.begin(), a.end());
    unordered_set<int> set_b(b.begin(), b.end());

    int diff = 0;

    for (int v : set_a)
        if (!set_b.count(v)) diff++;

    for (int v : set_b)
        if (!set_a.count(v)) diff++;

    return diff;
}

HEAResult hybrid_evolutionary(
    const vector<vector<int>>& dist_matrix,
    const vector<Node>& nodes,
    int population_number,
    double avg_msls_time_ms,
    Timer& timer,
    bool use_local_search
) {
    random_device rd;
    mt19937 rng(rd());
    uniform_real_distribution<double> choose_operator(0.0, 1.0);

    vector<vector<int>> population;
    int local_search_count = 0;
    int main_loop = 0;

    int attempts = 0;
    const int MAX_ATTEMPTS = population_number * 100;

    // START OF POPULATION
    while (population.size() < population_number && attempts < MAX_ATTEMPTS) {
        attempts++;

        vector<int> selectedNodes = selectNodes(nodes.size());
        vector<int> randPath = randomSolution(selectedNodes);
        vector<int> newPath = steepest_local_search(randPath, dist_matrix, "edge", nodes);

        local_search_count++;
        insert_to_population(population, newPath, dist_matrix, nodes, 20);
    }

    if (population.size() != population_number) {
        cerr << "Warning: population not full (" << population.size() << "/" << population_number << ")\n";
        throw runtime_error("Hybrid evolutionary: population is not full");
    }

    // MAIN LOOP
    while (timer.toc_ms() < avg_msls_time_ms) {
        main_loop++;

        // SELECT PARENTS
        auto [parent1, parent2] = select_parents(population, rng);

        // CHOOSE RECOMBINATION OPERATOR
        vector<int> offspring;

        if (choose_operator(rng) < 0.5) {
            offspring = operator1(parent1, parent2, nodes);
        } else {
            offspring = operator2(parent1, parent2, nodes, dist_matrix);
        }

        if (offspring.size() != 101) {
            cerr << "Error: offspring length is " << offspring.size() << ", expected 101." << endl;
            throw std::runtime_error("Offspring length is not 101!");
        }

        if (use_local_search) {
            offspring = steepest_local_search(offspring, dist_matrix, "edge", nodes);
            local_search_count++;

            if (offspring.size() != 101) {
                cerr << "Error: offspring length is " << offspring.size() << " after local search, expected 101." << endl;
                throw std::runtime_error("Offspring length is not 101 after local search!");
            }
        }

        bool inserted = insert_to_population(population, offspring, dist_matrix, nodes, 20);
    }

    vector<int> best = population.front();
    int best_score = computeObjective(best, dist_matrix, nodes);

    for (const auto& ind : population) {
        int score = computeObjective(ind, dist_matrix, nodes);
        if (score < best_score) {
            best_score = score;
            best = ind;
        }
    }

    return {best, local_search_count, main_loop};

}
