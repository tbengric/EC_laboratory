#pragma once
#include <vector>
#include <cmath>
#include "node.h"
#include <random>
#include "heuristics.h"
#include "distance_matrix.h"
#include <set>
#include <tuple>
#include <list>
#include <algorithm>

using namespace std;

double delta_intra_node(const vector<int>& path, int first_node_index, int second_node_index, const vector<vector<int>>& dist_matrix) {
    int path_size = path.size();
    if (path_size < 2) return 0.0;

    int prev_first = path[(first_node_index - 1 + path_size) % path_size];
    int first = path[first_node_index];
    int next_first = path[(first_node_index + 1) % path_size];

    int prev_second = path[(second_node_index - 1 + path_size) % path_size];
    int second = path[second_node_index];
    int next_second = path[(second_node_index + 1) % path_size];

    double old_cost = 0.0;
    double new_cost = 0.0;

    if ((first_node_index + 1) % path_size == second_node_index) {
        // adjacent nodes
        old_cost += dist_matrix[prev_first][first] + dist_matrix[first][second] + dist_matrix[second][next_second];
        new_cost += dist_matrix[prev_first][second] + dist_matrix[second][first] + dist_matrix[first][next_second];
    } else if ((second_node_index + 1) % path_size == first_node_index) {
        // adjacent nodes
        old_cost += dist_matrix[prev_second][second] + dist_matrix[second][first] + dist_matrix[first][next_first];
        new_cost += dist_matrix[prev_second][first] + dist_matrix[first][second] + dist_matrix[second][next_first];
    } else {
        // non-adjacent nodes
        old_cost += dist_matrix[prev_first][first] + dist_matrix[first][next_first];
        old_cost += dist_matrix[prev_second][second] + dist_matrix[second][next_second];

        new_cost += dist_matrix[prev_first][second] + dist_matrix[second][next_first];
        new_cost += dist_matrix[prev_second][first] + dist_matrix[first][next_second];
    }   
    double delta = new_cost - old_cost;
    return delta;
}

double delta_intra_edge(const vector<int>& path, int first_node_of_first_edge, int first_node_of_second_edge, const vector<vector<int>>& dist_matrix) {
    int path_size = path.size();
    if (path_size < 4) return 0.0;
    int first_node_first_edge = path[first_node_of_first_edge];
    int second_node_first_edge = path[(first_node_of_first_edge + 1) % path_size];
    int first_node_second_edge = path[first_node_of_second_edge];
    int second_node_second_edge = path[(first_node_of_second_edge + 1) % path_size];

    if (second_node_first_edge == first_node_second_edge) {
        // edges are adjacent
        return 0.0;
    }

    double old_cost = dist_matrix[first_node_first_edge][second_node_first_edge] +
                      dist_matrix[first_node_second_edge][second_node_second_edge];
    double new_cost = dist_matrix[first_node_first_edge][first_node_second_edge] +
                      dist_matrix[second_node_first_edge][second_node_second_edge];
    double delta = new_cost - old_cost;
    return delta;
}

double delta_inter_node(const vector<int>& path, int selected_node, int not_selected_node, const vector<vector<int>>& dist_matrix, const vector<Node>& nodes) {
    int path_size = path.size();
    if (path_size < 2) return 0.0;

    int node_selected = path[selected_node];
    int prev_node = path[(selected_node - 1 + path_size) % path_size];
    int next_node = path[(selected_node + 1) % path_size];

    double old_cost = dist_matrix[prev_node][node_selected] + dist_matrix[node_selected][next_node] + nodes[node_selected].cost;
    double new_cost = dist_matrix[prev_node][not_selected_node] + dist_matrix[not_selected_node][next_node] + nodes[not_selected_node].cost;

    double delta = new_cost - old_cost;
    return delta;
}

//Not sure if it works due to stop condition of the loop
vector<int> steepest_local_search(const vector<int>& initial_path, const vector<vector<int>>& dist_matrix, const string& intra_method, const vector<Node>& nodes) {
    int dist_size = dist_matrix.size();
    vector<int> current_path = initial_path;
    //double current_cost = initial_cost;

    while(true){
        double best_delta = 0.0;
        int best_move_type = -1; // 0: intra-node, 1: intra-edge, 2: inter-node
        int best_first_node = 0, best_second_node = 0;
        int best_not_selected_node = -1;

        int current_path_size = current_path.size();
        if (current_path_size < 2) break;

        set<int> selected_nodes(current_path.begin(), current_path.end());
        vector<int> not_selected_nodes;
        for (const auto& node : nodes) {
            if (selected_nodes.find(node.id) == selected_nodes.end()) {
                not_selected_nodes.push_back(node.id);
            }
        }

        if (intra_method == "node") {
            for (int i = 1; i < current_path_size - 1; ++i) {
                for (int j = i + 1; j < current_path_size - 1; ++j) {
                    double delta = delta_intra_node(current_path, i, j, dist_matrix);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 0;
                        best_first_node = i;
                        best_second_node = j;
                    }
                }
            }
        }else if (intra_method == "edge") {
            for (int i = 0; i < current_path_size - 1; ++i) {
                for (int j = i + 2; j < current_path_size - 1; ++j) {
                    double delta = delta_intra_edge(current_path, i, j, dist_matrix);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 1;
                        best_first_node = i;
                        best_second_node = j;
                    }
                }
            }
        }

        for (int i = 1; i < current_path_size - 1; ++i) {
            for (int not_selected_node : not_selected_nodes) {
                double delta = delta_inter_node(current_path, i, not_selected_node, dist_matrix, nodes);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 2;
                    best_first_node = i;
                    best_not_selected_node = not_selected_node;
                }
            }
        }

    

        if (best_move_type != -1 && best_delta < 0.0) {
            //current_cost += best_delta;
            if (best_move_type == 0) {
                // intra-node swap
                swap(current_path[best_first_node], current_path[best_second_node]);
            } else if (best_move_type == 1) {
                // intra-edge swap
                reverse(current_path.begin() + best_first_node + 1, current_path.begin() + best_second_node + 1);
            } else if (best_move_type == 2) {
                // inter-node swap
                current_path[best_first_node] = best_not_selected_node;
            }
        } else {
                break; // no improving move found
        }
        

    }
    return current_path;
}

// Structure to represent a saved move for delta evaluation
struct SavedMove {
    int move_type; // 0: intra-node, 1: intra-edge, 2: inter-node
    int first_param; // begin position
    int second_param; // end position
    int third_param; // for inter-node moves : id of outside node
    int removed_node1; // for edge moves (intra-edge): one endpoint of first removed edge
    int removed_node2; // for edge moves (intra-edge): other endpoint of first removed edge
    int removed_node3; // second removed edge endpoint 1
    int removed_node4; // second removed edge endpoint 2
    int edge_dir1; // recorded direction at save time (1 or -1) for edge1
    int edge_dir2; // recorded direction at save time (1 or -1) for edge2
    double delta;
    bool is_inverted; // whether this move involves inverted edges
    int saved_node_a; // node ids for intra-node moves (to find them again when re-applying)
    int saved_node_b;
};

// Helper function to check if an edge exists in the path and get its direction
// Returns: 0 if edge doesn't exist, 1 if edge exists in normal direction, -1 if edge exists in reverse direction
int edge_exists_and_direction(const vector<int>& path, int node_a, int node_b) {
    int path_size = path.size();
    for (int i = 0; i < path_size; ++i) {
        int current = path[i];
        int next = path[(i + 1) % path_size];
        
        if (current == node_a && next == node_b) {
            return 1;
        }
        if (current == node_b && next == node_a) {
            return -1;
        }
    }
    return 0;
}

int find_node_index(const vector<int>& path, int node_id) {
    for (int i = 0; i < (int)path.size(); ++i) {
        if (path[i] == node_id) return i;
    }
    return -1;
}

vector<int> steepest_local_search_with_lm(const vector<int>& initial_path, const vector<vector<int>>& dist_matrix, 
                                          const string& intra_method, const vector<Node>& nodes,
                                          list<SavedMove>& list_of_moves) {
    int dist_size = dist_matrix.size();
    vector<int> current_path = initial_path;

    while(true) {
        int current_path_size = current_path.size();
        if (current_path_size < 2) break;
        
        set<int> selected_nodes(current_path.begin(), current_path.end());
        vector<int> not_selected_nodes;
        for (const auto& node : nodes) {
            if (selected_nodes.find(node.id) == selected_nodes.end()) {
                not_selected_nodes.push_back(node.id);
            }
        }
        
        // PHASE 1: Check moves in LM list
        bool lm_move_applied = false;
        auto it = list_of_moves.begin();
        while (it != list_of_moves.end() && !lm_move_applied) {
            SavedMove& move = *it;
            
            if (move.move_type == 1) {
                // For intra-edge: check both removed edges
                int dir1 = edge_exists_and_direction(current_path, move.removed_node1, move.removed_node2);
                int dir2 = edge_exists_and_direction(current_path, move.removed_node3, move.removed_node4);

                if (dir1 == 0 || dir2 == 0) {
                    // At least one removed edge no longer exists -> remove move
                    it = list_of_moves.erase(it);
                    continue;
                }

                bool same_as_saved = (dir1 == move.edge_dir1 && dir2 == move.edge_dir2);
                bool both_reversed = (dir1 == -move.edge_dir1 && dir2 == -move.edge_dir2);

                if (same_as_saved || both_reversed) {
                    // Find positions and apply 2-opt
                    int pos1 = find_node_index(current_path, move.removed_node1);
                    int pos3 = find_node_index(current_path, move.removed_node3);
                    if (pos1 != -1 && pos3 != -1) {
                        int a = min(pos1, pos3);
                        int b = max(pos1, pos3);
                        // Re-evaluate delta before applying: only apply if still improving
                        double d = delta_intra_edge(current_path, a, b, dist_matrix);
                        if (d < 0.0) {
                            reverse(current_path.begin() + a + 1, current_path.begin() + b + 1);
                            it = list_of_moves.erase(it);
                            lm_move_applied = true;
                            continue;
                        } else {
                            it = list_of_moves.erase(it); // remove if not improving anymore
                            continue;
                        }
                    } else {
                        // if we can't find nodes anymore, remove
                        it = list_of_moves.erase(it);
                        continue;
                    }
                } else {
                    // Edges exist but not in the saved relative direction -> keep in LM and continue
                    ++it;
                    continue;
                }
            } else {
                // For node/inter moves, validate and apply if still improving
                if (move.move_type == 0) {
                    int posA = find_node_index(current_path, move.saved_node_a);
                    int posB = find_node_index(current_path, move.saved_node_b);
                    if (posA == -1 || posB == -1) {
                        it = list_of_moves.erase(it);
                        continue;
                    }
                    double d = delta_intra_node(current_path, posA, posB, dist_matrix);
                    if (d < 0.0) {
                        swap(current_path[posA], current_path[posB]);
                        it = list_of_moves.erase(it);
                        lm_move_applied = true;
                        continue;
                    } else {
                        ++it; // not improving now, keep it in LM
                        continue;
                    }
                } else if (move.move_type == 2) {
                    int pos = find_node_index(current_path, move.saved_node_a);
                    if (pos == -1) { it = list_of_moves.erase(it); continue; }
                    // Check candidate still not selected
                    if (selected_nodes.find(move.third_param) != selected_nodes.end()) { it = list_of_moves.erase(it); continue; }
                    double d = delta_inter_node(current_path, pos, move.third_param, dist_matrix, nodes);
                    if (d < 0.0) {
                        current_path[pos] = move.third_param;
                        it = list_of_moves.erase(it);
                        lm_move_applied = true;
                        continue;
                    } else {
                        ++it; // not improving now
                        continue;
                    }
                }
                ++it;
            }
        }
        
        if (lm_move_applied) {
            continue; // Go to next iteration
        }
        
        // PHASE 2: Evaluate neighborhood to find best move and collect improving moves
        double best_delta = 0.0;
        int best_move_type = -1;
        int best_first_node = 0, best_second_node = 0;
        int best_not_selected_node = -1;
        
        vector<SavedMove> new_improving_moves;
        
        if (intra_method == "edge") {
            for (int i = 0; i < current_path_size - 1; ++i) {
                for (int j = i + 2; j < current_path_size; ++j) {
                    double delta = delta_intra_edge(current_path, i, j, dist_matrix);
                    if (delta < 0.0) {
                        if (delta < best_delta) {
                            best_delta = delta;
                            best_move_type = 1;
                            best_first_node = i;
                            best_second_node = j;
                        }
                        // Store as improving move for LM
                        SavedMove new_move;
                        new_move.move_type = 1;
                        new_move.first_param = i;
                        new_move.second_param = j;
                        new_move.removed_node1 = current_path[i];
                        new_move.removed_node2 = current_path[i + 1];
                        new_move.removed_node3 = current_path[j];
                        new_move.removed_node4 = current_path[(j + 1) % current_path_size];
                        new_move.edge_dir1 = edge_exists_and_direction(current_path, new_move.removed_node1, new_move.removed_node2);
                        new_move.edge_dir2 = edge_exists_and_direction(current_path, new_move.removed_node3, new_move.removed_node4);
                        new_move.delta = delta;
                        new_move.is_inverted = false;
                        new_improving_moves.push_back(new_move);
                        // Also add a copy with inverted edges (for delayed application when direction flips)
                        SavedMove inv = new_move;
                        inv.is_inverted = true;
                        inv.edge_dir1 = -inv.edge_dir1;
                        inv.edge_dir2 = -inv.edge_dir2;
                        // Recompute delta for inverted (use same indexes a,b)
                        inv.delta = delta; // invariant for symmetric distances
                        new_improving_moves.push_back(inv);
                    }
                }
            }
        }
        
        for (int i = 1; i < current_path_size - 1; ++i) {
            for (int not_selected_node : not_selected_nodes) {
                double delta = delta_inter_node(current_path, i, not_selected_node, dist_matrix, nodes);
                if (delta < 0.0) {
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 2;
                        best_first_node = i;
                        best_not_selected_node = not_selected_node;
                    }
                    SavedMove new_move;
                    new_move.move_type = 2;
                    new_move.first_param = i;
                    new_move.third_param = not_selected_node;
                    new_move.saved_node_a = current_path[i];
                    new_move.saved_node_b = not_selected_node;
                    new_move.delta = delta;
                    new_move.is_inverted = false;
                    new_improving_moves.push_back(new_move);
                }
            }
        }
        
        // Sort new improving moves by delta (best first) and add to LM
        sort(new_improving_moves.begin(), new_improving_moves.end(),
             [](const SavedMove& a, const SavedMove& b) { return a.delta < b.delta; });
        
        for (const auto& m : new_improving_moves) {
            list_of_moves.push_back(m);
        }
        
        // Apply best move or terminate
        if (best_move_type != -1 && best_delta < 0.0) {
            if (best_move_type == 1) {
                reverse(current_path.begin() + best_first_node + 1, current_path.begin() + best_second_node + 1);
            } else if (best_move_type == 2) {
                current_path[best_first_node] = best_not_selected_node;
            }
            // Remove the applied move from LM (if it was inserted)
            for (auto it2 = list_of_moves.begin(); it2 != list_of_moves.end(); ++it2) {
                const SavedMove &mv = *it2;
                bool same = false;
                if (best_move_type == 1 && mv.move_type == 1) {
                    if (mv.removed_node1 == current_path[best_first_node] || mv.removed_node1 == current_path[best_second_node]) {
                        same = true;
                    }
                } else if (best_move_type == 2 && mv.move_type == 2) {
                    if (mv.first_param == best_first_node && mv.third_param == best_not_selected_node) same = true;
                }
                if (same) { list_of_moves.erase(it2); break; }
            }
        } else {
            break; // No improving move found
        }
    }
    
    return current_path;
}

vector<int> greedy_local_search(const vector<int>& initial_path, const vector<vector<int>>& dist_matrix, const string& intra_method, const vector<Node>& nodes) {
    int dist_size = dist_matrix.size();
    vector<int> current_path = initial_path;
    //double current_cost = initial_cost;

    while (true){
        int current_path_size = current_path.size();
        if (current_path_size < 2) break;

        set<int> selected_nodes(current_path.begin(), current_path.end());
        vector<int> not_selected_nodes;
        for (const auto& node : nodes) {
            if (selected_nodes.find(node.id) == selected_nodes.end()) {
                not_selected_nodes.push_back(node.id);
            }
        }

        vector<tuple<int, int, int, int>> possible_moves; // move_type, first_node, second_node, not_selected_node (in case of inter_node)

        if (intra_method == "node"){
            for (int i = 1; i < current_path_size - 1; ++i) {          // <-- pomijamy start/koniec
                for (int j = i + 1; j < current_path_size - 1; ++j) {
                    possible_moves.emplace_back(0, i, j, -1);
                }
            }
        } else if (intra_method == "edge"){
            for (int i = 0; i < current_path_size - 1; ++i) {
                for (int j = i + 2; j < current_path_size - 1; ++j) {  // <-- nie ruszamy ostatniego
                    possible_moves.emplace_back(1, i, j, -1);
                }
            }
        }

        for (int i = 0; i < current_path_size; ++i) {
            for (int not_selected_node : not_selected_nodes) {
                possible_moves.emplace_back(2, i, -1, not_selected_node);
            }
        }

        if (possible_moves.empty()) break;

        shuffle(possible_moves.begin(), possible_moves.end(), std::mt19937{std::random_device{}()});
        bool found_better_move = false;

        for (const auto& move : possible_moves) {
            int move_type = get<0>(move);
            int first_node = get<1>(move);
            int second_node = get<2>(move);
            int not_selected_node = get<3>(move);

            double delta = 0.0;
            if (move_type == 0) {
                delta = delta_intra_node(current_path, first_node, second_node, dist_matrix);
            } else if (move_type == 1) {
                delta = delta_intra_edge(current_path, first_node, second_node, dist_matrix);
            } else if (move_type == 2) {
                delta = delta_inter_node(current_path, first_node, not_selected_node, dist_matrix, nodes);
            }

            
            if (delta < 0.0) {
                //current_cost += delta;
                if (move_type == 0) {
                    swap(current_path[first_node], current_path[second_node]);
                } else if (move_type == 1) {
                    reverse(current_path.begin() + first_node + 1, current_path.begin() + second_node + 1);
                } else if (move_type == 2) {
                    current_path[first_node] = not_selected_node;
                }
                found_better_move = true;
                break; // exit after the first improving move
            }
        }

        if (!found_better_move) {
            break; // no improving move found
        }
    }
    
    return current_path;
}
