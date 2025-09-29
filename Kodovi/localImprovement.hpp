#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <set>
#include "lattice_planner/graphSearch.hpp"
#include "lattice_planner/optimizeHeading.hpp"
#include "Costs_Data_Holder.hpp"
#include "Solution.hpp"
#include "Edge.hpp"

#define INF 1000000.0
#define NUM_OF_DIRECTIONS 16

using namespace std;

bool inside_interval(int number, int interval_start, int interval_end) {
    return ((interval_start <= number) && (number <= interval_end));
}

// Prima dva stanja (ili čvora) u dvodimenzionalnom prostoru.
// Vraća euklidsku udaljenost između njih.
double Euclidean_distance(vector<int> state1, vector<int> state2) {
    double delta_x = state2[0] - state1[0];
    double delta_y = state2[1] - state1[1];
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

// Prima soluciju, novu soluciju dobivenu iz prethodne koja ima podniz čijem se čvorovima želi optimizirati stanja, indeks početka tog podniza i indeks kraja tog podniza.
// Funkcija izračuna donju među novog rješenja (korištenjem Euklidkse udaljenosti/ravnih linija između čvorova kojih se želi optimimizirati) i uspoređuje ju s duljinom originalne solucije.
// Vraća odgovor prihvaća li se solucija za daljnje optimiziranje. Ako se prihvaća, vraća se true. Inače se vraća false.
bool speed_up_heuristic_accept(Solution *solution, vector<int> new_solution, int subsequence_start, int subsequence_end,
                                vector<vector<int>> states, Costs_Data_Holder &cdh, int backwards_flag=1) {
    bool accept;
    double new_cost = 0.0;
    for (int i = 0; i < new_solution.size() - 1; i++) {
        int state1_index = new_solution[i];
        int state2_index = new_solution[i + 1];
        // Ako je barem trenutno ili barem slijedeće stanje jedno od stanja koje se želi optimizirati, onda se uračunava duljina ravne dužine izmeču čvorova u kojima se nalaze ta stanja.
        if (inside_interval(i, subsequence_start, subsequence_end) || inside_interval(i + 1, subsequence_start, subsequence_end)) {
            vector<int> state1 = states[state1_index], state2 = states[state2_index];
            cout << " Izmedu " << state1_index << " {" << state1[0] << ',' << state1[1] << "} i " << state2_index << " {" << state2[0] << ',' << state2[1] << "}:\n";
            cout << "  Euclidean_distance(state1, state2) = " << Euclidean_distance(state1, state2) << endl;
            new_cost += Euclidean_distance(state1, state2);
            cout << "  new_cost = " << new_cost << endl;
        }
        // Ako su i trenutno i slijedeće stanje izvan podniza koji se želi optimirati, uračunava se duljina putanje (koja može biti krivulja) između ta dva stanja.
        else {
            new_cost += cdh.determine_cost(state1_index, state2_index, states, backwards_flag);
            cout << " Izmedu " << state1_index << " {" << states[state1_index][0] << ',' << states[state1_index][1] << "} i " << state2_index << " {" << states[state2_index][0] << ',' << states[state2_index][1] << "}:\n";
            cout << "  determine_cost(state1_index, state2_index) = " << cdh.determine_cost(state1_index, state2_index, states, backwards_flag) << endl;
        }
    }

    cout << "(*solution).total_cost = " << (*solution).total_cost << endl;
    cout << "Total new_cost = " << new_cost << endl;
    if (new_cost >= (*solution).total_cost) {
        accept = false;
    }
    else {
        accept = true;
    }
    return accept;
}

void swaps(Solution *solution, vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, int backwards_flag) {
    vector<int> new_solution_states;
    int solution_size = (*solution).sol_states.size();
    // Prolazak kroz sva stanja u rješenju osim početnog i završnog
    for (int i = 1; i < solution_size - 3; i++) {
        for (int j = i + 2; j < solution_size - 1; j++) {
            new_solution_states = (*solution).sol_states;
            int tmp = new_solution_states[i];
            new_solution_states[i] = new_solution_states[j];
            new_solution_states[j] = tmp;
            Solution new_solution = Solution(new_solution_states, states, cdh, backwards_flag);
            if (new_solution.total_cost < (*solution).total_cost)
                (*solution) = new_solution;
        }
    }
}

double calculate_cost(vector<int> solution, vector<vector<int>> states, Costs_Data_Holder &cdh, int backwards_flag=1) {
    double total_c = 0.0, c;
    for (int i = 0; i < (int)solution.size() - 1; i++) {
        c = cdh.determine_cost(solution[i], solution[i+1], states, backwards_flag);
        if (c == INF)
            return INF;
        total_c += c;
    }
    return total_c;
}

void find_best_permutation(Solution *solution, int subsequence_start, int k, vector<int> subsequence_original, vector<vector<int>> nodes, vector<vector<int>> states,
                            Costs_Data_Holder &cdh, vector<int> *best_permutation_states, double *best_permutation_cost, int backwards_flag) {
    *best_permutation_states = (*solution).sol_states;
    *best_permutation_cost = (*solution).total_cost;

    int solution_size = (*solution).sol_states.size();
    int subsequence_arr[k];
    for (int i = 0; i < k; i++)
        subsequence_arr[i] = subsequence_original[i];
    int n = sizeof(subsequence_arr) / sizeof(subsequence_arr[0]);
    // Sortiranje subsequence_arr radi korištenja funkcije next_permutation()
    sort(subsequence_arr, subsequence_arr + n);
    do {
        // Ako je permutaciiju moguće dobiti i-neighbour swap algoritmom (i = 2, 3, ..., k - 1), onda ju preskoći.
        bool covered_by_i_neigh = false;
        for (int i = 0; i < k; i++)
            if (subsequence_original[i] == subsequence_arr[i]) {
                covered_by_i_neigh = true;
                break;
            }
        if (covered_by_i_neigh == true)
            continue;


        // Vektor će sadržavati soluciju za trenutnu permutaciju podniza
        vector<int> solution_for_subsequence = {};
        double cost_for_solution_for_subsequence;

        // Sastavljanje cijele solucije za trenutni subsequence_arr
        int count = 0;
        for (int i = 0; i < (int)solution_size; i++) {
            if (i >= subsequence_start && i < subsequence_start + k) {
                solution_for_subsequence.push_back(subsequence_arr[count]);
                count++;
            }
            else {
                solution_for_subsequence.push_back((*solution).sol_states[i]);
            }
        }

        cost_for_solution_for_subsequence = calculate_cost(solution_for_subsequence, states, cdh, backwards_flag);

        if (cost_for_solution_for_subsequence < *best_permutation_cost) {
            *best_permutation_cost = cost_for_solution_for_subsequence;
            *best_permutation_states = solution_for_subsequence;
        }
    } while (next_permutation(subsequence_arr, subsequence_arr + n));
}

void k_neighbour_swaps(Solution *solution, int k, vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th, int backwards_flag) {
    clock_t t_f_start = clock();

    int solution_size = (*solution).sol_states.size();
    vector<int> new_solution_states;
    double new_solution_cost;
    for (int subsequence_start = 1; subsequence_start < solution_size - k + 1 - 1; subsequence_start++) {
        // Stvaranje podniza
        vector<int> subsequence;
        for (int i = subsequence_start; i < subsequence_start + k; i++)
            subsequence.push_back((*solution).sol_states[i]);

        // Podatke o najboljem rješenju spremi u new_solution_states i new_solution_cost
        find_best_permutation(solution, subsequence_start, k, subsequence, nodes, states, cdh, &new_solution_states, &new_solution_cost, backwards_flag);
        
        // Ako permutacija poboljšava soluciju, solucija se ažurira
        if (new_solution_cost < (*solution).total_cost) {
            (*solution).sol_states = new_solution_states;
            (*solution).total_cost = new_solution_cost;
        }
    }

    // Kraj funckije
    clock_t t_f_finish = clock();
    if (k == 2)
        th.two_neighbour_swaps += (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
    else if (k == 3)
        th.three_neighbour_swaps += (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
    else if (k == 4)
        th.four_neighbour_swaps += (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
}

vector<int> two_opt_swap(vector<int> solution_states, int s1_index, int s2_index) {
    int solution_size = solution_states.size();
    vector<int> new_solution_states = {};
    int i = 0;
    for (int i = 0; i <= s1_index; i++)
        new_solution_states.push_back(solution_states[i]);
    for (int i = s2_index; i >= s1_index + 1; i--)
        new_solution_states.push_back(solution_states[i]);
    for (int i = s2_index + 1; i <= solution_size - 1; i++)
        new_solution_states.push_back(solution_states[i]);
    return new_solution_states;
}

void two_opt(Solution *solution, vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th, int backwards_flag) {
    clock_t t_f_start = clock();

    int solution_size = (*solution).sol_states.size();
    vector<int> new_solution_states;
    bool improvement = true;
    while (improvement == true) {
        improvement = false;

        for (int i = 0; i < solution_size - 1; i++) {
            for (int j = i+3; j < solution_size - 1; j++) {
                new_solution_states = two_opt_swap((*solution).sol_states, i, j);
                Solution new_solution = Solution(new_solution_states, states, cdh, backwards_flag);
                if (new_solution.total_cost < (*solution).total_cost) {
                    improvement = true;
                    (*solution) = new_solution;
                }
            }
        }
    }

    clock_t t_f_finish = clock();
    th.two_opt += (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
}

int get_index_in_solution_for_edge(vector<int> solution, Edge e) {
    int solution_size = solution.size();
    for (int i = 0; i < solution_size; i++) {
        if (solution[i] == e.s1)
            return i;
    }
    cerr << "NEOČEKIVANA VRIJEDNOST U get_index_in_solution_for_edge(): Nije pronađeno stanje koje sadrži brid e\n";
    return -1;
}

void direct_two_opt(Solution *solution, int number_of_longest, vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th, int backwards_flag) {
    clock_t t_f_start = clock();

    set<Edge> tmp;
    vector<Edge> longest;
    int solution_size = (*solution).sol_states.size();
    for (int i = 0; i < solution_size - 1; i++) {
        int s1 = (*solution).sol_states[i], s2 = (*solution).sol_states[i+1];
        Edge e = Edge(s1, s2, states, cdh);
        tmp.insert(e);
    }
    set<Edge>::iterator itr = tmp.begin();
    for (int i = 0; i < number_of_longest; i++) {
        longest.push_back(*itr);
        itr++;
    }

    // Za svaki najdulji brid, isprobaj zamjenu
    for (int i = 0; i < (int)longest.size() - (int)1; i++) {
        bool improvement = false;
        Edge e1 = longest[i];
        for (int j = i + 1; j < (int)longest.size(); j++) {
            Edge e2 = longest[j];
            if (! e1.adjecent(e2)) {
                int s1_index_in_solution = get_index_in_solution_for_edge((*solution).sol_states, e1);
                int s2_index_in_solution = get_index_in_solution_for_edge((*solution).sol_states, e2);

                // Funkciji two_opt_swap moraju se predati indeksi u redoslijedu u kojem se pojavljuju u rješenju - zato se koriste min i max.
                vector<int> new_solution_states = two_opt_swap((*solution).sol_states, min(s1_index_in_solution, s2_index_in_solution), max(s1_index_in_solution, s2_index_in_solution));
                Solution new_solution = Solution(new_solution_states, states, cdh, backwards_flag);

                if (new_solution.total_cost < (*solution).total_cost) {
                    improvement = true;
                    // Prihvati promjenu
                    // Brisanje promjenjenih bridova iz vektora najduljih
                    // Prvo izbrišemo drugi vektor (onaj na poziciji j), a zatim prvi brid (onaj na poziciji i).
                    longest.erase(longest.begin() + j);
                    // Kako se e2 nalazi iza e1, brisanje e2 neće promjeniti indeks e1
                    longest.erase(longest.begin() + i);
                    // Potrebno je ažurirati poredak stanja u bridovima za one bridove koji se nalaze između bridova koji su zamijenjeni
                    for (int k = 0; k < (int)longest.size(); k++) {
                        Edge ek = longest[k];
                        int sk_index_in_solution = get_index_in_solution_for_edge((*solution).sol_states, ek);
                        if (sk_index_in_solution > i && sk_index_in_solution < j) {
                            int tmp = ek.s1;
                            ek.s1 = ek.s2;
                            ek.s2 = tmp;
                            longest[k] = ek;
                        }
                    }
                    // Ažuriranje solucije
                    (*solution) = new_solution;
                    //Prekid prolaska jer su bridovi na i i j uklonjeni
                    break;
                }
            }
        }
        // Ako je pronađeno poboljšanje, znači da je došlo do zamjene birdova i da brid na i više nije prisutan u longest.
        // Zato se i umanji za 1 jer je idući najdulji brid sada na indeksu i. U for petlji će se i povećati za 1 pa tako idući i bude jednak trenutnom i.
        if (improvement == true)
            i--;
    }

    clock_t t_f_finish = clock();
    th.direct_two_opt += (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
}

void optimize_inserted_vertex(vector<int> *new_solution_states, int moved_state_index_index, vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, int backwards_flag) {
    int solution_size = (*new_solution_states).size();
    double min_cost = 0.0;
    // Indeks moved_state s obzirom na states
    int moved_state_index = (*new_solution_states)[moved_state_index_index];
    // Indeks najboljeg stanja. Indeks identificira tanje u states.
    int best_state_index = moved_state_index;
    // Indeksi susjeda od moved_state s obzirom na *new_solution_states
    int prev_state_index_index = moved_state_index_index - 1, foll_state_index_index = moved_state_index_index + 1;
    // Indeksi susjeda od moved_state s obzirom na "states"
    int prev_state_index, foll_state_index; 

    double c1, c2;
    if (prev_state_index_index >= 0) {
        prev_state_index = (*new_solution_states)[prev_state_index_index];
        c1 = cdh.determine_cost(prev_state_index, moved_state_index, states, backwards_flag);
        if (c1 == INF)
            min_cost = INF;
        else
            min_cost += c1;
    }

    if (c1 != INF && foll_state_index_index < solution_size) {
        foll_state_index = (*new_solution_states)[foll_state_index_index];
        c2 = cdh.determine_cost(moved_state_index, foll_state_index, states, backwards_flag);
        if (c2 == INF)
            min_cost = INF;
        else
            min_cost += c2;
    }
    
    // Indeks pomaknutog čvora s obzirom na nodes
    int moved_node_index = moved_state_index / NUM_OF_DIRECTIONS;
    
    for (int curr_state_index = moved_node_index * NUM_OF_DIRECTIONS; curr_state_index < moved_node_index * NUM_OF_DIRECTIONS + NUM_OF_DIRECTIONS; curr_state_index++) {
        double c1 = 0.0, c2 = 0.0;

        if (prev_state_index_index >= 0) {
            c1 = cdh.determine_cost(prev_state_index, curr_state_index, states, backwards_flag);

            // Nemogući put - isprobaj drugo stanje
            if (c1 == INF)
                continue;
        }

        if (foll_state_index_index < solution_size) {
            c2 = cdh.determine_cost(curr_state_index, foll_state_index, states, backwards_flag);

            // Nemogući put - isprobaj drugo stanje
            if (c2 == INF)
                continue;
        }

        if (c1 + c2 < min_cost) {
            min_cost = c1 + c2;
            best_state_index = curr_state_index;
        }
    }

    if (best_state_index != moved_node_index)
        (*new_solution_states)[moved_state_index_index] = best_state_index; 
}

void inserts(Solution *solution, vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th, int backwards_flag) {
    clock_t t_f_start = clock();

    int solution_size = (*solution).sol_states.size();
    // Stanje koje će se pomaknuti
    int moved_state;
    // indeksi stanja koja su bili susjedi stanju moved_state. Indeksi se odnose na indekse solution_x.
    int neigh_prev_index, neigh_next_index;
    // solution_x je kopija sadržaja iz (*solution).sol_states, ali bez moved_state
    // new_solution_states je solucija koja se dobije ubacivanjem moved_state stanja negdje u solution_x
    vector<int> solution_x, new_solution_states;
    double new_cost;
    for (int i = 1; i < solution_size - 1; i++) {
        solution_x = (*solution).sol_states;
        moved_state = solution_x[i];
        solution_x.erase(solution_x.begin() + i);
        neigh_prev_index = i - 1; neigh_next_index = i;

        for (int j = 1; j < (int)solution_x.size(); j++) {
            if (j != neigh_prev_index && j != neigh_next_index) {
                new_solution_states = solution_x;
                new_solution_states.insert(new_solution_states.begin() + j, moved_state);
                optimize_inserted_vertex(&new_solution_states, j, nodes, states, cdh, backwards_flag);
                Solution new_solution = Solution(new_solution_states, states, cdh, backwards_flag);
                if (new_solution.total_cost < (*solution).total_cost) {
                    (*solution) = new_solution;
                    break;
                }
            }
        }
    }

    clock_t t_f_finish = clock();
    th.inserts += (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
}

// Prima trenutnu populaciju
// Poboljšava trenutnu populaciju
void local_improvement_procedure_for_symmetric(set<Solution> *p_pop, vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th, int optimize_headings_flag, int backwards_flag=1) {
    th.inserts = 0.0;
    th.direct_two_opt = 0.0;
    th.two_opt = 0.0;
    th.two_neighbour_swaps = 0.0;
    th.three_neighbour_swaps = 0.0;
    th.four_neighbour_swaps = 0.0;
    th.optimizeHeadings = 0.0;
    
    set<Solution> improved_pop;
    set<Solution>::iterator itr;
    for (itr = (*p_pop).begin(); itr != (*p_pop).end(); itr++) {
        Solution curr_sol = (*itr);
        inserts(&curr_sol, nodes, states, cdh, th, backwards_flag);
        direct_two_opt(&curr_sol, nodes.size() / 4, nodes, states, cdh, th, backwards_flag);
        two_opt(&curr_sol, nodes, states, cdh, th, backwards_flag);
        k_neighbour_swaps(&curr_sol, 2, nodes, states, cdh, th, backwards_flag);
        k_neighbour_swaps(&curr_sol, 3, nodes, states, cdh, th, backwards_flag);
        k_neighbour_swaps(&curr_sol, 4, nodes, states, cdh, th, backwards_flag);
        
        if (optimize_headings_flag == 1) {
            vector<int> curr_order_of_nodes;
            for (int s : curr_sol.sol_states)
                curr_order_of_nodes.push_back((int)floor(s / NUM_OF_DIRECTIONS));
            curr_sol = Solution(optimizeHeadings(curr_order_of_nodes, nodes, states, cdh, th, backwards_flag), states, cdh, backwards_flag);
        }
        else {
            th.optimizeHeadings = 0.0;
        }

        improved_pop.insert(curr_sol);
    }

    (*p_pop) = improved_pop;

    // Kraj lokalnog poboljšanja
    th.improving = th.inserts + th.direct_two_opt + th.two_opt + th.two_neighbour_swaps + th.three_neighbour_swaps + th.four_neighbour_swaps + th.optimizeHeadings;
}
