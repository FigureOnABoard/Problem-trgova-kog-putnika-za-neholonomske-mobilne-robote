#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <ctime>
#include <algorithm>
#include <time.h>
#include <fstream>
#include <set>
#include <cmath>
#include <cstdio>

#include "headers/initialize_states.hpp"
#include "headers/reading_and_writing.hpp"
#include "Solution.hpp"
#include "Time_Holder.hpp"
#include "lattice_planner/graphSearch.hpp"
#include "lattice_planner/costFileReader.hpp"
#include "lattice_planner/optimizeHeading.hpp"
#include "localImprovement.hpp"

#define INF 1000000.0
#define NUM_OF_DIRECTIONS 16

using namespace std;

/*
    Program koristi memetički algoritam (engl. memetic algorithm) sličan onomu napravljenom u
    (Gutin, G. i Karapetyan, D. A Memetic Algorithm for the Generalized Traveling Salesman Problem),
    te je napravljen za planiranje puta neholonomskog robota kod kojeg svaki čvor koji treba posjetiti
    definira skup stanja.
    Unaprijed je definiran broj usmjerenja u koje robot može doći i on iznosi 16.
    Program ima definiranu listu čvorova (x_i, y_i) koje robot mora posjetiti. Drugim rječima, svaki čvor ima svoj jedinstven indeks.
    Program ima definiranu listu svih stanja (x_i, y_i, phi_i). Drugim rječima, svako stanje ima svoj jedistven indeks.
    Program rješeva generalizirani problem trgovačkog putnika u kojem mora posjetiti točno jedno stanje iz svakog čvora.
*/

/*
    Pokretanje programa: GTSP_Memetic_algorithm.exe optimize_headings_flag input_file output_file [backwards_flag]
    optimize_headings_flag može biti 0 ili 1. 0 Označava da se neće provoditi optimizeHeadings() tijekom local_improvement_search(), a 1 da hoće.
    input_file je datoteka (ime i put do datoteke) koju je napravio Cost_Data_Holder koja sadrži podatke o problemu.
    output_file je osnovni oblik naziva datoteke u koju će se spremati rezultati koji ukljućuju sve generacije i konačno rješenje. Datoteka će se uvijek
    spremati u mapu generations i imat će dodatak koji označava o kojoj je generaciji riječ. Ovisno o vrijednostima optimize_headings_flag i backwards_flag, lokacija
    i ime imat će zajedno oblik:
        "generations/with_optimizeHeadings/backwards/output_file-with_MA_gen{i}.txt" za optimize_headings_flag = 1 i backwards_flag = 1,
        "generations/with_optimizeHeadings/no_backwards/output_file-with_MA_gen{i}.txt" za optimize_headings_flag = 1 i backwards_flag = 0,
        "generations/without_optimizeHeadings/backwards/output_file-with_MA_gen{i}.txt" za optimize_headings_flag = 0 i backawrds_flag = 1,
        "generations/without_optimizeHeadings/no_backwards/output_file-with_MA_gen{i}.txt" za optimize_headings_flag = 0 i backwards_flag = 0,
    gdje je {i} redni broj generacije. Umjesto "output_file-with_MA_gen{i}.txt" stajat će "output_file-with_MA_FINISHED.txt" ako je riječ o datoteci koja sadrži
    podatke o konačnom rješenju ili će stajati "output_file-end_OH.txt" ako je riječ o datoteci s podacima o optimizaciji posljednjeg rjepenja kad se ne koristi
    optimizeHeadings u svakoj iteraciji (optimize_headings_flag = 0).
    [backwards_flag] može biti 0 ili 1. 0 onačava da se robot ne može kretati unatrag, a 1 da može. Ako se niti jedna vrijednost ne unese, [backwards_flag] se
    automatski postavlja na 1.
*/

// Poziva graphSearch uključen iz graphSearch.hpp
/*void invokeGraphSearch(int start[3], int finish[3], vector<double> &M, int sizeM[]) {
    string pathToVars = "lattice_planner/params/vars/csv_vars.csv";
    string pathToCost = "lattice_planner/params/cost/";
    vector<vector<short> > vMap;

    // Inicijalizacija vektorskog prikaza mape
    for (int i = 0; i < 1000; i++) {
        vector<short> row_i;
        for (int j = 0; j < 1000; j++) {
            row_i.push_back(short(0));
        }
        vMap.push_back(row_i);
    }

    // traženje najkraće izvedive putanje na grafu latice korištenjem A* algoritma
    graphSearch(pathToVars, pathToCost, vMap, start, finish, M, sizeM);
}*/

// Stvara i vraća vektor s koordinatama čvorova koje će se morati nalaziti na robotovom putu.
// Indeks čvora u vektoru je jedinstveni indetifikator/indeks tog čvora.
vector<vector<int>> define_problem() {
    vector<vector<int>> nodes = {{108,99}, {100,100}, {101,100}, {102,100}, {100,99}, {101,99}, {102,99}, {100,98}, {101,98}, {102,98}, {100,112}, {101,112}, {102,112}, {100,111}, {101,111}, {102,111}, {100,110}, {101,110}, {102,110}, {107,119}, {108,119}, {109,119}, {107,118}, {108,118}, {109,118}, {107,117}, {108,117}, {109,117}, {118,109}, {119,109}, {120,109}, {118,108}, {119,108}, {120,108}, {118,107}, {119,107}, {120,107}, {118,100}, {119,100}, {120,100}, {118,99}, {119,99}, {120,99}, {118,98}, {119,98}, {120,98}};
    return nodes;
}

/*map<int, map<int, double>> initialize_costs(vector<vector<int>> nodes, vector<vector<int>> states) {
cout << "Entered initialize_costs" << endl;
clock_t t;
double total_time = 0;
int pair_counter = 0;
    map<int, map<int, double>> costs;
    for (int i = 0; i < states.size() - 1; i++) {
        for (int j = i+1; j < states.size(); j++) {
            // Ako su i-to stanje i j-to stanje u različitim čvorovima, računa se udaljenost između njih. Inače, ništa ( jer se robot ne vraća u isti čvor)
            if (!(states[i][0] == states[j][0] && states[i][1] == states[j][1])) {
                vector<double> M = {};
                int sizeM[2], *vehicle_orientations = NULL;
                int start[3], finish[3];
                for (int k = 0; k < 3; k++) {
                    start[k] = states[i][k];
                    finish[k] = states[j][k];
                }
cout << "About to invokeGraphSearch on " << i << " and " << j << '.';
t = clock();
                invokeGraphSearch(start, finish, M, sizeM);
t = clock() - t;
total_time += ((double)t)/CLOCKS_PER_SEC;
cout << " Finished in " << ((double)t)/CLOCKS_PER_SEC << " seconds." << endl;
pair_counter++;
                costs[i][j] = get_cost_of_restructured_path(M, sizeM);
            }
        }
    }
cout << "Finished initialize_costs! Time took: " << total_time << " seconds. (" << pair_counter << " pairs)" << endl;
cout << "Average time: " << total_time / pair_counter << " seconds." << endl;
    return costs;
}*/

void set_random_seed() {
    time_t timestamp;
    time(&timestamp);
    long int seed = (long int) timestamp;
    seed %= 100;
    srand(seed);
    return;
}

int random_function(int i) {
    return rand() % i;
}

// Prima vektor svih čvorova nodes.
// Vraća nasumičan poredak čvorova iz nodes. Prvi i posljednji čvor su jedini koji nisu nasumično odabrani nego je za njih uzet prvi čvor iz nodes.
vector<int> random_node_selection(vector<vector<int>> nodes) {
    vector<int> order_of_nodes;
    for (int i = 1; i < nodes.size(); i++) {
        order_of_nodes.push_back(i);
    }
    random_shuffle(order_of_nodes.begin(), order_of_nodes.end(), random_function);
    
    order_of_nodes.insert(order_of_nodes.begin(), 0);
    order_of_nodes.insert(order_of_nodes.begin() + order_of_nodes.size(), 0);
    return order_of_nodes;
}

// Algoritam ima dva dijela:
// 1.) Konstruira nasumičnu permutaciju svih čvorova (konstruiraa vekor indeksa čvorova)
// 2.) Poziva optimizeHeadings() koji za svaki čvor iz 1.) odabire najbolje stanje. (Konstruira vektor indeksa stanja)
// Vraća put konstruiran u 2.)
set<Solution> semirandom_construction_heuristic(vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, int population_size, Time_Holder &th, int backwards_flag) {
    clock_t t_f_start = clock();

    set_random_seed();
    set<Solution> first_population;
    for (int i = 0; i < population_size; i++) {
        vector<int> order_of_nodes = random_node_selection(nodes);
        // Solution contains index of states in the order.
          // optimizeHeadings() će th.optimizeHeadings postaviti na vrijeme trajanje optimizeHeadings(), usprkos tome što th.optimizeHeadings označava vrijeme koje je proteklo za 
          // optimiueHeadings() tijekom lokalnog poboljšanja. Ovo nema veze jer će se nakon semirandom_construction_heuristic() izvoditi lokalno poboljšanje pa će se u njemu
          // u th.optimizeHeadings ponovno pisati.
        vector<int> solution = optimizeHeadings(order_of_nodes, nodes, states, cdh, th, backwards_flag);

        first_population.insert(Solution(solution, states, cdh, backwards_flag));
    }

    th.semirandom_constructon = ((double)(clock() - t_f_start)/CLOCKS_PER_SEC);
    th.generating = th.semirandom_constructon;
    th.reproduction = 0;
    th.crossover = 0;
    th.mutation = 0;
    return first_population;
}

void reproduction(set<Solution> population, set<Solution> *next_population,
                      vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th,
                      int number_of_repeats) {
    clock_t t_f_start = clock();

    int population_size = population.size();
    if (population_size == 0) {
        cout << "Izašao iz reproduction() jer je populacija roditelja prazna, a morala bi imati barem jedno rješenje.\n";
        return;
    }
    set<Solution>::iterator itr;
    itr = population.begin();
    // Kopira se prvih number_of_repeats najboljih rješenja iz population. Ako je broj rješenja population_size manji od number_of_repeats,
    // onda petlja prođe samo population_size puta.
    for (int i = 0; i < min(number_of_repeats, population_size); i++) {
        (*next_population).insert(*itr);
        itr++;
    }

    // Završetak funkcije
    clock_t t_f_finish = clock();
    th.reproduction = (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
    return;
}

void crossover(set<Solution> population, set<Solution> *next_population,
                vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th,
                int number_of_repeats, int backwards_flag) {
    clock_t t_f_start = clock();

    int population_size = population.size();
    if (population_size == 0) {
        cout << "Izašao iz crossover() jer je populacija roditelja prazna, a morala bi imati barem jedno rješenje.\n";
        th.crossover = 0.0;
        return;
    }

    if (round(0.33 * (double)population_size) < 2) {
        cout << "Preskočen crossover jer (round(0.33 * population_size) < 2), što za ovaj program ne omogućuje odabir dviju različitih roditelja.\n";
        th.crossover = 0.0;
        return;
    }

    for (int i = 0; i < number_of_repeats; i++) {
        int first_parent_index, second_parent_index;
        first_parent_index = rand() % (int)round(0.33 * (double)population_size);
        second_parent_index = rand() % (int)round(0.33 * (double)population_size);
        while (first_parent_index == second_parent_index)
            second_parent_index = rand() % (int)round(0.33 * (double)population_size);

        // Dohvaćanje dvaju roditelja nad kojima će se vršiti crossover.
        // Varijable u kojima se sprema roditelj ne sadrže ponavljajuće elementa, donosno, stanje iz početnog i završnog čvora pojavljuje se samo jednom.
        // Početno i završno stanje je ono čija je vrijednost u intervalu [0, 15]
        vector<int> parent1 = {}, parent2 = {};
        set<Solution>::iterator itr = population.begin();
        int parent_size;
        int count = 0;
        while (parent1.empty() || parent2.empty()) {
            if (count == first_parent_index) {
                parent1 = (*itr).sol_states;
                parent1.erase(parent1.begin() + parent1.size() - 1);
            }
            if (count == second_parent_index) {
                parent2 = (*itr).sol_states;
                parent2.erase(parent2.begin() + parent2.size() - 1);
            }
            count++;
            itr++;
        }
        parent_size = parent1.size();

        // Inicijalizacija djeteta
        vector<int> child = {};

        // Odabir fragmenta iz prvog roditelja koji počinje na nasumičnoj poziciji a i ima nasumičnu duljinu 1<=l<M
        // Stavljanje fragmenta na početak djeteta
        int a = rand() % parent_size;
        int l = 1 + (rand() % (parent_size - 1));
        for (int j = 0; j < l; j++)
            child.push_back(parent1[(j + a) % parent_size]);
        
        // Stvaranje pomoćnog niza parent2_x
        vector<int> parent2_x;
        for (int j = 1; j < parent_size + 1; j++)
            parent2_x.push_back(parent2[(j + a + l - 1) % parent_size]);
      
        // Stvaranje drugog pomoćnog niza parent2_xx
        // Niz sadrži sva stanja iz parent2_x osim onih koji se nalaze u čvorovima koji se već nalaze u child
        vector<int> parent2_xx;
        for (int state_index : parent2_x) {
            // Ako gledamo cjelobrojni dio rezultata djeljenja indeksa stanja s brojem orijentacija dobit ćemo indeks čvora u kojem se to stanje nalazi
            int node_index = int(state_index / NUM_OF_DIRECTIONS);
            // Provjera nalazi li se čvor u vektoru child
            bool present = false;
            for (int state_index_2 : child)
                if (int(state_index_2 / NUM_OF_DIRECTIONS) == node_index) {
                    present = true;
                    break;
                }
            
            // Ako se čvor ne nalazi u child, dodaj trenutni element u parent2
            if (present == false)
                parent2_xx.push_back(state_index);
        }

        // Završavanjem vektora child elementima iz present2_xx
        for (int j = 0; j < parent2_xx.size(); j++)
            child.push_back(parent2_xx[j]);

        // Rotacija elemenata tako da se stanje u prvom čvoru nađe na početku vektora
        while (int(child[0] / NUM_OF_DIRECTIONS) != 0) {
            int tmp = child[0];
            for (int j = 0; j < child.size() - 1; j++)
                child[j] = child[j + 1];
            child[child.size() - 1] = tmp;
        }

        // Djetetu se doda početno stanje kako bi se zatvorio puni krug
        child.push_back(child[0]);
        
        // Ako je rješenje izvedivo, dodaje ga se u novu populaciju
        Solution child_solution = Solution(child, states, cdh, backwards_flag);
        if (child_solution.total_cost < INF)
            (*next_population).insert(child_solution);
    }

    clock_t t_f_finish = clock();
    th.crossover = (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
    return;
}

void mutation(set<Solution> population, set<Solution> *next_population,
                vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th,
                int number_of_repeats, int backwards_flag) {
    clock_t t_f_start = clock();

    int population_size = population.size();
    if (population_size == 0) {
        cout << "Izašao iz mutation() jer je populacija roditelja prazna, a morala bi imati barem jedno rješenje.\n";
        th.mutation = 0.0;
        return;
    }

    set<Solution>::iterator itr;
    for (int i = 0; i < number_of_repeats; i++) {
        int parent_index = rand() % (int)round(0.75 * (double)population_size);
        
        // Dohvaćanje roditelja nad kojima će se vršiti mutation.
        // Varijable u kojima se sprema roditelj ne sadrže ponavljajuće elementa, donosno, stanje iz početnog i završnog čvora pojavljuje se samo jednom.
        // Početno i završno stanje je ono čija je vrijednost u intervalu [0, 15]
        itr = population.begin();
        for (int j = 0; j < parent_index; j++)
            itr++;
        vector<int> parent = (*itr).sol_states;
        parent.erase(parent.begin() + parent.size() - 1);
        int parent_size = parent.size();
               
        int a = rand() % parent_size;
        int l_min = (int)round(0.05 * (double)parent_size), l_max = (int)round(0.3 * (double)parent_size);
        if (l_max == 0) {
            cout << "Preskočen mutation() jer je maksimalna duljina fragmenta određena algoritmom za trenutni broj čvorova (" << parent_size << ") jednaka 0.\n";
            return;
        }
        int delta_l = l_max - l_min;
        int l = l_min + rand() % (delta_l + 1);

        vector<int> child;
        vector<int> fragment;
        vector<int> parent_x;
        for (int j = 0; j < l; j++)
            fragment.push_back(parent[(j + a) % parent_size]);

        for (int j = 0; j < parent_size; j++) {
            int parent_element = parent[j];
            bool present_in_fragment = false;
            for (int fragment_element : fragment) {
                if (parent_element == fragment_element) {
                    present_in_fragment = true;
                    break;
                }
            }
            if (present_in_fragment == false)
                parent_x.push_back(parent_element);
        }
        
        // b je nova pozicija fragmenta u odnosu na parent_x
        int b = rand() % parent_x.size();
        int parent_x_index = 0, fragment_index = 0;
        for (int j = 0; j < parent_size; j++)
            if (j >= b && j < b + l) {
                child.push_back(fragment[fragment_index]);
                fragment_index++;
            }
            else {
                child.push_back(parent_x[parent_x_index]);
                parent_x_index++;
            }

        // Rotacija elemenata tako da se stanje u prvom čvoru nađe na početku vektora
        while (int(child[0] / NUM_OF_DIRECTIONS) != 0) {
            int tmp = child[0];
            for (int j = 0; j < child.size() - 1; j++)
                child[j] = child[j + 1];
            child[child.size() - 1] = tmp;
        }

        // Djetetu se doda početno stanje kako bi se zatvorio puni krug
        child.push_back(child[0]);

        // Ako je rješenje izvedivo, dodaje ga se u novu populaciju
        Solution child_solution = Solution(child, states, cdh, backwards_flag);
        if (child_solution.total_cost < INF)
            (*next_population).insert(child_solution);
    }

    clock_t t_f_finish = clock();
    th.mutation = (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
    return;
}

void save_costs_to_file(string output_file_name, vector<vector<int>> nodes, vector<vector<int>> states, map<int, map<int, double>> costs) {
    ofstream output_file("results_costs/" + output_file_name);
    output_file << "This file contains costs between each two states from a different node.";
    output_file << " The first row contains a description about what this file is,";
    output_file << " the second row contains vector of nodes used in calculation,";
    output_file << " the third row contains discretization of orientations (in how many diferent orientiantions can a robot arrive to a node),";
    output_file << " the fourth row onward represents a matrix written in csv format.";
    output_file << " The matrix's first column contains index of the first state, the second column contains index of the second state and the third column contains the cost between thos two states.";
    output_file << endl;
    output_file << '{';
    for (int i = 0; i < nodes.size(); i++) {
        output_file << '{' << nodes[i][0] << ',' << nodes[i][1] << '}';
        if (i != nodes.size() - 1)
            output_file << ", ";
    }
    output_file << '}';
    output_file << endl;
    output_file << NUM_OF_DIRECTIONS;
    // Nema output_file << endl; jer će se retci matrice dodavati jedan po jedan, a da se sprijeći prazan redak na dnu datoteke, oznaka "endl" će se dodavati zajedno s podacima za redak.
    for (int i = 0; i < states.size() - 1; i++) {
        for (int j = i+1; j < states.size(); j++) {
            if (!(states[i][0] == states[j][0] && states[i][1] == states[j][1])) {
                output_file << endl;
                output_file << i << ',' << j << ',' << costs[i][j];
            }
        }
    }
    output_file.close();
}

void print_and_save_generation(set<Solution> *population, string problem_name, int count, Time_Holder &th) {
    string output_file_name = "generations/" + problem_name + "_gen" + to_string(count) + ".txt";
    ofstream output_file(output_file_name);

    cout << "Generation " << count << ".\n";
    output_file << "Generation " << count << ".\n";
    
    set<Solution>::iterator itr;
    for (itr = (*population).begin(); itr != (*population).end(); itr++) {
        cout << " {";
        output_file << " {";
        int s = (*itr).sol_states[0];
        cout << s;
        output_file << s;
        for (int i = 1; i < (*itr).sol_states.size(); i++) {
            s = (*itr).sol_states[i];
            cout << ", " << s;
            output_file << ", " << s;
        }
        cout << "} cost = " << (*itr).total_cost << endl;
        output_file << "} cost = " << (*itr).total_cost << endl;
    }

    output_file << endl;
    cout << "*total_elapsed_time: " << th.total_elapsed_time << " s\n";
    output_file << "*total_elapsed_time: " << th.total_elapsed_time << " s\n";
    cout << "*elapsed_time_for_current_generation: " << th.elapsed_time_for_current_generation << " s\n";
    output_file << "*elapsed_time_for_current_generation: " << th.elapsed_time_for_current_generation << " s\n";
    cout << "**generating: " << th.generating << " s\n";
    output_file << "**generating: " << th.generating << " s\n";
    cout << "***semirandom_constructon: " << th.semirandom_constructon << " s\n";
    output_file << "***semirandom_constructon: " << th.semirandom_constructon << " s\n";
    cout << "***reproduction: " << th.reproduction << " s\n";
    output_file << "***reproduction: " << th.reproduction << " s\n";
    cout << "***crossover: " << th.crossover << " s\n";
    output_file << "***crossover: " << th.crossover << " s\n";
    cout << "***mutation: " << th.mutation << " s\n";
    output_file << "***mutation: " << th.mutation << " s\n";
    cout << "**improving: " << th.improving << " s\n";
    output_file << "**improving: " << th.improving << " s\n";
    cout << "***inserts: " << th.inserts << " s\n";
    output_file << "***inserts: " << th.inserts << " s\n";
    cout << "***direct_two_opt: " << th.direct_two_opt << " s\n";
    output_file << "***direct_two_opt: " << th.direct_two_opt << " s\n";
    cout << "***two_opt: " << th.two_opt << " s\n";
    output_file << "***two_opt: " << th.two_opt << " s\n";
    cout << "***two_neighbour_swaps: " << th.two_neighbour_swaps << " s\n";
    output_file << "***two_neighbour_swaps: " << th.two_neighbour_swaps << " s\n";
    cout << "***three_neighbour_swaps: "<< th.three_neighbour_swaps << " s\n";
    output_file << "***three_neighbour_swaps: " << th.three_neighbour_swaps << " s\n";
    cout << "***four_neighbour_swaps: " << th.four_neighbour_swaps << " s\n";
    output_file << "***four_neighbour_swaps: " << th.four_neighbour_swaps << " s\n";
    cout << "***optimizeHeadings: " << th.optimizeHeadings << " s\n";
    output_file << "***optimizeHeadings: " << th.optimizeHeadings << " s\n";
/*
cout << "Elapsed time: " << ((double)(t_curr - t_start)/CLOCKS_PER_SEC) << " s" << endl;
output_file << "Elapsed time: " << ((double)(t_curr - t_start)/CLOCKS_PER_SEC) << " s" << endl;
*/
    output_file.close();
}

void print_and_save_OH_at_end(Solution best_solution, string problem_name, Time_Holder &th) {
    string output_file_name = "generations/" + problem_name + "-end_OH.txt";
    ofstream output_file(output_file_name);
    cout << "best_solution from optimizeHeadings() at final generation\n";
    output_file << "best_solution from optimizeHeadings() at final generation\n";
    cout << " {";
    output_file << " {";
    for (int i = 0; i < best_solution.sol_states.size() - 1; i++) {
        cout << best_solution.sol_states[i] << ", ";
        output_file << best_solution.sol_states[i] << ", ";
    }
    cout << best_solution.sol_states[best_solution.sol_states.size() - 1] << "} " << best_solution.total_cost << endl;
    output_file << best_solution.sol_states[best_solution.sol_states.size() - 1] << "} " << best_solution.total_cost << endl;
    output_file << endl;
    cout << "*total_elapsed_time: " << th.total_elapsed_time << " s\n";
    output_file << "*total_elapsed_time: " << th.total_elapsed_time << " s\n";
    cout << "*optimizeHeadings: " << th.optimizeHeadings << " s\n";
    output_file << "*optimizeHeadings: " << th.optimizeHeadings << " s\n";
    
    output_file.close();
}

void print_and_save_final_results(Solution best_solution, string output_file_name, vector<Time_Holder> &v_th) {
    // Računanje ukupnog trajanja rješavanja cijelog problema
    Time_Holder th_total;
    th_total.total_elapsed_time = v_th[v_th.size() - 1].total_elapsed_time;
    for (Time_Holder th : v_th) {
        th_total.generating += th.generating;
        th_total.semirandom_constructon += th.semirandom_constructon;
        th_total.reproduction += th.reproduction;
        th_total.crossover += th.crossover;
        th_total.mutation += th.mutation;

        th_total.improving += th.improving;
        th_total.inserts += th.inserts;
        th_total.direct_two_opt += th.direct_two_opt;
        th_total.two_opt += th.two_opt;
        th_total.two_neighbour_swaps += th.two_neighbour_swaps;
        th_total.three_neighbour_swaps += th.three_neighbour_swaps;
        th_total.four_neighbour_swaps += th.four_neighbour_swaps;
        th_total.optimizeHeadings += th.optimizeHeadings;
    }

    ofstream output_file(output_file_name);

    cout << "Best_solution: {";
    output_file << "Best_solution: {";
    for (int i = 0; i < best_solution.sol_states.size() - 1; i++) {
        cout << best_solution.sol_states[i] << ", ";
        output_file << best_solution.sol_states[i] << ", ";
    }
    cout << best_solution.sol_states[best_solution.sol_states.size() - 1] << "}\n";
    output_file << best_solution.sol_states[best_solution.sol_states.size() - 1] << "}\n";
    cout << "cost: " << best_solution.total_cost << endl;
    output_file << "cost: " << best_solution.total_cost << endl;
    cout << "*total_elapsed_time: " << th_total.total_elapsed_time << " s\n";
    output_file << "*total_elapsed_time: " << th_total.total_elapsed_time << " s\n";
    cout << "**generating: " << th_total.generating << " s\n";
    output_file << "**generating: " << th_total.generating << " s\n";
    cout << "***semirandom_construction: " << th_total.semirandom_constructon << " s\n";
    output_file << "***semirandom_construction: " << th_total.semirandom_constructon << " s\n";
    cout << "***reproduction: " << th_total.reproduction << " s\n";
    output_file << "***reproduction: " << th_total.reproduction << " s\n";
    cout << "***crossover: " << th_total.crossover << " s\n";
    output_file << "***crossover: " << th_total.crossover << " s\n";
    cout << "***mutation: " << th_total.mutation << " s\n";
    output_file << "***mutation: " << th_total.mutation << " s\n";
    cout << "**improving: " << th_total.improving << " s\n";
    output_file << "**improving: " << th_total.improving << " s\n";
    cout << "***inserts: " << th_total.inserts << " s\n";
    output_file << "***inserts: " << th_total.inserts << " s\n";
    cout << "***direct_two_opt: " << th_total.direct_two_opt << " s\n";
    output_file << "***direct_two_opt: " << th_total.direct_two_opt << " s\n";
    cout << "***two_opt: " << th_total.two_opt << " s\n";
    output_file << "***two_opt: " << th_total.two_opt << " s\n";
    cout << "***two_neighbour_swaps: " << th_total.two_neighbour_swaps << " s\n";
    output_file << "***two_neighbour_swaps: " << th_total.two_neighbour_swaps << " s\n";
    cout << "***three_neighbour_swaps: "<< th_total.three_neighbour_swaps << " s\n";
    output_file << "***three_neighbour_swaps: " << th_total.three_neighbour_swaps << " s\n";
    cout << "***four_neighbour_swaps: " << th_total.four_neighbour_swaps << " s\n";
    output_file << "***four_neighbour_swaps: " << th_total.four_neighbour_swaps << " s\n";
    cout << "***optimizeHeadings: " << th_total.optimizeHeadings << " s\n";
    output_file << "***optimizeHeadings: " << th_total.optimizeHeadings << " s\n";

    output_file.close();
}

/*void calculate_and_save_data_for_relative_positions() {
    // Program će pronalaziti sve puteve od početnog čvora {500, 500} do čvorova čije su koordinate x i y u odnosu na pripadne koordinate 500 i 500 udaljene za do 200.
    vector<vector<int>> nodes {{100,100}};
    vector<int> node_start = nodes[0];
    
    ifstream in_cost("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    // Ako datoteka ne postoji postoji, napravi je
    if (in_cost.good() == false) {
        in_cost.close();
        ofstream out("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
        out << "This file contains costs between states calculated by Costs_Data_Holder.";
        out << " The first row contains a description about what this file is,";
        out << " the second row contains vector of nodes used in calculation,";
        out << " the third row contains discretization of orientations (in how many different orientations can a robot arrive to a node),";
        out << " all the other rows contain either flags \"safe_area\", \"sensitive_area\" or keys and costs in csv format.";
        out << " The costs include costs from safe area in which case distance is calculated and stored based on a relative position of two states";
        out << " and costs from sensitive area in which case distance is calculated and stored based on an absolute position of two states.";
        out << " \"safe_area\" signifies that data about costs between relative states in safe area starts from the first row below and continues onwards until \"costs_sensitive_area\" is reached.";
        out << " \"sensitive_area\" signifies that data about costs between absolute states in safe area starts from the first row below and continues onwards until the end of the file is reached.";
        out << " In rows that isn't one of the first three rows or doesn't contain the flags \"safe_area\" nor \"sensitive_area\", one row contains information about one pair of states.";
        out << " Rows containing data about states in safe are have format \"delta_x,delta_y,fi_start,fi_finish,cost\",";
        out << " where delta_x and delta_y are differences in coordinates of the two states, fi_start and fi_finish are orientations of the first and the second state respectively";
        out << " and cost is the cost between those two states.";
        out << " Rows containing data about sensitive states have format \"state_start_index,state_finish_index,cost\",";
        out << " where state_start_index and state_finish_index are indexes of the first and the second state respectively and cost is a cost between those two states.";

        out << endl << '{';
        for (int i = 0; i < nodes.size(); i++) {
            out << '{' << nodes[i][0] << ',' << nodes[i][1] << '}';
            if (i < nodes.size() - 1)
                out << ", ";
        }
        out << '}';

        out << endl << NUM_OF_DIRECTIONS;
        out << endl << "safe_area";
        out << endl << "sensitive_area";

        ofstream out2("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
        out2 << "This file contains paths between states.";
        out2 << " One row represents the pair of states,";
        out2 << " the first row below it represents size of an array in which data about the path is stored";
        out2 << " and the second row below represents data about the path between them.";

        out2 << endl << '{';
        for (int i = 0; i < nodes.size(); i++) {
            out2 << '{' << nodes[i][0] << ',' << nodes[i][1] << '}';
            if (i < nodes.size() - 1)
                out2 << ", ";
        }
        out2 << '}';

        out2 << endl << NUM_OF_DIRECTIONS;
        out2 << endl << "safe_area";
        out2 << endl << "sensitive_area";
    }
    else {
        in_cost.close();
    }


    /*Costs_Data_Holder cdh1;
    cdh1.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cdh1.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    vector<pair<string, string>> dif;
    for (int i = 0; i <= 15; i++) {
        for (int j = 0; j <=15; j++) {
            for (int fi1 = 0; fi1 <= 15; fi1++) {
                for (int fi2 = 0; fi2 <= 15; fi2++) {
                    cout << i << ',' << j << ',' << fi1 << ',' << fi2 << endl;
                    // 5,0,4,8
                    if (i == 5 && j == 0 && fi1 == 4 && fi2 == 9) {
                        "Dosli smo do kraja\n";
                        cout << endl;
                        cout << "Pronadeni razliciti:\n";
                        for (pair<string, string> pr : dif) {
                            cout << ' ' << pr.first << " " << pr.second << endl;
                        }
                        exit(0);
                    }
                    if (i < 4) {
                        cout << "PRESKOCENO\n";
                        continue;
                    }
                    else if (i == 4) {
                        if (j < 4) {
                            cout << "PRESKOCENO\n";
                            continue;
                        }
                        else if (j == 4) {
                            if (fi1 < 14) {
                                cout << "PRESKOCENO\n";
                                continue;
                            }
                            else if (fi1 == 14) {
                                if (fi2 <= 2) {
                                    cout << "PRESKOCENO\n";
                                    continue;
                                }
                            }
                        }
                    }

                    if (cdh1.costs_safe_area[j][-i][cdh1.rotate_angle_for_degree(fi1, -4)][cdh1.rotate_angle_for_degree(fi2, -4)] != 0) {
                        string k1 = "[" + to_string(i) + "][" + to_string(j) + "][" + to_string(fi1) + "][" + to_string(fi2) + "]";
                        string k2 = "[" + to_string(j) + "][" + to_string(-i) + "][" + to_string(cdh1.rotate_angle_for_degree(fi1, -4)) + "][" + to_string(cdh1.rotate_angle_for_degree(fi2, -4)) + "]";
                        cout << "cdh1.costs_safe_area" + k1 + " = " << cdh1.costs_safe_area[i][j][fi1][fi2];
                        cout << " cdh1.costs_safe_area" + k2 + " = " << cdh1.costs_safe_area[j][-i][cdh1.rotate_angle_for_degree(fi1, -4)][cdh1.rotate_angle_for_degree(fi2, -4)] << endl;
                        Path p = cdh1.rotate_path_for_degree(cdh1.paths_safe_area[j][-i][cdh1.rotate_angle_for_degree(fi1, -4)][cdh1.rotate_angle_for_degree(fi2, -4)], 4);
                        cout << " Prethodni : ";
                        for (int el : cdh1.paths_safe_area[i][j][fi1][fi2].data) cout << el << ',';
                        cout << "\n Sada : ";
                        for (int el : p.data) cout << el << ',';
                        cout << endl;
                        bool razlicito = false;
                        if (cdh1.paths_safe_area[i][j][fi1][fi2].data.size() != p.data.size()) {
                            razlicito = true;
                        }
                        for (int k = 0; (k < p.data.size()) && (razlicito != true); k++) {
                            if (cdh1.paths_safe_area[i][j][fi1][fi2].data[k] != p.data[k]) {
                                razlicito = true;
                            }
                        }

                        if (razlicito == true) {
                            dif.push_back(make_pair(k1, k2));
                        }
                    }
                    if (cdh1.costs_safe_area[-j][i][cdh1.rotate_angle_for_degree(fi1, 4)][cdh1.rotate_angle_for_degree(fi2, 4)] != 0) {
                        string k1 = "[" + to_string(i) + "][" + to_string(j) + "][" + to_string(fi1) + "][" + to_string(fi2) + "]";
                        string k2 = "[" + to_string(-j) + "][" + to_string(i) + "][" + to_string(cdh1.rotate_angle_for_degree(fi1, 4)) + "][" + to_string(cdh1.rotate_angle_for_degree(fi2, 4)) + "]";
                        cout << "cdh1.costs_safe_area" + k1 + " = " << cdh1.costs_safe_area[i][j][fi1][fi2];
                        cout << " cdh1.costs_safe_area" + k2 + " = " << cdh1.costs_safe_area[-j][i][cdh1.rotate_angle_for_degree(fi1, 4)][cdh1.rotate_angle_for_degree(fi2, 4)] << endl;
                        Path p = cdh1.rotate_path_for_degree(cdh1.paths_safe_area[-j][i][cdh1.rotate_angle_for_degree(fi1, 4)][cdh1.rotate_angle_for_degree(fi2, 4)], -4);
                        cout << " Prethodni : ";
                        for (int el : cdh1.paths_safe_area[i][j][fi1][fi2].data) cout << el << ',';
                        cout << "\n Sada : ";
                        for (int el : p.data) cout << el << ',';
                        cout << endl;
                        bool razlicito = false;
                        if (cdh1.paths_safe_area[i][j][fi1][fi2].data.size() != p.data.size()) {
                            razlicito = true;
                        }
                        cout << "p.data.size() = " << p.data.size() << endl;
                        for (int k = 0; (k < p.data.size()) && (razlicito != true); k++) {
                            if (cdh1.paths_safe_area[i][j][fi1][fi2].data[k] != p.data[k]) {
                                razlicito = true;
                            }
                        }

                        if (razlicito == true) {
                            dif.push_back(make_pair(k1, k2));
                        }
                    }
                }
            }
        }
    }
    exit(0);*/

/*
    // TODO ne zaboraviti staviti i za negativne vrijednosti
    for (int i = (int)1; i <= (int)14; i++) {
        for (int j = (int)0; j <= (int)15; j++) {
            if (i != 0 || j != 0) {
                vector<int> node_curr = {nodes[0][0] + i, nodes[0][1] + j};
                nodes.push_back(node_curr);
                for (int fi1 = 0; fi1 < 16; fi1++) {
                    for (int fi2 = 0; fi2 < 16; fi2++) {
                        cout << "\nArrived at (i,j,fi1,fi2) = (" << i << ',' << j << ',' << fi1 << ',' << fi2 << ")\n";
                        
                        if (i < 14) {
                            cout << "skipping (i,j,fi1,fi2) = (" << i << ',' << j << ',' << fi1 << ',' << fi2 << ")\n";
                            continue;
                        }
                        if (i == 14) {
                            if (j < 8) {
                                cout << "skipping (i,j,fi1,fi2) = (" << i << ',' << j << ',' << fi1 << ',' << fi2 << ")\n";
                                continue;
                            }
                            else if (j == 8) {
                                if (fi1 < 15) {
                                    cout << "skipping (i,j,fi1,fi2) = (" << i << ',' << j << ',' << fi1 << ',' << fi2 << ")\n";
                                    continue;
                                }
                                else if (fi1 == 15) {
                                    if (fi2 <= 5) {
                                        cout << "skipping (i,j,fi1,fi2) = (" << i << ',' << j << ',' << fi1 << ',' << fi2 << ")\n";
                                        continue;
                                    }
                                }
                            }
                        }

                        Costs_Data_Holder cdh;

                        cdh.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
                        cdh.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");

                        vector<int> state1 = node_start, state2 = node_curr;
                        state1.push_back(fi1);
                        state2.push_back(fi2);

                        int delta_x = state2[0] - state1[0], delta_y = state2[1] - state1[1];
                        double t_elapsed = 0.0;

                        // Provjera nalazi li se već ključ u listi ključeva
                        cout << "Provjera je li ključ već unutra za {" << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << "} "; 
                        bool nalazi = false;
                        for (vector<int> key : cdh.costs_safe_area_keys) {
                            if (key[0] == delta_x && key[1] == delta_y && key[2] == fi1 && key[3] == fi2) {
                                nalazi = true;
                                break;
                            }
                        }
                        if (nalazi == true) {
                            cout << "PRESKOČENO jer se već nalazi.\n";
                            continue;
                        }
                        else {
                            cout << endl;
                        }

                        if (cdh.costs_safe_area[delta_x][delta_y][fi1][fi2] == 0) {
                            cout << "Trazenje za " << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << " (i,j)=(" << i << ',' << j <<")\n";
                            clock_t t_start = clock();
                            cdh.determine_cost_in_safe_area(state1, state2);
                            clock_t t_end = clock();
                            t_elapsed = (double)(t_end - t_start)/CLOCKS_PER_SEC;
                            cout << "Pronadeno za " << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << " (i,j)=(" << i << ',' << j <<") (time taken: " << t_elapsed << " s)\n";
                        }

                        cdh.save_cdh_to_file("precalculated_data/precalculated_data_for_safe_area-TEMP.txt", nodes, {});
                        remove("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
                        rename("results_costs/precalculated_data/precalculated_data_for_safe_area-TEMP.txt", "results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
                        cdh.save_paths_to_file("precalculated_data/precalculated_data_for_safe_area-TEMP.txt", nodes, {});
                        remove("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
                        rename("results_paths/precalculated_data/precalculated_data_for_safe_area-TEMP.txt", "results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
                        ifstream in("results_costs/precalculated_data/precalculated_data_for_safe_area-processed_states.txt");
                        vector<string> tmp;
                        string line;
                        while (getline(in, line))
                            tmp.push_back(line);
                        in.close();
                        ofstream out("results_costs/precalculated_data/precalculated_data_for_safe_area-processed_states-TEMP.txt");
                        for (int k = 0; k < tmp.size(); k++)
                            out << tmp[k] << endl;
                        out << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << " (i,j)=(" << i << ',' << j <<") (time taken: " << t_elapsed << " s)";
                        out.close();
                        remove("results_costs/precalculated_data/precalculated_data_for_safe_area-processed_states.txt");
                        rename("results_costs/precalculated_data/precalculated_data_for_safe_area-processed_states-TEMP.txt", "results_costs/precalculated_data/precalculated_data_for_safe_area-processed_states.txt");
                        cout << "Finished (i,j,fi1,fi2) = (" << i << ',' << j << ',' << fi1 << ',' << fi2 << ")\n";
                    }
                }
            }
        }
    }
    cout << "Završena cijela funkcija calculate_and_save_data_for_relative_positions()\n";
}*/

/*void calculate_data_for_circle_with_squares() {
    clock_t t_start = clock(), t_finish, t_1, t_2;
    vector<vector<int>> nodes = {{50,50},{43,57},{50,64},{57,57}};
    for (int i = 0; i < 4; i++) {
        vector<int> n = nodes[i];
        for (int dx : {-1, 1}) {
            for (int dy : {-1, 1}) {
                vector<int> n_new = n;
                n_new[0] += dx;
                n_new[1] += dy;
                nodes.push_back(n_new);
            }
        }
    }
    vector<vector<int>> states = initialize_states(nodes);
    Costs_Data_Holder cdh, cdh_temp;
    cout << "Čitanje duljina u cdh_temp\n";
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/sinusoid_02.txt");
    cout << "Čitanje puteva u cdh_temp\n";
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/sinusoid_02.txt");
    cout << "Traženje već poznatih stanja\n";
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            vector<int> state1 = states[s1], state2 = states[s2];
            cout << " Provjera za states[" << s1 << "] = {" << state1[0] << ',' << state1[1] << ',' << state1[2] << "}; states[" << s2 << "] = {" << state2[0] << ',' << state2[1] << ',' << state2[2] << "}\n"; 
            if (!(state1[0] == state2[0] && state1[1] == state2[1])) {
                cout << " Različiti čvorovi\n";
                int delta_x = state2[0] - state1[0], delta_y = state2[1] - state1[1], fi1 = state1[2], fi2 = state2[2];
                cout << " " << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << endl;
                if (cdh_temp.get_cost_for_safe_area(state1, state2) != 0) {
                    cdh.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.get_cost_for_safe_area(state1, state2);
                    cdh.costs_safe_area_keys.push_back({delta_x, delta_y, fi1, fi2});
                    cdh.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.paths_safe_area[delta_x][delta_y][fi1][fi2];
                }
            }
            cout << " -------------------\n";
        }
    }

    cout << "Određivanje nepoznatih putevna\n";
    for (int s1 = 0; s1 < states.size(); s1++)
        for (int s2 = 0; s2 < states.size(); s2++)
            if (!(states[s1][0] == states[s2][0] && states[s1][1] == states[s2][1])) {
                cout << " Računanje za s1 = " << s1 << "; s2 = " << s2 << "; {" << states[s1][0] << ',' << states[s1][1] << ',' << states[s1][2] << "}; {";
                cout << states[s2][0] << ',' << states[s2][1] << ',' << states[s2][2] << "}\n";
                t_1 = clock();
                cdh.determine_cost(s1, s2, states);
                t_2 = clock();
                cout << " Izračunato! (Time taken: " << (double)(t_2 - t_1)/CLOCKS_PER_SEC <<  "s; Total time elapsed: " << (double)(t_2 - t_start)/CLOCKS_PER_SEC << " s)\n";
            }

    
    cout << "Spremanje rezltata\n";
    cdh.save_cdh_to_file("circle_with_squares01.txt", nodes, states);
    cout << " Spremljene udaljenosti\n";
    cdh.save_paths_to_file("circle_with_squares01.txt", nodes, states);
    cout << " Spremljeni putevi\n";
    t_finish = clock();
    cout << "GOTOVO (Total time elapsed: " << (double)(t_finish - t_start)/CLOCKS_PER_SEC << " s\n";
}*/

/*void calculate_data_for_two_bushes() {
    clock_t t_start = clock();
    vector<vector<int>> nodes = {{47,50}, {50,50}, {47,47}, {50,47}, {60,50}, {63,50}, {60,53}, {63,53}};
    vector<vector<int>> states = initialize_states(nodes);

    Costs_Data_Holder cdh, cdh_temp;
    cout << "Čitanje duljina u cdh_temp\n";
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Čitanje puteva u cdh_temp\n";
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Traženje već poznatih stanja\n";
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            vector<int> state1 = states[s1], state2 = states[s2];
            cout << " Provjera za states[" << s1 << "] = {" << state1[0] << ',' << state1[1] << ',' << state1[2] << "}; states[" << s2 << "] = {" << state2[0] << ',' << state2[1] << ',' << state2[2] << "}\n"; 
            if (!(state1[0] == state2[0] && state1[1] == state2[1])) {
                cout << " Različiti čvorovi\n";
                int delta_x = state2[0] - state1[0], delta_y = state2[1] - state1[1], fi1 = state1[2], fi2 = state2[2];
                cout << " " << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << endl;
                if (cdh_temp.get_cost_for_safe_area(state1, state2) != 0) {
                    cdh.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.get_cost_for_safe_area(state1, state2);
                    cdh.costs_safe_area_keys.push_back({delta_x, delta_y, fi1, fi2});
                    cdh.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.paths_safe_area[delta_x][delta_y][fi1][fi2];
                }
            }
            cout << " -------------------\n";
        }
    }

    cout << "Određivanje nepoznatih putevna\n";
    for (int s1 = 0; s1 < states.size(); s1++)
        for (int s2 = 0; s2 < states.size(); s2++)
            if (!(states[s1][0] == states[s2][0] && states[s1][1] == states[s2][1])) {
                cout << " Računanje za s1 = " << s1 << "; s2 = " << s2 << "; {" << states[s1][0] << ',' << states[s1][1] << ',' << states[s1][2] << "}; {";
                cout << states[s2][0] << ',' << states[s2][1] << ',' << states[s2][2] << "}\n";
                cdh.determine_cost(s1, s2, states);
                cout << " Izračunato!\n";
            }

    cout << "Spremanje rezltata\n";
    cdh.save_cdh_to_file("problem_two_bushes_02.txt", nodes, states);
    cout << " Spremljene udaljenosti\n";
    cdh.save_paths_to_file("problem_two_bushes_02.txt", nodes, states);
    cout << " Spremljeni putevi\n";

    clock_t t_finish = clock();
    cout << "GOTOVO\n (time passed: " << (double)(t_finish - t_start)/CLOCKS_PER_SEC << " s)\n";
}*/

/*void calculate_data_for_line_with_bump() {
    clock_t t_start = clock();
    vector<vector<int>> nodes = {{50,50},{60,50},{60,53},{70,50}};
    vector<vector<int>> states = initialize_states(nodes);

    Costs_Data_Holder cdh, cdh_temp;
    cout << "Čitanje duljina u cdh_temp\n";
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Čitanje puteva u cdh_temp\n";
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Traženje već poznatih stanja\n";
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            vector<int> state1 = states[s1], state2 = states[s2];
            cout << " Provjera za states[" << s1 << "] = {" << state1[0] << ',' << state1[1] << ',' << state1[2] << "}; states[" << s2 << "] = {" << state2[0] << ',' << state2[1] << ',' << state2[2] << "}\n"; 
            if (!(state1[0] == state2[0] && state1[1] == state2[1])) {
                cout << " Različiti čvorovi\n";
                int delta_x = state2[0] - state1[0], delta_y = state2[1] - state1[1], fi1 = state1[2], fi2 = state2[2];
                cout << " " << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << endl;
                if (cdh_temp.get_cost_for_safe_area(state1, state2) != 0) {
                    cdh.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.get_cost_for_safe_area(state1, state2);
                    cdh.costs_safe_area_keys.push_back({delta_x, delta_y, fi1, fi2});
                    cdh.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.paths_safe_area[delta_x][delta_y][fi1][fi2];
                }
            }
            cout << " -------------------\n";
        }
    }

    cout << "Određivanje nepoznatih putevna\n";
    for (int s1 = 0; s1 < states.size(); s1++)
        for (int s2 = 0; s2 < states.size(); s2++)
            if (!(states[s1][0] == states[s2][0] && states[s1][1] == states[s2][1])) {
                cout << " Računanje za s1 = " << s1 << "; s2 = " << s2 << "; {" << states[s1][0] << ',' << states[s1][1] << ',' << states[s1][2] << "}; {";
                cout << states[s2][0] << ',' << states[s2][1] << ',' << states[s2][2] << "}\n";
                cdh.determine_cost(s1, s2, states);
                cout << " Izračunato!\n";
            }

    cout << "Spremanje rezltata\n";
    cdh.save_cdh_to_file("line_wiht_bump.txt", nodes, states);
    cout << " Spremljene udaljenosti\n";
    cdh.save_paths_to_file("line_wiht_bump.txt", nodes, states);
    cout << " Spremljeni putevi\n";
    clock_t t_finish = clock();
    cout << "GOTOVO\n (time passed: " << (double)(t_finish - t_start)/CLOCKS_PER_SEC << " s)\n";
}*/

/*void calculate_data_for_paralel_order() {
    clock_t t_start = clock();
    vector<vector<int>> nodes = {{50,50}, {50,53}, {50,56}, {50,59}, {60,50}, {60, 53}, {60, 56}, {60, 59}};
    vector<vector<int>> states = initialize_states(nodes);

    Costs_Data_Holder cdh, cdh_temp;
    cout << "Čitanje duljina u cdh_temp (1/3)\n";
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Čitanje duljina u cdh_temp (2/3)\n";
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/problem_two_bushes_01.txt");
    cout << "Čitanje duljina u cdh_temp (3/3)\n";
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/line_wiht_bump.txt");
    cout << "Čitanje puteva u cdh_temp (1/3)\n";
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Čitanje puteva u cdh_temp (2/3)\n";
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/problem_two_bushes_01.txt");
    cout << "Čitanje puteva u cdh_temp (3/3)\n";
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/line_wiht_bump.txt");
    cout << "Traženje već poznatih stanja\n";
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            vector<int> state1 = states[s1], state2 = states[s2];
            cout << " Provjera za states[" << s1 << "] = {" << state1[0] << ',' << state1[1] << ',' << state1[2] << "}; states[" << s2 << "] = {" << state2[0] << ',' << state2[1] << ',' << state2[2] << "}\n"; 
            if (!(state1[0] == state2[0] && state1[1] == state2[1])) {
                cout << " Različiti čvorovi\n";
                int delta_x = state2[0] - state1[0], delta_y = state2[1] - state1[1], fi1 = state1[2], fi2 = state2[2];
                cout << " " << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << endl;
                if (cdh_temp.get_cost_for_safe_area(state1, state2) != 0) {
                    cdh.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.get_cost_for_safe_area(state1, state2);
                    cdh.costs_safe_area_keys.push_back({delta_x, delta_y, fi1, fi2});
                    cdh.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.paths_safe_area[delta_x][delta_y][fi1][fi2];
                }
            }
            cout << " -------------------\n";
        }
    }

    cout << "Određivanje nepoznatih putevna\n";
    for (int s1 = 0; s1 < states.size(); s1++)
        for (int s2 = 0; s2 < states.size(); s2++)
            if (!(states[s1][0] == states[s2][0] && states[s1][1] == states[s2][1])) {
                cout << " Računanje za s1 = " << s1 << "; s2 = " << s2 << "; {" << states[s1][0] << ',' << states[s1][1] << ',' << states[s1][2] << "}; {";
                cout << states[s2][0] << ',' << states[s2][1] << ',' << states[s2][2] << "}\n";
                cdh.determine_cost(s1, s2, states);
                cout << " Izračunato!\n";
            }

    cout << "Spremanje rezltata\n";
    cdh.save_cdh_to_file("paralel_order.txt", nodes, states);
    cout << " Spremljene udaljenosti\n";
    cdh.save_paths_to_file("paralel_order.txt", nodes, states);
    cout << " Spremljeni putevi\n";
    clock_t t_finish = clock();
    cout << "GOTOVO\n (time passed: " << (double)(t_finish - t_start)/CLOCKS_PER_SEC << " s)\n";
}*/

/*void update_precalculated_data_files() {
    //vector<vector<int>> nodes = {{100,100}, {101,100}, {101,101}, {101,102}, {101,103}, {101,104}, {101,105}, {101,106}, {101,107}, {101,108}, {101,109}, {101,110}, {101,111}, {101,112}, {101,113}, {101,114}, {101,115}, {102,100}, {102,101}, {102,102}, {102,103}, {102,104}, {102,105}, {102,106}, {102,107}, {102,108}, {102,109}, {102,110}, {102,111}, {102,112}, {102,113}, {102,114}, {102,115}, {103,100}, {103,101}, {103,102}, {103,103}, {103,104}, {103,105}};
    vector<vector<int>> nodes = {{100,100}, {101,100}, {101,101}, {101,102}, {101,103}, {101,104}, {101,105}, {101,106}, {101,107}, {101,108}, {101,109}, {101,110}, {101,111}, {101,112}, {101,113}, {101,114}, {101,115}, {102,100}, {102,101}, {102,102}, {102,103}, {102,104}, {102,105}, {102,106}, {102,107}, {102,108}, {102,109}, {102,110}, {102,111}, {102,112}, {102,113}, {102,114}, {102,115}, {103,100}, {103,101}, {103,102}, {103,103}, {103,104}, {103,105}, {100,97}, {97,100}, {113,100}, {116,100}, {113,97}, {116,97}, {110,97}, {113,103}, {116,103}, {110,103}, {87,100}, {87,97}, {90,97}, {84,100}, {84,97}, {87,103}, {90,103}, {84,103}, {100,91}, {110,106}, {110,109}, {110,94}, {110,91}, {90,106}, {90,109}, {90,94}, {90,91}};

    Costs_Data_Holder cdh_prec, cdh_two_bush, cdh_line_bump, cdh_paralel;
    cout << "Čitanje duljina in precalculated_data_for_safe_area.txt\n";
    cdh_prec.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Čitanje puteva iz precalculated_data_for_safe_area.txt\n";
    cdh_prec.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    //cdh_two_bush.read_costs_from_file_with_cdh_format("results_costs/problem_two_bushes_01.txt");
    //cdh_two_bush.read_paths_from_file_with_cdh_format("results_paths/problem_two_bushes_01.txt");
    cout << "Čitanje duljina iz line_wiht_bump.txt\n";
    cdh_line_bump.read_costs_from_file_with_cdh_format("results_costs/line_wiht_bump.txt");
    cout << "Čitanje puteva iz line_wiht_bump.txt\n";
    cdh_line_bump.read_paths_from_file_with_cdh_format("results_paths/line_wiht_bump.txt");
    //cdh_paralel.read_costs_from_file_with_cdh_format("results_costs/paralel_order.txt");
    //cdh_paralel.read_paths_from_file_with_cdh_format("results_paths/paralel_order.txt");

    // two_bush
    for (vector<int> k1 : cdh_two_bush.costs_safe_area_keys) {
        bool present = false;
        for (vector<int> k2 : cdh_prec.costs_safe_area_keys) {
            if (k1 == k2) {
                present = true;
                break;
            }
        }
        if (present == false) {
            int delta_x = k1[0], delta_y = k1[1], fi1 = k1[2], fi2 = k1[3];
            nodes.push_back({100 + delta_x, 100 + delta_y});
            cdh_prec.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_two_bush.costs_safe_area[delta_x][delta_y][fi1][fi2];
            cdh_prec.costs_safe_area_keys.push_back(k1);
            cdh_prec.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_two_bush.paths_safe_area[delta_x][delta_y][fi1][fi2];
        }
    }
    // line_bump
    cout << "Dodavanje podataka u cdh_prec\n";
    int count = 0, velicina = cdh_line_bump.costs_safe_area_keys.size();
    bool node_inserted = false;
    for (vector<int> k1 : cdh_line_bump.costs_safe_area_keys) {
        count++;
        cout << " {" << k1[0] << ',' << k1[1] << ',' << k1[2] << ',' << k1[3] << "} " << count << '/' << velicina << endl;
        bool present = false;
        for (vector<int> k2 : cdh_prec.costs_safe_area_keys) {
            if (k1 == k2) {
                present = true;
                break;
            }
        }
        if (present == false) {
            cout << "  Dodavanje...\n";
            int delta_x = k1[0], delta_y = k1[1], fi1 = k1[2], fi2 = k1[3];
            cdh_prec.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_line_bump.costs_safe_area[delta_x][delta_y][fi1][fi2];
            cdh_prec.costs_safe_area_keys.push_back(k1);
            cdh_prec.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_line_bump.paths_safe_area[delta_x][delta_y][fi1][fi2];
            if (node_inserted == false) {
                nodes.push_back({100 + delta_x, 100 + delta_y});
                node_inserted = true;
            }
        }
    }
    // paralel
    for (vector<int> k1 : cdh_paralel.costs_safe_area_keys) {
        bool present = false;
        for (vector<int> k2 : cdh_prec.costs_safe_area_keys) {
            if (k1 == k2) {
                present = true;
                break;
            }
        }
        if (present == false) {
            int delta_x = k1[0], delta_y = k1[1], fi1 = k1[2], fi2 = k1[3];
            nodes.push_back({100 + delta_x, 100 + delta_y});
            cdh_prec.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_paralel.costs_safe_area[delta_x][delta_y][fi1][fi2];
            cdh_prec.costs_safe_area_keys.push_back(k1);
            cdh_prec.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_paralel.paths_safe_area[delta_x][delta_y][fi1][fi2];
        }
    }

    cout << "Spremanje cijene u results_costs/precalculated_data/precalculated_data_for_safe_area-TEMP.txt\n";
    cdh_prec.save_cdh_to_file("precalculated_data/precalculated_data_for_safe_area-TEMP.txt", nodes, {});
    cout << "Brisanje results_costs/precalculated_data/precalculated_data_for_safe_area.txt\n";
    remove("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Preimenovanje\n";
    rename("results_costs/precalculated_data/precalculated_data_for_safe_area-TEMP.txt", "results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Spremanje puteva u results_paths/precalculated_data/precalculated_data_for_safe_area-TEMP.txt\n";
    cdh_prec.save_paths_to_file("precalculated_data/precalculated_data_for_safe_area-TEMP.txt", nodes, {});
    cout << "Brisanje results_paths/precalculated_data/precalculated_data_for_safe_area.txt\n";
    remove("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "Preimenovanje\n";
    rename("results_paths/precalculated_data/precalculated_data_for_safe_area-TEMP.txt", "results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    cout << "GOTOVO!\n";
}*/

/*void calculate_data_for_sinusoid_02() {
    clock_t t_start = clock(), t_1, t_2;
    vector<vector<int>> nodes = {{100,100}, {101,103}, {102,105}, {104,106}, {106,105}, {107,103}, {108,100}, {109,97}, {110,95}, {112,94}, {114,95}, {115,97},
                                 {116,100}, {117,103}, {118,105}, {120,106}, {122,105}, {123,103}, {124,100}, {125,97}, {126,95}, {128,94}, {130,95}, {131,97},
                                 {132,100}};
    vector<vector<int>> states = initialize_states(nodes);

    // PROVJERI DO KOJEG JE STANJA RAČUNANJE DOŠLO
    /*Costs_Data_Holder c;
    c.read_costs_from_file_with_cdh_format("results_costs/sinusoid_02.txt");
    c.read_paths_from_file_with_cdh_format("results_paths/sinusoid_02.txt");
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            vector<int> ss1 = states[s1], ss2 = states[s2];
            cout << "s1,s2 = " << s1 << ',' << s2 << " ss1 = {";
            for (int el : ss1) cout << el << ", ";
            cout << "} ss2 = {";
            for (int el : ss2) cout << el << ", ";
            cout << "} ";

            if (ss1[0] == ss2[0] && ss1[1] == ss2[1]) {
                cout  << "ISTI CVOR\n";
                continue;
            }
            
            int dx = ss2[0] - ss1[0], dy = ss2[1] - ss1[1], fi1 = ss1[2], fi2 = ss2[2];
            if (c.costs_safe_area[dx][dy][fi1][fi2] != 0) {
                cout << "NADENO\n";
            }
            else {
                cout << "NIJE NADENO\n";
                exit(0);
            }
        }
    }*/

/*    Costs_Data_Holder cdh, cdh_temp;
    cout << "Čitanje duljina u cdh_temp\n";
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/sinusoid_01.txt");
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/sinusoid_02.txt");
    cout << "Čitanje puteva u cdh_temp\n";
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/sinusoid_01.txt");
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/sinusoid_02.txt");
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            vector<int> state1 = states[s1], state2 = states[s2];
            cout << " Provjera za states[" << s1 << "] = {" << state1[0] << ',' << state1[1] << ',' << state1[2] << "}; states[" << s2 << "] = {" << state2[0] << ',' << state2[1] << ',' << state2[2] << "}\n"; 
            if (!(state1[0] == state2[0] && state1[1] == state2[1])) {
                cout << " Različiti čvorovi\n";
                int delta_x = state2[0] - state1[0], delta_y = state2[1] - state1[1], fi1 = state1[2], fi2 = state2[2];
                cout << " " << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << endl;
                if (cdh_temp.get_cost_for_safe_area(state1, state2) != 0) {
                    cdh.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.get_cost_for_safe_area(state1, state2);
                    cdh.costs_safe_area_keys.push_back({delta_x, delta_y, fi1, fi2});
                    cdh.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.paths_safe_area[delta_x][delta_y][fi1][fi2];
                }
            }
            cout << " -------------------\n";
        }
    }

    cout << "Prvo spremanje u datoteku\n";
    cdh.save_cdh_to_file("sinusoid_02.txt", nodes, {});
    cdh.save_paths_to_file("sinusoid_02.txt", nodes, {});
    cout << "Prvo spremanje završeno\n";

    cout << "Određivanje nepoznatih putevna\n";
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            // Prvo provjeri jesu li stanja već prođena
            cout << "(s1,s2) = (" << s1 << ',' << s2 << ")\n";
            if (s1 < 13) {
                cout << "SKIPPED\n";
                continue;
            }
            else {
                if (s1 == 13) {
                    if (s2 <= 152) {
                        cout << "SKIPPED\n";
                        continue;
                    }
                }
            }


            if (!(states[s1][0] == states[s2][0] && states[s1][1] == states[s2][1])) {
                cout << " Računanje za s1 = " << s1 << "; s2 = " << s2 << "; {" << states[s1][0] << ',' << states[s1][1] << ',' << states[s1][2] << "}; {";
                cout << states[s2][0] << ',' << states[s2][1] << ',' << states[s2][2] << "}\n";
                t_1 = clock();
                cdh.determine_cost(s1, s2, states);
                t_2 = clock();
                cout << " Izračunato! (Time taken: " << (double)(t_2 - t_1)/CLOCKS_PER_SEC << " s; Total time elapsed: " << (double)(t_2 - t_start)/CLOCKS_PER_SEC << "s)\n";

                cout << " Spremanje cost u TEMP datoteku\n";
                cdh.save_cdh_to_file("sinusoid_02-TEMP.txt", nodes, {});
                cout << "Brisanje results_costs/sinusoid_02.txt\n";
                remove("results_costs/sinusoid_02.txt");
                cout << "Preimenovanje\n";
                rename("results_costs/sinusoid_02-TEMP.txt", "results_costs/sinusoid_02.txt");
                cout << "Spremanje path u TEMP datoteku\n";
                cdh.save_paths_to_file("sinusoid_02-TEMP.txt", nodes, {});
                cout << "Brisanje results_paths/sinusoid_02.txt\n";
                remove("results_paths/sinusoid_02.txt");
                cout << "Preimenovanje\n";
                rename("results_paths/sinusoid_02-TEMP.txt", "results_paths/sinusoid_02.txt");
            }
        }
    }

    clock_t t_finish = clock();
    cout << "GOTOVO\n (time passed: " << (double)(t_finish - t_start)/CLOCKS_PER_SEC << " s)\n";
}*/

void merge_two_files() {
    cout << "merge_two_files()\n";
    // Ovdje se moraju definirati elementi svih čvorova (nodes1 čvorovi zapisani u prvoj datoteci, a nodes2 čvorovi zapisani u drugoj datoteci)
    vector<vector<int>> nodes1 = {};
    vector<vector<int>> nodes2 = {};

    Costs_Data_Holder cdh1, cdh2;
    cdh1.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area-BIG_STEPS.txt");
    cdh1.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area-BIG_STEPS.txt");
    cdh2.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area-BIG_STEPS2.txt");
    cdh2.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area-BIG_STEPS2.txt");

    for (vector<int> n2 : nodes2) {
        cout << "Provjera prisutnosti n2 = {" << n2[0] << ',' << n2[1] << "} u nodes1\n";
        bool present = false;
        for (vector<int> n1 : nodes1) {
            if (n1 == n2) {
                present = true;
                break;
            }
        }
        if (present == false) {
            cout << " Nije prisutan (dodavanje)\n";
            nodes1.push_back(n2);
        }
        else {
            cout << " Prisutan (ne dodaje se)\n";
        }
    }

    for (vector<int> k2 : cdh2.costs_safe_area_keys) {
        // Provjera je li element već u cdh1
        cout << "Provjera pristunosti kljuca k2 = {" << k2[0] << ',' << k2[1] << ',' << k2[2] << ',' << k2[3] << "} u cdh1.costs_safe_area_keys\n";
        bool present = false;
        for (vector<int> k1 : cdh1.costs_safe_area_keys) {
            if (k1 == k2) {
                present = true;
                break;
            }
        }
        if (present == false) {
            cout <<" Nije prisutan (dodavanje)\n";
            int delta_x = k2[0], delta_y = k2[1], fi_start = k2[2], fi_finish = k2[3];
            cdh1.costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = cdh2.costs_safe_area[delta_x][delta_y][fi_start][fi_finish];
            cdh1.paths_safe_area[delta_x][delta_y][fi_start][fi_finish] = cdh2.paths_safe_area[delta_x][delta_y][fi_start][fi_finish];
            cdh1.costs_safe_area_keys.push_back(k2);
        }
        else {
            cout << " Prisutan (ne dodaje se)\n";
        }
    }

    cout << "SPREMANJE NOVOG cdh1 U  DATOTEKU...\n";
    cdh1.save_cdh_to_file("precalculated_data/precalculated_data_for_safe_area.txt", nodes1, {});
    cdh1.save_paths_to_file("precalculated_data/precalculated_data_for_safe_area.txt", nodes1, {});
    cout << "merge_two_files() zavrsio\n";
}

/*void make_files_for_problem_circle01() {
    clock_t t_start = clock();
    vector<vector<int>> nodes = {{50,50}, {43,57}, {50,64}, {57,57}};
    vector<vector<int>> states = initialize_states(nodes);

    Costs_Data_Holder cdh, cdh_temp;
    cout << "Čitanje duljina u cdh_temp\n";
    cdh_temp.read_costs_from_file_with_cdh_format("results_costs/problem_circle_with_squares01.txt");
    cout << "Čitanje puteva u cdh_temp\n";
    cdh_temp.read_paths_from_file_with_cdh_format("results_paths/problem_circle_with_squares01.txt");
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            vector<int> state1 = states[s1], state2 = states[s2];
            cout << " Provjera za states[" << s1 << "] = {" << state1[0] << ',' << state1[1] << ',' << state1[2] << "}; states[" << s2 << "] = {" << state2[0] << ',' << state2[1] << ',' << state2[2] << "}\n"; 
            if (!(state1[0] == state2[0] && state1[1] == state2[1])) {
                cout << " Različiti čvorovi\n";
                int delta_x = state2[0] - state1[0], delta_y = state2[1] - state1[1], fi1 = state1[2], fi2 = state2[2];
                cout << " " << delta_x << ',' << delta_y << ',' << fi1 << ',' << fi2 << endl;
                if (cdh_temp.get_cost_for_safe_area(state1, state2) != 0) {
                    cdh.costs_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.get_cost_for_safe_area(state1, state2);
                    cdh.costs_safe_area_keys.push_back({delta_x, delta_y, fi1, fi2});
                    cdh.paths_safe_area[delta_x][delta_y][fi1][fi2] = cdh_temp.paths_safe_area[delta_x][delta_y][fi1][fi2];
                }
            }
            cout << " -------------------\n";
        }
    }

    cout << "Spremanje rezltata\n";
    cdh.save_cdh_to_file("problem_circle01.txt", nodes, states);
    cout << " Spremljene udaljenosti\n";
    cdh.save_paths_to_file("problem_circle01.txt", nodes, states);
    cout << " Spremljeni putevi\n";
    clock_t t_finish = clock();
    cout << "GOTOVO\n (time passed: " << (double)(t_finish - t_start)/CLOCKS_PER_SEC << " s)\n";
}*/

int main(int argc, char *argv[]) {
    // Provjera jesu li zadani svi potrebni argumenti
    if (argc < 4) {
        cerr << "GREŠKA TIJEKOM POKRETANJA PROGRAMA! Nije predan dovoljan broj argumenata.\n";
        cerr << "Program se mora pokrenuti ovako: GTSP_Memetic_algorithm.exe optimize_headings_flag input_file output_file [backwards_flag]\n";
        cerr << "Pogledati izvorni kod GTSP_Memetic_aglorithm.cpp za detalje.\n";
        exit(1);
    }
    // Ako je optimize_headings_flag = 1, local_improvement_procedure_for_symmetric() će izvoditi optimizeHeadings().
    // Ako je optimize_headings_flag = 0, local_improvement_procedure_for_symmetric() neće izvoditi optimizeHeadings().
    int optimize_headings_flag = stoi(argv[1]), backwards_flag = 1;
    string input_file = argv[2], output_file = argv[3];
    if (argc >= 5) {
        backwards_flag = stoi(argv[4]);
        if (backwards_flag < 0 || backwards_flag > 1) {
            cerr << "GREŠKA! Kao zastava backwards_flag unesena je vrijednost " << backwards_flag << endl;
            exit(1);
        }
    }

    if (optimize_headings_flag < 0 || optimize_headings_flag > 1) {
        cerr << "GREŠKA! Kao zastava optimize_headings_flag unesena je vrijednost " << optimize_headings_flag << endl;
        exit(1);
    }
    // Ovisno o tome koristi li se optimizeHeadings ili ne te može li se robot kretati untrag ili ne, datoteka će se spremiti u drugi folder
    if (optimize_headings_flag == 0 && backwards_flag == 0) {
        output_file = "without_optimizeHeadings\\no_backwards\\" + output_file;
    }
    else if (optimize_headings_flag == 0 && backwards_flag == 1) {
        output_file = "without_optimizeHeadings\\backwards\\" + output_file;
    }
    else if (optimize_headings_flag == 1 && backwards_flag == 0) {
        output_file = "with_optimizeHeadings\\no_backwards\\" + output_file;
    }
    else if (optimize_headings_flag == 1 && backwards_flag == 1) {
        output_file = "with_optimizeHeadings\\backwards\\" + output_file;
    }

    // Provjera može li se spremati output_file u lokaciju u koja je navedena
    if (output_file.find("\\") != string::npos || output_file.find("/") != string::npos) {
        string output_file_x = "generations\\" + output_file;
        string path = "";
        for (int i = 0; i < output_file_x.size(); i++) {
            char c = output_file_x[i];
            path += c;
            if (c == '\\' || c == '/') {
                 if (filesystem::exists(path) == false) {
                    cerr << "GRESKA! Put " << path << " ne postoji.\n";
                    exit(1);
                 }
            }
        }
    }

    // Definiranje problema čitanjem podataka iz datoteke koju je kreirao Cost_Data_Holder
    vector<vector<int>> nodes, states;
    Costs_Data_Holder cdh;
    read_problem(input_file, nodes, states, cdh);

    clock_t t_start = clock(), t_curr;

    set<Solution> gen, improved_gen;
    vector<int> best_order;
    double min_cost = INF;
    bool improvement = true;
    int count = 1;
    vector<Time_Holder> v_th = {Time_Holder()};
    set<Solution>::iterator itr;

    // Početna generacija
    // Generiranje 1. generacije
    cout << "Generating generation 1.\n";
    gen = semirandom_construction_heuristic(nodes, states, cdh, 2 * nodes.size(), v_th[0], backwards_flag);
    // Lokalno poboljšavanje 1. generacije
    cout << "Improving generation 1.\n";
    local_improvement_procedure_for_symmetric(&gen, nodes, states, cdh, v_th[0], optimize_headings_flag);
    // Spremanje sveukupnog vremena trajanje obrade 1. generacije
    v_th[0].elapsed_time_for_current_generation = v_th[0].generating + v_th[0].improving;
    v_th[0].total_elapsed_time = v_th[0].elapsed_time_for_current_generation;
    // Ispis i spremanje 1. generacije i podataka i njezinom trajanju
    print_and_save_generation(&gen, output_file + "-with_MA", count, v_th[0]);
    cout << endl;

    min_cost = (*gen.begin()).total_cost;
    best_order = (*gen.begin()).sol_states;

    while (improvement == true) {
        clock_t t_curr_generation_start = clock();
        improvement = false;
        count++;
        v_th.push_back(Time_Holder());
        int r = int(round(double(0.2 * count + 0.05 * nodes.size() + 10)));

        // Obrada nove generacije
        set<Solution> next_gen;
        // Generiranje nove genracije
        cout << "Generating generation " << count << ".\n";
        reproduction(gen, &next_gen, nodes, states, cdh, v_th[count - 1], r);
        crossover(gen, &next_gen, nodes, states, cdh, v_th[count - 1], 8 * r, backwards_flag);
        mutation(gen, &next_gen, nodes, states, cdh, v_th[count - 1], 2 * r, backwards_flag);
        v_th[count - 1].generating = v_th[count - 1].reproduction + v_th[count - 1].crossover + v_th[count - 1].mutation;
        // Lokalno poboljšanje nove genracije
        cout << "Improving generation " << count << ".\n";
        local_improvement_procedure_for_symmetric(&next_gen, nodes, states, cdh, v_th[count- 1], optimize_headings_flag);

        gen = next_gen;
        if (min_cost > (*gen.begin()).total_cost) {
            min_cost = (*gen.begin()).total_cost;
            best_order = (*gen.begin()).sol_states;
            improvement = true;
        }
        
        clock_t t_curr_generation_end = clock();
        v_th[count - 1].elapsed_time_for_current_generation = (double)(t_curr_generation_end - t_curr_generation_start)/CLOCKS_PER_SEC;
        v_th[count - 1].total_elapsed_time = (double)(t_curr_generation_end - t_start)/CLOCKS_PER_SEC;
        print_and_save_generation(&gen, output_file + "-with_MA", count, v_th[count - 1]);
        cout << endl;
    }
    // Prije završetka algoritma, ako nije korišten optimizeHeadings tijekom local_improvement_procedure_for_symmetric(), onda se izvede jedan optimizeHeadings na najboljem rješenju.
    if (optimize_headings_flag == 0) {
        v_th.push_back(Time_Holder());

        vector<int> best_order_of_states = (*gen.begin()).sol_states;
        vector<int> best_order_of_nodes;
        for (int s : best_order_of_states)
            best_order_of_nodes.push_back((int)floor(s / NUM_OF_DIRECTIONS));
        best_order_of_states = optimizeHeadings(best_order_of_nodes, nodes, states, cdh, v_th[v_th.size() - 1], backwards_flag);
        v_th[v_th.size() - 1].improving = v_th[v_th.size() - 1].optimizeHeadings;
        v_th[v_th.size() - 1].total_elapsed_time = v_th[v_th.size() - 2].total_elapsed_time + v_th[v_th.size() - 1].optimizeHeadings;
        Solution best_solution = Solution(best_order_of_states, states, cdh, backwards_flag);
        gen.insert(best_solution);

        // Spremanje vremena utrošenog na optimizeHeadings()
        print_and_save_OH_at_end(best_solution, output_file + "-with_MA", v_th[v_th.size() - 1]);
        cout << endl;
    }

    // Ispis i spremanje rezultata
    Solution best_solution = (*gen.begin());
    print_and_save_final_results(best_solution, "generations/" + output_file + "-with_MA_FINISHED.txt", v_th);

    return 0;
}
