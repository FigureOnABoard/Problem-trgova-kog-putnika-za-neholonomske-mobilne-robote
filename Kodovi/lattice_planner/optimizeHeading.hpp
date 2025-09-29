#pragma once
#include <stdlib.h>
#include <vector>
#include <bits/stdc++.h>
#include "graphSearch.hpp"
#include "../Time_Holder.hpp"

#define INF 1000000.0

using namespace std;

struct statesInfo {
    // Roditelj stanja
    int parent;
    // Cijena do stanja
    double c;
};

vector<int> getNeighbourStates(int curr_state, vector<int> order_of_nodes, vector<vector<int>> nodes, vector<vector<int>> states) {
    int penultimate_node = order_of_nodes[order_of_nodes.size() - 2];
    vector<int> neighbour_states = {};
    // Čvor koji u order_of_nodes slijedi nakon čvora na kojem se nalazi curr_state
    int next_node_index;
    // Predzadnje stanje, donosno, stanje iz kojeg se jedino može rpijeći u dodatno stanje f={-1,-1,-1}
    if (states[curr_state][0] == nodes[penultimate_node][0] && states[curr_state][1] == nodes[penultimate_node][1]) {
        neighbour_states.push_back(-1);
        return neighbour_states;
    }
    else {
        for (int i = 0; i < order_of_nodes.size(); i++) {
            int node_index = order_of_nodes[i];
            vector<int> n = nodes[node_index];
            if (n[0] == states[curr_state][0] && n[1] == states[curr_state][1]) {
                next_node_index = order_of_nodes[i + 1];
                break;
            }
        }
    }
//cout << "Prije vector<int> next_node = nodes[next_node_index];\n";
//cout << "next_node_index = " << next_node_index << endl;
    vector<int> next_node = nodes[next_node_index];
//cout << "Prošao vector<int> next_node = nodes[next_node_index];\n";
    for (int i = 0; i < states.size(); i++) {
        vector<int> s = states[i];
        if (s[0] == next_node[0] && s[1] == next_node[1])
            neighbour_states.push_back(i);
    }

    return neighbour_states;
}

vector<int> trace_path_with_optimal_headings(map<int, statesInfo> statesInfoVar, int solution_size, int start_state_index) {
    vector<int> sol;
    for (int i = 0; i < solution_size; i++)
        sol.push_back(-1);
    int i = solution_size - 1, state_index = -1;
    while (state_index != -2 && i >= 0) {
        int new_state_index = statesInfoVar[state_index].parent;
        if (state_index == -1) {
            sol[i] = start_state_index;
        }
        else {
            sol[i] = state_index;
        }
        i--;
        state_index = new_state_index;
    }
    if (i >= 0) {
        cerr << "GREŠKA U trace_path_with_optimal_headings(): algoritam je od čvora -1 prošao kroz sve čvorove, a i je ostao veći od 0!";
        exit(1);
    }
    else if (state_index != -2) {
        cerr << "GREŠKA U trace_path_with_optimal_headings(): algoritam je ispunio varijablu sol a nije prošao kroz sve čvorove!";
        exit(1);
    }
    else {
        return sol;
    }
}

// A* algoritam
// Prima vektor redoslijeda čvorova rješenja, vektore čvorova i stanja i pokazivač na Cost_Data_Holder.
// Vraća vektor stanja čiji redoslijed odgovara redoslijedu čvorova i čiji je ciklus optimalan.
vector<int> optimizeHeadings(vector<int> order_of_nodes, vector<vector<int>> nodes, vector<vector<int>> states, Costs_Data_Holder &cdh, Time_Holder &th, int backwards_flag) {
    // Za traženje optimalnog ciklusa koristit će se prošireni vektor s dodatnim stanjem f = {-1,-1,-1} koji predstavlja isto stanje koje se nalazi na početku vektora.
    // Također, uvodi se indeks -2 za stanja stanje koje nije definirano ili ne spotoji.

    clock_t t_f_start = clock();

    // Trenutno rješenje i najbolje rješenje
    vector<int> curr_cycle, best_cycle;
    double curr_total_cost, best_total_cost = INF;
    
    int start_node = order_of_nodes[0];
    
    for (int start_state = start_node * 16; start_state < start_node * 16 + 16; start_state++) {
        // Definicija zatvorene liste
        map<int, bool> closedList;
        for (int i = -1; i < (int)order_of_nodes.size(); i++)
            closedList[i] = false;

        // Deklareacija mape za detalje o pojedinim stanjima
        map<int, statesInfo> statesInfoVar;

        for (int i = -1; i < (int)states.size(); i++) {
            statesInfoVar[i].c = INF;
            statesInfoVar[i].parent = -2;
        }

        // Inicijalizacija parametara za početno stanje
        statesInfoVar[start_state].parent = -2;
        statesInfoVar[start_state].c = 0;

        // Deklaracija otvorene liste s elementima <cijena, indeks stanja>
        set<pair<double, int>> openList;

        // Postavljanje početnog stanja u otvorenu list. Početna cijena je 0.0.
        openList.insert(make_pair(0.0, start_state));
        while (!openList.empty()) {
            pair<double, int> p = *openList.begin();

            // Uklanjanje tog stanja sa liste
            openList.erase(openList.begin());
            
            // Ako je stanje f=(-1,-1,-1) (konačno stanje), algoritam završava traženje puta za trenutno početno stanje i prijelazi na iduće stanje
            if (p.second == -1) {
                curr_cycle = trace_path_with_optimal_headings(statesInfoVar, order_of_nodes.size(), start_state);
                curr_total_cost = 0;
                for (int j = 0; j < curr_cycle.size() - 1; j++)
                    curr_total_cost += cdh.determine_cost(curr_cycle[j], curr_cycle[j+1], states, backwards_flag);
                
                // Ako je pronađeni put za trenutni start_state bolje od dosad nađenog boljeg, ažurira se best_cycle i best_total_cost
                if (best_total_cost == INF || curr_total_cost < best_total_cost) {
                    best_cycle = curr_cycle;
                    best_total_cost = curr_total_cost;
                }
                break;
            }

            // Dodavanje čvora u zatvorenu listu
            int current_state = p.second;
            closedList[current_state] = true;

            float c = p.first;
            float new_c;

            // Generiranje susjeda
            vector<int> neighbour_states = getNeighbourStates(current_state, order_of_nodes, nodes, states);
            for (int i = 0; i < neighbour_states.size(); i++) {
                int new_state = neighbour_states[i];
                // Računanje dodatnoe duljine od trenutnog stanja do idućeg stanja (new_state)
                double c_additional;
                if (new_state == -1) { // Ako novo stanje ima indeks -1, onda su
                    c_additional = cdh.determine_cost(current_state, start_state, states, backwards_flag);
                }
                else if (closedList[new_state] == false) {
                    c_additional = cdh.determine_cost(current_state, new_state, states, backwards_flag);
                }
                else { //closedList[new_state] == true
                    continue;
                }
                // Određivanje ukupne duljine do novogg stanja (duljine od početnog stanja do novog)
                if (c_additional != INF) {
                    new_c = c + c_additional;
                }
                else {
                    new_c = INF;
                }

                // Ako je new_c (cijena iz curr_state u neighbour_states[i]) manja od one koja je trenutno poznata, ažuriraju se podaci
                if (statesInfoVar[new_state].c == INF || statesInfoVar[new_state].c > new_c) {
                    openList.insert(make_pair(new_c, new_state));
                    statesInfoVar[new_state].c = new_c;
                    statesInfoVar[new_state].parent = current_state;
                }
            }
        }
    }

    if (best_total_cost == INF) {
        // Ako program ostane s praznom openList varijablom, onda nije uspio naći izvedivo rješenje.
        cerr << "GREŠKA U optimizeHeadings(): funkcija nije našla izvedivu putanju!\n";
        exit(1);
        // TODO dodati bolji kod za handlanje situacije kada je rješenje neizvedivo.
    }

    clock_t t_f_finish = clock();
    th.optimizeHeadings += (double)(t_f_finish - t_f_start)/CLOCKS_PER_SEC;
    return best_cycle;
}
