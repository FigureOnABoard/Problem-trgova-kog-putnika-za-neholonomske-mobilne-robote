#pragma once
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "initialize_states.hpp"
#include "transform_string_to_vector.hpp"
#include "../Costs_Data_Holder.hpp"

using namespace std;

// Definira varijable problema nodes, states i cdh koristeći podatke o problemu iz datoteke.
void read_problem(string file_name, vector<vector<int>> &nodes, vector<vector<int>> &states, Costs_Data_Holder &cdh) {
    ifstream f(file_name);
    string line;
    // Prva linija - opis datoteke
    getline(f, line);
    // Druga linija - čvorovi zadani u problemu
    getline(f, line);
    // Zatvaranje datoteke - imamo sve podatke potrebne za nodes i states, a cdh ima svoju vlastitu funkciju za čitanje datoteke.
    f.close();
    // Definiranje nodes
    nodes = transform_string_to_vector(line);
    // Definiranje states
    states = initialize_states(nodes);
    // Definiranje cdh
    cdh.read_costs_from_file_with_cdh_format(file_name);
    // Gotovo
}