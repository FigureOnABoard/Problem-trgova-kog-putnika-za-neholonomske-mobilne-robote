#pragma once
#include <string.h>
#include <iostream>
#include <fstream>
#include <map>

using namespace std;

void read_edges_costs_file(string file_path, map<int, map<int, double>> *costs) {
    ifstream input_file(file_path);
    string line;

    for (int i = 1; i <= 3; i++)
        getline(input_file, line);
    
    while (getline(input_file, line)) {
        int s1, s2;
        string curr_value = "";

        int i=0;
        while (line[i] != ',') {
            curr_value += line[i];
            i++;
        }
        s1 = stoi(curr_value);
        curr_value = "";
        i++;

        while (line[i] != ',') {
            curr_value += line[i];
            i++;
        }
        s2 = stoi(curr_value);
        curr_value = "";
        i++;

        while (i < line.size()) {
            curr_value += line[i];
            i++;
        }
        (*costs)[s1][s2] = stod(curr_value);
    }
}
