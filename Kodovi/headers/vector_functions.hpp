/*
SADRŽI:
    int get_index_of_node(vector<int> n, vector<vector<int>> nodes)
    double calculate_cost(vector<int> solution, Costs_Data_Holder &cdh, vector<vector<int>> states)
*/

#pragma once
#include <stdio.h>
#include <vector>

using namespace std;

// Vraća identifikacijski index za čvor n
int get_index_of_node(vector<int> n, vector<vector<int>> nodes) {
    for (int index = 0; index < nodes.size(); index++)
        if (nodes[index] == n)
            return index;
    return -1;
}
