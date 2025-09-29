#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "Costs_Data_Holder.hpp"

using namespace std;

// Jedna solucija u populaciji.
// Kao struktura, sadrži vekotr s indeksima stanja i ukupnu cijenu puta kroz ta stanja
struct Solution {
    vector<int> sol_states;
    double total_cost;

    Solution(vector<int> sol_states, vector<vector<int>> all_states, Costs_Data_Holder &cdh, int backwards_flag) {
        this->sol_states = sol_states;
        this->total_cost = 0.0;
        
        // Prolazi se kroz for petlju dok se ne prođe kroz sve parove susjednih stanja ILI dok se ne naleti na par stanja između kojih je put nemoguć (INF) nakon čega se petlja prekida
        for (int i=0; i < sol_states.size()-1; i++) {
            int s1 = sol_states[i], s2 = sol_states[i+1];
            double c = cdh.determine_cost(s1, s2, all_states, backwards_flag);
            
            if (c == INF) {
                this->total_cost = INF;
                break;
            }
            else {
                this->total_cost += c;
            }
        }
    }

    // Jedna solucija je manja od druge solucije ako je duljina puta kroz njezina stanja manja od duljine puta kroz stanja druge solucije
    bool operator<(const Solution other) const {
        if (this->total_cost < other.total_cost)
            return true;
        else if (this->total_cost == other.total_cost) {
            int solution_size = this->sol_states.size();
            for (int i = 0; i < solution_size; i++) {
                if (this->sol_states[i] < other.sol_states[i])
                    return true;
                else if (this->sol_states[i] > other.sol_states[i])
                    return false;
            }
            // AKo se prođe kroz čitavu for petlju bez izlaza, znači da su svi elementi jednaki.
            return false;
        }
        else  // (this->total_cost > other.total_cost)
            return false;
    }
};
