#pragma once
#include <iostream>
#include <vector>
#include <map>
#include "Costs_Data_Holder.hpp"
#include "lattice_planner/graphSearch.hpp"

#define INF 1000000.0
#define NUM_OF_DIRECTIONS 16

using namespace std;

struct Edge {
    double cost;
    int s1;
    int s2;

    Edge(int s1, int s2, vector<vector<int>> states, Costs_Data_Holder &cdh) {
        this->s1 = s1;
        this->s2 = s2;
        // TODO backwards_flag ne mora nužno biti 0
        this->cost = cdh.determine_cost(s1, s2, states, 0);
    }

    // Vraća true ako su dva brida susjeda, inače false
    bool adjecent(const Edge other) {
        return (this->s1 == other.s1 || this->s1 == other.s2 || this->s2 == other.s1 || this->s2 == other.s2);
    }

    bool operator<(const Edge other) const {
        if (this->cost < other.cost)
            return false;
        if (this->cost == other.cost) {
            if (this->s1 < other.s1) {
                return false;
            }
            else if (this->s1 == other.s1) {
                if (this->s2 < other.s2) {
                    return false;
                }
                else { // this->s2 > other.s22
                    return true;
                }
            }
            else { // this->s1 > other.s2
                return true;
            }
        }
        // (this->cost > other.cost)
        return true;
    }
};
