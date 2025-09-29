#pragma once
#include <map>

using namespace std;

// Za asimetrični problem, za spremanje rješenja se koristi gornjetrokutasta matrica.
// Ova funkcija za ulazni par stanja vraća vrijednost zapisan u mapi cost.
double getCostFromMapForAsymetric(map<int, map<int, double>> *p_costs, int state1, int state2) {
    if (state1 > state2)
        return (*p_costs)[state2][state1];
    return (*p_costs)[state1][state2];
}

void setCostToMapAsymetric(map<int, map<int, double>> *p_costs, int state1, int state2, double cost_value) {
    if (state1 > state2) {
        (*p_costs)[state2][state1] = cost_value;
    }
    else {
        (*p_costs)[state1][state2] = cost_value;
    }
}
