#pragma once
#include <stdio.h>
#include <vector>

using namespace std;

#define NUM_OF_DIRECTIONS 16

// Stvara i vraća vektor sa svim stanjima (koordinate i usmjerenja).
// Indeks stanja u vektoru e jedinstveni indetifikator/indeks tog stanja.
vector<vector<int>> initialize_states(vector<vector<int>> nodes) {
    vector<vector<int>> states;
    for (vector<int> node : nodes) {
        for (int fi = 0; fi < NUM_OF_DIRECTIONS; fi++) {
            vector<int> new_state = node;
            new_state.push_back(fi);
            states.push_back(new_state);
        }
    }
    return states;
}
