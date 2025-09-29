#pragma once
#include <string>

#include "../Costs_Data_Holder.hpp"

// Vraća true ako se stanja nalaze u istim čvorovima, inače false
bool check_if_nodes_equal(int s1, int s2, vector<vector<int>> states) {
    vector<int> state1 = states[s1], state2 = states[s2];
    return (state1[0] == state2[0] && state1[1] == state2[1]);
}

// Dodavanje svih traženih duljina iz cdh_precalculated u cdh
void copy_from_cdh_into_cdh(Costs_Data_Holder &cdh, vector<vector<int>> nodes, vector<vector<int>> states, string file_name, string file_name_TEMP, int backwards_flag) {
    cout << "\nDodavanje svih traženih duljina iz cdh_precalculated u cdh\n";
    Costs_Data_Holder cdh_precalculated;
    // Dohvat svih poznatih udaljenosti
    if (backwards_flag == 1) {
        cout << "Citanje podataka kad se MOZE ici unazad\n";
        cdh_precalculated.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/precalculated_data_for_safe_area.txt");
        cdh_precalculated.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/precalculated_data_for_safe_area.txt");
    } else {
        cout << "Citanje podataka kad se NE moze ici unazad\n";
        cdh_precalculated.read_costs_from_file_with_cdh_format("results_costs/precalculated_data/No_backwards/precalculated_data_for_safe_area.txt");
        cdh_precalculated.read_paths_from_file_with_cdh_format("results_paths/precalculated_data/No_backwards/precalculated_data_for_safe_area.txt");
    }
    
    // Dodavanje svih traženih duljina iz cdh_precalculated u cdh
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            if (check_if_nodes_equal(s1, s2, states) == false) {
                vector<int> state1 = states[s1], state2 = states[s2];
                int dx, dy, fi1, fi2;
                dx = state2[0] - state1[0];
                dy = state2[1] - state1[1];
                fi1 = state1[2];
                fi2 = state2[2];
                vector<int> k = {dx, dy, fi1, fi2};
                cout << " Provjera za s1 = " << s1 << " {" << state1[0] << ',' << state1[1] << ','  << state1[2] << "}, s2 = " << s2 << " {" << state2[0] << ',' << state2[1] << ',' << state2[2] << "} ";
                cout << "(dx,dy,fi1,fi2) = (" << dx << ',' << dy << ',' << fi1 << ',' << fi2 << ")\n";  
                if (cdh_precalculated.costs_safe_area[dx][dy][fi1][fi2] != 0) {
                    cdh.costs_safe_area[dx][dy][fi1][fi2] = cdh_precalculated.costs_safe_area[dx][dy][fi1][fi2];
                    cdh.costs_safe_area_keys.push_back(k);
                    cdh.paths_safe_area[dx][dy][fi1][fi2] = cdh_precalculated.paths_safe_area[dx][dy][fi1][fi2];
                }
            }
        }
    }
}

void determine_all_cdh_costs(Costs_Data_Holder &cdh, vector<vector<int>> nodes, vector<vector<int>> states, string file_name, string file_name_TEMP, int backwards_flag) {
    // Izračunaj sve nepoznate vrijednosti.
    cout << "Računanje svih vrijednosti\n";
    for (int s1 = 0; s1 < states.size(); s1++) {
        for (int s2 = 0; s2 < states.size(); s2++) {
            if (check_if_nodes_equal(s1, s2, states) == false) {
                cout << " determine_cost(" << s1 << ',' << s2 << ',' << "states)\n";
                cdh.determine_cost(s1, s2, states, backwards_flag);
            }
        }
        if ((s1 + 1) % 50 == 0) {
            cdh.save_cdh_to_file(file_name_TEMP, nodes, states);
            remove(("results_costs/" + file_name).c_str());
            rename(("results_costs/" + file_name_TEMP).c_str(), ("results_costs/" + file_name).c_str());
            cdh.save_paths_to_file(file_name_TEMP, nodes, states);
            remove(("results_paths/" + file_name).c_str());
            rename(("results_paths/" + file_name_TEMP).c_str(), ("results_paths/" + file_name).c_str());
        }
    }
}
