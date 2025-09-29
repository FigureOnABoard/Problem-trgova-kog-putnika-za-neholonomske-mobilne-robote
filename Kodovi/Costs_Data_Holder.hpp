#pragma once
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include "lattice_planner/graphSearch.hpp"
#include "lattice_planner/asymetricMapHandler.hpp"
#include "Path.hpp"

#define INF 1000000.0
#define NUM_OF_DIRECTIONS 16

using namespace std;

// Costs_Data_Folder.hpp sadrži definiciju strukture Cost_Data_Holder.
// Cost_Data_Holder sadrži podatke o duljinama puta između dva stanja čije su međusobne razlike u koordinatama delta_x i delta_y.
// Duljine se spremaju u mapu s ključevima delta_x, delta_y, fi_start, fi_finish.
// TODO dovršiti opis
struct Costs_Data_Holder {
    map<int, map<int, map<int, map<int, double>>>> costs_safe_area;
    vector<vector<int>> costs_safe_area_keys;
    map<int, map<int, map<int, map<int, Path>>>> paths_safe_area;
    map<int, map<int, double>> costs_sensitive_area;
    vector<vector<int>> costs_sensitive_area_keys;
    map<int, map<int, Path>> paths_sensitive_area;
    int node_dist = 5; // 5 - node distance/međusobna udaljenost dvaju čvorova
    int r = 3 * node_dist; // 3 - Broj indeksa čvorova u kojima je dugačak 1 radijus; 
    
    // Pretpostavka je da virtualni prikaz mape ima dimenzije 1000x1000 i da se proteže po čelijama 0-999. (0 i 999 su granice/rubovi preko kojih se ne može)
    // Kako neki putevi koji su optimalni na području udaljenom od rubova mape nisu izvedivi na području blizu ruba mape, duljine se spremaju u costs
    // samo ako nisu previše blizu rubovima mape. Pretpostavljamo da se stanje nalazi blizu ruba ako je od nekog ruba udaljena za 2r ili manje, gdje je 2r
    // dvostruka duljina maksimalnog radijusa skretanja r koji definira fizička ograničenost robota.
    bool check_if_state_near_edge(vector<int> state) {
        int m = node_dist * state[0];
        int n = node_dist * state[1];
        if (m <= 2 * r || n <= 2 * r || m >= 1000 - 2 * r || n >= 1000 - 2 * r)
            return true;
        return false;
    }

    int mirror_angle_in_x(int fi) {
        int mirror_fi;
        mirror_fi = - fi;
        if (mirror_fi < 0)
            mirror_fi += 16;
        if (mirror_fi >= 16)
            mirror_fi -= 16;
        return mirror_fi;
    }

    int mirror_angle_in_y(int fi) {
        int mirror_fi;
        mirror_fi = -fi + 8;
        if (mirror_fi < 0)
            mirror_fi += 16;
        if (mirror_fi >= 16)
            mirror_fi -= 16;
        return mirror_fi;
    }

    int rotate_angle_for_degree(int fi, int deg) {
        int rotated_fi = fi + deg;
        if (rotated_fi < 0)
            rotated_fi += 16;
        else if (rotated_fi >= 16)
            rotated_fi -= 16;
        return rotated_fi;
    }

    Path mirror_path_in_x(Path p) {
        Path mirrored_p;
        mirrored_p.size = p.size;
        mirrored_p.data = {};

        string pathToVars = "lattice_planner/params/vars/csv_vars.csv";
        vector<vector<double>> varVars = readVars(pathToVars);
        vector<pair<vector<int>, double> > graph[16];
        // m - broj redaka u datoteci csv_vars.csv
        // n - broj stupaca u datoteci csv_vars.csv
        int m = varVars.size();
        int n = varVars[0].size();
        vector<int> tmp;
        double sf;
        // petlja po svim segmentima u latici, počevši od segmenata s početnom orijentacijom 0
        for (int i = 0; i < m; i++) {
            tmp.clear();
            tmp.push_back((int)varVars[i][1]);    // x - koordinata susjednog čvora
            tmp.push_back((int)varVars[i][2]);    // y - koordinata susjednog čvora
            tmp.push_back((int)varVars[i][3]);    // konačna orijentacija u susjednom čvoru
            sf = varVars[i][4];                   // cijena (duljina pripadajućeg segmenta)
            // indexi elemenata vektora "graph" predstavljaju početne orijentacije (0-15)
            // svaki element vektora "graph" sadrži informacije o svim 
            // susjednim čvorovima u koje se može doći s dotičnom početnom orijentacijom
            // informacije o susjednim čvorovima sadržane su u parovima oblika: 
            // (x-y koordinate susjednog čvora + konačna orijentacija, cijena pripadajućeg segmenta)
            graph[(int)varVars[i][0]].push_back(make_pair(tmp, sf));
        }
        for (int k = 0; k < p.size; k+=2) {
            // Orijentacija
            mirrored_p.data.push_back(mirror_angle_in_x(p.data[k]));
            // Indeks putanje
            int x, y, fi_start, fi_finish;
            int index;
            fi_start = p.data[k];
            index = p.data[k + 1];
            x = graph[fi_start][index].first[0];
            y = graph[fi_start][index].first[1];
            fi_finish = graph[fi_start][index].first[2];

            // Vrijednosti zrcalne u odnosu na x-os
            int mirrored_x, mirrored_y, mirrored_fi_start, mirrored_fi_finish;
            int mirrored_index;
            mirrored_fi_start = mirror_angle_in_x(fi_start);
            mirrored_x = x;
            mirrored_y = -y;
            mirrored_fi_finish = mirror_angle_in_x(fi_finish);
            for (int i = 0; i < graph[mirrored_fi_start].size(); i++) {
                tmp = graph[mirrored_fi_start][i].first;
                if (tmp[0] == mirrored_x && tmp[1] == mirrored_y && tmp[2] == mirrored_fi_finish) {
                    mirrored_index = i;
                    break;
                }
            }
            
            mirrored_p.data.push_back(mirrored_index);
        }

        return mirrored_p;
    }

    Path mirror_path_in_y(Path p) {
        Path mirrored_p;
        mirrored_p.size = p.size;
        mirrored_p.data = {};

        string pathToVars = "lattice_planner/params/vars/csv_vars.csv";
        vector<vector<double>> varVars = readVars(pathToVars);
        vector<pair<vector<int>, double> > graph[16];
        // m - broj redaka u datoteci csv_vars.csv
        // n - broj stupaca u datoteci csv_vars.csv
        int m = varVars.size();
        int n = varVars[0].size();
        vector<int> tmp;
        double sf;
        // petlja po svim segmentima u latici, počevši od segmenata s početnom orijentacijom 0
        for (int i = 0; i < m; i++) {
            tmp.clear();
            tmp.push_back((int)varVars[i][1]);    // x - koordinata susjednog čvora
            tmp.push_back((int)varVars[i][2]);    // y - koordinata susjednog čvora
            tmp.push_back((int)varVars[i][3]);    // konačna orijentacija u susjednom čvoru
            sf = varVars[i][4];                   // cijena (duljina pripadajućeg segmenta)
            // indexi elemenata vektora "graph" predstavljaju početne orijentacije (0-15)
            // svaki element vektora "graph" sadrži informacije o svim 
            // susjednim čvorovima u koje se može doći s dotičnom početnom orijentacijom
            // informacije o susjednim čvorovima sadržane su u parovima oblika: 
            // (x-y koordinate susjednog čvora + konačna orijentacija, cijena pripadajućeg segmenta)
            graph[(int)varVars[i][0]].push_back(make_pair(tmp, sf));
        }
        for (int k = 0; k < p.size; k+=2) {
            // Orijentacija
            mirrored_p.data.push_back(mirror_angle_in_y(p.data[k]));
            // Indeks putanje
            int x, y, fi_start, fi_finish;
            int index;
            fi_start = p.data[k];
            index = p.data[k + 1];
            x = graph[fi_start][index].first[0];
            y = graph[fi_start][index].first[1];
            fi_finish = graph[fi_start][index].first[2];

            // Vrijednosti zrcalne u odnosu na y-os
            int mirrored_x, mirrored_y, mirrored_fi_start, mirrored_fi_finish;
            int mirrored_index;
            mirrored_fi_start = mirror_angle_in_y(fi_start);
            mirrored_x = -x;
            mirrored_y = y;
            mirrored_fi_finish = mirror_angle_in_y(fi_finish);
            for (int i = 0; i < graph[mirrored_fi_start].size(); i++) {
                tmp = graph[mirrored_fi_start][i].first;
                if (tmp[0] == mirrored_x && tmp[1] == mirrored_y && tmp[2] == mirrored_fi_finish) {
                    mirrored_index = i;
                    break;
                }
            }
            
            mirrored_p.data.push_back(mirrored_index);
        }

        return mirrored_p;
    }

    // Prima put p i stupanj deg (gledano na skali [0, 16]). Vraća put dobiven rotacijom p za deg.
    // Prihvaća se deg = 4 i  deg = -4. Ako je deg = 4, rotiraa se za 90 stupnjeva, a ako je -90 rotira se za -90 stupnjeva.
    Path rotate_path_for_degree(Path p, int deg) {
        if (abs(deg) != 4) {
            cerr << "Greška u funkciji rotate_path_for_degree()! Funkcija za vrijednost deg može primiti sam 4 ili -4, a primljena je vrijednost " << deg << endl;
            exit(1);
        }
        Path rotated_p;
        rotated_p.data = {};
        rotated_p.size = p.size;

        string pathToVars = "lattice_planner/params/vars/csv_vars.csv";
        vector<vector<double>> varVars = readVars(pathToVars);
        vector<pair<vector<int>, double> > graph[16];
        // m - broj redaka u datoteci csv_vars.csv
        // n - broj stupaca u datoteci csv_vars.csv
        int m = varVars.size();
        int n = varVars[0].size();
        vector<int> tmp;
        double sf;
        // petlja po svim segmentima u latici, počevši od segmenata s početnom orijentacijom 0
        for (int i = 0; i < m; i++) {
            tmp.clear();
            tmp.push_back((int)varVars[i][1]);    // x - koordinata susjednog čvora
            tmp.push_back((int)varVars[i][2]);    // y - koordinata susjednog čvora
            tmp.push_back((int)varVars[i][3]);    // konačna orijentacija u susjednom čvoru
            sf = varVars[i][4];                   // cijena (duljina pripadajućeg segmenta)
            // indexi elemenata vektora "graph" predstavljaju početne orijentacije (0-15)
            // svaki element vektora "graph" sadrži informacije o svim 
            // susjednim čvorovima u koje se može doći s dotičnom početnom orijentacijom
            // informacije o susjednim čvorovima sadržane su u parovima oblika: 
            // (x-y koordinate susjednog čvora + konačna orijentacija, cijena pripadajućeg segmenta)
            graph[(int)varVars[i][0]].push_back(make_pair(tmp, sf));
        }
        for (int k = 0; k < p.data.size(); k += 2) {
            // Rotacija
            rotated_p.data.push_back(rotate_angle_for_degree(p.data[k], deg));
            // Indeks putanje
            int x, y, fi_start, fi_finish;
            int index;
            fi_start = p.data[k];
            index = p.data[k + 1];
            x = graph[fi_start][index].first[0];
            y = graph[fi_start][index].first[1];
            fi_finish = graph[fi_start][index].first[2];

            // Vrijednosti koje se dobiju rotacijom za deg
            int rotated_x, rotated_y, rotated_fi_start, rotated_fi_finish;
            int rotated_index;
            if (deg == 4) {
                rotated_x = - y;
                rotated_y = x;
            }
            else { // deg == -4
                rotated_x = y;
                rotated_y = - x;
            }
            rotated_fi_start = rotate_angle_for_degree(fi_start, deg);
            rotated_fi_finish = rotate_angle_for_degree(fi_finish , deg);
            for (int i = 0; i < graph[rotated_fi_start].size(); i++) {
                tmp = graph[rotated_fi_start][i].first;
                if (tmp[0] == rotated_x && tmp[1] == rotated_y && tmp[2] == rotated_fi_finish) {
                    rotated_index = i;
                    break;
                }
            }

            rotated_p.data.push_back(rotated_index);
        }

        return rotated_p;
    }

    double get_cost_for_safe_area(vector<int> state_start, vector<int> state_finish) {
        int delta_x = state_finish[0] - state_start[0], delta_y = state_finish[1] - state_start[1];
        int fi_start = state_start[2], fi_finish = state_finish[2];
        if (costs_safe_area[delta_x][delta_y][fi_start][fi_finish] != 0) {
            return costs_safe_area[delta_x][delta_y][fi_start][fi_finish];
        }
        // Ako je delta_x' = -delta_x i delta_y' = delta_y, onda je put od početnog do završnog stanja isti kao putu zrcalnom u donosu na y-os.
        else if (costs_safe_area[-delta_x][delta_y][mirror_angle_in_y(fi_start)][mirror_angle_in_y(fi_finish)] != 0) {
            double cost = costs_safe_area[-delta_x][delta_y][mirror_angle_in_y(fi_start)][mirror_angle_in_y(fi_finish)];
            costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = cost;
            costs_safe_area_keys.push_back({delta_x, delta_y, fi_start, fi_finish});
            if (cost != INF)
                paths_safe_area[delta_x][delta_y][fi_start][fi_finish] = mirror_path_in_y(paths_safe_area[-delta_x][delta_y][mirror_angle_in_y(fi_start)][mirror_angle_in_y(fi_finish)]);
            return cost;
        }
        // Ako je delta_x' = delta_x i delta_y' = -delta_y, onda je put od početnog do završnog stanja isti kao putu zrcalnom u donosu na x-os.
        else if (costs_safe_area[delta_x][-delta_y][mirror_angle_in_x(fi_start)][mirror_angle_in_x(fi_finish)] != 0) {
            double cost = costs_safe_area[delta_x][-delta_y][mirror_angle_in_x(fi_start)][mirror_angle_in_x(fi_finish)];
            costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = cost;
            costs_safe_area_keys.push_back({delta_x, delta_y, fi_start, fi_finish});
            if (cost != INF)
                paths_safe_area[delta_x][delta_y][fi_start][fi_finish] = mirror_path_in_x(paths_safe_area[delta_x][-delta_y][mirror_angle_in_x(fi_start)][mirror_angle_in_x(fi_finish)]);
            return cost;
        }
        // Ako je delta_x' = -delta_x i delta_y' = -delta_y, onda je put od početnog do završnog stanja isti kao putu zrcalnom u donosu na x-os i y-os.
        else if (costs_safe_area[-delta_x][-delta_y][mirror_angle_in_y(mirror_angle_in_x(fi_start))][mirror_angle_in_y(mirror_angle_in_x(fi_finish))] != 0) {
            double cost = costs_safe_area[-delta_x][-delta_y][mirror_angle_in_y(mirror_angle_in_x(fi_start))][mirror_angle_in_y(mirror_angle_in_x(fi_finish))];
            costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = cost;
            costs_safe_area_keys.push_back({delta_x, delta_y, fi_start, fi_finish});
            if (cost != INF)
                paths_safe_area[delta_x][delta_y][fi_start][fi_finish] = mirror_path_in_y(mirror_path_in_x(paths_safe_area[-delta_x][-delta_y][mirror_angle_in_y(mirror_angle_in_x(fi_start))][mirror_angle_in_y(mirror_angle_in_x(fi_finish))]));
            return cost;
        }
        // Ako postoji put iz kojeg se trenutni put može dobiti rotacijom za 90 stupnjeva.
        // Taj bi u odnosu na trenutni put bio rotiran za -90 stupnjeva i imao delta_x' = delta_y, delta_y' = - delta_x, fi_start' = fi_start - 4 i fi_finish' = fi_finish - 4
        else if (costs_safe_area[delta_y][-delta_x][rotate_angle_for_degree(fi_start, -4)][rotate_angle_for_degree(fi_finish, -4)] != 0) {
            double cost = costs_safe_area[delta_y][-delta_x][rotate_angle_for_degree(fi_start,  -4)][rotate_angle_for_degree(fi_finish, -4)];
            costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = cost;
            costs_safe_area_keys.push_back({delta_x, delta_y, fi_start, fi_finish});
            if (cost != INF)
                paths_safe_area[delta_x][delta_y][fi_start][fi_finish] = rotate_path_for_degree(paths_safe_area[delta_y][-delta_x][rotate_angle_for_degree(fi_start, -4)][rotate_angle_for_degree(fi_finish, - 4)], 4);
            return cost;
        }
        // Ako postoji put iz kojeg se trenutni put može dobiti rotacijom za -90 stupnjeva.
        // Taj bi u odnosu na trenutni put bio rotiran za 90 stupnjeva i imao delta_x' = - delta_y, delta_y' = delta_x, fi_start' = fi_start + 4 i fi_finish' = fi_finish + 4
        else if (costs_safe_area[-delta_y][delta_x][rotate_angle_for_degree(fi_start, 4)][rotate_angle_for_degree(fi_finish, 4)] != 0) {
            double cost = costs_safe_area[-delta_y][delta_x][rotate_angle_for_degree(fi_start, 4)][rotate_angle_for_degree(fi_finish, 4)];
            costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = cost;
            costs_safe_area_keys.push_back({delta_x, delta_y, fi_start, fi_finish});
            if (cost != INF)
                paths_safe_area[delta_x][delta_y][fi_start][fi_finish] = rotate_path_for_degree(paths_safe_area[-delta_y][delta_x][rotate_angle_for_degree(fi_start, 4)][rotate_angle_for_degree(fi_finish, 4)], -4);
            return cost;
        }
        /*
        // Ako postoji put iz kojeg se trenutni put može dobiti rotacijom za 90 stupnjeva i zrcaljenjem s x-osi.
        // Taj bi u odnosu na trenutni put bio oritran za -90 stupnjeva i zrcaljen s x-osi.
        // Imao bi koordinate delta_x' = delta_y, delta_y' = delta_x i kuteve fi_start' = mirror_angle_in_x(fi_start - 4) i fi_finish' = mirror_angle_in_x(fi_finish - 4).
        else if (costs_safe_area[])*/
        else {
            return 0;
        }
    }

    // Vraća duljinu brida za par stanja koja nisu blizu rubovima
    // Ulazni argumenti: početno stanje, završno stanje i  zastavica koja signalizira može li se vozilo kretati unazad ili ne
    // Izlazni argument: duljina puta između početnog i završnog stanja
    double determine_cost_in_safe_area(vector<int> state_start, vector<int> state_finish, int backwards_flag) {
        int delta_x = state_finish[0] - state_start[0], delta_y = state_finish[1] - state_start[1], fi_start = state_start[2], fi_finish = state_finish[2];
        if (get_cost_for_safe_area(state_start, state_finish) != 0) {
            return get_cost_for_safe_area(state_start, state_finish);
        }
        else { // Nema para iz kojeg bi se mogla isčitati analogna situacija
            int start[3], finish[3];
            vector<double> M = {};
            int sizeM[2], *vehicle_orientations;
            for (int i = 0; i < 3; i++) {
                start[i] = state_start[i];
                finish[i] = state_finish[i];
            }
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
            graphSearch(pathToVars, pathToCost, vMap, start, finish, M, sizeM, backwards_flag);

            if (M.size() == 0) {
                costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = INF;
                costs_safe_area_keys.push_back({delta_x, delta_y, fi_start, fi_finish});
                return INF;
            }
            else {
                costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = get_cost_of_restructured_path(M, sizeM);
                costs_safe_area_keys.push_back({delta_x, delta_y, fi_start, fi_finish});
                paths_safe_area[delta_x][delta_y][fi_start][fi_finish].data = M;
                paths_safe_area[delta_x][delta_y][fi_start][fi_finish].size = sizeM[0] * sizeM[1];
                return costs_safe_area[delta_x][delta_y][fi_start][fi_finish];
            }
        }
    }

    // Vraća duljinu brida za par stanja koja su blizu rubovima.
    // TODO poboljšati opis
    // U ovom slučaju gledaju se udaljenosti između specifičnih stanja, a ne između relativno jednakih stanja.
    // Ulazni argumenti: jedinstveni indeksi početnog i završnog stanje, početno i završno stanje i  zastavica koja signalizira može li se vozilo kretati unazad ili ne
    // Izlazni argument: duljina puta između početnog i završnog stanja
    double determine_cost_in_sensitive_area(int state_start_index, int state_finish_index, vector<int> state_start, vector<int> state_finish, int backwards_flag) {
        double c = getCostFromMapForAsymetric(&costs_sensitive_area, state_start_index, state_finish_index);
        if (c != 0) {
            return c;
        }
        else {
            int start[3], finish[3];
            vector<double> M = {};
            int sizeM[2], *vehicle_orientations;
            for (int i = 0; i < 3; i++) {
                start[i] = state_start[i];
                finish[i] = state_finish[i];
            }
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
            graphSearch(pathToVars, pathToCost, vMap, start, finish, M, sizeM, backwards_flag);

            c = get_cost_of_restructured_path(M, sizeM);

            setCostToMapAsymetric(&costs_sensitive_area, state_start_index, state_finish_index, c);
            if (state_start_index < state_finish_index) {
                costs_sensitive_area_keys.push_back({state_start_index, state_finish_index});
                paths_sensitive_area[state_start_index][state_finish_index].data = M;
                paths_sensitive_area[state_start_index][state_finish_index].size = sizeM[0] * sizeM[1];
            }
            else {
                costs_sensitive_area_keys.push_back({state_finish_index, state_start_index});
                paths_sensitive_area[state_finish_index][state_start_index].data = M;
                paths_sensitive_area[state_finish_index][state_start_index].size = sizeM[0] * sizeM[1];
            }

            return c;
        }
    }

    double determine_cost(int start_index, int finish_index, vector<vector<int>> states, int backwards_flag) {
        vector<int> start = states[start_index], finish = states[finish_index];
        if (check_if_state_near_edge(start) == false && check_if_state_near_edge(finish) == false) {
            return determine_cost_in_safe_area(start, finish, backwards_flag);
        }
        else {
            return determine_cost_in_sensitive_area(start_index, finish_index, start, finish, backwards_flag);
        }
    }

    void read_costs_from_file_with_costs_map_format(string input_file_name, vector<vector<int>> states) {
        ifstream f(input_file_name);
        string line;

        for (int i = 0; i < 3; i++)
            getline(f, line);

        while (getline(f, line)) {
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
            
            vector<int> state1 = states[s1], state2 = states[s2];
            // Ako su u sigurnoj površnini, odnosno, nisu blizu rubovima
            if (check_if_state_near_edge(state1) == false && check_if_state_near_edge(state2) == false) {
                if (get_cost_for_safe_area(state1, state2) == 0) {
                    int delta_x = state2[0] - state1[0], delta_y = state2[1] - state1[1];
                    int fi1 = state1[2], fi2 = state2[2];
                    costs_safe_area[delta_x][delta_y][fi1][fi2] = stod(curr_value);
                    costs_safe_area_keys.push_back({delta_x, delta_y, fi1, fi2});
                }
            }
            else { // Ako su na osjetljivoj površini,  donosno, ako je barem jedan od stanja blizu nekom od rubova
                if (getCostFromMapForAsymetric(&costs_sensitive_area, s1, s2) == 0) {
                    costs_sensitive_area[s1][s2] = stod(curr_value);
                    costs_sensitive_area_keys.push_back({s1, s2});
                }
            }
        }
    }

    // Spremanje izračunatih udaljenosti u datoteku
    // Funkcija sprema sve ključeve iz costs_safe_area_keys i costs_sensitive_area_keys te uz njih pripadne udaljenosti uz costs_safe_area i costs_sensitive_area.
    // TODO Podijeliti save to file na dvije funkcije: 1 za duljine, 1 za putanje
    void save_cdh_to_file(string output_file_name, vector<vector<int>> nodes, vector<vector<int>> states) {
        ofstream output_file("results_costs/" + output_file_name);
        output_file << "This file contains costs between states calculated by Costs_Data_Holder.";
        output_file << " The first row contains a description about what this file is,";
        output_file << " the second row contains vector of nodes used in calculation,";
        output_file << " the third row contains discretization of orientations (in how many different orientations can a robot arrive to a node),";
        output_file << " all the other rows contain either flags \"safe_area\", \"sensitive_area\" or keys and costs in csv format.";
        output_file << " The costs include costs from safe area in which case distance is calculated and stored based on a relative position of two states";
        output_file << " and costs from sensitive area in which case distance is calculated and stored based on an absolute position of two states.";
        output_file << " \"safe_area\" signifies that data about costs between relative states in safe area starts from the first row below and continues onwards until \"costs_sensitive_area\" is reached.";
        output_file << " \"sensitive_area\" signifies that data about costs between absolute states in safe area starts from the first row below and continues onwards until the end of the file is reached.";
        output_file << " In rows that isn't one of the first three rows or doesn't contain the flags \"safe_area\" nor \"sensitive_area\", one row contains information about one pair of states.";
        output_file << " Rows containing data about states in safe are have format \"delta_x,delta_y,fi_start,fi_finish,cost\",";
        output_file << " where delta_x and delta_y are differences in coordinates of the two states, fi_start and fi_finish are orientations of the first and the second state respectively";
        output_file << " and cost is the cost between those two states.";
        output_file << " Rows containing data about sensitive states have format \"state_start_index,state_finish_index,cost\",";
        output_file << " where state_start_index and state_finish_index are indexes of the first and the second state respectively and cost is a cost between those two states.";

        output_file << endl << '{';
        for (int i = 0; i < nodes.size(); i++) {
            output_file << '{' << nodes[i][0] << ',' << nodes[i][1] << '}';
            if (i < nodes.size() - 1)
                output_file << ", ";
        }
        output_file << '}';

        output_file << endl << NUM_OF_DIRECTIONS;

        output_file << endl << "safe_area";
        for (vector<int> k : costs_safe_area_keys) {
            int delta_x = k[0], delta_y = k[1], fi_start = k[2], fi_finish = k[3];
            output_file << endl << delta_x << ',' << delta_y << ',' << fi_start << ',' << fi_finish << ',' << costs_safe_area[delta_x][delta_y][fi_start][fi_finish];
        }

        output_file << endl << "sensitive_area";
        for (vector<int> k : costs_sensitive_area_keys) {
            int s_start_index = k[0], s_finish_index = k[1];
            output_file << endl << s_start_index << ',' << s_finish_index << ',' << costs_sensitive_area[s_start_index][s_finish_index];
        }

        output_file.close();
    }

    void save_paths_to_file(string output_file_name, vector<vector<int>> nodes, vector<vector<int>> states) {
        ofstream output_file("results_paths/" + output_file_name);
        output_file << "This file contains paths between states.";
        output_file << " One row represents the pair of states,";
        output_file << " the first row below it represents size of an array in which data about the path is stored";
        output_file << " and the second row below represents data about the path between them.";
        
        output_file << endl << '{';
        for (int i = 0; i < nodes.size(); i++) {
            output_file << '{' << nodes[i][0] << ',' << nodes[i][1] << '}';
            if (i < nodes.size() - 1)
                output_file << ", ";
        }
        output_file << '}';

        output_file << endl << NUM_OF_DIRECTIONS;

        output_file << endl << "safe_area";
        for (vector<int> k : costs_safe_area_keys) {
            int delta_x = k[0], delta_y = k[1], fi_start = k[2], fi_finish = k[3];
            output_file << endl << delta_x << ',' << delta_y << ',' << fi_start << ',' << fi_finish;
            Path p = paths_safe_area[delta_x][delta_y][fi_start][fi_finish];
            output_file << endl << p.size;
            output_file << endl;
            for (int k = 0; k < p.size - 1; k++)
                output_file << p.data[k] << ',';
            output_file << p.data[p.size - 1];
        }

        output_file << endl << "sensitive_area";
        for (vector<int> k : costs_sensitive_area_keys) {
            int s_start_index = k[0], s_finish_index = k[1];
            output_file << endl << s_start_index << ',' << s_finish_index;
            Path p = paths_sensitive_area[s_start_index][s_finish_index];
            output_file << endl << p.size;
            output_file << endl;
            for (int k = 0; k < p.size - 1; k++)
                output_file << p.data[k] << ',';
            output_file << p.data[p.size - 1];
        }

        output_file.close();
    }

    void read_costs_from_file_with_cdh_format(string input_file_name) {
        ifstream f(input_file_name);
        string line;
        
        for (int i = 1; i <= 3; i++)
            getline(f, line);

        getline(f, line);
        if (line != "safe_area") {
            cerr << "GREŠKA u funkciji read_costs_from_file_with_cdh_format()! U datoteci " + input_file_name + " na 4. linije je očekivana vrijednost \"safe_area\", a nalazi se vrijednost \"" + line + "\".\n";
            exit(1);
        }
        getline(f, line);
        while (line != "sensitive_area") {
            int delta_x, delta_y, fi_start, fi_finish;
            double c;

            string curr_value = "";
            int i = 0;
            while (line[i] != ',')
                curr_value += line[i++];
            delta_x = stoi(curr_value);

            curr_value = "";
            i++;
            while (line[i] != ',')
                curr_value += line[i++];
            delta_y = stoi(curr_value);

            curr_value = "";
            i++;
            while (line[i] != ',')
                curr_value += line[i++];
            fi_start = stoi(curr_value);

            curr_value = "";
            i++;
            while (line[i] != ',')
                curr_value += line[i++];
            fi_finish = stoi(curr_value);

            curr_value = "";
            i++;
            while (i < line.size())
                curr_value += line[i++];
            c = stod(curr_value);

            costs_safe_area[delta_x][delta_y][fi_start][fi_finish] = c;
            costs_safe_area_keys.push_back({delta_x, delta_y, fi_start, fi_finish});

            if (! getline(f, line)) {
                cerr << "GREŠKA u funkciji read_costs_from_file_with_cdh_format()! U datoteci " + input_file_name + " nije pronađena vrijednost \"sensitive_area\"";
                exit(1);
            }
        }

        while (getline(f, line)) { // sensitive area
            int state_start_index, state_finish_index;
            double c;

            string curr_value = "";
            int i = 0;

            while (line[i] != ',')
                curr_value += line[i++];
            state_start_index = stoi(curr_value);

            curr_value = "";
            i++;
            while (line[i] != ',')
                curr_value += line[i++];
            state_finish_index = stoi(curr_value);

            curr_value = "";
            i++;
            while (i < line.size())
                curr_value += line[i++];
            c = stod(curr_value);

            costs_sensitive_area[state_start_index][state_finish_index] = c;
            costs_sensitive_area_keys.push_back({state_start_index, state_finish_index});
        }

        f.close();
    }

    void read_paths_from_file_with_cdh_format(string input_file_name) {
        ifstream f(input_file_name);
        string line;
        
        for (int i = 1; i <= 3; i++)
            getline(f, line);

        getline(f, line);
        if (line != "safe_area") {
            cerr << "GREŠKA u funkciji read_costs_from_file_with_cdh_format()! U datoteci " + input_file_name + " na 4. linije je očekivana vrijednost \"safe_area\", a nalazi se vrijednost \"" + line + "\".\n";
            exit(1);
        }
        getline(f, line);
        while (line != "sensitive_area") {
            int delta_x, delta_y, fi_start, fi_finish;
            Path p;

            string curr_value = "";
            int i = 0;
            while (line[i] != ',')
                curr_value += line[i++];
            delta_x = stoi(curr_value);

            curr_value = "";
            i++;
            while (line[i] != ',')
                curr_value += line[i++];
            delta_y = stoi(curr_value);

            curr_value = "";
            i++;
            while (line[i] != ',')
                curr_value += line[i++];
            fi_start = stoi(curr_value);

            curr_value = "";
            i++;
            while (i < line.size())
                curr_value += line[i++];
            fi_finish = stoi(curr_value);

            getline(f, line);
            p.size = stoi(line);

            getline(f, line);
            p.data = {};
            curr_value = "";
            for (int i = 0; i < line.size(); i++) {
                if (line[i] != ',')
                    curr_value += line[i];
                else { // line[i] == ','
                    p.data.push_back(stod(curr_value));
                    curr_value = "";
                }
            }
            p.data.push_back(stod(curr_value));

            paths_safe_area[delta_x][delta_y][fi_start][fi_finish] = p;

            if (! getline(f, line)) {
                cerr << "GREŠKA u funkciji read_costs_from_file_with_cdh_format()! U datoteci " + input_file_name + " nije pronađena vrijednost \"sensitive_area\"";
                exit(1);
            }
        }

        while (getline(f, line)) { // sensitive area
            int state_start_index, state_finish_index;
            Path p;

            string curr_value = "";
            int i = 0;

            while (line[i] != ',')
                curr_value += line[i++];
            state_start_index = stoi(curr_value);

            curr_value = "";
            i++;
            while (i < line.size())
                curr_value += line[i++];
            state_finish_index = stoi(curr_value);

            getline(f, line);
            p.size = stoi(line);

            getline(f, line);
            p.data = {};
            curr_value = "";
            for (int i = 0; i < line.size(); i++) {
                if (line[i] != ',')
                    curr_value += line[i];
                else {
                    p.data.push_back(stod(curr_value));
                    curr_value = "";
                }
            }
            p.data.push_back(stod(curr_value));

            paths_sensitive_area[state_start_index][state_finish_index] = p;
        }

        f.close();
    }
};
