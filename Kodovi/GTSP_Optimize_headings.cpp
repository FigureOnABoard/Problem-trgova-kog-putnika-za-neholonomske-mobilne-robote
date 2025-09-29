#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <time.h>

#include "Time_Holder.hpp"
#include "Costs_Data_Holder.hpp"
#include "lattice_planner/optimizeHeading.hpp"
#include "headers/transform_string_to_vector.hpp"
#include "headers/vector_functions.hpp"

#define INF 1000000.0
#define NUM_OF_DIRECTIONS 16

using namespace std;

/*
Program preuzima rješenje TSP-a i iz njega napravi rješenje GTSP-a koje ima isti poredak čvorova i čije orijentacije u čvorovima daju najkraći put za taj poredak čvorova.
*/

/*
Program se pokreće ovako: GTSP_Optimize_headings.exe datoteka1 datoteka2 [backwards_flag]
gdje je "datoteka1" datoteka s podacima o problemu koju je napravio Cast_Data_Holder iz koje se definira problem, "datoteka2" datoteka u koju se sprema rješenja ovog programa,
a [backwards_flag] zastavica koja označava može li se robot kretati unatrag ili ne. Ako je [backwards_flag] jednak 0, ne može se kretati unatrag, a ako je 1 onda se može kretati
unatrag. Ako se [backwards_flag] ne unese, automatski se postavlja na 1.
*/

// Stvara i vraća vektor s koordinatama čvorova koje će se morati nalaziti na robotovom putu.
// Indeks čvora u vektoru je jedinstveni indetifikator/indeks tog čvora.
vector<vector<int>> define_problem() {
    vector<vector<int>> nodes = {{108,100}, {100,100}, {101,100}, {102,100}, {100,99}, {101,99}, {102,99}, {100,98}, {101,98}, {102,98}, {100,112}, {101,112}, {102,112}, {100,111}, {101,111}, {102,111}, {100,110}, {101,110}, {102,110}, {107,119}, {108,119}, {109,119}, {107,118}, {108,118}, {109,118}, {107,117}, {108,117}, {109,117}, {118,109}, {119,109}, {120,109}, {118,108}, {119,108}, {120,108}, {118,107}, {119,107}, {120,107}, {118,100}, {119,100}, {120,100}, {118,99}, {119,99}, {120,99}, {118,98}, {119,98}, {120,98}};
    return nodes;
}

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

// Čita podatke iz datoteke koju je generirao OR-Tools_TSP_solver.py koja sadrži rješenje TSP problema.
// Sprema poredak čvorova dobiven rješavanjem TSP-a i vrijeme koliko je trajalo nalaženje tog rješenja u varijable solution_nodes i 
void read_TSP_result_from_file(string file_name, vector<vector<int>> &solution_nodes, double &execution_time_TSP) {
    ifstream f(file_name);
    string line;
    // Linija s rješenjem TSP-a
    getline(f, line);
    string solution_string = "";
    int i = 0;
    while (line[i] != '{')
        i++;
    while (i < line.size()) {
        solution_string += line[i];
        i++;
    }
    // Transformacija string vektora u pravi vektor
    solution_nodes = transform_string_to_vector(solution_string);
    
    // Linija s vremenom trajanja TSP solvera
    getline(f, line);
    string execution_time_TSP_string = "";
    i = 0;
    while (isdigit(line[i]) == false)
        i++;
    while (i < line.size() && line[i] != ' ' && line[i] != 's') {
        execution_time_TSP_string += line[i];
        i++;
    }
    // Transformacija string broja u pravi double
    execution_time_TSP = stod(execution_time_TSP_string);

    // Zatvaranje datoteke
    f.close();

    // Završetak
}

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

double calculate_cost(vector<int> solution, Costs_Data_Holder &cdh, vector<vector<int>> states, int backwards_flag) {
    double cost = 0.0;
    int s1, s2;
    for (int i = 0; i < solution.size() - 1; i++) {
        s1 = solution[i];
        s2 = solution[i + 1];
        cost += cdh.determine_cost(s1, s2, states, backwards_flag);
    }
    return cost;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        cerr << "GRESKA! Nedovoljno argumenata pri pozivu programa.\n";
        cerr << "Program se pokreće ovako: GTSP_Optimize_headings.exe datoteka1 datoteka2 [backwards_flag]\n";
        cerr << "gdje je \"datoteka1\" datoteka s podacima o problemu koju je napravio Cast_Daata_Holder iz koje se definira problem, ";
        cerr << "a \"datoteka2\" datoteka u koju se sprema rješenja ovog programa.\n";
        cerr << "\"backwards_flag\" je zastava koja iznačava može li se vozilo kretati unazad i može biti 0 ili 1.\n";
        cerr << "AKo je backwards_flag 0, onda se robot ne može kretati unatrag. Ako je 1, onda se može. Ako se \"backwards_flag\" ne unese, defaultno se postavlja na 1.\n";
        cerr << "(Ako se sve duljine putanja nalaze u \"datoteka1\", onda je svejedno hoće li se unsti \"backwards_flag\" ili ne.)";
        exit(1);
    }
    int backwards_flag = 1;
    if (argc >= 4)
        backwards_flag = stoi(argv[3]);

    clock_t start_time, end_time;
    double execution_time_TSP, execution_time_optimize_headings, execution_time;

    // Poredak čvorova u rješenju u formatu {{x0,y0}, {x1,y1}, ...}
    vector<vector<int>> solution_nodes;
    read_TSP_result_from_file("GTSP_results/TSP_result.txt", solution_nodes, execution_time_TSP);
    vector<vector<int>> nodes, states;
    Costs_Data_Holder cdh;
    cout << "read_problem()\n";
    read_problem(argv[1], nodes, states, cdh);

    // Ispis svih čvorova i njihovih indeksa za provjeru
    for (int i = 0; i < nodes.size(); i++)
        cout << i << "->{" << nodes[i][0] << ',' << nodes[i][1] << "}, ";
    cout << endl;

    // Poredak čvorova u rješenju prikazan preko indeksa
    vector<int> order_of_nodes = {};
    for (vector<int> n : solution_nodes)
        order_of_nodes.push_back(get_index_of_node(n, nodes));
    
    // Odabir najboljih stanja čiji poredak odgovara poretku čvorova u order_of_nodes
    Time_Holder th;
    vector<int> solution = optimizeHeadings(order_of_nodes, nodes, states, cdh, th, backwards_flag);
    // Duljina dobivenog rješenja
    double min_cost = calculate_cost(solution, cdh, states, backwards_flag);
    execution_time_optimize_headings = th.optimizeHeadings;
    execution_time = execution_time_TSP + execution_time_optimize_headings;

    // Zapisivanje konačnog rezultata
    string output_file_name = argv[2];
    if (backwards_flag == 0)
        output_file_name = "NO_backwards/" + output_file_name;
    else
        output_file_name = "backwards/" + output_file_name;
    ofstream f("GTSP_results/Decoupled-OH results/" + output_file_name);
    cout << "\nFinished! Best solution: {";
    f << "Finished! Best solution: {";
    int s_index = solution[0];
    cout << s_index;
    f << s_index;
    for (int i = 1; i < solution.size(); i++) {
        s_index = solution[i];
        cout << ", " << s_index;
        f << ", " << s_index;
    }
    cout << "}\n";
    f << "}\n";
    cout << "cost: " << min_cost << endl;
    f << "cost: " << min_cost << endl;
    cout << "Execution time: " << execution_time << " s\n";
    f << "Execution time: " << execution_time << " s\n";
    cout << "Execution time (OR-Tools): " << execution_time_TSP << " s\n";
    f << "Execution time (OR-Tools): " << execution_time_TSP << " s\n";
    cout << "Execution time (Optimize headings): " << execution_time_optimize_headings << " s\n";
    f << "Execution time (Optimize headings): " << execution_time_optimize_headings << " s\n";
    // Zatvaranje datoteke
    f.close();

    return 0;
}
