#pragma once
#include <iostream>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "../Solution.hpp"
#include "../Costs_Data_Holder.hpp"
#include "transform_string_to_vector.hpp"

using namespace std;

// Pomoćni header za čitanje generacije i duljinu trajanja programa do izvođenja te generacije.
void read_generation(string input_file, vector<vector<int>> states, Costs_Data_Holder cdh, set<Solution> &generation, double &elapsed_time, int backwards_flag) {
    cout << "\n!!! Pokrenut je read_generation(). Namiješteno je da će čitati iz datoteke\n";
    cout << "\t" << input_file << endl;
    cout << "Želite li nastaviti s čitanjem te datoteke (y/n)? ";
    string r = "";
    cin >> r;
    while (r != "y" && r != "n") {
        cout << "Odgovor nije prepoznat. ";
        cin >> r;
    }
    if (r == "n") {
        cout << "Prekid programa.\n";
        exit(0);
    }
    else { // r == "y"
        ifstream f(input_file);
        string line;
        // Redak u kojem piše "Generacija x"
        getline(f, line);
        // Retci u formatu {s1, s2, s3, ..., s1} cost = number
        getline(f, line);
        while (line[0] == ' ' && line[1] == '{') {
            string solution_string = "";
            vector<int> solution;
            for (int i = 1; line[i] != '}'; i++)
                solution_string += line[i];
            solution_string += '}';
            // Pretvorba vektora u strin formatu p pravi vektor
            solution = transform_string_int_to_vector_int(solution_string);
            
            // Dodavanje solucije u generaciju
            Solution solution_solution = Solution(solution, states, cdh, backwards_flag);
            generation.insert(solution_solution);

            // Iduća linija u datoteci
            getline(f, line);
            if (line.length() == 0) {
                getline(f, line);
                break;
            }
        }
        // Redak koji sadrži do tad prođeno vrijeme (ne radimo getlne() jer je on već napravljen u while petlji)
        int i = 0;
        while (line[i] != ':')
            i++;
        i += 2;
        string elapsed_time_string = "";
        while (line[i] != ' ' && line[i] != 's')
            elapsed_time_string += line[i++];
        // Pretvorba stringa u double
        elapsed_time = stod(elapsed_time_string);
    }
}
