#pragma once
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#define INF 1000000.0
#define NUM_OF_DIRECTIONS 16

using namespace std;

// Prima string koji sadrži vektor vektora cijelih brojeva u C++ formatu
// Vraća pravi vektor.
vector<vector<int>> transform_string_to_vector(string s) {
    // Uklanjanje rubnih zagrada
    s.erase(s.size() - 1, 1);
    s.erase(0, 1);
    // Dodavanje elemenata u vektor vektora
    vector<vector<int>> points;
    vector<int> new_point = {};
    string new_coordinate = "";
    bool reading_point = false;
    for (char c : s) {
        if (reading_point == true) {
            // Kraj "točke"/zadnja koordinata točke
            if (c == '}') {
                reading_point = false;
                new_point.push_back(stoi(new_coordinate));
                points.push_back(new_point);
                new_coordinate = "";
                new_point = {};
            }
            else if (c == ',') { // završena koordinata
                new_point.push_back(stoi(new_coordinate));
                new_coordinate = "";
            }
            else { // još uvijek se čita koordinata
                new_coordinate += c;
            }
        }
        else { // reading_point == false
            if (c == '{') { // Početak "točke" - slijedeći znak je prva koordinata
                reading_point = true;
                new_coordinate = "";
            }
            // else c == ',' ili c == ' '
        }
    }
    // Vraćanje rezultata
    return points;
}

// Iz datoteke koja ima točke zapisane u vitičastim zagradama se čita i zapisuje točke u vektor.
// Npr. ako datoteke ima zapis {{2,5}, {5,5}, {10,10}, {2,5}}, podaci se spremaju u C++ vektor vektora {{2,5}, {5,5}, {10,10}, {2,5}}
// Slično funkciji readPointsFromCurlyBrackets() iz pathPlotter.py
vector<vector<int>> read_points_from_curly_brackets(string filename) {
    ifstream f(filename);
    string line;
    getline(f, line);
    // Zatvaranje datoteke
    f.close();
    if (line[0] != '{' || line[line.size() - 1] != '}') {
        cerr << "Greška u read_points_from_curly_brackets()! Datoteka ne započinje s \'{\' i(li) ne završava s \'}\'";
        exit(1);
    }
    vector<vector<int>> points = transform_string_to_vector(line);
    // Vraćanje rezultata
    return points;
}

// Prima string koji sadrži vekotr cijelih brojeva.
// Vraća pravi vektor
vector<int> transform_string_int_to_vector_int(string s) {
    // Dodavanje elemenata u vektor cijelih brojeva
    vector<int> v;
    string number;
    char c;
    // Ako je pročitani znak ',', znači da su se pročitale sve znamenke cijelog broja. Pročitani broj se dodaju u v i number se resetira.
    // Ako je pričitani znak znamenka, ona se uzima kao trenutni broj i dodaje se na kraj stringa u varijabli number.
    // Ako je pročitani znak ' ', znači da se nalazi na razmaku između dva broja (ili možda kraja vektora) i ne događa se ništa.
    // Ako je pročitani znak '}', znači da se je došlo do kraja vektora.
    int i = 0;
    while (true) {
        c = s[i];
        if (c == '{') {
            number = "";
        }
        else if (c == ',') {
            v.push_back(stoi(number));
            number = "";
        }
        else if (c == ' ') {
        }
        else if (c == '}') {
            v.push_back(stoi(number));
            break;
        }
        else { // c je znamenka
            number += c;
        }
        i++;
    }
    // Vraćanje dobivenog vektora
for (int el : v) cout << el << ' ';
cout << endl;
    return v;
}
