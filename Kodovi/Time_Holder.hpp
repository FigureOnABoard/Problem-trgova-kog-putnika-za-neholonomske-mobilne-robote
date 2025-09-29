#pragma once

using namespace std;

// Sadrži sve podatke o proteknutom vremenu za jednu generaciju
struct Time_Holder {
    // Varijable reproduction, crossover i mutation se postavljaju na 0 za prvu generaciju, jer se u prvoj generaciji pripadne funckije ne izvode.
    double total_elapsed_time = 0.0; // Ukupno vrijeme koje je proteklo od početka ozvođenja 1. generacije do ove generaciju.
        double elapsed_time_for_current_generation = 0.0; // Vrijeme koje je proteklo od početka izvođenja ove generacije (početka generiranja) do kraja njezinog izvršavanja (do kraja lokalnog poboljšanja).
            double generating = 0.0; // Vrijeme koje je proteklo za generiranje ove generacije;
                double semirandom_constructon = 0.0; // Vrijeme koje je proteklo da se polunasumično generira prva generacija
                double reproduction = 0.0; // Vrijeme proteklo za izvođenje funckije reproduction().
                double crossover = 0.0; // Vrijeme proteklo za izvođenje funkcije crossover().
                double mutation = 0.0; // Vrijeme proteklo za izvođenje funkcije mutation().
            double improving = 0.0; // Vrijeme koje je proteklo za poboljšavanje ove generacije
                double inserts = 0.0; // Vrijeme koje je proteklo za izvođenje funkcije inserts().
                double direct_two_opt = 0.0; // Vrijeme koje je proteklo za izvođenje funkcije direct_two_opt().
                double two_opt = 0.0; // Vrijeme koje je proteklo za izvođenje funkcije two_opt().
                double two_neighbour_swaps = 0.0; // Vrijeme koje je proteklo za izvođenje funkcije two_neighbour_swaps().
                double three_neighbour_swaps = 0.0; // Vrijeme koje je proteklo za izvođenje funkcije k_neighbour_swaps() s argumentom k=2
                double four_neighbour_swaps = 0.0; // Vrijeme koje je proteklo za izvođenje funkcije k_neighbour_swaps() s argumentom k=3
                double optimizeHeadings = 0.0; // Vrijeme koje je proteklo za izvođenje funkcije k_neighbour_swaps() s argumentom k=4

    Time_Holder() {
        this->total_elapsed_time = 0.0;
        this->elapsed_time_for_current_generation = 0.0;
            this->generating = 0.0;
                this->semirandom_constructon = 0.0;
                this->reproduction = 0.0;
                this->crossover = 0.0;
                this->mutation = 0.0;
            this->improving = 0.0;
                this->inserts = 0.0;
                this->direct_two_opt = 0.0;
                this->two_opt = 0.0;
                this->two_neighbour_swaps =  0.0;
                this->three_neighbour_swaps = 0.0;
                this->four_neighbour_swaps = 0.0;
                this->optimizeHeadings = 0.0;
    }
};
