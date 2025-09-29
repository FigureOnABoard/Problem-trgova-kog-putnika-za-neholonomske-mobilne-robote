#pragma once
#include "defines.h"
#include <math.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cstring>

#define INF 1000000.0

using namespace std;

int xf, yf, fif;

float mapResolution = 0.05;
int node_dist = 5;

class Vector : public vector<int>
{
public:
    //operator size_t() const { return (*this).size(); };
    operator size_t() const { return (*this)[0]*1600 + (*this)[1]*16 + (*this)[2]; };
};

// zamjena mjesta elemenata na indexima i,j u vektoru
void swap(int i, int j, vector<pair<Vector, double> > *v, map<Vector, int> *h)
{
    pair<Vector, double> tmp;
    tmp = (*v)[i];
    (*v)[i] = (*v)[j];
    (*v)[j] = tmp;

    (*h)[(*v)[i].first] = i;
    (*h)[(*v)[j].first] = j;
}

// podizanje elementa na odgovarajući nivo u binarnom stablu
void bubble_up(int index, vector<pair<Vector, double> > *v, map<Vector, int> *h)
{
    int new_ind;
    
    while (index > 0)
    {
        new_ind = (index - 1)/2;
        
        if ((*v)[index].second < (*v)[new_ind].second)
        {
            swap(index, new_ind, v, h);
            index = new_ind;
        } 
        else 
        {
            break;
        }
    }
}

// spuštanje elementa na odgovarajući nivo u binarnom stablu
void bubble_down(int index, vector<pair<Vector, double> > *v, map<Vector, int> *h)
{
    int size = v->size();
    int new_ind;
    
    while (true)
    {
        if (2 * index + 2 >= size)
        {
            if (2 * index + 1 >= size)
            {
                break;
            } 
            else 
            {
                new_ind = 2 * index + 1;
            }
        } 
        else 
        {
            if ((*v)[2 * index + 1].second < (*v)[2 * index + 2].second)
            {
                new_ind = 2 * index + 1;
            } 
            else 
            {
                new_ind = 2 * index + 2;
            }
        }
        
        if ((*v)[index].second > (*v)[new_ind].second)
        {
            swap(index, new_ind, v, h);
            index = new_ind;
        } 
        else 
        {
            break;
        }
    }
}

// provjera izvedivosti i određivanje cijene segmenta 
// ako je segment izvediv vraća se njegova cijena, u suprotnom se vraća vrijednost INF
double cost(int x, int y, int ei, int ej, vector<vector<pair<int, int> > > *cm, const vector<vector<short> >& map, vector<pair<vector<int>, double> > *g)
{
    double rez = 0.0;
    int m, n, mapWidth, mapHeight;
    double h = 0;
    
    mapWidth = map.size();
    
    if(mapWidth > 0)
        mapHeight = map[0].size();
    else return INF;


    // ***********************************************************************
    // ************* PROVJERA KOLIZIJE IZMEĐU VOZILA I PREPREKA  *************
    // ***********************************************************************
        
    for (unsigned int i = 0; i < cm[ei][ej].size(); i++)
    {
      m = node_dist * x + cm[ei][ej][i].first;
      n = node_dist * y + cm[ei][ej][i].second;
      
      // ako se bilo koja od čelija koju vozilo okupira tijekom izvođenja segmenta putanje nalazi 
      // izvan granica karte, vraća se vrijednost 100000  
      if (m < 0 || n < 0 || m > (mapWidth - 1) || n > (mapHeight - 1))
      {
        return INF;
      }
      else 
      {
        if (map[m][n] < 0)  // ako čelija zahvaća prepreku vraća se vrijednost 100000
          return INF;
        else
          rez += map[m][n];
      }
    }
    // **********************************************************************
    
    int vx = x + g[ei][ej].first[0];
    int vy = y + g[ei][ej].first[1];
        
    int dx = xf - vx;
    int dy = yf - vy;
        
    h = sqrt(dx * dx + dy * dy);    // heuristična komponenta cijene
    //fprintf(stderr, "xf: %d yf: %d vx: %d vy: %d h: %f g: %f\n", xf, yf, vx, vy, h, g[ei][ej].second);

    return (g[ei][ej].second + h);  // ako sve čelije segmenta putanje leže u slobodnom prostoru karte, vraća se cijena segmenta
}

vector<vector<double> > readVars(string pathToVars) {
    // Cecking if pathToVars includes the name of the file. If not, then the name is added.
    if (pathToVars.substr(pathToVars.length() - 4) != ".csv") {
        if (pathToVars.back() != '/') {
            pathToVars += "/";
        }
        pathToVars += "csv_vars.csv";
    }
    // Matrix/vector representation of the data from varVars variable from csv_vars.csv file
    vector<vector<double> > varVars;
    ifstream fileVars(pathToVars);
    string line;

    while (getline(fileVars, line)) {
        // currValue - the current value between the commas as a string
        // lineValues - vector containing double values in the current line. This will be added as a vector element to varVars.
        string currValue = "";
        vector<double> lineValues;
        for (char c : line) {
            if (c != ',') {
                currValue += c;
            }
            else {
                // currValueDouble - the current value between the commas as a double
                double currValueDouble = stod(currValue);
                lineValues.push_back(currValueDouble);
                currValue = "";
            }
        }
        // currValueDouble - the current value between the commas as a double
        double currValueDouble = stod(currValue);
        // Adding the final value - the one which isn't followed by a comma
        lineValues.push_back(currValueDouble);
        // Adding the first row to the matrix/multidimensional vector
        varVars.push_back(lineValues);
    }

    fileVars.close();

    return varVars;
}

// Reads the data from a file "csv_cost_j.csv", where j is an ordinal number of the file.
// j is received as an argument of the function.
vector<vector<double> > readCost(string pathToCost, int j) {
    string pathToFile;
    if (pathToCost[-1] != '/')
        pathToFile = pathToCost + "/csv_cost_" + to_string(j) + ".csv";
    else
        pathToFile = pathToCost + "csv_cost_" + to_string(j) + ".csv";
    // Matrix/vector representation of the data from varCost variable from csv_cost_j.csv file
    vector<vector<double> > varCost;

    ifstream fileCost(pathToFile);
    string line;

    while (getline(fileCost, line)) {
        // currValue - the current value between the commas as a string
        // lineValues - vector containing double values in the current line. This will be added as a vector element to varVars.
        string currValue = "";
        vector<double> lineValues;
        for (char c : line) {
            if (c != ',') {
                currValue += c;
            }
            else {
                // currValueDouble - the current value between the commas as a double
                double currValueDouble = stod(currValue);
                lineValues.push_back(currValueDouble);
                currValue = "";
            }
        }
        // currValueDouble - the current value between the commas as a double
        double currValueDouble = stod(currValue);
        // Adding the final value - the one which isn't followed by a comma
        lineValues.push_back(currValueDouble);
        // Adding the first row to the matrix/multidimensional vector
        varCost.push_back(lineValues);
    }

    fileCost.close();

    return varCost;
}

// Kreće se stanje po stanje i vraća ukupnu duljinu od početnog do završnog stanja
double get_cost_of_restructured_path(vector<double> &M, int sizeM[]) {
    if (M.size() == 0)
        return INF;
    string pathToVars = "lattice_planner/params/vars/csv_vars.csv";
    double trajectory_cost = 0.0;
    int k = 0;
    vector<vector<double> > varVars = readVars(pathToVars);
    for (int i = 0; i < sizeM[1]; i++) {
        // The row from which the new state begins
        int new_state_row = 0;
        while (varVars[new_state_row][0] != M[k])
            new_state_row++;
        trajectory_cost += varVars[new_state_row + (int)M[k+1]][4];
        k += 2;
    }
    return trajectory_cost;
}

// graph search algoritm (Dijkstra + A*)
// Rezultat: M
// M za svaki čvor v sadrži početan smjer kretanja u v (iz raspona 0-15 i 16-31) i index segmenta koji uz danu početnu orijentaciju vodi do slijedećeg čvora.
void graphSearch(string pathToVars, string pathToCost, const vector<vector<short> >& vMap, int start[], int finish[],
                 vector<double> &M, int sizeM[], int backwards_flag)
{
cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    // početne koordinate i orijentacija vozila
    int xs =  start[0];
    int ys =  start[1];
    int fis = start[2];
    
    xf = finish[0];
    yf = finish[1];
    fif = finish[2];

    map<Vector, int> hm;
    map<Vector, pair<int, int> > hm_sol;

    // n = broj stupaca u datoteci csv_vars.csv
    // m = broj redaka u datoteci csv_vars.csv
    // nc = broj stupaca u datotecti csv_cost_j.csv
    // mc = broj redaka u datoteci dcv_cost_j.csv
    int m, n, mc, nc;
    double *data_in, *data_out, *cell_in, *map_in;

    vector<pair<vector<int>, double> > graph[16];
    vector<vector<pair<int, int> > > cost_map[16];
    
    vector<int> tmp;
    vector<pair<int, int> > vptmp;
    double sf;

    // Read data from csv_vars.csv
    vector<vector<double> > varVars = readVars(pathToVars);
    // m - broj redaka
    // n - broj stupaca
    m = varVars.size();
    n = varVars[0].size();

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

        // Read data from csv_cost_j.csv, where j = i + 1
        vector<vector<double> > varCost = readCost(pathToCost, i + 1);
        // mc - broj redaka u varCost
        // nc - broj stupaca u varCost
        mc = varCost.size();
        nc = varCost[0].size();

        vptmp.clear();
        for (int j = 0; j < nc; j++) {
            vptmp.push_back(make_pair(varCost[0][j], varCost[1][j]));
        }

        // indexi elemenata vektora "cost_map" predstavljaju početne orijentacije (0-15)
        // svaki element vektora "cost_map" sadrži vektore parova s
        // x-y koordinatama svih čelija kroz koje vozilo prolazi prilikom 
        // izvođenja pojedinog segmenta s dotičnom početnom orijentacijom
        cost_map[(int)varVars[i][0]].push_back(vptmp);
    }

    // definicija vektora za pohranu posjećenih čvorova prilikom pretraživanja grafa
    // koristi se kao sortirano binarno stablo (min-heap)
    // korijen stabla (prvi element vektora) je čvor s najmanjom cijenom
    vector<pair<Vector, double> > heap;

    double c;
    double new_c;
    //double pen = 0.0; // penalizacija promjena kuta
    bool path_exist = true;

    Vector v;
    Vector new_v;

    // spremanje početnih koordinata i orijentacije vozila u vektor "v"
    v.push_back(xs);
    v.push_back(ys);
    v.push_back(fis);

    heap.push_back(make_pair(v, 0.0));

    bool found;

    while (true) {
        if (heap.size() == 0)
        {
            path_exist = false;
            break;
        }
        swap(0, heap.size() - 1, &heap, &hm);   // zamjena mjesta prvog i posljednjeg čvora u stablu -> čvor s najmanjom cijenom dolazi na dno stabla

        // pomicanje u sljedeći čvor s najmanjom cijenom iz kojega će se dalje provjeravati ukupne cijene dolaska u njegove susjedne čvorove 
        v = heap.back().first;  // koordinate trenutnog čvora (x,y,fi)
        c = heap.back().second; // ukupna minimalna cijena dolaska u trenutni čvor iz početnog čvora
        hm[v] = -1;

        heap.pop_back();    // uklanjanje čvora s najmanjom cijenom iz binarnog stabla (to je trenutno zadnji čvor u heap-u)
        bubble_down(0, &heap, &hm); // spuštanje elementa koji se privremeno našao na root mjestu binarnog stabla (nakon prethodne swap operacije)

        // određivanje ciljnih koordinata kod removinga
        if (v[0] == xf  &&  v[1] == yf  &&  v[2] == fif) // ako su koord. tenutnog čvora jednake koordinatama ciljnog čvora..
        {
            //fprintf(stderr, "Pronađen ciljni čvor!!\n");
            break;  // prekida se proces planiranja putanje..
        }

        // određivanje najkraćih putanja do svih susjednih čvorova pod pretpostavkom da se vozilo kreće unaprijed
        for (unsigned int i = 0; i < graph[v[2]].size(); i++)   // graph[v[2]] sadrži podatke o svim susjednim čvorovima u koje se može doći iz trenutnog čvora "v" uz početnu orijentaciju v[2] 
        {
            // u "new_v" se upisuju x-y koordinate i konačne orijentacije i-tog susjednog čvora koji je povezan s čvorom "v",
            // a u koji se iz čvora "v" može doći uz početnu orijentaciju v[2]
            new_v.clear();
            new_v.push_back(v[0] + graph[v[2]][i].first[0]);    // x - koordinata i-tog susjednog čvora
            new_v.push_back(v[1] + graph[v[2]][i].first[1]);    // y - koordinata i-tog susjednog čvora
            new_v.push_back(graph[v[2]][i].first[2]);           // konačna orijentacija u i-tom susjednom čvoru

            if (hm.find(new_v) == hm.end()) // ako "new_v" još nije dodan u mapu "hm"..
            {
                //heap.push_back(make_pair(new_v, c + graph[v[2]][i].second + pen * abs(new_v[2] - v[2])));
                new_c = cost(v[0], v[1], v[2], i, cost_map, vMap, graph);   // provjera izvedivosti i određivanje cijene segmenta
                        
                if (new_c < INF)    // ako je segment izvediv
                {
                    heap.push_back(make_pair(new_v, c + new_c));    // čvor se dodaje na kraj stabla zajedno s pripadajućom ukupnom cijenom
                    hm[new_v] = heap.size() - 1;        // pamti se index čvora u stablu
                    hm_sol[new_v] = make_pair(v[2], i); // pamti se orijentacija u prethodnom čvoru i index segmenta do čvora "new_v"
                    bubble_up(heap.size() - 1, &heap, &hm); // čvor se podiže na odgovarajuće mjesto u stablu tako da stablo bude sortirano 
                }
            } 
            else 
            {
                if (hm[new_v] != -1)    
                {
                    // ako je cijena dolaska u susjedni čvor "new_v" manja od prehodne cijene za taj čvor..
                    if (c + cost(v[0], v[1], v[2], i, cost_map, vMap, graph) < heap[hm[new_v]].second)
                    {
                        if(heap[hm[new_v]].first[0] != new_v[0] || heap[hm[new_v]].first[1] != new_v[1] || heap[hm[new_v]].first[2] != new_v[2])
                            fprintf(stderr, "Error in graph search!\n");

                        //heap[hm[new_v]].second = c + graph[v[2]][i].second + pen * (abs(new_v[2] - v[2]));
                        heap[hm[new_v]].second = c + cost(v[0], v[1], v[2], i, cost_map, vMap, graph); // upisuje se nova cijena do čvora "new_v"
                        hm_sol[new_v] = make_pair(v[2], i); // pamti se orijentacija u prethodnom čvoru i index segmenta do čvora "new_v"  
                        bubble_up(hm[new_v], &heap, &hm);   // modificira se mjesto "new_v" čvora u stablu tako da stablo ponovno bude sortirano
                    }
                }
            }
        }

        // određivanje najkraćih putanja do svih čvorova pod pretpostavkom da se vozilo kreće unatrag
        if (backwards_flag == 1) {
            for (unsigned int i = 0; i < graph[(v[2] + 8) % 16].size(); i++)    // graph[(v[2] + 8) % 16] sadrži podatke o svim susjednim čvorovima u koje se može doći iz trenutnog čvora "v" kretanjem unatrag 
            {
                // u "new_v" se upisuju x-y koordinate i konačne orijentacije i-tog susjednog čvora koji je povezan s čvorom "v",
                // a u koji se iz čvora "v" može doći uz početnu orijentaciju (v[2] + 8) % 16 (vožnjom unatrag)
                new_v.clear();
                new_v.push_back(v[0] + graph[(v[2] + 8) % 16][i].first[0]);     // x - koordinata i-tog susjednog čvora
                new_v.push_back(v[1] + graph[(v[2] + 8) % 16][i].first[1]);     // y - koordinata i-tog susjednog čvora
                new_v.push_back((graph[(v[2] + 8) % 16][i].first[2] + 8) % 16); // konačna orijentacija u i-tom susjednom čvoru

                if (hm.find(new_v) == hm.end()) // ako "new_v" još nije dodan u mapu "hm"..
                {
                    //heap.push_back(make_pair(new_v, c + graph[v[2]][i].second + pen * abs(new_v[2] - v[2])));
                    new_c = 2 * cost(v[0], v[1], (v[2] + 8) % 16, i, cost_map, vMap, graph);    // provjera izvedivosti i određivanje cijene segmenta
                    if (new_c < INF)    // ako je segment izvediv
                    {
                        heap.push_back(make_pair(new_v, c + new_c));    // čvor se dodaje na kraj stabla zajedno s pripadajućom ukupnom cijenom
                        hm[new_v] = heap.size() - 1;    // pamti se index čvora u stablu
                        hm_sol[new_v] = make_pair((v[2] + 8) % 16 + 16, i); // pamti se orijentacija u prethodnom čvoru i index segmenta do čvora "new_v"
                        bubble_up(heap.size() - 1, &heap, &hm); // čvor se podiže na odgovarajuće mjesto u stablu tako da stablo bude sortirano
                    }
                } 
                else 
                {
                    if (hm[new_v] != -1)
                    {
                        // ako je cijena dolaska u susjedni čvor "new_v" manja od prehodne cijene za taj čvor..
                        if (c + 2*cost(v[0], v[1], (v[2] + 8) % 16, i, cost_map, vMap, graph) < heap[hm[new_v]].second)
                        {
                            if(heap[hm[new_v]].first[0] != new_v[0] ||  heap[hm[new_v]].first[1] != new_v[1] || heap[hm[new_v]].first[2] != new_v[2])
                                fprintf(stderr, "Error in graph search!\n");

                            //heap[hm[new_v]].second = c + graph[v[2]][i].second + pen * (abs(new_v[2] - v[2]));
                            heap[hm[new_v]].second = c + 2*cost(v[0], v[1], (v[2] + 8)%16, i, cost_map, vMap, graph); // upisuje se nova cijena do čvora "new_v"
                            hm_sol[new_v] = make_pair((v[2] + 8) % 16 + 16, i); // pamti se orijentacija u prethodnom čvoru i index segmenta do čvora "new_v"
                            bubble_up(hm[new_v], &heap, &hm);   // modificira se mjesto "new_v" čvora u stablu tako da stablo ponovno bude sortirano
                        }
                    }
                }
            }
        }
    }

    if (path_exist) // ako je pronađena izvediva putanja
    {
        vector<pair<int, int> > sol; // sol se odnosi na par (orijantacija u čvoru, index segmenta do slijedećeg čvora)
        pair<int, int> p;
        vector<int> orient; // "Vektor stvarnih ( < 16) orijentacija robota"
            
        v[0] = xf; v[1] = yf; v[2] = fif;
            
        // kretanje unatrag od ciljnog čvora do početnog čvora
        while (true)
        {
            // kada se dođe do početnog čvora prekida se while petlja
            if (v[0] == xs  &&  v[1] == ys  &&  v[2] == fis)
            {
                break;
            }

            p = hm_sol[v];
            sol.push_back(p);   // pohrana početne orijentacije u prethodnom čvoru i indexa segmenta putanje koji povezuje prethodni čvor s trenutnim čvorom "v" 

            // pomicanje u prethodni čvor (tj. u jedan čvor bliže početnom čvoru) 
            v[0] = v[0] - graph[p.first % 16][p.second].first[0];
            v[1] = v[1] - graph[p.first % 16][p.second].first[1];

            if (p.first > 15)
            {
                v[2] = (p.first + 8) % 16;
            }
            else
                v[2] = p.first % 16;
                
            orient.push_back(v[2]);
        }
 
        M = {};
        sizeM[0] = 2;
        sizeM[1] = sol.size();  // ukupan broj segmenata od kojih se sastoji izračunata putanja
        //sizeM[1] = 336;   // ukupan broj segmenata od kojih se sastoji izračunata putanja

        for (int i = sol.size() - 1; i >= 0; i--)
        {
            M.push_back(sol[i].first % 16);    // početna orijentacija u i-tom čvoru
            M.push_back(sol[i].second);      // index segmenta koji uz danu početnu orijentaciju vodi do sljedećeg čvora na putanji
        }
        
        // ZA ISCRTAVANJE CIJELE LATICE:
        /*for(int j = 0; j < 16; j++)
        {
            for (unsigned int i = 0; i < graph[j].size(); i++)
            {
                data_out[k] = j;        // početna orijentacija u i-tom čvoru
                data_out[k+1] = i;      // index segmenta koji uz danu početnu orijentaciju vodi do sljedećeg čvora na putanji
                k += 2;
            }
        }*/

/*
        ////////////////////////////////////////////////////////////////
        //      PRIKAZ DOBIVENOG RJEŠENJA           ////////////////////
        ////////////////////////////////////////////////////////////////
        cout << "######################" << endl;
        for (int i = 0; i < sol.size(); i++) {
            cout << "(" << sol[i].first << "," << sol[i].second << ") ";
        }
        cout << endl;
        ////////////////////////////////////////////////////////////////
        //      SPREMANJE DOBIVENOG RJEŠENJA U COMMA-SEPARATED     ///
        //      VALUES (.csv) DATOETEKU                              ///
        ////////////////////////////////////////////////////////////////
        ofstream result_file("results/res_test_09_backwards.csv");
        for (int i = sol.size() - 1; i >= 0; i--) {
            result_file << sol[i].first << "," << sol[i].second;
            if (i != 0) 
                result_file << endl;
        }
        result_file.close();
*/
    }
    else
    {
        M = {};
        sizeM[0] = 0;
        sizeM[1] = 0;
    }
}
