#pragma once
#include <iostream>
#include <vector>

using namespace std;

struct Path {
    // data sadrži informacije o putanji. Na prnim indeksima (k = 0, 2, 4,...) nalaze se orijen ntacija u trenutnom čvoru,
    // a na neparnim indeksima k+1 = (1, 3, 5,...) nalaze se indeksi segmenta koji uz orijentaciju na k-tom indeksu vode do idućeg čvora.
    vector<double> data;
    // path sadrži duljinu niza na koji pokazuje data
    int size;
};