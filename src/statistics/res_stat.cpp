#include <iostream>
#include <fstream>
#include <cstring>

#include "DataCruncher.h"

int main(int argc, char** argv) {

    if ((argc != 3 || argc == 2) &&
        ((strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0))) {
        std::cout << "Usage: " << argv[0] << " <in.csv> <out.csv>\n";
    }

    std::string inFile  = std::string(argv[1]);
    std::string outFile = std::string(argv[2]);
    swlexp::DataCruncher dataCruncher(inFile, outFile);
    
    dataCruncher.crunch();

    return 0;
}