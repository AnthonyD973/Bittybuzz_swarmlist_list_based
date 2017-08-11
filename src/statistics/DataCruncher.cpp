#include <stdexcept>
#include <sstream>

#include "DataCruncher.h"

swlexp::DataCruncher::DataCruncher(
        const std::string& inFile,
        const std::string& outFile)
    : m_inFile(inFile.c_str())
    , m_outFile(outFile.c_str())
{
    if (m_inFile.fail()) {
        throw std::runtime_error("File \"" + inFile + "\" could not be opened for reading.");
    }
    if (m_outFile.fail()) {
        throw std::runtime_error("File \"" + outFile + "\" could not be opened for writing.");
    }
}

/****************************************/
/****************************************/

swlexp::DataCruncher::~DataCruncher() {

}

/****************************************/
/****************************************/

void swlexp::DataCruncher::crunch() {
    DataEntry dataEntry;
    std::string line;
    std::getline(m_inFile, line); // Ignore header line
    while (1) {
        long long unsigned int totalConsensusTime = 0;
        unsigned int numExperiments = 0;

        std::getline(m_inFile, line);
        std::istringstream lineStrm(line);
        try {
            lineStrm >> dataEntry;
        }
        catch (std::length_error& e) {
            break;
        }

        totalConsensusTime += dataEntry.getConsensusTime();
        ++numExperiments;

        if (std::getline(m_inFile, line)) {
            lineStrm.clear();
            DataEntry deLoop;
            try {
                lineStrm >> deLoop;
            } catch (std::length_error& e) {}
            while (dataEntry.isSameExpConfigAs(deLoop)) {
                totalConsensusTime += deLoop.getConsensusTime();
                ++numExperiments;
            }
        }
    }
}

/****************************************/
/****************************************/

swlexp::DataCruncher::DataEntry::DataEntry() {

}

/****************************************/
/****************************************/

swlexp::DataCruncher::DataEntry::~DataEntry() {

}

/****************************************/
/****************************************/

bool swlexp::DataCruncher::DataEntry::isSameExpConfigAs(
        const DataEntry& other) const {
    return m_protocol  == other.m_protocol  &&
           m_topology  == other.m_topology  &&
           m_numRobots == other.m_numRobots &&
           m_dropProb  == other.m_dropProb;
}

/****************************************/
/****************************************/

std::istream& swlexp::operator>>(
        std::istream& istrm,
        DataCruncher::DataEntry& de) {
    // Clear values in case we throw an error.
    de.m_protocol  = "";
    de.m_topology  = "";
    de.m_numRobots = 0;
    de.m_dropProb  = 0.0;
    de.m_consensusTime = ~0;

    // Get string size.
    std::streampos initPos = istrm.tellg();
    istrm.seekg(0, std::ios::end);
    std::streampos size = istrm.tellg();
    istrm.seekg(initPos);

    if (size != 0) {
        char dummy; // Buffer for the comma separator.
        std::getline(istrm, de.m_protocol, ',');
        std::getline(istrm, de.m_topology, ',');
        istrm >> de.m_numRobots >> dummy >>
                 de.m_dropProb  >> dummy >>
                 de.m_consensusTime;
    }
    else {
        throw std::length_error("Tried to extract data from empty stream.");
    }
    return istrm;
}