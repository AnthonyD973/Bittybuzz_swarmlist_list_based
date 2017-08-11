#ifndef DATA_CRUNCHER_H
#define DATA_CRUNCHER_H

#include <iostream>
#include <fstream>
#include <cinttypes>

namespace swlexp {

    class DataCruncher {

    public:
        class DataEntry {
        protected:
            friend std::istream& operator>>(
                    std::istream& istrm,
                    DataEntry& dataEntry);
        public:
            DataEntry();
            ~DataEntry();

            bool isSameExpConfigAs(const DataEntry& other) const;


            inline
            std::string getProtocol() const { return m_protocol; }

            inline
            std::string getTopology() const { return m_topology; }

            inline
            uint32_t    getNumRobots() const { return m_numRobots; }

            inline
            double      getDropProb() const { return m_dropProb; }

            inline
            uint32_t    getConsensusTime() const { return m_consensusTime; }

        private:
            std::string m_protocol;
            std::string m_topology;
            uint32_t    m_numRobots;
            double      m_dropProb;
            uint32_t    m_consensusTime;
        };

    public:

        DataCruncher(const std::string& inFile, const std::string& outFile);
        ~DataCruncher();

        void crunch();
    
    private:

    private:
        std::ifstream m_inFile;
        std::ofstream m_outFile;
    };

    std::istream& operator>>(
            std::istream& istrm,
            swlexp::DataCruncher::DataEntry& dh);

}

#endif // !DATA_CRUNCHER_H