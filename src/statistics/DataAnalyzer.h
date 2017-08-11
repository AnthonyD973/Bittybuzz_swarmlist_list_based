#ifndef DATA_ANALYZER_H
#define DATA_ANALYZER_H

#include <vector>

#include "DataCruncher.h"

namespace swlexp {

    class DataAnalyzer {
    public:
        DataAnalyzer();
        ~DataAnalyzer();
        void addEntry();
        std::string produceResults() const;
        void clearEntries();

    private:
        std::vector<DataCruncher::DataEntry> m_entries;
    };

}

#endif // !DATA_ANALYZER_H