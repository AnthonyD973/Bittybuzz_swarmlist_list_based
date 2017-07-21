#include <algorithm>
#include <vector>
#include <unordered_map>
#include <cinttypes>
#include <sstream>

#include "Swarmlist.h"

namespace swlexp {
    bool Swarmlist::c_entriesShouldBecomeInactive;
    argos::UInt64 Swarmlist::c_totalNumActive = 0;
}

/****************************************/
/****************************************/

swlexp::Swarmlist::Swarmlist()
    : m_numActive(0)
    , m_next(0)
{
    m_idToIndex.clear();
    m_data.clear();
}

/****************************************/
/****************************************/

swlexp::Swarmlist::~Swarmlist() {

}

/****************************************/
/****************************************/
 
const swlexp::Swarmlist::Entry& swlexp::Swarmlist::get(RobotId robot) const {
    return m_data[m_idToIndex.at(robot)];
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::update(RobotId robot,
                               argos::UInt8 swarmMask,
                               swlexp::Lamport8 lamport) {
    // Does the entry already exist?
    const swlexp::Swarmlist::Entry* existingEntry;
    bool existed;
    try {
        existingEntry = &get(robot);
        existed = true;
    }
    catch (std::out_of_range& err) {
        existed = false;
    }

    bool shouldUpdate;
    if (existed) {
        // Yes.
         swlexp::Lamport8 oldLamport = existingEntry->getLamport();

        // Is entry active?
        if (existingEntry->isActive(m_id)) {
            // Yes ; use circular lamport clock model to determine
            // whether the entry should be updated.
            shouldUpdate = lamport.isNewerThan(oldLamport);
        }
        else {
            // No ; the entry is newer if the lamport clocks are different.
            shouldUpdate = (lamport != oldLamport);
            if (shouldUpdate) {
                ++m_numActive;
                ++c_totalNumActive;
            }
        }
    }
    else {
        // No ; it's a new entry.
        shouldUpdate = 1;
        ++m_numActive;
        ++c_totalNumActive;
    }

    if (shouldUpdate) {
        // Create a new entry (which also resets the timer)
        swlexp::Swarmlist::Entry newEntry = swlexp::Swarmlist::Entry(
            robot, swarmMask, lamport);
        _set(newEntry);
    }
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::tick() {
    if (c_entriesShouldBecomeInactive) {
        for (uint8_t i = 0; i < m_data.size(); ++i) {
            // Deal with entries in inactive mode
            swlexp::Swarmlist::Entry& curr = m_data[i];
            if (curr.isActive(m_id)) {
                curr.tick();
                if (!curr.isActive(m_id)) {
                    --m_numActive;
                    --c_totalNumActive;
                }
                _set(curr);
            }
        }
    }
}

/****************************************/
/****************************************/

std::string swlexp::Swarmlist::serializeData(char elemDelim, char entryDelim) const {
    std::ostringstream sstrm;

    for (const swlexp::Swarmlist::Entry& e : m_data) {
        sstrm << '(' <<
                 std::to_string(e.getRobotId())        << elemDelim <<
                 std::to_string(e.getLamport())        << elemDelim <<
                 std::to_string(e.getTimeToInactive()) << elemDelim <<
                 ')' << entryDelim;
    }

    return sstrm.str();
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::_set(const swlexp::Swarmlist::Entry& entry) {
    try {
        // Get existing entry.
        argos::UInt32 idx = m_idToIndex.at(entry.getRobotId());

        // Didn't throw an exception ; the entry already existed.
        m_data[idx] = entry;
        m_idToIndex[entry.getRobotId()] = idx;
    }
    catch (std::out_of_range& e) {
        // Throwed an exception ; entry doesn't exist yet.
        m_data.push_back(entry);
        m_idToIndex[entry.getRobotId()] = (argos::UInt32)m_data.size() - 1;
    }
}

// ==============================
// =      SWARMLIST ENTRY       =
// ==============================

swlexp::Swarmlist::Entry::Entry(RobotId robot,
                                argos::UInt8 swarmMask,
                                swlexp::Lamport8 lamport)
    : m_robot(robot)
    , m_swarmMask(swarmMask)
    , m_lamport(lamport)
{
    resetTimer();
}