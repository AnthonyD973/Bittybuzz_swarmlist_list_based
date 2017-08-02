#include <algorithm>
#include <vector>
#include <unordered_map>
#include <cinttypes>
#include <sstream>
#include <random> // std::default_random_engine
#include <chrono> // std::chrono
#include <argos3/core/utility/math/rng.h> // argos::CRandom

#include "Swarmlist.h"

namespace swlexp {
    argos::UInt32       Swarmlist::Entry::c_ticksToInactive = 1;

    bool                Swarmlist::c_entriesShouldBecomeInactive;
    argos::UInt64       Swarmlist::c_totalNumActive = 0;
    argos::UInt16       Swarmlist::c_numEntriesPerSwarmMsg;
    const argos::UInt16 Swarmlist::c_SWARM_ENTRY_SIZE = sizeof(RobotId) + sizeof(argos::UInt8) + sizeof(Lamport32);
    const argos::UInt8  Swarmlist::c_ROBOT_ID_POS     = 0;
    const argos::UInt8  Swarmlist::c_SWARM_MASK_POS   = 0 + sizeof(RobotId);
    const argos::UInt8  Swarmlist::c_LAMPORT_POS      = 0 + sizeof(RobotId) + sizeof(argos::UInt8);
    }

/****************************************/
/****************************************/

swlexp::Swarmlist::Swarmlist(Messenger* msn)
    : m_msn(msn)
    , m_swMsgCb(this)
{
    m_numActive = 0;
    m_msn->registerCallback(Messenger::MSG_TYPE_SWARM, m_swMsgCb);
}

/****************************************/
/****************************************/

swlexp::Swarmlist::~Swarmlist() {
    c_totalNumActive -= m_numActive;
    m_msn->removeCallback(Messenger::MSG_TYPE_SWARM, m_swMsgCb);
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::init(RobotId id) {
    m_id = id;
    reset();
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::reset() {
    m_data.clear();
    m_idToIndex.clear();
    m_data.shrink_to_fit();

    // Reinitialize stuff
    c_totalNumActive -= m_numActive;
    m_numActive       = 0;
    m_next            = 0;
    m_numMsgsTx       = 0;
    m_numMsgsRx       = 0;
    m_highestTti      = 0;
    m_ttiSum          = 0;
    m_numUpdates      = 0;

    c_numEntriesPerSwarmMsg =
        (getPacketSize() - 1) /
        (sizeof(RobotId) + sizeof(argos::UInt8) + sizeof(Lamport32));

    _update(m_id, 0, 0);
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::controlStep() {
    _sendSwarmChunk();
    _tick();
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::forceConsensus(const std::vector<RobotId>& existingRobots) {
    static argos::CRandom::CRNG* argosRng = argos::CRandom::CreateRNG("argos");
    reset();

    // Make a vector containing the id of all robots.
    std::vector<RobotId> robots = existingRobots;
    unsigned int seed = argosRng->Uniform(argos::CRange<argos::UInt32>(0, UINT32_MAX));
    std::default_random_engine rng(seed);
    std::shuffle(robots.begin(), robots.end(), rng);

    for (RobotId id : robots) {
        if (id != m_id) {
            _update(id, 0, 0);
        }
    }
    m_next = argosRng->Uniform(argos::CRange<argos::UInt32>(0, m_data.size()));
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::setSwarmMask(argos::UInt8 swarmMask) {
    m_data[m_idToIndex[m_id]].setSwarmMask(swarmMask);
}

/****************************************/
/****************************************/

std::string swlexp::Swarmlist::serializeData(char elemDelim, char entryDelim) const {
    std::ostringstream sstrm;

    for (const swlexp::Swarmlist::Entry& e : m_data) {
        sstrm << '(' <<
                 std::to_string(e.getRobotId())        << elemDelim <<
                 std::to_string(e.getLamport())        << elemDelim <<
                 std::to_string(e.getTimeToInactive()) <<
                 ')' << entryDelim;
    }

    return sstrm.str();
}

/****************************************/
/****************************************/
 
const swlexp::Swarmlist::Entry& swlexp::Swarmlist::_get(RobotId robot) const {
    return m_data[m_idToIndex.at(robot)];
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::_update(RobotId robot,
                                argos::UInt8 swarmMask,
                                Lamport32 lamport) {
    // Does the entry already exist?
    const swlexp::Swarmlist::Entry* existingEntry;
    bool existed;
    try {
        existingEntry = &_get(robot);
        existed = true;
    }
    catch (std::out_of_range& err) {
        existed = false;
    }

    bool shouldUpdate;
    if (existed) {
        // Yes.
        Lamport32 oldLamport = existingEntry->getLamport();
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
        // Change the lowest TTI for the statistical analysis.
        if (existed && robot != m_id) {
            const argos::UInt32 TTI = -(existingEntry->getTimeToInactive());
            m_ttiSum += TTI;
            ++m_numUpdates;
            if (TTI > m_highestTti) {
                m_highestTti = TTI;
            }
        }
        // Create a new entry (which also resets the timer)
        swlexp::Swarmlist::Entry newEntry =
            swlexp::Swarmlist::Entry(robot, swarmMask, lamport);
        _set(newEntry);
    }
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::_tick() {
    if (c_entriesShouldBecomeInactive) {
        for (argos::UInt32 i = 0; i < m_data.size(); ++i) {
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

/****************************************/
/****************************************/

argos::CByteArray swlexp::Swarmlist::_makeNextMessage() {
    argos::CByteArray swarmMsg(getPacketSize());
    swarmMsg[0] = Messenger::MSG_TYPE_SWARM;

    // Send some entries
    for (argos::UInt16 i = 0; i < c_numEntriesPerSwarmMsg; ++i) {
        Entry entry = _getNext();

        // Don't send the info of inactive robots.
        // At worst, only the robot's own data is active,
        // so we don't risk falling in infinite loops.
        while (!entry.isActive(m_id)) {
            _next();
            entry = _getNext();
        }

        // Go to next robot. If we don't have enough entries, we'll
        // send the same entry several times, but that's OK, since
        // in most practical cases we have the information of many robots.
        // Besides, if we have few entries then cost of handling the same
        // entry several times is very low.
        _next();
        writeInPacket(swarmMsg, entry, i);
    }
    return std::move(swarmMsg);
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::_sendSwarmChunk() {

    // Send several swarm messages
    m_numMsgsTx += 1;
    // Send a swarm message
    m_msn->sendMsgTx(std::move(_makeNextMessage()));
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::_next() {
    ++m_next;
    if (m_next >= m_data.size()) {
        m_next = 0;
    }
}

/****************************************/
/****************************************/

swlexp::Swarmlist::Entry swlexp::Swarmlist::_getNext() {
    Entry* e = &m_data[m_next];
    // Increment our own Lamport clock so that others are aware
    // that we still exist.
    if (e->getRobotId() == m_id)
        e->incrementLamport();
    return *e;
}

// ==============================
// =      SWARMLIST ENTRY       =
// ==============================

swlexp::Swarmlist::Entry::Entry(RobotId robot,
                                argos::UInt8 swarmMask,
                                Lamport32 lamport)
    : m_robot(robot)
    , m_swarmMask(swarmMask)
    , m_lamport(lamport)
{
    resetTimer();
}

// ==============================
// =     SWARM_MSG_CALLBACK     =
// ==============================

void swlexp::Swarmlist::SwarmMsgCallback::operator()(
    const argos::CCI_RangeAndBearingSensor::SPacket& packet)
{
    const argos::UInt8* SWARM_MSG = packet.Data.ToCArray();
    for (argos::UInt8 j = 0; j < c_numEntriesPerSwarmMsg; ++j) {
        RobotId robot = *(const RobotId*)&SWARM_MSG[1+c_SWARM_ENTRY_SIZE*j+c_ROBOT_ID_POS];
        // We have the most updated info about ourself ;
        // don't update our info.
        if (robot != m_swarmlist->m_id) {
            argos::UInt8 swarmMask  = SWARM_MSG[1+c_SWARM_ENTRY_SIZE*j+c_SWARM_MASK_POS];
            Lamport32 lamport = *(Lamport32*)&SWARM_MSG[1+c_SWARM_ENTRY_SIZE*j+c_LAMPORT_POS];
            m_swarmlist->_update(robot, swarmMask, lamport);
        }
    }
    m_swarmlist->m_numMsgsRx += 1;
}

/****************************************/
/****************************************/

// ==============================
// =      GLOBAL FUNCTIONS      =
// ==============================

void swlexp::writeInPacket(argos::CByteArray& packet,
                           const swlexp::Swarmlist::Entry& entry,
                           argos::UInt16 idx) {
    argos::UInt8* data = packet.ToCArray();
    const argos::UInt16 ENTRY_POS = 1 + Swarmlist::c_SWARM_ENTRY_SIZE * idx;
    *(RobotId*)     &data[ENTRY_POS+Swarmlist::c_ROBOT_ID_POS]   = entry.getRobotId();
    *(argos::UInt8*)&data[ENTRY_POS+Swarmlist::c_SWARM_MASK_POS] = entry.getSwarmMask();
    *(Lamport32*)   &data[ENTRY_POS+Swarmlist::c_LAMPORT_POS]    = entry.getLamport();
}
