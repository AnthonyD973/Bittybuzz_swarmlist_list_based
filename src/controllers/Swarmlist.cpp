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
    bool                Swarmlist::c_entriesShouldBecomeInactive;
    argos::UInt64       Swarmlist::c_totalNumActive = 0;
    argos::UInt16       Swarmlist::c_numEntriesPerSwarmMsg;
    const argos::UInt16 Swarmlist::c_SWARM_ENTRY_SIZE = sizeof(RobotId) + sizeof(argos::UInt8) + sizeof(Lamport8);
    const argos::UInt8  Swarmlist::c_ROBOT_ID_POS     = 0;
    const argos::UInt8  Swarmlist::c_SWARM_MASK_POS   = 0 + sizeof(RobotId);
    const argos::UInt8  Swarmlist::c_LAMPORT_POS      = 0 + sizeof(RobotId) + sizeof(argos::UInt8);
    argos::Real         Swarmlist::c_targetBroadcastSuccessProb = 55.0;
}

/****************************************/
/****************************************/

swlexp::Swarmlist::Swarmlist(Messenger* msn)
    : m_msn(msn)
    , m_swMsgCb(this)
    , m_shouldRebroadcast(false)
    , m_msgsTillNoNew(0)
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
    m_newData.clear();
    m_idToIndex.clear();
    m_data.shrink_to_fit();
    m_newData.shrink_to_fit();

    // Reinitialize stuff
    c_totalNumActive    -= m_numActive;
    m_numActive          = 0;
    m_next               = 0;
    m_newNext            = 0;
    m_stepsTillChunk     = STEPS_PER_CHUNK - 1;
    m_stepsTillTick      = STEPS_PER_TICK  - 1;
    m_numMsgsTx          = 0;
    m_numMsgsRx          = 0;


    c_numEntriesPerSwarmMsg =
        (getPacketSize() - 1) /
        (sizeof(RobotId) + sizeof(argos::UInt8) + sizeof(Lamport8));
    
    m_numRebroadcasts = 0;
    argos::Real targetBroadcastFailProb = (100.0 - c_targetBroadcastSuccessProb) / 100.0;
    argos::Real currBroadcastFailProb = 1.0;
    while (currBroadcastFailProb > targetBroadcastFailProb) {
        ++m_numRebroadcasts;
        currBroadcastFailProb *= getPacketDropProb();
    }

    _update(m_id, 0, 0);
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::controlStep() {
    if (m_stepsTillChunk == 0) {
        m_stepsTillChunk = STEPS_PER_CHUNK;
        _sendSwarmChunk();
    }
    --m_stepsTillChunk;

    if (m_stepsTillTick == 0) {
        m_stepsTillTick = STEPS_PER_TICK;
        _tick();
    }
    --m_stepsTillTick;
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::forceConsensus(const std::vector<RobotId>& existingRobots) {
    static argos::CRandom::CRNG* argosRng = argos::CRandom::CreateRNG("argos");
    reset();

    std::vector<RobotId> robots = existingRobots;
    unsigned int seed = argosRng->Uniform(argos::CRange<argos::UInt32>(0, UINT32_MAX));
    std::default_random_engine rng(seed);
    std::shuffle(robots.begin(), robots.end(), rng);

    for (RobotId id : robots) {
        if (id != m_id) {
            Lamport8 lamport = rng();
            _update(id, 0, lamport);
        }
    }
    m_newData.clear();
    m_newData.shrink_to_fit();
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
                               Lamport8 lamport) {
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

        // Since it's a new entry, also add it in the "new entry" vector.
        if (m_newData.empty()) {
            m_newNext = 0;
        }
        swlexp::Swarmlist::NewData nd = {
            .entryIdx = (argos::UInt32)m_data.size(),
            .numRebroadcastsLeft = m_numRebroadcasts
        };
        m_newData.push_back(nd);
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

void swlexp::Swarmlist::_tick() {
    if (c_entriesShouldBecomeInactive) {
        for (argos::UInt8 i = 0; i < m_data.size(); ++i) {
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
    static const argos::UInt8 NUM_MSGS_BETWEEN_NO_NEW = 3;
    argos::CByteArray swarmMsg(getPacketSize());
    swarmMsg[0] = Messenger::MSG_TYPE_SWARM;

    argos::UInt16 msgIdx = 0; // Place of the entry in the message.
    argos::UInt16 numNewEntriesToSend;
    // Determine how many new entries to send.
    // If entries do not become inactive, there is no need to
    // send the info of robots we already broadcast the info of.
    if (!c_entriesShouldBecomeInactive || m_msgsTillNoNew != 0) {
        numNewEntriesToSend =
            (m_newData.size() < c_numEntriesPerSwarmMsg) ?
            (m_newData.size()) :
            (c_numEntriesPerSwarmMsg);
    }
    else {
        m_msgsTillNoNew = NUM_MSGS_BETWEEN_NO_NEW;
        numNewEntriesToSend = 0;
    }
    --m_msgsTillNoNew;

    // Send new entries
    for (argos::UInt16 i = 0; i < numNewEntriesToSend; ++i) {
        Entry entry = _getNewNext();
        _newNext();
        writeInPacket(swarmMsg, entry, msgIdx);
    }

    // Send non-new entries
    for (argos::UInt16 i = 0; i < c_numEntriesPerSwarmMsg - numNewEntriesToSend; ++i) {
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
        writeInPacket(swarmMsg, entry, msgIdx);
    }
    return std::move(swarmMsg);
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::_sendSwarmChunk() {

    // Send several swarm messages
    argos::UInt32 numMsgsRequiredToSendAllEntries = (m_numActive / c_numEntriesPerSwarmMsg + 1);

    const argos::UInt8 NUM_MSGS_TX =
        (numMsgsRequiredToSendAllEntries >= SWARM_CHUNK_AMOUNT) ?
        (SWARM_CHUNK_AMOUNT) :
        (numMsgsRequiredToSendAllEntries);

    m_numMsgsTx += NUM_MSGS_TX;

    for (argos::UInt8 i = 0; i < NUM_MSGS_TX; ++i) {
        // Send a swarm message
        m_msn->sendMsgTx(std::move(_makeNextMessage()));
    }
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

void swlexp::Swarmlist::_newNext() {
    ++m_newNext;
    if (m_newNext >= m_newData.size()) {
        m_newNext = 0;
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

/****************************************/
/****************************************/

swlexp::Swarmlist::Entry swlexp::Swarmlist::_getNewNext() {
    Entry* e = &m_data[m_newData[m_newNext].entryIdx];
    // Increment our own Lamport clock so that others are aware
    // that we still exist.
    if (e->getRobotId() == m_id) {
        e->incrementLamport();
    }

    // Do not consider the entry to be new if we will not rebroadcast it.
    --m_newData[m_newNext].numRebroadcastsLeft;
    if (!m_shouldRebroadcast || m_newData[m_newNext].numRebroadcastsLeft == 0) {
        // Entry no longer new.
        m_newData[m_newNext] = m_newData.back();
        m_newData.pop_back();

        --m_newNext;
    }
    return *e;
}

/****************************************/
/****************************************/

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

/****************************************/
/****************************************/

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
            Lamport8 lamport = SWARM_MSG[1+c_SWARM_ENTRY_SIZE*j+c_LAMPORT_POS];
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
    *(Lamport8*)    &data[ENTRY_POS+Swarmlist::c_LAMPORT_POS]    = entry.getLamport();
}
