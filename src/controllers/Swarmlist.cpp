#include <algorithm>
#include <vector>
#include <unordered_map>
#include <cinttypes>
#include <sstream>

#include "Swarmlist.h"

namespace swlexp {
    bool                Swarmlist::c_entriesShouldBecomeInactive;
    argos::UInt64       Swarmlist::c_totalNumActive = 0;
    argos::UInt16       Swarmlist::c_numEntriesPerSwarmMsg;
    const argos::UInt16 Swarmlist::c_SWARM_ENTRY_SIZE = sizeof(RobotId) + sizeof(argos::UInt8) + sizeof(Lamport8);
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
    m_msn->registerCallback(Messenger::MSG_TYPE_SWARM, m_swMsgCb);
}

/****************************************/
/****************************************/

swlexp::Swarmlist::~Swarmlist() {
    m_msn->removeCallback(Messenger::MSG_TYPE_SWARM, m_swMsgCb);
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::setSwarmMask(argos::UInt8 swarmMask) {
    m_data[m_idToIndex.at(m_id)].setSwarmMask(swarmMask);
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
        m_newData.push_back(swlexp::Swarmlist::Entry(robot, swarmMask, lamport));
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

void swlexp::Swarmlist::_next() {
    if (m_newData.empty()) {
        ++m_next;
        if (m_next >= m_data.size())
            m_next = 0;
    }
    else {
        ++m_newNext;
        if (m_newNext >= m_newData.size())
            m_newNext = 0;
    }
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::_sendSwarmChunk() {

    // Send several swarm messages
    argos::UInt32 numMsgsRequired = (m_numActive / c_numEntriesPerSwarmMsg + 1);

    const argos::UInt8 NUM_MSGS_TX =
        (numMsgsRequired >= SWARM_CHUNK_AMOUNT) ?
        (SWARM_CHUNK_AMOUNT) :
        (numMsgsRequired);

    m_numMsgsTx += NUM_MSGS_TX;

    for (argos::UInt8 i = 0; i < NUM_MSGS_TX; ++i) {
        // Send a swarm message
        argos::CByteArray msgTx(getPacketSize());
        msgTx[0] = Messenger::MSG_TYPE_SWARM;
        for (argos::UInt8 j = 0; j < c_numEntriesPerSwarmMsg; ++j) {
            swlexp::Swarmlist::Entry entry = _getNext(m_id);

            // Don't send the info of inactive robots.
            // At worst, only the robot's own data is active,
            // so we don't risk falling in infinite loops.
            while (!entry.isActive(m_id)) {
                _next();
                entry = _getNext(m_id);
            }

            // Append the next entry's data
            Messenger::MsgType* msgTxData = (Messenger::MsgType*)(msgTx.ToCArray());
            *(RobotId*)     &msgTxData[1+c_SWARM_ENTRY_SIZE*j+c_ROBOT_ID_POS]   = entry.getRobotId();
            *(argos::UInt8*)&msgTxData[1+c_SWARM_ENTRY_SIZE*j+c_SWARM_MASK_POS] = entry.getSwarmMask();
            *(Lamport8*)    &msgTxData[1+c_SWARM_ENTRY_SIZE*j+c_LAMPORT_POS]    = entry.getLamport();

            // Go to next robot (if we don't have enough entries, we'll
            // send the same entry several times, but that's OK, since
            // in most practical cases we have the information of many robots).
            // Besides, if we have few entries then cost of handling the same
            // entry several times is very low.
            _next();
        }
        m_msn->sendMsgTx(msgTx);
    }
}

/****************************************/
/****************************************/

void swlexp::Swarmlist::_init() {
    m_numActive          = 0;
    m_next               = 0;
    m_stepsTillChunk     = STEPS_PER_CHUNK - 1;
    m_stepsTillTick      = STEPS_PER_TICK  - 1;
    m_numMsgsTx          = 0;
    m_numMsgsRx          = 0;

    c_numEntriesPerSwarmMsg =
        (getPacketSize() - 1) /
        (sizeof(RobotId) + sizeof(argos::UInt8) + sizeof(Lamport8));
}

/****************************************/
/****************************************/

swlexp::Swarmlist::Entry swlexp::Swarmlist::_getNext(RobotId id) {
    Entry* e;
    if (m_newData.empty()) {
        e = &m_data[m_next];
    }
    else {
        e = &m_newData[m_newNext];
    }
    // Increment our own Lamport clock so that others are aware
    // that we still exist.
    if (e->getRobotId() == id)
        e->incrementLamport();
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
