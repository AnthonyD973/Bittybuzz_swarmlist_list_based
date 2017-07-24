#include <argos3/core/simulator/simulator.h>
#include <sstream>
#include <numeric> // std::accumulate
#include <cstring> // std::memcpy

#include "FootbotController.h"

namespace swlexp {
    std::unordered_set<FootbotController*> FootbotController::c_controllers;
    argos::UInt16       FootbotController::c_packetSize;
    argos::UInt16       FootbotController::c_numEntriesPerSwarmMsg;
    const argos::UInt16 FootbotController::c_SWARM_ENTRY_SIZE = sizeof(RobotId) + sizeof(argos::UInt8) + sizeof(Lamport8);
    const argos::UInt8  FootbotController::c_ROBOT_ID_POS     = 0;
    const argos::UInt8  FootbotController::c_SWARM_MASK_POS   = 0 + sizeof(RobotId);
    const argos::UInt8  FootbotController::c_LAMPORT_POS      = 0 + sizeof(RobotId) + sizeof(argos::UInt8);
}

/****************************************/
/****************************************/

swlexp::FootbotController::FootbotController()
{
    // Add this process to the list of processes.
    c_controllers.insert(this);
}

/****************************************/
/****************************************/

swlexp::FootbotController::~FootbotController() {
    c_controllers.erase(this);
}

/****************************************/
/****************************************/

void swlexp::FootbotController::Init(argos::TConfigurationNode& t_node) {
    std::string idStr = GetId().substr(std::string("fb").size());
    m_id = std::stoi(idStr);
    m_swarmlist.setOwnerId(m_id);

    // Initialize stuff
    m_timeAtLastLog      = -1;
    m_numMsgsTxSinceLog  = 0;
    m_numMsgsRxSinceLog  = 0;
    m_numMsgsTx          = 0;
    m_numMsgsRx          = 0;
    m_stepsTillTick      = STEPS_PER_TICK;
    m_stepsTillNextChunk = SWARM_CHUNK_DELAY;
    m_localSwarmMask     = 0x01;
    m_numSends           = 0;

    // Get actuators and sensors
    m_leds    = GetActuator<argos::CCI_LEDsActuator           >("leds"             );
    m_rabAct  = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");

    m_rabSens = GetSensor  <argos::CCI_RangeAndBearingSensor  >("range_and_bearing");

    // Configure swarm message stuff.
    bool entriesShouldBecomeInactive;
    argos::GetNodeAttribute(t_node, "entries_should_become_inactive", entriesShouldBecomeInactive);
    swlexp::Swarmlist::setEntriesShouldBecomeInactive(entriesShouldBecomeInactive);

    argos::GetNodeAttribute(t_node, "packet_size", c_packetSize);
    c_numEntriesPerSwarmMsg =
        (c_packetSize - 1) /
        (sizeof(RobotId) + sizeof(argos::UInt8) + sizeof(Lamport8));
    m_swarmlist.update(m_id, m_localSwarmMask, 0);
}

/****************************************/
/****************************************/

void swlexp::FootbotController::ControlStep() {
    static argos::CSpace& space =
        argos::CSimulator::GetInstance().GetSpace();

    _processMsgsRx();

    // Do not send messages when you don't need to.
    if (m_numSends > 0) {
        --m_numSends;
    }
    if (_isRabFree()) {
        m_rabAct->ClearData();
    }

    if (m_stepsTillNextChunk > 0) {
        --m_stepsTillNextChunk;
    }
    if (m_stepsTillNextChunk == 0) {
        m_stepsTillNextChunk = SWARM_CHUNK_DELAY;
        _sendNextSwarmChunk();
    }

    // switch(m_swarmlist.getNumActive() % 8) {
    //     case 0: {
    //         m_leds->SetAllColors(argos::CColor::BLACK);
    //         break;
    //     }
    //     case 1: {
    //         m_leds->SetAllColors(argos::CColor::RED);
    //         break;
    //     }
    //     case 2: {
    //         m_leds->SetAllColors(argos::CColor::YELLOW);
    //         break;
    //     }
    //     case 3: {
    //         m_leds->SetAllColors(argos::CColor::GREEN);
    //         break;
    //     }
    //     case 4: {
    //         m_leds->SetAllColors(argos::CColor::CYAN);
    //         break;
    //     }
    //     case 5: {
    //         m_leds->SetAllColors(argos::CColor::BLUE);
    //         break;
    //     }
    //     case 6: {
    //         m_leds->SetAllColors(argos::CColor::MAGENTA);
    //         break;
    //     }
    //     case 7: {
    //         m_leds->SetAllColors(argos::CColor::WHITE);
    //         break;
    //     }
    //     default: ;
    // }

    --m_stepsTillTick;
    if (m_stepsTillTick == 0) {
        m_stepsTillTick = STEPS_PER_TICK;
        m_swarmlist.tick();
    }

    const argos::UInt32 TIME = space.GetSimulationClock();
    if (m_id == 0 && TIME % 1000 == 0) {
        std::cout << "Robot #"     << m_id << "\t; " <<
                     "Timesteps: " << TIME << "\t; " <<
                     "Swarmlist size: " << m_swarmlist.getSize() << "\n";
    }
}

/****************************************/
/****************************************/

void swlexp::FootbotController::Destroy() {

}

/****************************************/
/****************************************/

std::string swlexp::FootbotController::getCsvStatusLog() {
    std::ostringstream logData;

    const argos::UInt32 TIME = argos::CSimulator::GetInstance().
                               GetSpace().GetSimulationClock();

    argos::Real bwTx;
    argos::Real bwRx;

    if (TIME != m_timeAtLastLog) {
        bwTx = (argos::Real)m_numMsgsTxSinceLog / (TIME - m_timeAtLastLog);
        bwRx = (argos::Real)m_numMsgsRxSinceLog / (TIME - m_timeAtLastLog);
    }
    else {
        bwTx = 0;
        bwRx = 0;
    }

    logData << GetId()                    << c_CSV_DELIM <<
               TIME                       << c_CSV_DELIM <<
               m_numMsgsTxSinceLog        << c_CSV_DELIM <<
               bwTx                       << c_CSV_DELIM <<
               m_numMsgsRxSinceLog        << c_CSV_DELIM <<
               bwRx                       << c_CSV_DELIM <<
               m_swarmlist.getSize()      << c_CSV_DELIM <<
               m_swarmlist.getNumActive() << c_CSV_DELIM <<
               '"' << m_swarmlist.serializeData(c_CSV_DELIM, ';') << "\"\n";

    m_numMsgsTxSinceLog = 0;
    m_numMsgsRxSinceLog = 0;
    m_timeAtLastLog = TIME;

    return std::move(logData.str());
}

/****************************************/
/****************************************/

void swlexp::FootbotController::_sendNextSwarmChunk() {
    // Send several swarm messages
    const argos::UInt8 NUM_MSGS_TX =
        (m_swarmlist.getNumActive() / c_numEntriesPerSwarmMsg + 1 >= SWARM_CHUNK_AMOUNT) ?
        (SWARM_CHUNK_AMOUNT) :
        (m_swarmlist.getNumActive() / c_numEntriesPerSwarmMsg + 1);

    m_numMsgsTxSinceLog += NUM_MSGS_TX;
    m_numMsgsTx         += NUM_MSGS_TX;

    for (uint8_t i = 0; i < NUM_MSGS_TX; ++i) {
        // Send a swarm message
        argos::CByteArray msgTx(c_packetSize);
        msgTx[0] = 1;
        for (uint8_t j = 0; j < c_numEntriesPerSwarmMsg; ++j) {
            // Increment our own Lamport clock so that others are aware
            // that we still exist.
            swlexp::Swarmlist::Entry entry = m_swarmlist.getNext(m_id);

            // Don't send the info of inactive robots.
            // At worst, only the robot's own data is active,
            // so we don't risk falling in infinite loops.
            while (!entry.isActive(m_id)) {
                m_swarmlist.next();
                entry = m_swarmlist.getNext(m_id);
            }

            // Append the next entry's data
            argos::UInt8* msgTxData = msgTx.ToCArray();
            *(RobotId*)     &msgTxData[c_SWARM_ENTRY_SIZE*j+1+c_ROBOT_ID_POS]   = entry.getRobotId();
            *(argos::UInt8*)&msgTxData[c_SWARM_ENTRY_SIZE*j+1+c_SWARM_MASK_POS] = entry.getSwarmMask();
            *(Lamport8*)    &msgTxData[c_SWARM_ENTRY_SIZE*j+1+c_LAMPORT_POS]    = entry.getLamport();

            // Go to next robot (if we only have one or two robots, we'll
            // send the same robot info several times, but that's OK, since
            // in most practical cases we have the information of many robots).
            m_swarmlist.next();
        }
        _sendMsgTx(msgTx);
    }
}

/****************************************/
/****************************************/

using argos::CARGoSException; // Required because of the THROW_ARGOSEXCEPTION macro

void swlexp::FootbotController::_processMsgsRx() {
    const argos::CCI_RangeAndBearingSensor::TReadings& msgsRx = m_rabSens->GetReadings();

    for (argos::CCI_RangeAndBearingSensor::SPacket msgRx : msgsRx) {
        argos::UInt8 msgType = msgRx.Data[0];
        switch (msgType) {
            case 1: {
                m_numMsgsRxSinceLog += 1;
                m_numMsgsRx         += 1;
                for (uint8_t j = 0; j < c_numEntriesPerSwarmMsg; ++j) {
                    RobotId robot = *(RobotId*)&msgRx.Data[c_SWARM_ENTRY_SIZE*j+1+c_ROBOT_ID_POS];
                    // We have the most updated info about ourself ;
                    // don't update our info.
                    if (robot != m_id) {
                        argos::UInt8 swarmMask  = msgRx.Data[c_SWARM_ENTRY_SIZE*j+1+c_SWARM_MASK_POS];
                        Lamport8 lamport = msgRx.Data[c_SWARM_ENTRY_SIZE*j+1+c_LAMPORT_POS];
                        m_swarmlist.update(robot, swarmMask, lamport);
                    }
                }
                break;
            }
            case 0: {
                // This type of packet is ignored.
                break;
            }
            default: THROW_ARGOSEXCEPTION("Unknown message type: " << msgType);
        }
    }

}

/****************************************/
/****************************************/

argos::UInt64 swlexp::FootbotController::getTotalNumMessagesTx() {
    return std::accumulate(c_controllers.begin(), c_controllers.end(), 0, _msgTxElemSum);
}

/****************************************/
/****************************************/

argos::UInt64 swlexp::FootbotController::getTotalNumMessagesRx() {
    return std::accumulate(c_controllers.begin(), c_controllers.end(), 0, _msgTxElemSum);
}

/****************************************/
/****************************************/

void swlexp::FootbotController::writeStatusLogHeader(std::ostream& o) {
    static const std::string STATUS_LOG_HEADER = std::string() +
        "ID"                                   + c_CSV_DELIM +
        "Time (timesteps)"                     + c_CSV_DELIM +
        "Number of messages sent"              + c_CSV_DELIM +
        "Avg. sent bandwidth (B/timestep)"     + c_CSV_DELIM +
        "Number of messages received"          + c_CSV_DELIM +
        "Avg. received bandwidth (B/timestep)" + c_CSV_DELIM +
        "Swarmlist size"                       + c_CSV_DELIM +
        "Swarmlist number of active entries"   + c_CSV_DELIM +
        "\"Swarmlist data (robot ID,lamport,ticks to inactive)\"\n";
    o << STATUS_LOG_HEADER;
}

/****************************************/
/****************************************/

bool swlexp::FootbotController::isConsensusReached() {
    const argos::UInt32 NUM_CONTROLLERS = getNumControllers();
    return swlexp::Swarmlist::getTotalNumActive() == NUM_CONTROLLERS * NUM_CONTROLLERS;
}

/****************************************/
/****************************************/

using swlexp::FootbotController;
REGISTER_CONTROLLER(FootbotController, "footbot_controller")