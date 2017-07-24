#include <argos3/core/simulator/simulator.h>
#include <sstream>
#include <numeric> // std::accumulate
#include <cstring> // std::memcpy

#include "FootbotController.h"

namespace swlexp {
    std::unordered_set<FootbotController*> FootbotController::c_controllers;
    argos::UInt16       FootbotController::c_packetSize;
}

/****************************************/
/****************************************/

argos::UInt16 swlexp::getPacketSize() {
    return swlexp::FootbotController::getPacketSize();
}

/****************************************/
/****************************************/

swlexp::FootbotController::FootbotController()
    : m_swarmlist(&m_msn)
{
    // Add this controller to the list of controller.
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
    m_localSwarmMask = 0x01;

    // Get actuators and sensors and build the messenger
    m_leds    = GetActuator<argos::CCI_LEDsActuator>("leds");
    argos::CCI_RangeAndBearingActuator* rabAct  = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");
    argos::CCI_RangeAndBearingSensor*   rabSens = GetSensor  <argos::CCI_RangeAndBearingSensor  >("range_and_bearing");
    m_msn.plugComponents(rabAct, rabSens);

    // Configure swarm message stuff.
    bool entriesShouldBecomeInactive;
    argos::GetNodeAttribute(t_node, "entries_should_become_inactive", entriesShouldBecomeInactive);
    swlexp::Swarmlist::setEntriesShouldBecomeInactive(entriesShouldBecomeInactive);

    argos::GetNodeAttribute(t_node, "packet_size", c_packetSize);
    m_swarmlist.setSwarmMask(m_localSwarmMask);
    m_swarmlist._init();
}

/****************************************/
/****************************************/

void swlexp::FootbotController::ControlStep() {
    static argos::CSpace& space =
        argos::CSimulator::GetInstance().GetSpace();

    m_msn._controlStep();
    m_swarmlist._controlStep();

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

std::string swlexp::FootbotController::getCsvStatusLog(bool sideEffect) {
    argos::UInt32 timeAtLastLog = (argos::UInt32)-1;
    argos::UInt64 numMsgsTxAtLastLog = 0;
    argos::UInt64 numMsgsRxAtLastLog = 0;

    std::ostringstream logData;

    const argos::UInt32 TIME = argos::CSimulator::GetInstance().
                               GetSpace().GetSimulationClock();

    argos::UInt64 NUM_MSGS_TX = m_swarmlist.getNumMsgsTx();
    argos::UInt64 NUM_MSGS_RX = m_swarmlist.getNumMsgsRx();
    argos::UInt64 NUM_MSGS_TX_SINCE_LOG = NUM_MSGS_TX - numMsgsTxAtLastLog;
    argos::UInt64 NUM_MSGS_RX_SINCE_LOG = NUM_MSGS_RX - numMsgsRxAtLastLog;
    argos::Real bwTx;
    argos::Real bwRx;

    if (TIME != timeAtLastLog) {
        bwTx = (argos::Real)(NUM_MSGS_TX_SINCE_LOG) / (TIME - timeAtLastLog);
        bwRx = (argos::Real)(NUM_MSGS_RX_SINCE_LOG) / (TIME - timeAtLastLog);
    }
    else {
        bwTx = 0;
        bwRx = 0;
    }

    logData << GetId()                    << c_CSV_DELIM <<
               TIME                       << c_CSV_DELIM <<
               NUM_MSGS_TX_SINCE_LOG      << c_CSV_DELIM <<
               bwTx                       << c_CSV_DELIM <<
               NUM_MSGS_RX_SINCE_LOG      << c_CSV_DELIM <<
               bwRx                       << c_CSV_DELIM <<
               m_swarmlist.getSize()      << c_CSV_DELIM <<
               m_swarmlist.getNumActive() << c_CSV_DELIM <<
               '"' << m_swarmlist.serializeData(c_CSV_DELIM, ';') << "\"\n";

    if (sideEffect) {
        numMsgsTxAtLastLog = NUM_MSGS_TX;
        numMsgsRxAtLastLog = NUM_MSGS_RX;
        timeAtLastLog      = TIME;
    }

    return std::move(logData.str());
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