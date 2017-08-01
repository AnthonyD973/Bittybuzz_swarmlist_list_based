#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <sstream>
#include <numeric> // std::accumulate
#include <cstring> // std::memcpy

#include "FootbotController.h"

namespace swlexp {
    std::unordered_set<FootbotController*> FootbotController::c_controllers;
    argos::Real   FootbotController::c_packetDropProb;
}

/****************************************/
/****************************************/

argos::Real swlexp::getPacketDropProb() {
    return swlexp::FootbotController::getPacketDropProb();
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

using argos::CARGoSException; // Required because of the THROW_ARGOSEXCEPTION macro.

void swlexp::FootbotController::Init(argos::TConfigurationNode& t_node) {
    // Set numeric ID.
    std::string idStr = GetId().substr(std::string("fb").size());
    m_id = std::stoi(idStr);

    // Get packet drop probability.
    argos::TConfigurationNode& controllers        = argos::GetNode(argos::CSimulator::GetInstance().GetConfigurationRoot(), "controllers");
    argos::TConfigurationNode& footbot_controller = argos::GetNode(controllers,        "footbot_controller");
    argos::TConfigurationNode& sensors            = argos::GetNode(footbot_controller, "sensors");
    argos::TConfigurationNode& rab                = argos::GetNode(sensors,            "range_and_bearing");
    argos::GetNodeAttribute(rab, "packet_drop_prob", c_packetDropProb);

    // Get actuators and sensors and build the messenger
    m_leds    = GetActuator<argos::CCI_LEDsActuator>("leds");
    argos::CCI_RangeAndBearingActuator* rabAct  = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");
    argos::CCI_RangeAndBearingSensor*   rabSens = GetSensor  <argos::CCI_RangeAndBearingSensor  >("range_and_bearing");
    m_msn.init(rabAct, rabSens);

    // Init the swarmlist.
    m_swarmlist.init(m_id);
    m_localSwarmMask = 0x01;
    m_swarmlist.setSwarmMask(m_localSwarmMask);
}

/****************************************/
/****************************************/

void swlexp::FootbotController::ControlStep() {
    static argos::CSpace& space =
        argos::CSimulator::GetInstance().GetSpace();

    m_msn.controlStep();
    m_swarmlist.controlStep();

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
    if (m_id == c_controllers.size() - 1 && TIME % 1000 == 0) {
        std::cout << "Robot #"     << m_id << "\t; " <<
                     "Timesteps: " << TIME << "\t; " <<
                     "Swarmlist num active: " << m_swarmlist.getNumActive() << "\n";
    }
}

/****************************************/
/****************************************/

void swlexp::FootbotController::Destroy() {

}

/****************************************/
/****************************************/

std::string swlexp::FootbotController::getCsvStatusLog(bool sideEffect) {
    std::ostringstream logData;

    const argos::UInt32 TIME = argos::CSimulator::GetInstance().
                               GetSpace().GetSimulationClock();

    argos::UInt64 NUM_MSGS_TX = m_swarmlist.getNumMsgsTx();
    argos::UInt64 NUM_MSGS_RX = m_swarmlist.getNumMsgsRx();
    argos::UInt64 NUM_MSGS_TX_SINCE_LOG = NUM_MSGS_TX - m_numMsgsTxAtLastLog;
    argos::UInt64 NUM_MSGS_RX_SINCE_LOG = NUM_MSGS_RX - m_numMsgsTxAtLastLog;
    argos::Real bwTx;
    argos::Real bwRx;

    if (TIME != m_timeAtLastLog) {
        bwTx = (argos::Real)(NUM_MSGS_TX_SINCE_LOG) / (TIME - m_timeAtLastLog);
        bwRx = (argos::Real)(NUM_MSGS_RX_SINCE_LOG) / (TIME - m_timeAtLastLog);
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
               m_swarmlist.getNumActive() << "\n";

    if (sideEffect) {
        m_numMsgsTxAtLastLog = NUM_MSGS_TX;
        m_numMsgsRxAtLastLog = NUM_MSGS_RX;
        m_timeAtLastLog      = TIME;
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

void swlexp::FootbotController::forceConsensus() {
    argos::CSpace::TMapPerType& entities =
        argos::CSimulator::GetInstance().
            GetSpace().GetEntitiesByType("foot-bot");

    std::vector<RobotId> existingRobots;

    std::transform(
        entities.begin(),
        entities.end(),
        back_inserter(existingRobots),
        [](std::pair<std::string, argos::CAny> elem) {
            std::string idStr =
                argos::any_cast<argos::CFootBotEntity*>(elem.second)->GetId();
            return std::stoi(idStr.substr(std::string("fb").size()));
        });

    // Force each swarmlist's consensus.
    for (FootbotController* ctrl : c_controllers) {
        ctrl->m_swarmlist.forceConsensus(existingRobots);
    }
}

/****************************************/
/****************************************/

void swlexp::FootbotController::writeStatusLogHeader(std::ostream& o) {
    static const std::string STATUS_LOG_HEADER = std::string() +
        "ID"                       + c_CSV_DELIM +
        "Time (ts)"                + c_CSV_DELIM +
        "Num msgs tx"              + c_CSV_DELIM +
        "Avg. tx bandwidth (B/ts)" + c_CSV_DELIM +
        "Num msgs rx"              + c_CSV_DELIM +
        "Avg. rx bandwidth (B/ts)" + c_CSV_DELIM +
        "Swl size"                 + c_CSV_DELIM +
        "Swl num active\n";
    o << STATUS_LOG_HEADER;
}

/****************************************/
/****************************************/

void swlexp::FootbotController::writeTtiData(std::ostream& o) {
    argos::UInt64 totalTtiRequired = 0;
    argos::UInt32 maxTtiRequired = 0;
    argos::Real sumAvgTtis = 0.0;
    for (FootbotController* controller : c_controllers) {
        const argos::UInt32 TTI_REQUIRED = controller->m_swarmlist.getHighestTti();
        if (TTI_REQUIRED > maxTtiRequired) {
            maxTtiRequired = TTI_REQUIRED;
        }
        totalTtiRequired += TTI_REQUIRED;
        sumAvgTtis += controller->m_swarmlist.getAverageTti();
        o << TTI_REQUIRED << c_CSV_DELIM;
    }
    o << '\n';
    o << (sumAvgTtis / c_controllers.size()) << '\n';
    o << ((argos::Real)totalTtiRequired / c_controllers.size()) << '\n';
    o << maxTtiRequired << '\n';
}

/****************************************/
/****************************************/

bool swlexp::FootbotController::isConsensusReached() {
    const argos::UInt32 NUM_CONTROLLERS = getNumControllers();
    return Swarmlist::getTotalNumActive() == NUM_CONTROLLERS * NUM_CONTROLLERS;
}

/****************************************/
/****************************************/

using swlexp::FootbotController;
REGISTER_CONTROLLER(FootbotController, "footbot_controller")