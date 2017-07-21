#include <algorithm>
#include <cstdio>

#include "ExpLoopFunc.h"

namespace swlexp {
    const argos::Real   ExpLoopFunc::c_RAB_RANGE   = 0.5;
    argos::UInt16       ExpLoopFunc::c_packetSize;
}

/****************************************/
/****************************************/

swlexp::ExpLoopFunc::ExpLoopFunc()
{ }

/****************************************/
/****************************************/

swlexp::ExpLoopFunc::~ExpLoopFunc() {

}

/****************************************/
/****************************************/

using argos::CARGoSException; // Required because of the THROW_ARGOSEXCEPTION macro.

void swlexp::ExpLoopFunc::Init(argos::TConfigurationNode& t_tree) {

    // Get experiment params
    argos::TConfigurationNode controllers        = argos::GetNode(GetSimulator().GetConfigurationRoot(), "controllers");
    argos::TConfigurationNode footbot_controller = argos::GetNode(controllers,        "footbot_controller");
    argos::TConfigurationNode sensors            = argos::GetNode(footbot_controller, "sensors");
    argos::TConfigurationNode rab                = argos::GetNode(sensors,            "range_and_bearing");
    argos::GetNodeAttributeOrDefault(rab, "packet_drop_prob", m_msgDropProb, 0.0);

    argos::TConfigurationNode params             = argos::GetNode(footbot_controller, "params");
    argos::GetNodeAttribute(params, "packet_size", c_packetSize);

    argos::GetNodeAttribute(t_tree, "res", m_expResName);
    argos::GetNodeAttribute(t_tree, "log", m_expLogName);
    argos::GetNodeAttribute(t_tree, "fb_csv", m_expFbCsvName);
    argos::GetNodeAttribute(t_tree, "fb_status_log_delay", m_expStatusLogDelay);
    argos::UInt32 numRobots;
    argos::GetNodeAttribute(t_tree, "num_robots", numRobots);
    std::string topology;
    argos::GetNodeAttribute(t_tree, "topology", topology);
    if (topology == "line") {
        _placeLine(numRobots);
    }
    else {
        THROW_ARGOSEXCEPTION("Unknown topology: " << topology);
    }
    

    // Open files.
    m_expFbCsv.open(m_expFbCsvName, std::ios::app);
    if (m_expFbCsv.fail()) {
        THROW_ARGOSEXCEPTION("Could not open CSV file \"" << m_expFbCsvName << "\".");
    }
    m_expRes.open(m_expResName, std::ios::app);
    if (m_expRes.fail()) {
        THROW_ARGOSEXCEPTION("Could not open CSV file \"" << m_expResName << "\".");
    }
    m_expLog.open(m_expLogName, std::ios::app);
    if (m_expLog.fail()) {
        THROW_ARGOSEXCEPTION("Could not open log file \"" << m_expLogName << "\".");
    }

    // Write experiment params to result and log files.
    m_expRes << '\n' <<
                topology      << c_CSV_DELIM <<
                numRobots     << c_CSV_DELIM <<
                m_msgDropProb;

    m_expLog << "---EXPERIMENT START---\n"
                "Topology: " << topology << "\n" <<
                "Number of robots: " << numRobots << "\n"
                "Drop probability: " << (m_msgDropProb * 100) << "%\n";

    argos::LOG << "--------------------------------------\n"
                  "TOPOLOGY: " << topology << "\n" <<
                  "NUMBER OF ROBOTS: " << numRobots << "\n"
                  "PACKET DROP PROBABILITY: " << (m_msgDropProb * 100) << "%\n"
                  "--------------------------------------\n";
    m_expLog.flush();

    // Write header in the status logs file and perform the first status log.
    swlexp::FootbotController::writeStatusLogHeader(m_expFbCsv);
    swlexp::FootbotController::writeStatusLogs(m_expFbCsv);
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::Destroy() {
    m_expRes.flush();
    m_expLog.flush();
    m_expFbCsv.flush();

    // Other resources are closed when they get out of scope.
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::PostStep() {
    static argos::UInt32 callsTillMetaStuff = m_expStatusLogDelay - 1;

    if (callsTillMetaStuff == 0) {
        callsTillMetaStuff = m_expStatusLogDelay;
        swlexp::FootbotController::writeStatusLogs(m_expFbCsv);
    }

    --callsTillMetaStuff;
}

/****************************************/
/****************************************/

bool swlexp::ExpLoopFunc::IsExperimentFinished() {
    bool isFinished = swlexp::FootbotController::isConsensusReached();
    if (isFinished) {
        _finishExperiment(swlexp::ExpLoopFunc::ExitCode::NORMAL);
    }
    return isFinished;
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::_placeLine(argos::UInt32 numRobots) {
    // static const argos::Real X_SPACING = 0.00, Y_SPACING = 0.06;
    // static const argos::Real X_BASEPOS = 0.00, Y_BASEPOS = 0.00, Z_BASEPOS = 0.00;

    // // Resize arena.
    // const argos::Real X_SIZE = 1.0 + X_BASEPOS + (numRobots) * X_SPACING;
    // const argos::Real Y_SIZE = 1.0 + Y_BASEPOS + (numRobots) * Y_SPACING;
    // const argos::Real Z_SIZE = 1.0;
    // const argos::CVector3 ARENA_SIZE = argos::CVector3(X_SIZE, Y_SIZE, Z_SIZE);
    // const argos::CVector3 ARENA_CENTER = argos::CVector3(
    //     (X_BASEPOS + X_SIZE) / 2,
    //     (Y_BASEPOS + Y_SIZE) / 2,
    //     (Z_BASEPOS + Z_SIZE) / 2);
    //     std::cout.flush();

    // GetSpace().SetArenaSize(ARENA_SIZE);
    // GetSpace().SetArenaCenter(ARENA_CENTER);

    // Place the footbots
    argos::CRange<argos::CVector3> arenaLimits = GetSpace().GetArenaLimits();
    argos::Real maxY = arenaLimits.GetMax().GetY() - 0.10;
    for (argos::UInt32 i = 0; i < numRobots; ++i) {
        argos::CVector3 pos = argos::CVector3(0, maxY - i * 0.40, 0);
        argos::CQuaternion orient = argos::CQuaternion();

        argos::CFootBotEntity* fb = new argos::CFootBotEntity(
            "fb" + std::to_string(i),
            "fb_ctrl",
            pos,
            orient,
            c_RAB_RANGE,
            c_packetSize);
        
        AddEntity(*fb);
    }
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::_finishExperiment(swlexp::ExpLoopFunc::ExitCode exitCode) {
    m_expLog << "---END---\n";

    if (exitCode == NORMAL) {
        const argos::UInt32 NUM_FOOTBOTS =
            swlexp::FootbotController::getNumControllers();
        const argos::UInt64 NUM_MSGS_TX =
            swlexp::FootbotController::getTotalNumMessagesTx();
        const argos::UInt64 NUM_MSGS_RX =
            swlexp::FootbotController::getTotalNumMessagesRx();
        double bwTx = ((double)NUM_MSGS_TX / GetSpace().GetSimulationClock() /
                    NUM_FOOTBOTS) * c_packetSize;
        double bwRx = ((double)NUM_MSGS_RX / GetSpace().GetSimulationClock() /
                    NUM_FOOTBOTS) * c_packetSize;

        m_expRes << c_CSV_DELIM << GetSpace().GetSimulationClock() <<
                    c_CSV_DELIM << NUM_MSGS_TX <<
                    c_CSV_DELIM << NUM_MSGS_RX <<
                    c_CSV_DELIM << bwTx <<
                    c_CSV_DELIM << bwRx;
        m_expRes.flush();
        m_expLog << "Consensus (ts): " << GetSpace().GetSimulationClock() << "\n"
                    "Msgs sent (total): " << NUM_MSGS_TX << "\n"
                    "Msgs received (total): " << NUM_MSGS_RX << "\n"
                    "Avg. sent bandwidth (B/(timestep*foot-bot)): " << bwTx << "\n"
                    "Avg. received bandwidth (B/(timestep*foot-bot)): " << bwRx << "\n"
                    "\n";
        m_expLog.flush();

        swlexp::FootbotController::writeStatusLogs(m_expFbCsv);
        m_expFbCsv.flush();

        argos::LOG << "Experiment finished normally in " << GetSpace().GetSimulationClock() <<
                    " timesteps. See \"" << m_expLogName << "\" for results.\n";
    }
    else {
        remove(m_expFbCsvName.c_str());
        remove(m_expResName.c_str());
        m_expLog      << "[ERROR] " << _exitCodeToString(exitCode) << "\n";
        m_expLog.flush();
    }
}

/****************************************/
/****************************************/

std::string swlexp::ExpLoopFunc::_exitCodeToString(swlexp::ExpLoopFunc::ExitCode exitCode) {
    if (exitCode == NORMAL) {
        return "";
    }
    else {
        THROW_ARGOSEXCEPTION("Unknown exit code: " << exitCode);
    }
}

/****************************************/
/****************************************/

using swlexp::ExpLoopFunc;

REGISTER_LOOP_FUNCTIONS(ExpLoopFunc, "exp_loop_func");
