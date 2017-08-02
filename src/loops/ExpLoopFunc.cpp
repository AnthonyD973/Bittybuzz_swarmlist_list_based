#include <algorithm>
#include <list>

#include "ExpLoopFunc.h"

namespace swlexp {
    argos::UInt16 ExpLoopFunc::c_packetSize;
}

/****************************************/
/****************************************/

argos::UInt16 swlexp::getPacketSize() {
    return ExpLoopFunc::getPacketSize();
}

/****************************************/
/****************************************/

swlexp::ExpLoopFunc::ExpLoopFunc()
    : m_state(nullptr)
{ }

/****************************************/
/****************************************/

swlexp::ExpLoopFunc::~ExpLoopFunc() {

}

/****************************************/
/****************************************/

using argos::CARGoSException; // Required because of the THROW_ARGOSEXCEPTION macro.

void swlexp::ExpLoopFunc::Init(argos::TConfigurationNode& t_tree) {

    m_timeBeginning = std::time(NULL);

    // Get experiment params
    argos::TConfigurationNode& controllers        = argos::GetNode(GetSimulator().GetConfigurationRoot(), "controllers");
    argos::TConfigurationNode& footbot_controller = argos::GetNode(controllers,        "footbot_controller");
    argos::TConfigurationNode& sensors            = argos::GetNode(footbot_controller, "sensors");
    argos::TConfigurationNode& rab                = argos::GetNode(sensors,            "range_and_bearing");
    argos::GetNodeAttributeOrDefault(rab, "packet_drop_prob", m_msgDropProb, 0.0);

    argos::GetNodeAttribute(t_tree, "res", m_expResName);
    argos::GetNodeAttribute(t_tree, "log", m_expLogName);
    argos::GetNodeAttribute(t_tree, "fb_csv", m_expFbCsvName);
    argos::GetNodeAttribute(t_tree, "realtime_output_file", m_expRealtimeOutputName);
    argos::GetNodeAttribute(t_tree, "fb_status_log_delay", m_expStatusLogDelay);
    argos::GetNodeAttribute(t_tree, "steps_to_stall", m_expStepsToStall);
    argos::GetNodeAttribute(t_tree, "packet_size", c_packetSize);
    argos::GetNodeAttribute(t_tree, "protocol", m_protocol);
    argos::GetNodeAttribute(t_tree, "topology", m_topology);
    argos::GetNodeAttribute(t_tree, "num_robots", m_numRobots);
    std::string jobId;
    argos::GetNodeAttribute(t_tree, "job_id", jobId);

    std::string walltimeStr;
    argos::GetNodeAttribute(t_tree, "walltime", walltimeStr);
    const char* wtCStr = walltimeStr.c_str();
    m_expWalltime = 0;
    while (wtCStr[0] != '\0') {
        while (wtCStr[1] != ':'  &&
               wtCStr[1] != '\0' &&
               wtCStr[0] == '0') {
            ++wtCStr;
        }
        m_expWalltime *= 60;
        m_expWalltime += std::atoi(wtCStr);
        while (wtCStr[0] != ':' && wtCStr[1] != '\0') {
            ++wtCStr;
        }
        ++wtCStr;
    }

    // Open files.
    m_expFbCsv.open(m_expFbCsvName, std::ios::trunc);
    if (m_expFbCsv.fail()) {
        THROW_ARGOSEXCEPTION("Could not open CSV file \"" << m_expFbCsvName << "\".");
    }
    if (m_expRealtimeOutputName != "") {
        m_expRealtimeOutput.open(m_expRealtimeOutputName, std::ios::trunc);
        if (m_expRealtimeOutput.fail()) {
            THROW_ARGOSEXCEPTION("Could not open realtime CSV file \"" << m_expFbCsvName << "\".");
        }
    }
    else {
        argos::LOG << "No realtime file used.\n";
    }
    m_expRes.open(m_expResName, std::ios::trunc);
    if (m_expRes.fail()) {
        THROW_ARGOSEXCEPTION("Could not open CSV file \"" << m_expResName << "\".");
    }
    m_expLog.open(m_expLogName, std::ios::trunc);
    if (m_expLog.fail()) {
        THROW_ARGOSEXCEPTION("Could not open log file \"" << m_expLogName << "\".");
    }

    // Write experiment params to result and log files.
    m_expRes << '\n' <<
                m_protocol    << c_CSV_DELIM <<
                m_topology    << c_CSV_DELIM <<
                m_msgDropProb << c_CSV_DELIM <<
                m_numRobots;

    m_expLog << "---EXPERIMENT START---\n";
    if (jobId != "") {
        m_expLog << "Job ID: " << jobId << "\n";
    }
    m_expLog << "Protocol: " << m_protocol << "\n"
                "Topology: " << m_topology << "\n"
                "Drop probability: " << (m_msgDropProb * 100) << "%\n"
                "Number of robots: " << m_numRobots << "\n";

    std::string toDisplayWalltime;
    if (m_expWalltime > 0) {
        toDisplayWalltime = walltimeStr + " (" + std::to_string(m_expWalltime) + " seconds)";
    }
    else {
        toDisplayWalltime = "none";
    }

    argos::LOG << "--------------------------------------\n"
                  "WALLTIME:         " << toDisplayWalltime << "\n"
                  "PROTOCOL:         " << m_protocol << "\n"
                  "TOPOLOGY:         " << m_topology << "\n"
                  "DROP PROBABILITY: " << (m_msgDropProb * 100) << "%\n"
                  "NUMBER OF ROBOTS: " << m_numRobots << "\n"
                  "--------------------------------------\n";
    m_expLog.flush();

    // Prepare the aparatus for protocol
    if (m_protocol == "consensus") {
        m_state = new ExpStateConsensus(*this);
    }
    else if (m_protocol == "adding") {
        m_state = new ExpStateAdding(*this);
    }
    else if (m_protocol == "removing") {
        m_state = new ExpStateRemoving(*this);
    }
    else if (m_protocol == "all") {
        m_state = new ExpStateAll(*this);
    }
    else {
        THROW_ARGOSEXCEPTION("Unknown protocol: \"" << m_protocol << "\"");
    }
    m_state->init(m_topology, m_numRobots);

    // Setup realtime output.
    m_timeAtLastRealtimeOutput = std::time(NULL);
    swlexp::FootbotController::writeStatusLogHeader(m_expRealtimeOutput);
    swlexp::FootbotController::writeStatusLogs(m_expRealtimeOutput, false);
    m_expRealtimeOutput.flush();

    // Write header in the status logs file and perform the first status log.
    swlexp::FootbotController::writeStatusLogHeader(m_expFbCsv);
    swlexp::FootbotController::writeStatusLogs(m_expFbCsv, true);
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::Destroy() {
    m_expRes.flush();
    m_expLog.flush();
    m_expFbCsv.flush();
    delete m_state;
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::PostStep() {
    static argos::UInt32 callsTillStatusLog = m_expStatusLogDelay - 1;

    // Write to realtime status log every hour.
    static const argos::UInt32 DELAY_FOR_REALTIME_LOG = 3600;
    std::time_t time = std::time(NULL);
    if (time - m_timeAtLastRealtimeOutput >= DELAY_FOR_REALTIME_LOG) {
        m_timeAtLastRealtimeOutput = time;
        // Erase realtime file and write the new data to it.
        m_expRealtimeOutput.close();
        m_expRealtimeOutput = std::ofstream(m_expRealtimeOutputName.c_str(), std::ios::trunc);
        FootbotController::writeStatusLogHeader(m_expRealtimeOutput);
        FootbotController::writeStatusLogs(m_expRealtimeOutput, false);
        m_expRealtimeOutput.flush();
    }

    if (callsTillStatusLog == 0) {
        callsTillStatusLog = m_expStatusLogDelay;
        FootbotController::writeStatusLogs(m_expFbCsv, true);
    }

    --callsTillStatusLog;
}

/****************************************/
/****************************************/

bool swlexp::ExpLoopFunc::IsExperimentFinished() {
    std::time_t time = std::time(NULL);
    bool isExperimentFinished = m_state->isFinished();
    bool isWalltimeReached = (m_expWalltime != 0 && (time - m_timeBeginning >= m_expWalltime));
    bool isExperimentStalling = false;//m_state->isExperimentStalling(m_expStepsToStall);
    if (isExperimentFinished) {
        _finishExperiment(ExitCode::NORMAL);
    }
    else if (isWalltimeReached) {
        _finishExperiment(ExitCode::WALLTIME_REACHED);
    }
    else if (isExperimentStalling) {
        _finishExperiment(ExitCode::STALLING_EXPERIMENT);
    }

    bool isFinished = isExperimentFinished;
    return isFinished;
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
        argos::Real bwTx = ((argos::Real)NUM_MSGS_TX / GetSpace().GetSimulationClock() /
            NUM_FOOTBOTS) * c_packetSize;
        argos::Real bwRx = ((argos::Real)NUM_MSGS_RX / GetSpace().GetSimulationClock() /
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

        swlexp::FootbotController::writeStatusLogs(m_expFbCsv, true);
        m_expFbCsv.flush();

        argos::LOG << "Experiment finished normally in " << GetSpace().GetSimulationClock() <<
                    " timesteps. See \"" << m_expLogName << "\" for results.\n";
    }
    else {
        remove(m_expFbCsvName.c_str());
        remove(m_expResName.c_str());
        m_expLog << "[ERROR] " << _exitCodeToString(exitCode) << "\n";
        m_expLog.flush();
        THROW_ARGOSEXCEPTION(_exitCodeToString(exitCode));
    }
}

/****************************************/
/****************************************/

std::string swlexp::ExpLoopFunc::_exitCodeToString(swlexp::ExpLoopFunc::ExitCode exitCode) {
    if (exitCode == ExitCode::NORMAL) {
        return "";
    }
    else if (exitCode == ExitCode::WALLTIME_REACHED) {
        return "Walltime reached.";
    }
    else if (exitCode == ExitCode::STALLING_EXPERIMENT) {
        return "No progress has been observed for a long time.";
    }
    else {
        THROW_ARGOSEXCEPTION("Unknown exit code: " << exitCode);
    }
}

/****************************************/
/****************************************/

using swlexp::ExpLoopFunc;

REGISTER_LOOP_FUNCTIONS(ExpLoopFunc, "exp_loop_func");
