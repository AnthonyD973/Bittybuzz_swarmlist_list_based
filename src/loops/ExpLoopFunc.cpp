#include <algorithm>

#include "ExpLoopFunc.h"

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
    // Get arguments from configuration file.
    argos::GetNodeAttribute(t_tree, "res", m_expResName);
    argos::GetNodeAttribute(t_tree, "log", m_expLogName);
    argos::GetNodeAttribute(t_tree, "kb_csv", m_expKbCsvName);
    argos::GetNodeAttribute(t_tree, "kb_meta_stuff_delay", m_expMetaStuffDelay);
    argos::GetNodeAttribute(t_tree, "check_if_finished_delay", m_expCheckIfFinishedDelay);
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

    // Get other experiment params
    argos::TConfigurationNode media = argos::GetNode(GetSimulator().GetConfigurationRoot(), "media");
    argos::TConfigurationNode kilocomm = argos::GetNode(media, "kilobot_communication");
    argos::GetNodeAttributeOrDefault(kilocomm, "message_drop_prob", m_msgDropProb, 0.0);

    // Open files.
    m_expKbCsv.open(m_expKbCsvName, std::ios::app);
    if (m_expKbCsv.fail()) {
        THROW_ARGOSEXCEPTION("Could not open CSV file \"" << m_expKbCsvName << "\".");
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
                topology      << CSV_DELIM <<
                numRobots     << CSV_DELIM <<
                m_msgDropProb;

    m_expKbCsv << "ID,"
                  "Time (timesteps),"
                  "Number of messages sent,"
                  "Avg. sent bandwidth (B/timestep),"
                  "Number of messages received,"
                  "Avg. received bandwidth (B/timestep),"
                  "Swarmlist size,"
                  "Swarmlist number of active entries,"
                  "\"Swarmlist data (robot ID,lamport,ticks to inactive)\"\n";

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
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::Destroy() {
    m_expRes.flush();
    m_expLog.flush();
    m_expKbCsv.flush();

    // Other resources are closed when they get out of scope.
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::Reset() {
    // Go through the eperiment data of each kilobot process
    for (auto it = m_kilobotProcesses.begin(); it < m_kilobotProcesses.end(); ++it) {
        it->reset();
    }
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::PostStep() {
    static argos::UInt32 callsTillMetaStuff = 0;

    if (callsTillMetaStuff == 1) {
        const argos::UInt32 TIME = GetSpace().GetSimulationClock();
        argos::UInt32 id = 0;
        for (auto it = m_kilobotProcesses.begin(); it < m_kilobotProcesses.end(); ++it) {
            exp_data_t* expData = it->getExpData();
            expData->time = TIME;
            set_meta_info(expData, EXP_DATA_META_INFO_SHOULD_LOG_STATUS);
            if (!get_and_clear_meta_info(expData, EXP_DATA_META_INFO_IS_ALIVE)) {
                m_expLog      << "Kilobot #" << id << "'s process has died unexpectedly.\n";
                m_expLog.flush();
                _finishExperiment();
                THROW_ARGOSEXCEPTION("Kilobot #" << id << "'s process has died unexpectedly.");
                return;
            }
            ++id;
        }
    }
    else if (callsTillMetaStuff == 0) {
        callsTillMetaStuff = m_expMetaStuffDelay;
        for (auto it = m_kilobotProcesses.begin(); it < m_kilobotProcesses.end(); ++it) {
            std::string logData = it->getLogData();
            if (logData != "") {
                m_expKbCsv << logData << '\n';
                m_expKbCsv.flush();
            }
        }
    }

    --callsTillMetaStuff;
}

/****************************************/
/****************************************/

bool swlexp::ExpLoopFunc::IsExperimentFinished() {
    static argos::UInt32 callsTillCheck = 0;

    bool isFinished;
    if (callsTillCheck == 0) {
        callsTillCheck = m_expCheckIfFinishedDelay;

        isFinished = true;
        const argos::UInt32 NUM_KILOBOTS = m_kilobotProcesses.size();
        for (auto it = m_kilobotProcesses.begin(); it < m_kilobotProcesses.end(); ++it) {
            if (it->getExpData()->swarmlist.num_active != NUM_KILOBOTS) {
                isFinished = false;
                break;
            }
        }
        if (isFinished) {
            _finishExperiment();
        }
    }
    else {
        isFinished = false;
    }

    --callsTillCheck;
    return isFinished;
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::_placeLine(argos::UInt32 numRobots) {
    argos::CRange<argos::CVector3> arenaLimits = GetSpace().GetArenaLimits();
    argos::Real maxY = arenaLimits.GetMax().GetY() - 0.03;
    for (argos::UInt32 i = 0; i < numRobots; ++i) {
        argos::CVector3 pos = argos::CVector3(0, maxY - i * 0.06, 0);
        argos::CQuaternion orient = argos::CQuaternion();

        swlexp::KilobotProcess&& kProc = KilobotProcess(*this, i, "kb_ctrl", pos, orient);
        m_kilobotProcesses.push_back(std::move(kProc));
    }
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::_finishExperiment() {
    const argos::UInt32 NUM_KILOBOTS = m_kilobotProcesses.size();
    const argos::UInt64 NUM_MSGS_TX  = KilobotProcess::getTotalNumMessagesTx();
    const argos::UInt64 NUM_MSGS_RX  = KilobotProcess::getTotalNumMessagesRx();
    double bw_tx    = ((double)NUM_MSGS_TX / GetSpace().GetSimulationClock() /
                   NUM_KILOBOTS) * 13;
    double bw_rx = ((double)NUM_MSGS_RX / GetSpace().GetSimulationClock() /
                   NUM_KILOBOTS) * 13;

    m_expRes << CSV_DELIM << GetSpace().GetSimulationClock() <<
                CSV_DELIM << NUM_MSGS_TX <<
                CSV_DELIM << NUM_MSGS_RX <<
                CSV_DELIM << bw_tx <<
                CSV_DELIM << bw_rx;
    m_expRes.flush();
    m_expLog << "---END---\n"
                "Consensus (ts): " << GetSpace().GetSimulationClock() << "\n"
                "Msgs sent (total): " << NUM_MSGS_TX << "\n"
                "Msgs received (total): " << NUM_MSGS_RX << "\n"
                "Avg. sent bandwidth (B/(ts*kb)): " << bw_tx << "\n"
                "Avg. received bandwidth (B/(ts*kb)): " << bw_rx << "\n"
                "\n";
    m_expLog.flush();

    argos::LOG << "Experiment finished in " << GetSpace().GetSimulationClock() <<
                  " timesteps. See \"" << m_expLogName << "\" for results.\n";
}

/****************************************/
/****************************************/

using swlexp::ExpLoopFunc;

REGISTER_LOOP_FUNCTIONS(ExpLoopFunc, "exp_loop_func");