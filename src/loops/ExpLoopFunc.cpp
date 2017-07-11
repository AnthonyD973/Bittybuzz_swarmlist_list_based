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

    // Open result and log files.
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

    m_expLog << "---EXPERIMENT START---\n"
                "Topology: " << topology << "\n" <<
                "Number of robots: " << numRobots << "\n"
                "Drop probability: " << m_msgDropProb << "\n";
    m_expLog.flush();
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::Destroy() {
    m_expLog.flush();

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
    static const argos::UInt32 STEPS_TO_MSG_CHECK = 30;
    static argos::UInt32 stepsTillNumMsgCheck = 0;

    if (stepsTillNumMsgCheck == 0) {
        _checkNumMessages();
        stepsTillNumMsgCheck = STEPS_TO_MSG_CHECK;
    }
    --stepsTillNumMsgCheck;
}

/****************************************/
/****************************************/

bool swlexp::ExpLoopFunc::IsExperimentFinished() {
    static const argos::UInt32 CALLS_TO_CHECK = 10;
    static argos::UInt32 callsTillCheck = 0;

    bool isFinished;
    if (callsTillCheck == 0) {
        callsTillCheck = CALLS_TO_CHECK;

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

void swlexp::ExpLoopFunc::_checkNumMessages() {
    for (auto it = m_kilobotProcesses.begin(); it < m_kilobotProcesses.end(); ++it) {
        uint8_t* numMsgs = &it->getExpData()->num_msgs_in_timestep;
        m_numMsgsSent += *numMsgs;
        *numMsgs = 0;
    }
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::_finishExperiment() {
    const argos::UInt32 NUM_KILOBOTS = m_kilobotProcesses.size();
    _checkNumMessages();
    double bw = ((double)m_numMsgsSent / GetSpace().GetSimulationClock() /
                 NUM_KILOBOTS * 13);
    double bw_rx = bw * (1.0 - m_msgDropProb);

    m_expRes << CSV_DELIM << GetSpace().GetSimulationClock() <<
                CSV_DELIM << m_numMsgsSent <<
                CSV_DELIM << bw <<
                CSV_DELIM << bw_rx;
    m_expLog << "---END---\n"
                "Consensus (ts): " << GetSpace().GetSimulationClock() << "\n"
                "Msgs sent (total): " << m_numMsgsSent << "\n"
                "Avg. bandwidth (B/(ts*kb)): " << bw << "\n"
                "Avg. received bandwidth (B/(ts*kb)): " << bw_rx << "\n"
                "\n";
    m_expLog.flush();

    argos::LOG << "Experiment finished. See \"" << m_expLogName << "\" for results.\n";
}

/****************************************/
/****************************************/

using swlexp::ExpLoopFunc;

REGISTER_LOOP_FUNCTIONS(ExpLoopFunc, "exp_loop_func");