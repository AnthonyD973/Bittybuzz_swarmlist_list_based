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

void swlexp::ExpLoopFunc::Init(argos::TConfigurationNode& t_tree) {
    argos::UInt32 numRobots;
    argos::GetNodeAttribute(t_tree, "robots", numRobots);
    std::string topology;
    argos::GetNodeAttribute(t_tree, "topology", topology);
    if (topology == "line") {
        _placeLine(numRobots);
    }
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::Destroy() {
    // Nothing to do ; resources are closed when they get out of scope.
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

}

/****************************************/
/****************************************/

bool swlexp::ExpLoopFunc::IsExperimentFinished() {
    const argos::UInt32 NUM_KILOBOTS = m_kilobotProcesses.size();
    for (auto it = m_kilobotProcesses.begin(); it < m_kilobotProcesses.end(); ++it) {
        if (it->getExpData()->swarmlist.num_active < NUM_KILOBOTS) {
            return false;
        }
    }
    argos::LOG << "Experiment finished. Timesteps taken: " << GetSpace().GetSimulationClock() << ".\n";
    return true;
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

using swlexp::ExpLoopFunc;

REGISTER_LOOP_FUNCTIONS(ExpLoopFunc, "exp_loop_func");