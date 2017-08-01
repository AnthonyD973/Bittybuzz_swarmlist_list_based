#include <argos3/core/simulator/simulator.h>

#include "ExpState.h"
#include "RobotPlacer.h"
#include "FootbotController.h"
#include "Swarmlist.h"

// ==============================
// =    EXP_STATE_CONSENSUS     =
// ==============================

swlexp::ExpStateConsensus::ExpStateConsensus(argos::CLoopFunctions& loops)
    : ExpStateBase(loops)
{
    Swarmlist::setEntriesShouldBecomeInactive(true);
    Swarmlist::Entry::setTicksToInactive((argos::UInt32)-1);
    argos::TConfigurationNode& loop_functions = argos::GetNode(argos::CSimulator::GetInstance().GetConfigurationRoot(), "loop_functions");
    argos::GetNodeAttribute(loop_functions, "tti_file", m_ttiFileName);
}

/****************************************/
/****************************************/

swlexp::ExpStateConsensus::~ExpStateConsensus() {

}

/****************************************/
/****************************************/

void swlexp::ExpStateConsensus::init(
        std::string topology,
        argos::UInt32 numRobots) {
    // Place robots.
    RobotPlacer::getInst().placeRobots(topology, numRobots, *m_loops);
}

/****************************************/
/****************************************/

bool swlexp::ExpStateConsensus::isExperimentStalling(
        argos::UInt32 stepsToStall) {
    const argos::UInt32 TIME =
        argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
    if (FootbotController::getTotalNumActive() > m_lastTotalNumActive) {
        m_lastTotalNumActive = FootbotController::getTotalNumActive();
        m_timeSinceLastIncrease = TIME;
        return false;
    }
    return  (TIME - m_timeSinceLastIncrease > stepsToStall);
}

/****************************************/
/****************************************/

bool swlexp::ExpStateConsensus::isFinished() {
    bool isFinished = FootbotController::isConsensusReached();
    if (isFinished) {
        std::ofstream ttiFile(m_ttiFileName.c_str());
        FootbotController::writeTtiData(ttiFile);
        ttiFile.close();
    }
    return isFinished;
}

// ==============================
// =      EXP_STATE_ADDING      =
// ==============================

swlexp::ExpStateAdding::ExpStateAdding(argos::CLoopFunctions& loops)
    : ExpStateBase(loops)
{
    Swarmlist::setEntriesShouldBecomeInactive(false);
}

/****************************************/
/****************************************/

swlexp::ExpStateAdding::~ExpStateAdding() {

}

/****************************************/
/****************************************/

void swlexp::ExpStateAdding::init(
        std::string topology,
        argos::UInt32 numRobots) {
    // Place robots.
    RobotPlacer::getInst().placeRobots(topology, numRobots, *m_loops);

    // Remove one robot and force consensus.
    argos::CFootBotEntity* farthestRobot =
        &dynamic_cast<argos::CFootBotEntity&>(
            RobotPlacer::getInst().findFarthestFromOrigin());
    // Entities do no have a copy constructor. We must find and save the
    // parameters to use in the normal constructor.
    std::string farthestRobotStrId = farthestRobot->GetId();
    argos::CVector3 pos = farthestRobot->GetEmbodiedEntity().GetOriginAnchor().Position;
    argos::CQuaternion orient = farthestRobot->GetEmbodiedEntity().GetOriginAnchor().Orientation;

    // Remove the foot-bot, force consensus and add a new foot-bot
    // that is identical to the one we removed.
    m_loops->RemoveEntity(*farthestRobot);
    FootbotController::forceConsensus();
    argos::CEntity* robotCopy = new argos::CFootBotEntity(
        farthestRobotStrId,
        RobotPlacer::getControllerName(),
        pos,
        orient,
        RobotPlacer::getRabRange(),
        getPacketSize());
    m_loops->AddEntity(*robotCopy);
}

/****************************************/
/****************************************/

bool swlexp::ExpStateAdding::isExperimentStalling(
        argos::UInt32 stepsToStall) {
    const argos::UInt32 TIME =
        argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
    if (FootbotController::getTotalNumActive() > m_lastTotalNumActive) {
        m_lastTotalNumActive = FootbotController::getTotalNumActive();
        m_timeSinceLastIncrease = TIME;
        return false;
    }
    return  (TIME - m_timeSinceLastIncrease > stepsToStall);
}

/****************************************/
/****************************************/

bool swlexp::ExpStateAdding::isFinished() {
    return FootbotController::isConsensusReached();
}

// ==============================
// =     EXP_STATE_REMOVING     =
// ==============================

swlexp::ExpStateRemoving::ExpStateRemoving(argos::CLoopFunctions& loops)
    : ExpStateBase(loops)
{
    Swarmlist::setEntriesShouldBecomeInactive(true);
}

/****************************************/
/****************************************/

swlexp::ExpStateRemoving::~ExpStateRemoving() {

}

/****************************************/
/****************************************/

void swlexp::ExpStateRemoving::init(
        std::string topology,
        argos::UInt32 numRobots) {
    // Place robots.
    RobotPlacer::getInst().placeRobots(topology, numRobots, *m_loops);

    // Force consensus.
    FootbotController::forceConsensus();

    // Remove one robot.
    argos::CFootBotEntity* farthestRobot =
        &dynamic_cast<argos::CFootBotEntity&>(
            RobotPlacer::getInst().findFarthestFromOrigin());
    m_loops->RemoveEntity(*farthestRobot);

    m_lastTotalNumActive = FootbotController::getTotalNumActive();
}

/****************************************/
/****************************************/

bool swlexp::ExpStateRemoving::isExperimentStalling(
        argos::UInt32 stepsToStall) {
    const argos::UInt32 TIME =
        argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
    if (FootbotController::getTotalNumActive() < m_lastTotalNumActive) {
        m_lastTotalNumActive = FootbotController::getTotalNumActive();
        m_timeSinceLastIncrease = TIME;
        return false;
    }
    return  (TIME - m_timeSinceLastIncrease > stepsToStall);
}

/****************************************/
/****************************************/

bool swlexp::ExpStateRemoving::isFinished() {
    return FootbotController::isConsensusReached();
}

// ==============================
// =       EXP_STATE_ALL        =
// ==============================

swlexp::ExpStateAll::ExpStateAll(argos::CLoopFunctions& loops)
    : ExpStateBase(loops)
    , m_subState(nullptr)
{
    m_subState = new ExpStateConsensus(loops);
}

/****************************************/
/****************************************/

swlexp::ExpStateAll::~ExpStateAll() {
    delete m_subState;
}

/****************************************/
/****************************************/

void swlexp::ExpStateAll::init(
        std::string topology,
        argos::UInt32 numRobots) {
    m_progression = Progression::CONSENSUS;

    // Place robots.
    RobotPlacer::getInst().placeRobots(topology, numRobots, *m_loops);

    // Entities do no have a copy constructor. We must find and save the
    // parameters to use in the normal constructor.
    m_farthestRobot =
        &dynamic_cast<argos::CFootBotEntity&>(
            RobotPlacer::getInst().findFarthestFromOrigin());
    m_farthestRobotStrId  = m_farthestRobot->GetId();
    m_farthestRobotPos    = m_farthestRobot->GetEmbodiedEntity().GetOriginAnchor().Position;
    m_farthestRobotOrient = m_farthestRobot->GetEmbodiedEntity().GetOriginAnchor().Orientation;
}

/****************************************/
/****************************************/

bool swlexp::ExpStateAll::isExperimentStalling(
        argos::UInt32 stepsToStall) {
    return m_subState != nullptr && m_subState->isExperimentStalling(stepsToStall);
}

/****************************************/
/****************************************/

using argos::CARGoSException; // Required because of the THROW_ARGOSEXCEPTION macro.

bool swlexp::ExpStateAll::isFinished() {
    if (m_subState->isFinished()) {
        delete m_subState;
        
        switch (m_progression) {
            case Progression::CONSENSUS: {
                m_subState = new ExpStateRemoving(*m_loops);
                m_loops->RemoveEntity(*m_farthestRobot);
                m_progression = Progression::REMOVING;
                break;
            }
            case Progression::REMOVING: {
                m_subState = nullptr;
                m_progression = Progression::DONE;
                break;
            }
            default: THROW_ARGOSEXCEPTION("Unknown progression in " << __PRETTY_FUNCTION__);
        }
    }

    return (m_progression == Progression::DONE);
}
