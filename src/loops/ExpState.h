#ifndef EXP_STATE_H
#define EXP_STATE_H

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <string>
#include <iostream>

#include "include.h"

namespace swlexp {

    class ExpStateBase {
    public:
        virtual ~ExpStateBase() {}
        virtual void init(std::string topology, argos::UInt32 numRobots) {}
        /**
         * Determines whether the experiment has not been making any
         * progress for a while.
         * We define "progress" to be an increase (or decrease, depending on
         * what we are expecting) in the total number of active entries.
         * @param[in] stepsToStall The number of timesteps without any
         * progress after which we declare the experiment as stalling.
         * @return Whether the experiment is stalling.
         */
        virtual bool isExperimentStalling(argos::UInt32 stepsToStall) = 0;
        virtual bool isFinished() = 0;

    protected:
        ExpStateBase(argos::CLoopFunctions& loops) : m_loops(&loops) {}
    
    protected:
        argos::CLoopFunctions* m_loops;
    };


    class ExpStateConsensus : public ExpStateBase {
    public:
        ExpStateConsensus(argos::CLoopFunctions& loops);
        virtual ~ExpStateConsensus();
        virtual void init(std::string topology, argos::UInt32 numRobots);
        virtual bool isExperimentStalling(argos::UInt32 stepsToStall);
        virtual bool isFinished();
    
    private:
        argos::UInt32 m_timeSinceLastIncrease = 0;
        argos::UInt64 m_lastTotalNumActive = 0;
        std::string m_ttiFileName;
    };


    class ExpStateAdding : public ExpStateBase {
    public:
        ExpStateAdding(argos::CLoopFunctions& loops);
        virtual ~ExpStateAdding();
        virtual void init(std::string topology, argos::UInt32 numRobots);
        virtual bool isExperimentStalling(argos::UInt32 stepsToStall);
        virtual bool isFinished();
    
    private:
        argos::UInt32 m_timeSinceLastIncrease = 0;
        argos::UInt64 m_lastTotalNumActive = 0;
    };


    class ExpStateRemoving : public ExpStateBase {
    public:
        ExpStateRemoving(argos::CLoopFunctions& loops);
        virtual ~ExpStateRemoving();
        virtual void init(std::string topology, argos::UInt32 numRobots);
        virtual bool isExperimentStalling(argos::UInt32 stepsToStall);
        virtual bool isFinished();
    
    private:
        argos::UInt32 m_timeSinceLastIncrease = 0;
        argos::UInt64 m_lastTotalNumActive;
    };


    class ExpStateAll : public ExpStateBase {
    private:
        enum class Progression {
            CONSENSUS = 0,
            REMOVING,
            DONE
        };

    public:
        ExpStateAll(argos::CLoopFunctions& loops);
        virtual ~ExpStateAll();
        virtual void init(std::string topology, argos::UInt32 numRobots);
        virtual bool isExperimentStalling(argos::UInt32 stepsToStall);
        virtual bool isFinished();

    private:
        Progression m_progression;
        ExpStateBase* m_subState;
        argos::CFootBotEntity* m_farthestRobot;
        std::string m_farthestRobotStrId;
        argos::CVector3 m_farthestRobotPos;
        argos::CQuaternion m_farthestRobotOrient;

    };
}

#endif // !EXP_STATE_H