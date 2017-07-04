#include <argos3/core/simulator/loop_functions.h>
#include <string>
#include <vector>

#include "KilobotProcess.h"

#ifndef EXP_LOOP_FUNC_H
#define EXP_LOOP_FUNC_H

namespace swlexp {

    class ExpLoopFunc : public argos::CLoopFunctions {

    // ==============================
    // =          METHODS           =
    // ==============================

    public:

        ExpLoopFunc();
        virtual ~ExpLoopFunc();

        virtual void Init(argos::TConfigurationNode& t_tree);
        virtual void Destroy();
        virtual void Reset();

        virtual void PostStep();
        virtual bool IsExperimentFinished();

    private:

        /**
         * Places a certain number of robots in a line.
         * @param[in] numRobots The number of robots to place.
         */
        void _placeLine(argos::UInt32 numRobots);

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

        /**
         * Shared memory maps between the ARGoS and the kilobot processes.
         * This allows ARGoS to gather data about the kilobots' state, for example
         * to know when the experiment is finished.
         */
        std::vector<swlexp::KilobotProcess> m_kilobotProcesses;

    };

}

#endif // !EXP_LOOP_FUNC_H