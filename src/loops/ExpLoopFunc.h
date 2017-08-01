#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>

#include "FootbotController.h"
#include "ExpState.h"

#ifndef EXP_LOOP_FUNC_H
#define EXP_LOOP_FUNC_H

namespace swlexp {


    class ExpLoopFunc : public argos::CLoopFunctions {

    // ==============================
    // =       NESTED SYMBOLS       =
    // ==============================

    private:

        /**
         * Enum that gives info about why we are exiting.
         */
        enum ExitCode : argos::SInt16 {
            NORMAL          = 0, ///< Exited normally.
            WALLTIME_REACHED,    ///< Reached walltime before the experiment could finish.
            STALLING_EXPERIMENT, ///< No new data had been propagated for a while.
        };

    // ==============================
    // =          METHODS           =
    // ==============================

    public:

        ExpLoopFunc();
        virtual ~ExpLoopFunc();

        virtual void Init(argos::TConfigurationNode& t_tree);
        virtual void Destroy();

        virtual void PostStep();
        virtual bool IsExperimentFinished();

    private:

        /**
         * Finishes the experiment, in effect writing experiment data to the log
         * file.
         * @param[in] exitCode Code for how to finish the experiment.
         */
        void _finishExperiment(swlexp::ExpLoopFunc::ExitCode exitCode);

    // ==============================
    // =       STATIC METHODS       =
    // ==============================

    public:

        /**
         * Determines the size of a packet.
         * @return The size of a packet.
         */
        static
        argos::UInt16 getPacketSize() { return c_packetSize; }

    private:

        /**
         * Determines the string that corresponds to an exit code.
         * @param[in] exitCode The exit code.
         * @return The string corresponding to the exit code.
         */
        static
        std::string _exitCodeToString(swlexp::ExpLoopFunc::ExitCode exitCode);

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

        /**
         * State of the experiment.
         */
        ExpStateBase* m_state;

        /**
         * Path to the experiment's param & result file.
         */
        std::string m_expResName;

        /**
         * File that will contain the experiment's parameters and its results.
         * This is a CSV file that can be opened with LibreOffice Calc
         * (possibly also MS Excel).
         */
        std::ofstream m_expRes;

        /**
         * Path to the experiment's log file.
         */
        std::string m_expLogName;

        /**
         * Log file that will contain data about the experiment.
         */
        std::ofstream m_expLog;

        /**
         * Path to the file that footbots log status data to every few timesteps.
         */
        std::string m_expFbCsvName;

        /**
         * File that footbots log status data to every few timesteps.
         */
        std::ofstream m_expFbCsv;

        /**
         * Path to the file that we perform status logs into. Unlike the
         * foot-bots' status logs, whose contents we do not know on the batch
         * server until the end of the experiment, we can view the contents
         * of this file while the experiment is running.
         */
        std::string m_expRealtimeOutputName;

        /**
         * File that we perform status logs into. Unlike the
         * foot-bots' status logs, whose contents we do not know on the batch
         * server until the end of the experiment, we can view the contents
         * of this file while the experiment is running.
         */
        std::ofstream m_expRealtimeOutput;

        /**
         * Time for the realtime output file.
         */
        std::time_t m_timeAtLastRealtimeOutput;

        /**
         * Time at which the experiment began.
         */
        std::time_t m_timeBeginning;

        /**
         * Maximum time (in sec) after which we kill the experiment.
         */
        argos::UInt32 m_expWalltime;

        /**
         * Number of timesteps without progress after which we consider that
         * the experiment is stalling and kill it.
         */
        argos::UInt32 m_expStepsToStall;

        /**
         * @brief How long (in timesteps) we wait until we request the
         * foot-bots to log their status.
         */
        argos::UInt32 m_expStatusLogDelay;

        /**
         * @brief Experiment param. Specifies what the experiment does.
         * @details Possible values:
         * 1) "consensus" -- Runs an experiment from scratch and stops when
         * all the robots have the data of all the others.
         * 2) "adding" -- Forces consensus on all the robots at the
         * beginning of the experiment, and immediately adds another robot.
         */
        std::string m_protocol;

        /**
         * @brief Experiment param. Specifies what topology we are in.
         */
        std::string m_topology;

        /**
         * @brief Experiment param. Specifies the probablility that a
         * message that should have been received is dropped.
         */
        argos::Real m_msgDropProb;

        /**
         * @brief Experiment param. Specifies how many robots are placed.
         */
        argos::UInt32 m_numRobots;

    // ==============================
    // =       STATIC MEMBERS       =
    // ==============================

        /**
         * Delimiter of the CSV file.
         */
        static const char c_CSV_DELIM = ',';

        /**
         * Size of a communication packet.
         */
        static argos::UInt16 c_packetSize;

    };

}

#endif // !EXP_LOOP_FUNC_H