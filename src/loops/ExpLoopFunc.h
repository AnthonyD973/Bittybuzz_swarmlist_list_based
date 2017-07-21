#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <fstream>
#include <string>
#include <vector>

#include "FootbotController.h"

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
            NORMAL          = 0x00, ///< Exited normally.
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
         * Places a certain number of robots in a line.
         * @param[in] numRobots The number of robots to place.
         */
        void _placeLine(argos::UInt32 numRobots);

        /**
         * Finishes the experiment, in effect writing experiment data to the log
         * file.
         * @param[in] exitCode Code for how to finish the experiment.
         */
        void _finishExperiment(swlexp::ExpLoopFunc::ExitCode exitCode);

        /**
         * Determines the string that corresponds to an exit code.
         * @param[in] exitCode The exit code.
         * @return The string corresponding to the exit code.
         */
        static std::string _exitCodeToString(swlexp::ExpLoopFunc::ExitCode exitCode);

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

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
         * Path to the file that footbots log CSV data to every few timesteps.
         */
        std::string m_expFbCsvName;

        /**
         * File that footbots log CSV data to every few timesteps.
         */
        std::ofstream m_expFbCsv;

        /**
         * @brief How often (in timesteps) wait until we do meta things, i.e.,
         * 1) request the footbots to log their status.
         */
        argos::UInt32 m_expStatusLogDelay;

        /**
         * Experiment param. Specifies the probablility that a
         * message that should have been received is dropped.
         */
        argos::Real m_msgDropProb;

        /**
         * Delimiter of the CSV file.
         */
        static const char c_CSV_DELIM = ',';

        /**
         * Range of communication of the foot-bot.
         */
        static const argos::Real c_RAB_RANGE;

        /**
         * Size of a communication packet.
         */
        static argos::UInt16 c_packetSize;

    };

}

#endif // !EXP_LOOP_FUNC_H