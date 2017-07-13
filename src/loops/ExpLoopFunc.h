#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <fstream>
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

        /**
         * Finishes the experiment, in effect writing experiment data to the log
         * file.
         */
        void _finishExperiment();

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
         * Path to the file that kilobots log CSV data to every few timesteps.
         */
        std::string m_expKbCsvName;

        /**
         * File that kilobots log CSV data to every few timesteps.
         */
        std::ofstream m_expKbCsv;

        /**
         * @brief How often (in timesteps) wait until we do meta things, i.e.,
         * 1) request the kilobots to log their status, and
         * 2) check if the kilobot processes are still alive.
         */
        argos::UInt32 m_expMetaStuffDelay;

        /**
         * How often (in timesteps) we wait until we check if the experiment
         * is finished.
         */
        argos::UInt32 m_expCheckIfFinishedDelay;

        /**
         * Experiment param. Specifies the probablility that a
         * message that should have been received is dropped.
         */
        argos::Real m_msgDropProb;

        /**
         * Delimiter of the CSV file.
         */
        static const char CSV_DELIM = ',';

    };

}

#endif // !EXP_LOOP_FUNC_H