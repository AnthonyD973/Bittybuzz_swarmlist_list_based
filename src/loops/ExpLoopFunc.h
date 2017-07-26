#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>

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
         * Places a certain number of robots in one large cluster in
         * such a way that robots have a high number of neighbors in their
         * communication range.
         * @param[in] numRobots The number of robots to place.
         */
        void _placeScaleFree(argos::UInt32 numRobots);

        /**
         * Places a certain number of robots randomly inside a square.
         * @param[in] numRobots The number of robots to place.
         */
        void _placeCluster(argos::UInt32 numRobots);

        /**
         * Places a certain number of robots uniformly in an area.
         * @param[in] numRobots The number of robots to place.
         * @param[in] area The area to place the robots in.
         */
        void _placeUniformly(argos::UInt32 numRobots, argos::CRange<argos::Real> area);

        /**
         * Places the walls of the arena.
         * @param[in] numRobots The number of robots to place.
         * @param[in] density Density of the robots.
         */
        void _placeWalls(argos::UInt32 numRobots);

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
         * Gets the communication range of the foot-bots.
         */
        static
        argos::Real getRabRange() { return c_rabRange; }

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
        std::time_t m_timeSinceLastRealtimeOutput;

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

    // ==============================
    // =       STATIC MEMBERS       =
    // ==============================

        /**
         * Delimiter of the CSV file.
         */
        static const char c_CSV_DELIM = ',';

        /**
         * Range of communication of the foot-bot.
         */
        static argos::Real c_rabRange;

        /**
         * Size of a communication packet.
         */
        static argos::UInt16 c_packetSize;

    };

}

#endif // !EXP_LOOP_FUNC_H