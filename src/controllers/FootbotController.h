/**
 * @file FootbotController.h
 * @brief Definition of the FootbotController class.
 */

#ifndef FOOTBOT_CONTROLLER_H
#define FOOTBOT_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <unordered_set>

#include "Swarmlist.h"
#include "Messenger.h"

namespace swlexp {

    /**
     * Foot-bots' controller for the swarmlist simulation.
     */
    class FootbotController : public argos::CCI_Controller {

    // ==============================
    // =          METHODS           =
    // ==============================

    public:

        /**
         * Main constructor.
         */
        FootbotController();
        ~FootbotController();

        FootbotController(FootbotController&) = delete;
        FootbotController& operator=(FootbotController&) = delete;

        // Init, Step, Destroy (we do not support Reset for the experiment)

        virtual void Init(argos::TConfigurationNode& t_node);

        virtual void ControlStep();

        virtual void Destroy();

        // Other functions

        /**
         * Constructs a string that contains the status of this footbot
         * in a CSV file format.
         * The values are comma-separated, and the string consits of a
         * single line terminated by a new-line character.
         * @param[in] sideEffect Whether a call to this function sets
         * the time at which we last called it.
         * @return The CSV-formatted status of the Footbot.
         */
        std::string getCsvStatusLog(bool sideEffect);

    // ==============================
    // =       STATIC METHODS       =
    // ==============================

    public:

        /**
         * Gets the number of messages sent by all the footbots.
         */
        static
        argos::UInt64 getTotalNumMessagesTx();

        /**
         * Gets the number of messages received by all the footbots.
         */
        static
        argos::UInt64 getTotalNumMessagesRx();

        /**
         * Gets the size of a sent packet.
         */
        inline static
        argos::UInt16 getPacketSize() { return c_packetSize; }

        /**
         * Gets the packet drop probability.
         */
        inline static
        argos::Real getPacketDropProb() { return c_packetDropProb; }

        /**
         * Gets the total number of controllers.
         */
        static
        argos::UInt32 getNumControllers() { return c_controllers.size(); }

        /**
         * Makes all robots reach conensus immediately.
         * This is used when we want to see how long it would take
         * for a new robot's data to be propagated through an
         * existing swarm.
         */
        static
        void forceConsensus();

        /**
         * Determines the first ("header") line of the CSV log file.
         * @param[in,out] o The stream to write the header line into.
         * @param[in] sideEffect Whether a call to this function sets
         * the time at which we last called it.
         */
        static
        void writeStatusLogHeader(std::ostream& o);

        /**
         * Writes the status log of all footbots inside a stream.
         * @param[in,out] o The stream to write the status logs into.
         * @param[in] sideEffect Whether a call to this function sets
         * the time at which we last called it.
         */
        inline static
        void writeStatusLogs(std::ostream& o, bool sideEffect) {
            for (FootbotController* controller : c_controllers)
                o << controller->getCsvStatusLog(sideEffect);
        }

        /**
         * Determines whether the number of active foot-bot entries of all
         * foot-bots' swarmlists is equal to the number of foot-bots.
         * @return Whether the number of active foot-bot entries of all
         * foot-bots' swarmlists is equal to the number of foot-bots.
         */
        static
        bool isConsensusReached();
    
    private:

        inline static
        argos::UInt64 _msgTxElemSum(argos::UInt64 lhs, const swlexp::FootbotController* rhs)
        { return lhs + rhs->m_swarmlist.getNumMsgsTx(); }

        inline static
        argos::UInt64 _msgRxElemSum(argos::UInt64 lhs, const swlexp::FootbotController* rhs)
        { return lhs + rhs->m_swarmlist.getNumMsgsRx(); }

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

        RobotId m_id;                        ///< Numeric ID of this foot-bot.
        swlexp::Messenger m_msn;             ///< Messenger.
        swlexp::Swarmlist m_swarmlist;       ///< Swarmlist of the foot-bot.
        argos::UInt8 m_localSwarmMask;       ///< Swarm mask of the current robot.
        
        argos::UInt16 m_stepsTillTick;       ///< Number of control steps until a swarmlist tick is sent.

        argos::UInt32 m_timeAtLastLog = (argos::UInt32)-1; ///< Simulation time at the last status log.
        argos::UInt64 m_numMsgsTxAtLastLog = 0; ///< Number of sent messages at the last status log.
        argos::UInt64 m_numMsgsRxAtLastLog = 0; ///< Number of receieved messages at the last status log.


        // Sensors/Actuators

        argos::CCI_LEDsActuator* m_leds;       ///< For setting the LEDs' color.

    // ==============================
    // =       STATIC MEMBERS       =
    // ==============================

    private:

        static const char c_CSV_DELIM = ',';   ///< Delimiter between two CSV values.

        static std::unordered_set<FootbotController*> c_controllers; ///< Existing controllers.

        static argos::UInt16 c_packetSize;     ///< Size of a message.
        static argos::Real   c_packetDropProb; ///< Probability of occurrence of a packet drop.
    };

}

#endif // !FOOTBOT_CONTROLLER_H