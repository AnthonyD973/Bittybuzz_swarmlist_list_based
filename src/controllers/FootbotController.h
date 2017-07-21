/**
 * @file FootbotController.h
 * @brief Definition of the FootbotController class.
 */

#ifndef FOOTBOT_PROCESS_H
#define FOOTBOT_PROCESS_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <unordered_set>

#include "Swarmlist.h"

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


        // Getters

        /**
         * Gets the number of messages sent by the footbots of this process.
         * @return The number of messages sent by the footbots of this
         * process since the beginning of the experiment.
         */
        inline
        argos::UInt64 getNumMessagesTx() const { return m_numMsgsTx; }

        /**
         * Gets the number of messages received by the footbots of this
         * process since the beginning of the experiment.
         * @return The number of messages received by the footbots of this
         * process since the beginning of the experiment.
         */
        inline
        argos::UInt64 getNumMessagesRx() const { return m_numMsgsRx; }

        // Other functions

        /**
         * Constructs a string that contains the status of this footbot
         * in a CSV file format.
         * The values are comma-separated, and the string consits of a
         * single line terminated by a new-line character.
         * @return The CSV-formatted status of the Footbot.
         */
        std::string getCsvStatusLog();

    private:

        /**
         * Sends a set of messages
         */
        void _sendNextSwarmChunk();

        inline
        void _sendMsgTx(const argos::CByteArray& msgTx, argos::UInt8 numSends = 1) {
            m_rabAct->SetData(msgTx);
            m_numSends = numSends;
        }

        void _processMsgsRx();

        inline
        bool _isRabFree() const { return m_numSends == 0; }

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
         * Gets the total number of controllers.
         */
        static
        argos::UInt32 getNumControllers() { return c_controllers.size(); }

        /**
         * Determines the first ("header") line of the CSV log file.
         * @param[in,out] o The stream to write the header line into.
         */
        static
        void writeStatusLogHeader(std::ostream& o);

        /**
         * Writes the status log of all footbots inside a stream.
         * @param[in,out] o The stream to write the status logs into.
         */
        inline static
        void writeStatusLogs(std::ostream& o) {
            for (FootbotController* controller : c_controllers)
                o << controller->getCsvStatusLog();
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
        { return lhs + rhs->getNumMessagesTx(); }

        inline static
        argos::UInt64 _msgRxElemSum(argos::UInt64 lhs, const swlexp::FootbotController* rhs)
        { return lhs + rhs->getNumMessagesRx(); }

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

        RobotId m_id;                        ///< Numeric ID of this foot-bot.
        swlexp::Swarmlist m_swarmlist;       ///< Swarmlist of the foot-bot.
        argos::UInt8 m_localSwarmMask;       ///< Swarm mask of the current foot-bot.

        argos::UInt64 m_numMsgsTxSinceLog;   ///< Number of messages transmitted since the last status log.
        argos::UInt64 m_numMsgsRxSinceLog;   ///< Number of messages received since the last status log.
        argos::UInt64 m_numMsgsTx;           ///< Number of messages transmitted since the beginning of the experiment.
        argos::UInt64 m_numMsgsRx;           ///< Number of messages received since the beginning of the experiment.
        
        argos::UInt16 m_numSends;            ///< How many times we should send the message that is already set inside the RAB actuator.
        argos::UInt16 m_stepsTillTick;       ///< Number of control steps until a swarmlist tick is sent.
        argos::UInt16 m_stepsTillNextChunk;  ///< Number of control steps until a swarm chunk is triggered.

        argos::UInt32 m_timeAtLastLog;       ///< Simulation time at the last status log.


        // Actuators
        
        argos::CCI_LEDsActuator* m_leds;     ///< For setting the LEDs' color.
        argos::CCI_RangeAndBearingActuator*
                                  m_rabAct;  ///< Communication actuator.
        argos::CCI_RangeAndBearingSensor*
                                 m_rabSens;  ///< Communication sensor.

    // ==============================
    // =       STATIC MEMBERS       =
    // ==============================

    private:

        static std::unordered_set<FootbotController*> c_controllers; ///< Existing controllers.

        static const char c_CSV_DELIM = ','; ///< Delimiter between two CSV values.

        static argos::UInt16 c_packetSize;             ///< Size of a message.
        static argos::UInt16 c_numEntriesPerSwarmMsg;  ///< The number of data entries we transmit about other robots per packet.
        static const argos::UInt16 c_SWARM_ENTRY_SIZE; ///< Size of a single swarmlist entry in a message.
        static const argos::UInt8 c_ROBOT_ID_POS;      ///< Offset, inside a swarmlist entry, of the robot's ID.
        static const argos::UInt8 c_SWARM_MASK_POS;    ///< Offset, inside a swarmlist entry, of the swarm mask.
        static const argos::UInt8 c_LAMPORT_POS;       ///< Offset, inside a swarmlist entry, of the lamport clock.
    };

}

#endif // !FOOTBOT_PROCESS_H