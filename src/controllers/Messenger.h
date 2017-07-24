/**
 * @file Messenger.h
 * @brief Definition of the Messenger class.
 */

#ifndef MESSENGER_H
#define MESSENGER_H

#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <unordered_map>

#include "include.h"

namespace swlexp {

    /**
     * Encapsulates sending and receiving messages.
     */
    class Messenger {

    protected:

        friend class FootbotController;

    // ==============================
    // =       NESTED SYMBOLS       =
    // ==============================

    public:

        /**
         * Type for a callback object.
         */
        class Callback {
        public:
            virtual
            ~Callback() {}

            virtual
            void operator()(const argos::CCI_RangeAndBearingSensor::SPacket& packet) { }

            virtual
            bool operator==(const Callback& other) { return this == &other; }
        };

        /**
         * Type of a message.
         */
        enum MsgType : argos::UInt8 {
            MSG_TYPE_NOTHING = 0,
            MSG_TYPE_SWARM
        };

        /**
         * Hash functor for the type buckets.
         */
        struct MyHash {
            inline
            size_t operator()(MsgType type) const { return (size_t)type; }
        };

    // ==============================
    // =          METHODS           =
    // ==============================

    public:

        Messenger();

        ~Messenger();

        /**
         * Sets the transmitter and the receiver.
         */
        inline
        void plugComponents(argos::CCI_RangeAndBearingActuator* transmitter,
                            argos::CCI_RangeAndBearingSensor* receiver)
        { m_transmitter = transmitter; m_receiver = receiver; }

        /**
         * Sets a message to be sent.
         * @warning This overrides any previous message that would have
         * been sent.
         * @see isFree()
         * @param[in] msgTx The message to send.
         * @note Inside the byte array, the type is the first byte, i.e.,
         * msgTx.ToCArray()[0]
         */
        void sendMsgTx(const argos::CByteArray& msgTx);

        /**
         * Sets a message to be sent.
         * @warning This overrides any previous message that would have
         * been sent.
         * @param[in] msgTx The message to send.
         * @note Inside the byte array, the type is the first byte, i.e.,
         * msgTx.ToCArray()[0]
         */
        void sendMsgTx(argos::CByteArray&& msgTx);

        /**
         * Fetches the messages received since the last timestep.
         * @return The messages received since the last timestep.
         */
        const argos::CCI_RangeAndBearingSensor::TReadings& getMsgsRx() const;

        /**
         * Determines whether the messenger is free to send a message.
         */
        inline
        bool isFree() const { return m_isFree == 0; }

        /**
         * Registers a callback function that will be called whenever a message
         * of a certain type is received.
         * @param[in] type The type of message to call the callback on.
         * @param[in] callback The function to call whenever a message of
         * the desired type is received.
         * @warning Message type 0 is possible but should be avoided.
         */
        void registerCallback(MsgType type, Callback& callback);

        /**
         * Removes a callback function.
         * @param[in] type The message type the callback was called with.
         * @param[in] callback The function to remove.
         */
        void removeCallback(MsgType type, Callback& callback);

        /**
         * Removes a callback function.
         * @param[in] callback The function to remove.
         * @warning This removes the first matching
         * callback function.
         */
        void removeCallback(Callback& callback);

    private:

        Messenger(Messenger&) = delete;
        Messenger& operator=(const Messenger&) = delete;

        /**
         * Function that should be called exactly once every timestep.
         */
        void _controlStep();

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

        std::unordered_multimap<MsgType, Callback*, MyHash> m_callbacks; ///< Callback functions
        argos::CCI_RangeAndBearingActuator* m_transmitter; ///< Communication actuator.
        argos::CCI_RangeAndBearingSensor*   m_receiver;    ///< Communication sensor.
        argos::UInt8 m_isFree; ///< Whether the messenger is free to send another message.

    };
}

#endif // !MESSENGER_H