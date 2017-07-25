/**
 * @file include.h
 * @brief Definition of symbols that should be defined everywhere.
 */

#ifndef INCLUDE_H
#define INCLUDE_H

#include <argos3/core/utility/datatypes/datatypes.h>

namespace swlexp {

    // ===============================
    // =     CONFIGURABLE VALUES     =
    // ===============================

    /**
     * The number of timesteps between the emission of swarm chunks.
     */
    #define STEPS_PER_CHUNK 1

    /**
     * The maximum number of swarm messages to send per swarm chunk.
     * We send less messages if we don't have enough data to send.
     */
    #define SWARM_CHUNK_AMOUNT 1 // Doesn't work if it's not equal to 1.

    /**
     * The number of swarmlist ticks before a swarm entry
     * is considered inactive.
     * @note <b>Max value: 255</b>
     */
    #define SWARMLIST_TICKS_TO_INACTIVE (argos::UInt8)10

    /**
     * The number of timesteps before each swarmlist tick.
     * 1 means "tick every timestep". This value has the effect of a
     * multiplier to SWARMLIST_TICKS_TO_INACTIVE.
     * @note <b>Max value: 65535</b>
     * @see SWARMLIST_TICKS_TO_INACTIVE
     */
    #define STEPS_PER_TICK (argos::UInt16)10

    /**
     * The maximum number of ticks a Lamport clock sould be above an old
     * Lamport clock so that the new Lamport clock is considered as 'new'.
     */
    #define LAMPORT_THRESHOLD 50

    // ===============================
    // =      TYPE DEFINITIONS       =
    // ===============================

    /**
     * Type for the ID of a robot.
     */
    typedef argos::UInt32 RobotId;

    /**
     * Lamport clock type.
     */
    template <class UIntType>
    class Lamport {
    public:
        inline
        Lamport(UIntType lamport) { m_lamport = lamport; }
        inline
        operator UIntType&()       { return m_lamport; }
        inline
        operator UIntType () const { return m_lamport; }

        bool isNewerThan(Lamport<UIntType> other) {
            // This function uses a circular Lamport model (0 == 255 + 1).
            // A Lamport clock is 'newer' than an old Lamport clock if its value
            // is less than 'LAMPORT_THRESHOLD' ticks ahead of the old clock.

            bool overflow = (static_cast<UIntType>(-1) - other < LAMPORT_THRESHOLD);
            if (overflow) {
                return m_lamport > other.m_lamport ||
                       m_lamport <= static_cast<UIntType>(other.m_lamport + LAMPORT_THRESHOLD);
            }
            else {
                return m_lamport >  other.m_lamport &&
                       m_lamport <= other.m_lamport + LAMPORT_THRESHOLD;
            }
        }

    private:
        UIntType m_lamport;
    };

    typedef Lamport<argos::UInt8 > Lamport8;
    typedef Lamport<argos::UInt16> Lamport16;
    typedef Lamport<argos::UInt32> Lamport32;

    // ==============================
    // =     UTILITY FUNCTIONS      =
    // ==============================

    /**
     * Returns the size of a packet (in bytes).
     */
    argos::UInt16 getPacketSize();
    
    /**
     * Returns the probability (between 0.0 and 1.0) of packet drop.
     */
    argos::Real getPacketDropProb();

}

#endif // !INCLUDE_H