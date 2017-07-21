/**
 * @file swarmlist.h
 * @brief Definition of the Swarmlist class.
 */

#ifndef SWARMLIST_H
#define SWARMLIST_H

#include <stdexcept> // std::domain_error
#include <unordered_map>
#include <string>

#include "include.h"

namespace swlexp {

    /**
     * The data that we know about other robots.
     */
    class Swarmlist {

    // ==============================
    // =       NESTED SYMBOLS       =
    // ==============================

    public:

        /**
         * Entry of the Swarmlist.
         */
        class Entry {

        public:

            Entry(RobotId robot, argos::UInt8 swarmMask, swlexp::Lamport8 lamport);

            inline
            RobotId getRobotId() const { return m_robot; }
            inline
            argos::UInt8 getSwarmMask() const { return m_swarmMask; }
            inline
            swlexp::Lamport8 getLamport() const { return m_lamport; }
            inline
            argos::UInt8 getTimeToInactive() const { return m_timeToInactive; }

            /**
             * Determines whether the entry is active.
             * The entry of the current robot is always active.
             * @param[in] id ID of the current robot.
             * @return Whether the entry is active.
             */
            inline
            bool isActive(RobotId id) const { return m_timeToInactive != 0 || m_robot == id; }

            /**
             * Removes 1 from the timer.
             */
            void tick() { --m_timeToInactive; }

            /**
             * Resets the entry's timer.
             */
            void resetTimer() { m_timeToInactive = SWARMLIST_TICKS_TO_INACTIVE; }

            /**
             * Increments the entry's lamport clock.
             */
            void incrementLamport() { ++m_lamport; }

        private:
            RobotId m_robot;               ///< Robot ID this entry is for.
            argos::UInt8 m_swarmMask;      ///< Data that we wish to share.
            swlexp::Lamport8 m_lamport;    ///< Time at which the entry was last updated.
            argos::UInt8 m_timeToInactive; ///< Number of swarmlist ticks until we consider this robot to be inactive.
        };

    // ==============================
    // =          METHODS           =
    // ==============================

    public:

        Swarmlist();

        ~Swarmlist();

        inline
        argos::UInt32 getSize() const { return m_data.size(); }

        inline
        argos::UInt32 getNumActive() const { return m_numActive; }

        inline
        Entry getNext(RobotId id) {
            Entry& e = m_data[m_next];
            if (e.getRobotId() == id)
                e.incrementLamport();
            return e;
        }

        /**
         * Gets an entry of the swarmlist given its robot ID.
         * @throw std::out_of_range The ID is not found.
         * @param[in] robot The robot ID whose data to fetch.
         * @return The entry of the swarmlist corresponding to the passed ID.
         */
        const Entry& get(RobotId robot) const;

        /**
         * Sets which robot owns this swarmlist.
         * @param[in] id The robot owning this swarmlist.
         */
        inline
        void setOwnerId(RobotId id) { m_id = id; }


        /**
         * Sets the entry inside the swarmlist, and resets the entry's timer.
         * @param[in] robot The robot ID this entry is for.
         * @param[in] swarmMask The payload data.
         * @param[in] lamport The time at which this entry was created by
         * 'robot'.
         */
        void update(RobotId robot, argos::UInt8 swarmMask, swlexp::Lamport8 lamport);

        /**
         * Removes 1 from all timers, and deals with old entries.
         */
        void tick();

        /**
         * Changes the next robot whose data to send.
         */
        inline
        void next() {
            ++m_next;
            if (m_next >= m_data.size())
                m_next = 0;
        }

        /**
         * Composes a string consisting of a set of
         * "(ID,lamport since update, time to inactive)"
         * entries.
         * @param[in] elemDelim Delimiter bewteen each entry's element.
         * @param[in] entryDelim Delimiter bewteen each entry.
         * @return The serialized data.
         */
        std::string serializeData(char elemDelim, char entryDelim) const;

    private:

        /**
         * Adds/Modifies an entry of the swarmlist.
         */
        void _set(const Entry& entry);

    // ==============================
    // =       STATIC METHODS       =
    // ==============================

    public:

        /**
         * Determines the total number of swarmlist entries in the entire swarm.
         */
        inline static
        argos::UInt64 getTotalNumActive() { return c_totalNumActive; }


        /**
         * Determines whether existing entries should become inactive after a while.
         */
        inline static
        bool getEntriesShouldBecomeInactive() { return c_entriesShouldBecomeInactive; }

        /**
         * Sets whether existing entries should become inactive after a while.
         */
        inline static
        void setEntriesShouldBecomeInactive(bool shouldBecomeInactive) { c_entriesShouldBecomeInactive = shouldBecomeInactive; }

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

        RobotId m_id;                                           ///< ID of the robot whose swarmlist this is.
        std::vector<Entry> m_data;                              ///< Index    => Entry in O(1)
        std::unordered_map<RobotId, argos::UInt32> m_idToIndex; ///< Robot ID => Index in O(1)
        argos::UInt32 m_numActive;                              ///< Number of active entries.
        argos::UInt32 m_next;                                   ///< The index of the next entry to send via a swarm chunk.

    // ==============================
    // =       STATIC MEMBERS       =
    // ==============================

    private:

        static bool c_entriesShouldBecomeInactive; ///< Whether existing entrie should become inactive after a while.
        static argos::UInt64 c_totalNumActive;     ///< The sum, over all robots, of the number of active entries.

    };
}

#endif // !SWARMLIST_H