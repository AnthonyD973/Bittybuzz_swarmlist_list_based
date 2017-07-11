/**
 * @file swarmlist.h
 * @brief Implements the swarmlist functions as two std::unordered_map objects.
 */

#ifndef SWARMLIST_H
#define SWARMLIST_H

#include "include.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * The data that we know about other robots.
 * @note We assume there is only one instance of this structure.
 */
typedef struct PACKED {
    uint32_t size;         ///< Number of entries.
    uint32_t num_active;   ///< Number of active entries.
    uint32_t next_to_send; ///< The index of the next entry to send via a swarm chunk.
} swarmlist_t;

/**
 * Swarmlist single instance.
 */
extern swarmlist_t* swarmlist;

/**
 * Constructs the swarmlist.
 */
void swarmlist_construct();

/**
 * Function implemented in a C++ file. Uses a std::unordered_map to retreive the entry.
 * @param[in] key The robot ID whose entry to retrieve.
 * @param[in,out] value Buffer for the entry. Left unset if the entry isn't found.
 * @return Nonzero if the key-value pair existed, otherwise 0.
 */
uint8_t swarmlist_get(robot_id_t key, swarmlist_entry_t const ** value);

/**
 * Stores an entry.
 * @param[in] entry The entry to store.
 */
void swarmlist_set(const swarmlist_entry_t* entry);

/**
 * Updates/creates an entry in the swarm list. Uses swarmlist_get/set.
 * @param[in] robot The robot's ID whose entry to update.
 * @param[in] swarm_mask The swarms this robot is a member of.
 * @param[in] lamport The lamport clock of the message whence the call to
 * this function comes. Determines whether the entry should be updated.
 */
void swarmlist_update(robot_id_t robot,
                      uint8_t swarm_mask,
                      lamport8_t lamport);

/**
 * Function implemented in a C++ file. Passes on to the next swarm entry.
 */
ALWAYS_INLINE
void swarmlist_next() {
    ++swarmlist->next_to_send;
    if (swarmlist->next_to_send >= swarmlist->size)
        swarmlist->next_to_send = 0;
}

/**
 * Function implemented in a C++ file. Uses a std::unordered_map to
 * get the next swarmlist entry to send.
 * @return The next swarmlist entry to send. Must be freed by the caller.
 */
swarmlist_entry_t swarmlist_get_next();

/**
 * Determines the size of the swarmlist.
 */
ALWAYS_INLINE
uint8_t swarmlist_size() { return swarmlist->size; }

/**
 * Determines the number of active entries in the swarmlist.
 */
ALWAYS_INLINE
uint8_t swarmlist_num_active() { return swarmlist->num_active; }

/**
 * Removes 1 from all timers, and deals with old entries.
 */
void swarmlist_tick();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !SWARMLIST_H