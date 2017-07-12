/**
 * @file swarmlist.cpp
 * @brief Implements the swarmlist functions as two std::unordered_map objects.
 * @details The only reason why this is a C++ file is because we
 * want to use std::unordered_map objects. This is not meant to be a "true"
 * C++ file.
 */

#include <algorithm>
#include <vector>
#include <unordered_map>
#include <cinttypes>

#include "swarmlist_list_based.h"
#include "swarmlist.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

struct entry_index_t {
    swarmlist_entry_t entry;
    uint32_t index;
};

std::vector<swarmlist_entry_t> data_by_index;             ///< Index    => Entry
std::unordered_map<robot_id_t, entry_index_t> data_by_id; ///< Robot ID => Entry, Index

/****************************************/
/****************************************/

void swarmlist_construct() {
    swarmlist->size = 0;
    swarmlist->num_active = 0;
    swarmlist->next_to_send = 0;
    data_by_id.clear();
    data_by_index.clear();
}

/****************************************/
/****************************************/

void swarmlist_set(const swarmlist_entry_t* entry) {
    try {
        // Get existing entry.
        entry_index_t ei = data_by_id.at(entry->robot);

        // Didn't throw an exception ; the entry already existed.
        data_by_index[ei.index] = *entry;
        data_by_id[entry->robot] = {.entry = *entry, .index = ei.index};
    }
    catch (std::out_of_range& e) {
        // Throwed an exception ; entry doesn't exist yet.
        data_by_index.push_back(*entry);
        data_by_id[entry->robot] = {.entry = *entry, .index = (uint32_t)data_by_index.size() - 1};
    }
}

/****************************************/
/****************************************/
 
uint8_t swarmlist_get(robot_id_t key, swarmlist_entry_t const ** value) {    
    try {
        *value = &data_by_id.at(key).entry;
        return true;
    }
    catch(std::out_of_range& e) {
        return false;
    }
}

/****************************************/
/****************************************/


void swarmlist_update(robot_id_t robot,
                      uint8_t swarm_mask,
                      lamport8_t lamport) {

    // Does the entry already exist?
    uint8_t should_update;
    const swarmlist_entry_t* E = 0;
    uint8_t existed = swarmlist_get(robot, &E);
    if (existed) {
        // Yes.
        lamport8_t old_lamport = E->lamport;

        // Is entry active?
        if (swarmlist_entry_isactive(E)) {
            // Yes ; use circular lamport clock model to determine
            // whether the entry should be updated.
            should_update = lamport_isnewer(lamport, old_lamport);
        }
        else {
            // No ; the entry is newer if the lamport clocks are different.
            should_update = (lamport != old_lamport);
            if (should_update) {
                ++swarmlist->num_active;
            }
        }
    }
    else {
        // No ; it's a new entry.
        should_update = 1;
        ++swarmlist->size;
        ++swarmlist->num_active;
    }

    if (should_update) {
        // printf("%s(%d): %s\n", __FILE__, __LINE__, __PRETTY_FUNCTION__);
        swarmlist_entry_t entry = {
            .robot            = robot,
            .swarm_mask       = swarm_mask,
            .lamport          = lamport,
            .time_to_inactive = SWARMLIST_TICKS_TO_INACTIVE
        };
        swarmlist_set(&entry);
    }
}

/****************************************/
/****************************************/

swarmlist_entry_t swarmlist_get_next() {
    return data_by_index.at(swarmlist->next_to_send);
}

/****************************************/
/****************************************/

#ifdef SWARMLIST_REMOVE_OLD_ENTRIES

void swarmlist_tick() {
    for (uint8_t i = 0; i < swarmlist->size; ++i) {
       // Deal with entries in inactive mode
       swarmlist_entry_t entry_cpy = data_by_index[i];
        if (swarmlist_entry_isactive(&entry_cpy)) {
            --entry_cpy.time_to_inactive;

            if (!swarmlist_entry_isactive(&entry_cpy)) {
                entry_cpy.time_to_removal = SWARMLIST_TICKS_TO_REMOVAL;
                --swarmlist->num_active;
            }
            swarmlist_set(&entry_cpy);
        }
        else {
            // Deal with old inactive entries
            if (!swarmlist_entry_shouldremove(&entry_cpy)) {
                --entry_cpy.time_to_removal;
                swarmlist_set(&entry_cpy);
            }
            else {
                // Removal

                cli(); // No interrups ; don't accept messages during removal.
                // Erase from unordered_map
                data_by_id.erase(entry_cpy.robot);
                // Erase from vector (replace deleted entry by the last entry).
                data_by_id[data_by_index[swarmlist->size-1].robot].index = i;
                data_by_index[i] = data_by_index[swarmlist->size-1];
                data_by_index.pop_back();
                --swarmlist->size;
                sei();
            }
        }
    }
}

#else // SWARMLIST_REMOVE_OLD_ENTRIES

void swarmlist_tick() {
    for (uint8_t i = 0; i < swarmlist->size; ++i) {
       // Deal with entries in inactive mode
        if (swarmlist_entry_isactive(&data_by_index[i])) {
            swarmlist_entry_t entry = data_by_index[i];
            --entry.time_to_inactive;
            swarmlist_set(&entry);
        }
    }
}

#endif // SWARMLIST_REMOVE_OLD_ENTRIES

void swarmlist_foreach(void (*elem_funp)(const swarmlist_entry_t * entry, void* params), void* params) {
    for (auto it = data_by_index.begin(); it < data_by_index.end(); ++it) {
        elem_funp(&*it, params);
    }
}

#ifdef __cplusplus
}
#endif // __cplusplus
