/**
 * @file exp_data.h
 * @brief Definition of the exp_data_t structure, which contains the data
 * shared between the controller and the behavior processes.
 */

#ifndef EXP_DATA_H
#define EXP_DATA_H

#include <inttypes.h>

#include "behaviors/swarmlist_list_based.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * Data shared between the controller and the behavior processes.
 */
typedef struct {
    swarmlist_t swarmlist; ///< The swarmlist.
    uint64_t num_msgs_tx;  ///< Number of messages sent since the beginning of the experiment.
    uint64_t num_msgs_rx;  ///< Number of messages received since the beginning of the experiment.
    char csv_path[];       ///< Path to csv file.
} exp_data_t;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !EXP_DATA_H