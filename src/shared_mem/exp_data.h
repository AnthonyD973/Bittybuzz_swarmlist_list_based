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
    uint8_t meta_info;     ///< Info such as commands from/to ARGoS or whether the subprocess is still alive.
    swarmlist_t swarmlist; ///< The swarmlist.
    uint64_t num_msgs_tx;  ///< Number of messages sent since the beginning of the experiment.
    uint64_t num_msgs_rx;  ///< Number of messages received since the beginning of the experiment.
    uint64_t time;         ///< Elapsed time (in timesteps) since the beginning of the experiment.
    char log_data[];       ///< Data to log by ARGoS.
} exp_data_t;

/**
 * In the 'meta_info' field of exp_data_t, bit which tells whether the
 * subprocess is alive.
 * The subprocess has to set this bit, and the ARGoS side checks it and
 * clears it regularly.
 */
#define EXP_DATA_META_INFO_IS_ALIVE (1 << 0)

/**
 * In the 'meta_info' field of exp_data_t, bit which tells whether the
 * subprocess should log its status to the 'log_data' subfield.
 */
#define EXP_DATA_META_INFO_SHOULD_LOG_STATUS (1 << 1)

/**
 * Gets and clears a meta-info bit.
 * @param[in,out] exp_data The experiment data of the process.
 * @param[in] which_bit The bit to get and clear.
 * @return Nonzero if the meta-info bit is set.
 */
inline static
uint8_t get_and_clear_meta_info(exp_data_t* exp_data, uint8_t which_bit) {
    uint8_t meta_info_bit = (exp_data->meta_info & which_bit);
    exp_data->meta_info &= ~which_bit;
    return meta_info_bit;
}

/**
 * Sets a meta-info bit of a process.
 * @param[in,out] exp_data The experiment data of the process.
 * @param[in] which_bit The bit to set.
 */
inline static
void set_meta_info(exp_data_t* exp_data, uint8_t which_bit) {
    exp_data->meta_info |= which_bit;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !EXP_DATA_H