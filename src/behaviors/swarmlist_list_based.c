/**
 * @file swarmlist_list_based.c
 * @brief Test of a "List-based" swarm-list strategy.
 * The difference with swarmlist_list_based.c is that robot IDs
 * are 32-bit values instead of 8-bit. We need more than 256 robots
 * for the simulation.
 */

#include <kilolib.h>
#include <message_crc.h>
#include <stdio.h>

#include "swarmlist_list_based.h"

// ===============================
// =  GENERAL GLOBAL VARIABLES   =
// ===============================

swarmlist_t* swarmlist   = 0;
uint8_t local_swarm_mask = 0x01;
lamport8_t local_lamport = 0;

// ===============================
// =         TX MESSAGES         =
// ===============================

message_t msg_tx;
volatile uint8_t should_send_tx = 0;
volatile uint8_t msg_tx_sent = 0;

#include <signal.h>
void preloop();
void postloop();
#define msg_tx_busy_wait()                                              \
    while(!msg_tx_sent) {                                               \
        /* Suspend process, waiting for ARGoS controller's resume signal */ \
        raise(SIGTSTP);                                                 \
        /* Update state */                                              \
        preloop();                                                      \
        /* Send messages */                                             \
        postloop();                                                     \
    }

message_t* which_msg_tx() {
    if (should_send_tx) {
        return &msg_tx;
    }
    return 0;
}

/**
 * Called when a message has been sent.
 */
void msg_tx_success() {
    // printf("#%d: sent #%d.\n", kilo_uid, *(robot_id_t*)&msg_tx.data[SWARM_ENTRY_SZ*0+ROBOT_ID_POS]);
    should_send_tx = 0;
    msg_tx_sent = 1;
    ++*num_msgs_in_timestep;
}


/**
 * Number of swarmlist entries in a swarm message.
 */
#define SWARM_ENTRY_SZ (                                                \
        sizeof(robot_id_t) +                                            \
        sizeof(uint8_t) +                                               \
        sizeof(lamport8_t)                                              \
    )

/**
 * Number of entries per swarm message.
 */
#define NUM_ENTRIES_PER_SWARM_MSG (9/SWARM_ENTRY_SZ)

/**
 * Offset, inside a message, of the robot ID of the first
 * entry of the message.
 */
#define ROBOT_ID_POS 0

/**
 * Offset, inside a message, of the swarm mask of the first
 * entry of the message.
 */
#define SWARM_MASK_POS (ROBOT_ID_POS + sizeof(robot_id_t))

/**
 * Offset, inside a message, of the Lamport clock of the first
 * entry of the message.
 */
#define LAMPORT_POS (SWARM_MASK_POS + sizeof(uint8_t))

void send_next_swarm_chunk() {
    if (swarmlist->size != 0) {
        // Send several swarm messages
        const uint8_t NUM_MSGS = (swarmlist->size / NUM_ENTRIES_PER_SWARM_MSG + 1 >= SWARM_CHUNK_AMOUNT) ?
                                 (SWARM_CHUNK_AMOUNT) :
                                 (swarmlist->size / NUM_ENTRIES_PER_SWARM_MSG + 1);

        for (uint8_t i = 0; i < NUM_MSGS; ++i) {
            // Send a swarm message
            msg_tx.type = SWARM;
            for (uint8_t j = 0; j < NUM_ENTRIES_PER_SWARM_MSG; ++j) {
                // Increment our own Lamport clock so that others are aware
                // that we still exist.
                swarmlist_entry_t entry = swarmlist_get_next();

                // Don't send the info of inactive robots.
                // A robot always has at least its own entry active,
                // so we don't risk falling in infinite loops.
                while (!swarmlist_entry_isactive(&entry)) {
                    swarmlist_next();
                    entry = swarmlist_get_next();
                }

                // Increment Lamport clock when sending our own info.
                if (entry.robot == kilo_uid) {
                    ++local_lamport;
                    entry.lamport = local_lamport;
                }

                // Append the next entry's data
                *(robot_id_t*)&msg_tx.data[SWARM_ENTRY_SZ*j+ROBOT_ID_POS] = entry.robot;
                msg_tx.data[SWARM_ENTRY_SZ*j+SWARM_MASK_POS] = entry.swarm_mask;
                msg_tx.data[SWARM_ENTRY_SZ*j+LAMPORT_POS]    = entry.lamport;

                // Go to next robot (if we only have one or two robots, we'll
                // send the same robot info several times, but that's OK).
                swarmlist_next();
            }
            msg_tx.crc = message_crc(&msg_tx);
            should_send_tx = 1;
            // printf("#%d: sending #%d.\n", kilo_uid, *(robot_id_t*)&msg_tx.data[SWARM_ENTRY_SZ*0+ROBOT_ID_POS]);
            // msg_tx_busy_wait();
        }
    }
    else {
        // printf("Current robot (ID %d): Own swarmlist entry \n"
        //        "was deleted. Creating it again.\n");
        swarmlist_update(kilo_uid, local_swarm_mask, local_lamport);
    }
}

// ===============================
// =         RX MESSAGES         =
// ===============================

void process_msg_rx(message_t* msg_rx, distance_measurement_t* d) {
    switch(msg_rx->type) {
        case SWARM: {
            process_msg_rx_swarm(msg_rx); break;
        }
        default: LED(3,0,3); delay(65535); break;
    }
}

void process_msg_rx_swarm(message_t* msg_rx) {
    for (uint8_t j = 0; j < NUM_ENTRIES_PER_SWARM_MSG; ++j) {
        robot_id_t robot = *(robot_id_t*)&msg_rx->data[SWARM_ENTRY_SZ*j+ROBOT_ID_POS];
        // We have the most updated info about ourself ;
        // don't update our info.
        if (robot != kilo_uid) {
            uint8_t swarm_mask  = msg_rx->data[SWARM_ENTRY_SZ*j+SWARM_MASK_POS];
            lamport8_t lamport  = msg_rx->data[SWARM_ENTRY_SZ*j+LAMPORT_POS];
            swarmlist_update(robot, swarm_mask, lamport);
        }
    }
}

// ===============================
// =       OTHER FUNCTIONS       =
// ===============================

uint8_t lamport_isnewer(lamport8_t lamport, lamport8_t old_lamport) {
    // This function uses a circular Lamport model (0 == 255 + 1).
    // A Lamport clock is 'newer' than an old Lamport clock if its value
    // is less than 'LAMPORT_THRESHOLD' ticks ahead of the old clock.

    uint8_t lamport_overflow = (UINT8_MAX - old_lamport < LAMPORT_THRESHOLD);
    if (lamport_overflow) {
        return lamport > old_lamport ||
               lamport <= (uint8_t)(old_lamport + LAMPORT_THRESHOLD);
    }
    else {
        return lamport >  old_lamport &&
               lamport <= old_lamport + LAMPORT_THRESHOLD;
    }
}

// ===============================
// =  SETUP AND LOOP FUNCTIONS   =
// ===============================

void setup() {
    rand_seed(rand_hard());
    open_resources();
    swarmlist_construct();
    swarmlist_update(kilo_uid, local_swarm_mask, local_lamport);
}

uint32_t n_loops = 0;
uint16_t loops_till_tick = 1;
uint16_t loops_till_next_chunk = 1;
void loop() {

    if (loops_till_next_chunk > 0) {
        --loops_till_next_chunk;
    }

    if (loops_till_next_chunk == 0 && !should_send_tx) {
        loops_till_next_chunk = (rand_soft() >> 7) + SWARM_CHUNK_DELAY;
        send_next_swarm_chunk();
    }

    switch(swarmlist_num_active() % 8) {
        case 0: {
            LED(0,0,0);
            break;
        }
        case 1: {
            LED(1,0,0);
            break;
        }
        case 2: {
            LED(1,1,0);
            break;
        }
        case 3: {
            LED(0,1,0);
            break;
        }
        case 4: {
            LED(0,2,1);
            break;
        }
        case 5: {
            LED(0,0,1);
            break;
        }
        case 6: {
            LED(1,0,1);
            break;
        }
        case 7: {
            LED(1,1,1);
            break;
        }
        default: ;
    }

    --loops_till_tick;
    if (loops_till_tick == 0) {
        loops_till_tick = LOOPS_PER_TICK;
        swarmlist_tick();
    }

    ++n_loops;
    if (kilo_uid == 0 && n_loops % 1000 == 0) {
        printf("Robot #%d\t; Timesteps: %d\t; swarmlist size: %d\n", kilo_uid, n_loops, swarmlist->size);
    }

}

// ===============================
// =        MAIN FUNCTION        =
// ===============================

int main() {
    kilo_init();

    kilo_message_tx = which_msg_tx;
    kilo_message_tx_success = msg_tx_success;
    kilo_message_rx = process_msg_rx;

    kilo_start(setup, loop);

    return 0;
}