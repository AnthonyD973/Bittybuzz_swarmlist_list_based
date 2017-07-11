#include <sys/mman.h> // shm_open, shm_unlink
#include <sys/stat.h> // S_IRUSR, S_IWUSR (in some configurations)
#include <fcntl.h>    // O_RDWR, O_CREAT, S_IRUSR, S_IWUSR
#include <errno.h>    // errno
#include <unistd.h>   // ftruncate
#include <malloc.h>   // malloc
#include <string.h>   // sprintf
#include <signal.h>   // SIGKILL, sigaction, struct sigaction, __sighandler_t
#include <stdint.h>   // PRIu64, ...
#include <stdlib.h>   // exit

#include "include/exp_data.h" // exp_data_t
#include "include.h"

static char* exp_data_name;
int exp_data_fd;
exp_data_t* exp_data;
void (*kilolib_sigterm_handler)(int) = 0;

FILE* kilobot_csv;
uint64_t* num_msgs_tx;
uint64_t* num_msgs_rx;

/**
 * Closes all the open files and memory maps.
 */
void exp_sigterm_handler(int i) {
    if (kilolib_sigterm_handler) {
        kilolib_sigterm_handler(i);
    }
    munmap(exp_data, sizeof(*exp_data));
    close(exp_data_fd);
    shm_unlink(exp_data_name);
    free(exp_data_name);
}

void open_resources() {
    // Invent a memory object name by using the kilo_uid.
    exp_data_name = (char*)malloc(100);
    sprintf(exp_data_name, "/exp_data%" PRIu32, kilo_uid);
    exp_data_name = (char*)realloc(exp_data_name, strlen(exp_data_name)+1); // Shrink to size

    // Get experiment data memory.
    exp_data_fd = shm_open(exp_data_name, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);

    // Create virtual memory map to experiment data and get its pointer
    exp_data = mmap(0, sizeof(exp_data_t), PROT_READ | PROT_WRITE, MAP_SHARED, exp_data_fd, 0);
    // Set 'extern' pointers.
    swarmlist = &exp_data->swarmlist;
    num_msgs_tx = &exp_data->num_msgs_tx;
    num_msgs_rx = &exp_data->num_msgs_rx;

    // Open csv file, whose path was already written by ARGoS.
    kilobot_csv = fopen(exp_data->csv_path, "w");
    if (!kilobot_csv) {
        fprintf(stderr, "ERROR for kilobot #%d: Failed to open file \"%s\": %s\n", kilo_uid, exp_data->csv_path, strerror(errno));
        exit(-1);
    }

    // Remove the CSV file path from the shared memory ; we don't need it anymore.
    // Also resets everything to zero.
    int trunc_ret = ftruncate(exp_data_fd, sizeof(exp_data_t));

    if (trunc_ret < 0) {
        fprintf(stderr, "ERROR for kilobot #%d: %s", kilo_uid, strerror(errno));
        swarmlist = 0;
        num_msgs_tx = 0;
    }

    // Set our own SIGTERM handler and get the old one.
    kilolib_sigterm_handler = signal(SIGTERM, exp_sigterm_handler);

    fprintf(kilobot_csv,
            "ID;"
            "Number of messages sent,"
            "Number of messages received,"
            "Swarmlist size,"
            "Swarmlist num active,"
            "Swarmlist next to send,"
            "\"Swarmlist data (robot ID;lamport;time to inactive)\"\n");
    fflush(kilobot_csv);
}


void log_elem_fun(const swarmlist_entry_t* entry, void* params) {
    fprintf(kilobot_csv,
        "(%d,%d,%d);",
        entry->robot,
        entry->lamport,
        entry->time_to_inactive);
}

void log_status() {
    fprintf(kilobot_csv,
        "%d,%"PRIu64",%"PRIu64",%d,%d,%d,\"",
        kilo_uid, *num_msgs_tx, *num_msgs_rx, swarmlist->size, swarmlist->num_active, swarmlist->next_to_send);

    swarmlist_foreach(log_elem_fun, 0);

    fprintf(kilobot_csv, "\"\n");
    fflush(kilobot_csv);
}
