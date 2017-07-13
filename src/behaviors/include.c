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
#include <stdarg.h>   // va_list, va_start
#include <stdio.h>

#include "shared_mem/exp_data.h" // exp_data_t
#include "include.h"

static char* exp_data_name;
int exp_data_fd;
exp_data_t* exp_data;
void (*kilolib_sigterm_handler)(int) = 0;

uint64_t* num_msgs_tx;
uint64_t* num_msgs_rx;

/**
 * Number of characters (apart from the null byte) that can currently
 * be written to the log.
 */
uint32_t log_data_len;

/**
 * Appends data to the experiment data log.
 * @details This function appends the data after the first encountered null
 * string.
 * @param[in] format A printf-style format for the log.
 * @param[in] ... Arguments to the format.
 */
void log_printf(const char* format, ...) {
    // Find where to write.
    char* last_alloc_char = exp_data->log_data + log_data_len;
    char* where_to_append = exp_data->log_data;
    while (where_to_append <= last_alloc_char && *where_to_append != '\0') {
        ++where_to_append;
    }
    // Did we move past the log data buffer without finding a null byte?
    if (where_to_append > last_alloc_char) {
        // Yes. The log just contains junk data.
        where_to_append = exp_data->log_data;
    }

    // Find the number of character that the log should have.
    va_list args, args_cpy;
    va_start(args, format);
    va_copy(args_cpy, args);
    size_t chars_needed = vsnprintf(NULL, 0, format, args);
    va_end(args);

    // Resize log if necessary.
    uint32_t log_data_len_needed = (uint32_t)(where_to_append - exp_data->log_data)
                                   / sizeof(char) + chars_needed;
    uint8_t must_resize = (log_data_len_needed >  log_data_len ||
                           log_data_len_needed <= log_data_len - 100);
    if (must_resize) {
        log_data_len = log_data_len_needed;
        int trunc_ret = ftruncate(exp_data_fd, sizeof(exp_data_t) + log_data_len + 1);
        if (trunc_ret < 0) {
            fprintf(stderr, "ERROR for kilobot #%d: %s", kilo_uid, strerror(errno));
            fflush(stderr);
        }
    }

    // Write to log.
    vsprintf(where_to_append, format, args_cpy);

    va_end(args_cpy);
}

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

    int trunc_ret = ftruncate(exp_data_fd, sizeof(exp_data_t) + 1);
    log_data_len = 0;

    if (trunc_ret < 0) {
        fprintf(stderr, "ERROR for kilobot #%d: %s", kilo_uid, strerror(errno));
        fflush(stderr);
        exit(-1);
    }

    // Set our own SIGTERM handler and get the old one.
    kilolib_sigterm_handler = signal(SIGTERM, exp_sigterm_handler);
}


void log_elem_fun(const swarmlist_entry_t* entry, void* params) {
    log_printf("(%d,%d,%d);",
               entry->robot,
               entry->lamport,
               entry->time_to_inactive);
}

void log_status() {
    static uint64_t former_num_msgs_tx = 0;
    static uint64_t former_num_msgs_rx = 0;

    uint64_t num_msgs_tx_since_log = *num_msgs_tx - former_num_msgs_tx;
    uint64_t num_msgs_rx_since_log = *num_msgs_rx - former_num_msgs_rx;
    former_num_msgs_tx = *num_msgs_tx;
    former_num_msgs_rx = *num_msgs_rx;

    double bw_tx = (double)(num_msgs_tx_since_log) / STEPS_TO_LOG;
    double bw_rx = (double)(num_msgs_rx_since_log) / STEPS_TO_LOG;

    log_printf("%d,%"PRIu64",%"PRIu64",%f,%"PRIu64",%f,%d,%d,\"",
        kilo_uid, exp_data->time, num_msgs_tx_since_log, bw_tx, num_msgs_rx_since_log, bw_rx, swarmlist->size, swarmlist->num_active);

    swarmlist_foreach(log_elem_fun, 0);

    log_printf("\"");
}

void do_meta_stuff() {
    set_meta_info(exp_data, EXP_DATA_META_INFO_IS_ALIVE);
    if (get_and_clear_meta_info(exp_data, EXP_DATA_META_INFO_SHOULD_LOG_STATUS)) {
        log_status();
    }
}
