#include <sys/mman.h> // shm_open, shm_unlink
#include <sys/stat.h> // S_IRUSR, S_IWUSR (in some configurations)
#include <fcntl.h>    // O_RDWR, O_CREAT, S_IRUSR, S_IWUSR
#include <errno.h>    // errno
#include <unistd.h>   // ftruncate
#include <malloc.h>   // malloc
#include <string.h>   // sprintf
#include <signal.h>   // SIGKILL, sigaction, struct sigaction, __sighandler_t

#include "include/exp_data.h" // exp_data_t
#include "include.h"

static char* exp_data_name;
int exp_data_fd;
exp_data_t* exp_data;

uint8_t* num_msgs_in_timestep;

void (*kilolib_sigterm_handler)(int) = 0;

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

exp_data_t exp_d;

void open_resources() {
    // Invent a memory object name by using the kilo_uid.
    exp_data_name = (char*)malloc(100);
    sprintf(exp_data_name, "exp_data%" PRIu32, kilo_uid);
    exp_data_name = (char*)realloc(exp_data_name, strlen(exp_data_name)+1); // Shrink to size

    // Create/get experiment data memory
    exp_data_fd = shm_open(exp_data_name, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    // Increase shared memory size to fit the size of the experiment data.
    // Also sets everything to zero.
    int trunc_ret = ftruncate(exp_data_fd, sizeof(exp_data_t));

    if (trunc_ret >= 0) {
        // Create virtual memory map to experiment data and get its pointer
        exp_data = mmap(0, sizeof(exp_data_t), PROT_READ | PROT_WRITE, MAP_SHARED, exp_data_fd, 0);

        // Set 'extern' pointers.
        swarmlist = &exp_data->swarmlist;
        num_msgs_in_timestep = &exp_data->num_msgs_in_timestep;
    }
    else {
        fprintf(stderr, "ERROR for kilobot #%d: %s", kilo_uid, strerror(errno));
        swarmlist = 0;
        num_msgs_in_timestep = 0;
    }

    // Set our own SIGTERM handler and get the old one.
    kilolib_sigterm_handler = signal(SIGTERM, exp_sigterm_handler);
}
