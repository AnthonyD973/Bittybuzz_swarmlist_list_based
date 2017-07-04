#include <sys/mman.h> // shm_open, shm_unlink
#include <fcntl.h>    // O_RDWR, O_CREAT, S_IRUSR, S_IWUSR
#include <unistd.h>   // ftruncate
#include <malloc.h>   // malloc
#include <string.h>   // sprintf
#include <signal.h>   // SIGKILL, sigaction, struct sigaction, __sighandler_t

#include "include/exp_data.h" // exp_data_t
#include "include.h"

static char* exp_data_name;
int exp_data_fd;
exp_data_t* exp_data;

__sighandler_t kilolib_sigkill_handler;

/**
 * Closes all the open files and memory maps.
 */
void sigkill_hander(int i) {
    if (kilolib_sigkill_handler) {
        kilolib_sigkill_handler(i);
    }
    munmap(exp_data, sizeof(*exp_data));
    close(exp_data_fd);
    shm_unlink(exp_data_name);
}

void open_resources() {
    // Invent a memory object name by using the kilo_uid.
    exp_data_name = (char*)malloc(100);
    sprintf(exp_data_name, "exp_data%" PRIu32, kilo_uid);
    exp_data_name = (char*)realloc(exp_data_name, strlen(exp_data_name)+1); // Shrink to size

    // Create/get experiment data memory
    exp_data_fd = shm_open(exp_data_name, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    // Increase shared memory size to fit the size of the experiment data
    int trunc_ret = ftruncate(exp_data_fd, sizeof(exp_data_t));

    if (trunc_ret >= 0) {
        // Create virtual memory map to experiment data and get its pointer
        exp_data = mmap(0, sizeof(exp_data_t), PROT_READ | PROT_WRITE, MAP_SHARED, exp_data_fd, 0);

        // Set swarmlist pointer
        swarmlist = &exp_data->swarmlist;
    }
    else {
        swarmlist = 0;
    }

    // Get current SIGKILL handler.
    struct sigaction oldact;
    sigaction(SIGKILL, 0, &oldact);
    kilolib_sigkill_handler = oldact.sa_sigaction;
    signal(SIGTERM, sigkill_hander);
}
