#include <time.h>
#include <assert.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>

//#include "srsran/srsran.h"
#include "ngscope/hdr/dciLib/dci_sink_client.h"
#include "ngscope/hdr/dciLib/dci_sink_client_beamforming.h"
#include "ngscope/hdr/dciLib/dci_sink_sock.h"
#include "ngscope/hdr/dciLib/dci_sink_dci_recv.h"
#include "ngscope/hdr/dciLib/dci_sink_ring_buffer.h"
#include "ngscope/hdr/dciLib/load_config_remote.h"

extern bool go_exit;
extern ngscope_dci_sink_CA_t dci_CA_buf;
extern ngscope_beamforming_config_t config;

// Success and error codes
#define ACK_SUCCESS 0xFF
#define ACK_ERROR 0xAA


void ngscope_beamforming_state_init(ngscope_beamforming_state_t* bf_st, int cell){
    bf_st->cell_idx     = cell;
	bf_st->prevtti 	    = 0;
	bf_st->step_size    = INIT_STEP_SIZE;
    bf_st->max_rate     = 0;
    bf_st->prevheader   = 0;

    for(int i = 0; i < N_ELEMENTS; ++i) {
        bf_st->phase_array[i] = random_phase(0,360);
    }
    write_phase_to_Arduino(&dci_CA_buf, bf_st);

	return;
}

// Generate a random phase change between min and max.
// We use 4-bit phase shifters, so the phase change is a multiple of 22.5 degrees.
double random_phase(double min, double max) {
    int imin = min / 22.5;
    int imax = max / 22.5;
    int irandom = rand() % (imax - imin + 1) + imin;
    return irandom * 22.5;
}

void write_phase_to_Arduino(ngscope_dci_sink_CA_t *q, ngscope_beamforming_state_t* bf_st) {
    // Set the number of phase values that each Arduino device will receive
    int phases_per_device = 6;

    // Set the number of Arduino devices
    int n_devices = 8;

    // update ring buffer states
    int cell_idx = bf_st->cell_idx;
    
    // Define the preamble based on cell_idx value
    // cell_idx = 0: 0xAAAA,
    // cell_idx = 1: 0xBBBB,
    // cell_idx = 2: 0xCCCC,
    // cell_idx = 3: 0xDDDD.
    uint16_t preamble;
    switch (cell_idx) {
        case 0:
            preamble = 0xAAAA;
            break;
        case 1:
            preamble = 0xBBBB;
            break;
        case 2:
            preamble = 0xCCCC;
            break;
        case 3:
            preamble = 0xDDDD;
            break;
        default:
            printf("Invalid cell_idx value: %d\n", cell_idx);
            return;
    }
        

    // Loop over each Arduino device
    for (int i = 0; i < n_devices; ++i) {
        // Define the device name
        char device_name[23];
        sprintf(device_name, "/dev/arduinoMKR1010_%d", i + 1);

        // Open the device
        int fd = open(device_name, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            // Handle the error if the device cannot be opened
            printf("Error %i from open: %s\n", errno, strerror(errno));
            return;
        }

        // Define the serial port parameters
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            close(fd);
            return;
        }

        // Set the input and output baud rates
        // Arduino MKR WIFI 1010 uses the Atmel SAMD21, UART-over-USB for serial communication.
        // Higher baud rates 230400, 460800, 921600 are possible, need test
        cfsetospeed(&tty, (speed_t)B460800);
        cfsetispeed(&tty, (speed_t)B460800);

        // Configure the serial port parameters
        tty.c_cflag     &=  ~PARENB; // Clear parity bit, disabling parity
        tty.c_cflag     &=  ~CSTOPB; // Clear stop field, only one stop bit used in communication
        tty.c_cflag     &=  ~CSIZE; // Clear all the size bits
        tty.c_cflag     |=  CS8; // 8 bits per byte

        tty.c_lflag     =   0; // No signaling chars, no echo, no canonical processing
        tty.c_oflag     =   0; // No remapping, no delays
        tty.c_cc[VMIN]      =   0; // Read doesn't block
        tty.c_cc[VTIME]     =   5; // 0.5 seconds read timeout

        tty.c_cflag     |=  CREAD | CLOCAL; // Enable receiver, ignore modem control lines
        tty.c_iflag     &=  ~(IXON | IXOFF | IXANY); // Disable software flow control

        tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // Disable canonical mode, and echo and other options
        tty.c_oflag     &=  ~OPOST; // Disable output processing

        // Apply the settings
        tcflush(fd, TCIFLUSH);
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            close(fd);
            return;
        }

        uint8_t ack = 0x00;
        int success = 0;

        // Get the start time
        struct timespec start, now, start_serial;
        // Set the timeout in microseconds
        const int TIMEOUT = 5000;        

        
        // Loop until success acknowledgment is received
        while(!success && !go_exit) {
            tcflush(fd, TCIFLUSH);
            
            // Get the start time
            clock_gettime(CLOCK_MONOTONIC, &start_serial); 

            // Write the preamble to the device
            if (write(fd, &preamble, sizeof(preamble)) != sizeof(preamble)) {
                // Handle the error if the preamble cannot be written
                printf("Error %i from write preamble: %s\n", errno, strerror(errno));
                close(fd);
                return;
            }

            // Loop over each phase value for the current device
            for (int j = 0; j < phases_per_device; ++j) {
                // Get the phase value from the state structure
                double phase = bf_st->phase_array[i * phases_per_device + j];
                // Convert the phase value to an index
                int phase_ind = (int)(phase / 22.5);

                // Convert the phase value to a binary string
                // Add padding 2 bits at the start and end
                char phaseByte = (char) ((phase_ind & 0b1111) << 2);
                int n = 1; // Number of bytes to write

                // Write the phase value to the device
                if (write(fd, &phaseByte, n) != n) {
                    // Handle the error if the phase value cannot be written
                    printf("Error %i from write phase: %s\n", errno, strerror(errno));
                    close(fd);
                    return;
                }
            }

            // Get the current time as the start time
            clock_gettime(CLOCK_MONOTONIC, &start);    
            // Initialize numRead
            int numRead = 0;    

            while (!go_exit) {
                // Try to read the ack
                numRead = read(fd, &ack, 1);
                // If read was successful, break the loop
                if (numRead > 0) {
                    break;
                }

                // Get the current time
                clock_gettime(CLOCK_MONOTONIC, &now);
                // If the difference between now and start is greater than the timeout, break the loop
                if ((now.tv_sec - start.tv_sec) * 1000000 + (now.tv_nsec - start.tv_nsec) / 1000 > TIMEOUT) {
                    printf("Timeout while waiting for acknowledgement\n");
                    break;
                }
            }

            if (ack == ACK_SUCCESS) {
                // Break out of the while loop
                clock_gettime(CLOCK_MONOTONIC, &now);
                // print the time elapsed in us
                printf("Arduino %d Success! Time elapsed: %ld us\n", i, (now.tv_sec - start_serial.tv_sec) * 1000000 + (now.tv_nsec - start_serial.tv_nsec) / 1000);

                success = 1;
            } else if (ack == ACK_ERROR) {
                // Retry the transmission
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("Arduino %d Fail! Time elapsed: %ld us\n", i, (now.tv_sec - start_serial.tv_sec) * 1000000 + (now.tv_nsec - start_serial.tv_nsec) / 1000);
                continue;
            } else {
                printf("Unexpected response from device: %02x\n", ack);
                close(fd);
                return;
            }
        }

        // Close the device
        close(fd);
    }

    ngscope_dci_sink_cell_t* dci_cell_buf = &q->cell_dci[cell_idx];
    // lock dci_cell_buf, update nof_logged_tti and nof_unused_dci
    pthread_mutex_lock(&dci_cell_buf->mutex);
    dci_cell_buf->nof_unused_dci = 0; 
    dci_cell_buf->nof_logged_tti = 0;
    pthread_mutex_unlock(&dci_cell_buf->mutex);

    return;
}


// Helper function to calculate rate for each RNTI
double calculate_rate_for_RNTI(dci_perRNTI_t *data) {
    // Calculate sums for tbs0, tbs1 and prb for current RNTI
    int sum_tbs0 = 0;
    int sum_tbs1 = 0;
    int sum_prb = 0;
    for(int i = 0; i < data->count; i++){
        sum_tbs0 += data->tbs0[i];
        sum_tbs1 += data->tbs1[i];
        sum_prb += data->prb[i];
    }

    data->sum_prb = sum_prb; // Save sum of prb into the structure

    // Handle case when sum_prb is zero
    if (sum_prb == 0) {
        printf("Sum of PRB for RNTI %d is zero. Rate cannot be calculated.\n", data->rnti);
        return 0;
    }

    // Calculate the rate for current RNTI
    double rate = (sum_tbs0 + sum_tbs1) / (double)sum_prb;
    data->rate = rate;  // Save rate into the structure

    return rate;
}

// Get feedback.
double get_feedback(ngscope_dci_sink_CA_t *q, ngscope_beamforming_state_t *bf_st, ngscope_beamforming_config_t *cfg) {
    int cell_idx = bf_st->cell_idx;
    ngscope_dci_sink_cell_t* dci_cell_buf = &q->cell_dci[cell_idx];

    struct timespec delay;
    delay.tv_sec = 0;
    delay.tv_nsec = 500000;  // 0.5 ms
    double new_rate = 0;
    double feedback = -1;

    dci_perRNTI_wrapper_t dci_perRNTI;
    memset(&dci_perRNTI, 0, sizeof(dci_perRNTI));
    // assign the rnti of cfg to each RNTI data 
    for(int i = 0; i < cfg->cellRNTI[cell_idx].nof_rnti; i++){
        dci_perRNTI.rnti_data[i].rnti = cfg->cellRNTI[cell_idx].rnti[i];
    }

    while (!go_exit) {
        pthread_mutex_lock(&dci_cell_buf->mutex);
        // If enough ttis have been logged.
        if (dci_cell_buf->nof_logged_tti >= FEEDBACK_WINDOW) {

            // update tti and header
            int header = (dci_cell_buf->header - 1 + NOF_LOG_DCI) % NOF_LOG_DCI;
            uint16_t curr_tti = dci_cell_buf->dci[header].tti;

            int oldest_data_index = (dci_cell_buf->header - dci_cell_buf->nof_unused_dci + NOF_LOG_DCI) % NOF_LOG_DCI;  
            if (cfg->enable_fixed_rnti) {
                // Calculating new_rate using new method: only collecting DCI from several fixed RNTIs
                for (int i = oldest_data_index; i != dci_cell_buf->header; i = (i + 1) % NOF_LOG_DCI) {
                    if (dci_cell_buf->dci[i].dl_ul_flag == 0) {
                        int rnti_index = -1;
                        for(int j = 0; j < cfg->cellRNTI[cell_idx].nof_rnti; j++){
                            if(cfg->cellRNTI[cell_idx].rnti[j] == dci_cell_buf->dci[i].rnti){
                                rnti_index = j;
                                break;
                            }
                        }
                        // if we have found the matching RNTI, then add the data to the dci_perRNTI struct
                        if (rnti_index != -1) { 
                            dci_perRNTI_t *data = &dci_perRNTI.rnti_data[rnti_index];
                            data->tbs0[data->count] = dci_cell_buf->dci[i].tbs0;
                            data->tbs1[data->count] = dci_cell_buf->dci[i].tbs1;
                            data->rv0[data->count] = dci_cell_buf->dci[i].rv0;
                            data->rv1[data->count] = dci_cell_buf->dci[i].rv1;
                            data->prb[data->count] = dci_cell_buf->dci[i].prb;
                            data->count++;
                        }
                    }
                }

                // Variables to keep track of the total sum of prbs and the total weighted rates
                double total_prb = 0;
                double total_weighted_rate = 0;

                // Go through each RNTI and calculate its rate
                for(int j = 0; j < cfg->cellRNTI[cell_idx].nof_rnti; j++){
                    dci_perRNTI_t *data = &dci_perRNTI.rnti_data[j];
                    double rate = calculate_rate_for_RNTI(data);
                    
                    // Accumulate the total sum of prbs and the total weighted rates
                    total_prb += data->sum_prb;
                    total_weighted_rate += rate * data->sum_prb;
                }

                // Finally, calculate the new_rate as the weighted average of all RNTI's rate
                new_rate = total_weighted_rate / total_prb;
                

            } else {
                // Calculating new_rate using old method: collecting DCI from all RNTIs
                int count = 0;            
                for (int i = oldest_data_index; i != dci_cell_buf->header; i = (i + 1) % NOF_LOG_DCI) {
                    if (dci_cell_buf->dci[i].dl_ul_flag == 0) {
                        dci_perRNTI.tbs0[count] = dci_cell_buf->dci[i].tbs0;
                        dci_perRNTI.tbs1[count] = dci_cell_buf->dci[i].tbs1;
                        dci_perRNTI.rv0[count] = dci_cell_buf->dci[i].rv0;
                        dci_perRNTI.rv1[count] = dci_cell_buf->dci[i].rv1;
                        dci_perRNTI.prb[count] = dci_cell_buf->dci[i].prb;
                        count++;
                    }
                }

                int sum_tbs0 = 0;
                int sum_tbs1 = 0;
                int sum_rv0 = 0;
                int sum_rv1 = 0;
                int sum_prb = 0;

                for (int i = 0; i < dci_cell_buf->nof_unused_dci; i++) {
                    sum_tbs0 += dci_perRNTI.tbs0[i];
                    sum_tbs1 += dci_perRNTI.tbs1[i];
                    sum_rv0 += dci_perRNTI.rv0[i];
                    sum_rv1 += dci_perRNTI.rv1[i];
                    sum_prb += dci_perRNTI.prb[i];
                }

                new_rate = (sum_tbs0 + sum_tbs1) / (double)sum_prb;
            }

            printf("Started calculating feedback! nof_unused_dci:%d curr_tti:%d \n", dci_cell_buf->nof_unused_dci, curr_tti);
            printf("new_rate:%f old_rate:%f \n", new_rate, bf_st->max_rate);
            if (new_rate > bf_st->max_rate) {
                bf_st->max_rate = new_rate;
                feedback = 1;
            }

            bf_st->prevtti = curr_tti;
            bf_st->prevheader = header;
            dci_cell_buf->nof_unused_dci = 0; 
            dci_cell_buf->nof_logged_tti = 0;
            pthread_mutex_unlock(&dci_cell_buf->mutex);
            return feedback;
        }
        pthread_mutex_unlock(&dci_cell_buf->mutex);

        // Wait for 0.5ms before checking again.
        nanosleep(&delay, NULL);
    }
    pthread_mutex_unlock(&dci_cell_buf->mutex);
    return feedback;
}

// Adjust step size. takes n_explore_hist as input.

void adjust_stepsize(int n_explore_hist[], int n_random_guess, ngscope_beamforming_state_t *bf_st) {
    int sum_explore = 0;
    for(int i = 0; i < n_random_guess; ++i) {
        sum_explore += n_explore_hist[i];
    }
    if(sum_explore/n_random_guess > 1 && bf_st->step_size < STEP_SIZE_MAX) {
        bf_st->step_size += 22.5;
    } else if(sum_explore/n_random_guess <= 1 && bf_st->step_size > STEP_SIZE_MIN) {
        bf_st->step_size -= 22.5;
    }

    return;
}

void *dci_beamform_client_thread(void *arg) {

    // Casting the void* pointer to an int* pointer
    int* index_ptr = (int*) arg;
    // Dereferencing the pointer to get the actual int value
    int cell_idx = *index_ptr;

    // Step 1: Initialization
    struct timespec delay;
    delay.tv_sec = 10;  // 10 s
    delay.tv_nsec = 0;  // 0 ms    

    ngscope_beamforming_state_t bf_state;
    // Initialize the ngscope_beamforming_state_t struct
    ngscope_beamforming_state_init(&bf_state, cell_idx);    

    double phase_change[N_ELEMENTS]; // The phase change of recent change.
    int n_random_guess = 0; // The number of random guesses.
    int n_step_adjust = 0; // The number of step size adjustments.
    int n_explore = 0; // The number of explores.
    int n_explore_hist[N_STEP_ADJUST_THRESH+1]; // The number of explores in the last N_STEP_ADJUST_THRESH guesses.

    // The feedback value.
    double feedback = 0;

    srand(time(NULL));  // initialize random seed

    while (!go_exit) {
        // Step 2: Random Guess
        for (int i = 0; i < N_ELEMENTS; ++i) {
            phase_change[i] = random_phase(-bf_state.step_size, bf_state.step_size);
            bf_state.phase_array[i] += phase_change[i];
            bf_state.phase_array[i] = fmod(bf_state.phase_array[i]+360, 360);
        }

        write_phase_to_Arduino(&dci_CA_buf, &bf_state);
        feedback = get_feedback(&dci_CA_buf, &bf_state, &config);
        if (feedback < 0) {
            for (int i = 0; i < N_ELEMENTS; ++i) {
                phase_change[i] *= -1;  // reverse the sign of phase changes
                bf_state.phase_array[i] += 2*phase_change[i];
                bf_state.phase_array[i] = fmod(bf_state.phase_array[i]+360, 360);
            }

            write_phase_to_Arduino(&dci_CA_buf, &bf_state);
            feedback = get_feedback(&dci_CA_buf, &bf_state, &config);
            if (feedback < 0) {
                for (int i = 0; i < N_ELEMENTS; ++i) {
                    bf_state.phase_array[i] -= phase_change[i];
                    bf_state.phase_array[i] = fmod(bf_state.phase_array[i]+360, 360);
                }
                write_phase_to_Arduino(&dci_CA_buf, &bf_state);
                printf("Feedback is still negative! n_random_guess:%d n_explore:%d \n", n_random_guess, n_explore);
                n_explore_hist[n_random_guess] = n_explore;
                n_explore = 0;                
                n_random_guess++;                         
            }
        }


        // Step 3: Explore
        if (feedback>0){
            printf("Explore started!\n");
            n_explore++;
            while (n_explore <= N_EXPLORE) {
                for (int i = 0; i < N_ELEMENTS; ++i) {
                    phase_change[i] = (phase_change[i] > 0 ? 1 : -1) * EXPLORE_STEP_SIZE;
                    bf_state.phase_array[i] += phase_change[i];
                    bf_state.phase_array[i] = fmod(bf_state.phase_array[i]+360, 360);                
                }

                write_phase_to_Arduino(&dci_CA_buf, &bf_state);
                feedback = get_feedback(&dci_CA_buf, &bf_state, &config);
                if (feedback < 0) {
                    for (int i = 0; i < N_ELEMENTS; ++i) {
                        bf_state.phase_array[i] -= phase_change[i];
                        bf_state.phase_array[i] = fmod(bf_state.phase_array[i]+360, 360);
                    }
                    write_phase_to_Arduino(&dci_CA_buf, &bf_state);
                    break;
                }

                n_explore++;
            }
            printf("Explore finished! n_random_guess:%d n_explore:%d \n", n_random_guess, n_explore);
            n_explore_hist[n_random_guess] = n_explore;
            n_explore = 0;
            n_random_guess++;
        }


        // Step 4: Step Size Adjustment
        if (n_random_guess >= N_STEP_ADJUST_THRESH) {
            // adjust_stepsize takes n_explore_hist as input
            adjust_stepsize(n_explore_hist, n_random_guess, &bf_state);
            n_step_adjust++;
            n_random_guess = 0;
        }

        // Step 5: Stop Condition
        if (bf_state.max_rate >= DATA_RATE_MAX || n_step_adjust >= N_STEP_ADJUST_MAX) {
            // Wait for 10s before running again.
            nanosleep(&delay, NULL);
            // break;
        }
    }

    return NULL;
}

