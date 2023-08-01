#ifndef DCI_SINK_CLIENT_BEAMFORMING_H
#define DCI_SINK_CLIENT_BEAMFORMING_H

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

#include "ngscope/hdr/dciLib/dci_sink_ring_buffer.h"
#include "ngscope/hdr/dciLib/load_config_remote.h"

#define N_ELEMENTS 48
#define INIT_STEP_SIZE 90
#define STEP_SIZE_MIN 22.5
#define STEP_SIZE_MAX 135
#define FEEDBACK_WINDOW 20
#define N_EXPLORE 4
#define EXPLORE_STEP_SIZE 22.5
#define N_STEP_ADJUST_THRESH 10
#define DATA_RATE_MAX 1200 // bits per ms per RB, (tbso+tbs1 = bits per ms)
#define N_STEP_ADJUST_MAX 20


typedef struct {
    int     tbs0[FEEDBACK_WINDOW+1];
    int     tbs1[FEEDBACK_WINDOW+1];
    int     rv0[FEEDBACK_WINDOW+1];
    int     rv1[FEEDBACK_WINDOW+1];
    int     prb[FEEDBACK_WINDOW+1];
    int     count; // tracks how many entries are in the arrays
    uint16_t rnti;
    int     sum_prb;
    double  rate;
} dci_perRNTI_t;

typedef struct {
    int tbs0[NOF_LOG_DCI];
    int tbs1[NOF_LOG_DCI];
    int rv0[NOF_LOG_DCI];
    int rv1[NOF_LOG_DCI];
    int prb[NOF_LOG_DCI];    
    dci_perRNTI_t rnti_data[MAX_NOF_UE_PERCELL];
} dci_perRNTI_wrapper_t;


typedef struct{
    int8_t   cell_idx;
	uint16_t prevtti;
    uint16_t prevheader;

    float step_size;
	double max_rate;
    double phase_array[N_ELEMENTS];
}ngscope_beamforming_state_t;

void* dci_beamform_client_thread(void* p);
double random_phase(double min, double max);
void ngscope_beamforming_state_init(ngscope_beamforming_state_t* q, int cell_idx);
void write_phase_to_Arduino(ngscope_dci_sink_CA_t *q, ngscope_beamforming_state_t* bf_st);

#endif