#ifndef _CONFIG_REMOTE_H_
#define _CONFIG_REMOTE_H_

#include <stdio.h>
#include "task_scheduler.h"

#define MAX_NOF_UE_PERCELL 4

typedef struct{
    int        nof_rnti;
    int        rnti[MAX_NOF_UE_PERCELL];
}cellRNTI_config_t;

typedef struct{
    int        nof_cell;
    int        enable_fixed_rnti;
    cellRNTI_config_t     cellRNTI[MAX_NOF_RF_DEV]; // MAX_NOF_RF_DEV = 4, is also the max number of cells
}ngscope_beamforming_config_t;

int ngscope_beamforming_read_config(ngscope_beamforming_config_t* config);
#endif
