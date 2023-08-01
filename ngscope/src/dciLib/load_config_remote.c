#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <libconfig.h>
#include "ngscope/hdr/dciLib/load_config_remote.h"

int compar(const void* a,const void* b)
{
    return (*(long long*)a - *(long long*)b);
}

bool containsDuplicate(long long* nums, int numsSize){
    int i,j;
    qsort(nums, numsSize,sizeof(long long),compar);
    for(i = 0,j = 1;j < numsSize;i++,j++)
    {
        if(nums[i] == nums[j])
        {
            return true;
        }
    }
    return false;
}

int ngscope_beamforming_read_config(ngscope_beamforming_config_t* config){
    config_t* cfg = (config_t *)malloc(sizeof(config_t));
    config_init(cfg);

    printf("read remote config!\n");
    if(! config_read_file(cfg, "./config_remote.cfg")){
        fprintf(stderr, "%s:%d - %s\n", config_error_file(cfg),
                config_error_line(cfg), config_error_text(cfg));
        config_destroy(cfg);
        return(EXIT_FAILURE);
    }

    if(! config_lookup_int(cfg, "nof_cell", &config->nof_cell)){
        printf("ERROR: reading nof_cell\n");
    }
    printf("read nof cell:%d\n", config->nof_cell);    

    if(! config_lookup_bool(cfg, "enable_fixed_rnti", &config->enable_fixed_rnti)){
        printf("ERROR: reading enable_fixed_rnti\n");
    }
    printf("read enable_fixed_rnti:%d\n", config->enable_fixed_rnti);    

    for(int i=0;i<config->nof_cell;i++){
        char name[50];
        sprintf(name, "cellRNTI_config%d.nof_rnti",i);
        if(! config_lookup_int(cfg, name, &config->cellRNTI[i].nof_rnti)){
            printf("ERROR: reading nof_rnti\n");
        }else{
            printf("cell id: %d, nof_rnti:%d ", i, config->cellRNTI[i].nof_rnti);
        }

        for(int j=0;j<config->cellRNTI[i].nof_rnti;j++){
            sprintf(name, "cellRNTI_config%d.rnti%d",i,j);
            if(! config_lookup_int(cfg, name, &config->cellRNTI[i].rnti[j])){
                printf("ERROR: reading rnti\n");
            }else{
                printf("rnti%d:%d ", j, config->cellRNTI[i].rnti[j]);
            }
        }
    }

    return 0;
}