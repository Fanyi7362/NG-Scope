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

#include "srsran/common/crash_handler.h"

#include "srsran/srsran.h"
#include "ngscope/hdr/dciLib/dci_sink_client.h"
#include "ngscope/hdr/dciLib/dci_sink_client_beamforming.h"
#include "ngscope/hdr/dciLib/dci_sink_sock.h"
#include "ngscope/hdr/dciLib/dci_sink_dci_recv.h"
#include "ngscope/hdr/dciLib/dci_sink_ring_buffer.h"
#include "ngscope/hdr/dciLib/load_config_remote.h"

bool go_exit = false;
ngscope_dci_sink_CA_t dci_CA_buf;
ngscope_beamforming_config_t config;

void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  } else if (signo == SIGSEGV) {
    exit(1);
  }
}

int main(int argc, char** argv){
  srsran_debug_handle_crash(argc, argv);

  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, SIGINT);
  sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  signal(SIGINT, sig_int_handler);

  // Load the configurations
  ngscope_beamforming_read_config(&config);  

  pthread_t client_thd;
  // Create a thread to receive socket data.
  pthread_create(&client_thd, NULL, dci_sink_client_thread, NULL);  

  pthread_t beamform_thd[MAX_NOF_RF_DEV];
  int indices[MAX_NOF_RF_DEV];

  // Create a thread for each cell to perform beamforming.
  // config.nof_cell is always less than or equal to MAX_NOF_RF_DEV.
  for(int i = 0; i < config.nof_cell; i++){
      indices[i] = i;
      pthread_create(&beamform_thd[i], NULL, dci_beamform_client_thread, &indices[i]);
  }
  
  // Wait for the socket thread to finish.
	pthread_join(client_thd, NULL);
  // Wait for the beamforming thread to finish.
  for(int i = 0; i < config.nof_cell; i++){
      pthread_join(beamform_thd[i], NULL);
  }

	printf("abs");
  return 1;
}
