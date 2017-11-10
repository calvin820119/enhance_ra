#ifndef _EXT_RAR_
#define _EXT_RAR_

typedef enum arrival_e{
    one_shot,
    poisson
}arrival_t;

typedef enum events_e{
	event_ra_period=0,

	event_stop,
	num_normal_event
}events_t;

typedef enum ue_state_e{
	idle=0,	//	before select preamble
	backoff,	//	during backoff counting
	state2,	//	after NPRACH, before receive DCI
	state3,	//	(retransmission)after receive DCI, before receive DCI
	state1	//	after select preamble, before NPRACH
}ue_state_t;

typedef struct ue_s{
    
	int done;  //
	ue_state_t state;  //
	//	follow exponetial, indicate next arrival time.
	float arrival_time;    //
	
	///	MSG1
	//	preamble index selected for random access procedure.
	int preamble_index;    //
	//	backoff triggered-> retransmit+1
	int retransmit_counter;    //
	
	/// MSG2
	//	timing advance value
	int ta;
	float ta_mean;	//	sample mean
	int ta_count;	//	sample numbers
	int ta_min;		//	sample maximum
	int ta_max;		//	sample minimum
	
	/// MSG3
	//	msg3 HARQ round times
	int eNB_process_msg3;  //
	int msg3_harq_round;
	double msg3_grant;
	int msg3_ack;
	
	/// MSG4
	int ue_id; //
	/// GLOBAL
	//	used for msg1, msg3
	struct ue_s *next; //
	struct ue_s *prev; //
	//	NB-IoT UE fixed location
	float location_x;
	float location_y;
	float distance;
	
	///	statistic
	float access_delay;    //
	
}ue_t;

typedef struct preamble_s{
	int num_selected;
	ue_t ue_list;
}preamble_t;

typedef enum distribution_e{
	dist_normal = 0,
	dist_determine
}distribution_t;

typedef struct simulation_s{
    
    int total_ras;
    int number_of_preamble;
    int num_ue;
    
	float mean_interarrival;	
    float mean_rar_latency;	
    float mean_msg3_latency;	
    float mean_msg3_retransmit_latency;	
    
    float ra_period;
    ue_t *ue_list;
    int max_retransmit;	
    preamble_t *preamble_table;	
    int back_off_window_size;	
    int ras;//
    int failed;//
    int attempt;//
    int success;//
    int collide;//
    int collide_preamble;//
    int retransmit;//
    int trial;
    int msg3_harq_round_max;	
	float total_access_delay;//
	float eNB_radius;	//
	float normal_std;
    int retransmit_count;//
	int one_shot_ue;//
	float ps;
	float avg_trial;
	float threshold;
}simulation_t;


float exponetial(float mean);
void ue_backoff_process(simulation_t *inst);
void ra_procedure(simulation_t *inst);
void ue_selected_preamble(simulation_t *inst, ue_t *ue);
void ue_arrival(simulation_t *inst, ue_t *ue);
void initialize(simulation_t *inst);
void initialize_ue_preamble(simulation_t *inst);
void timing(simulation_t *inst); 
void report(simulation_t *inst);

#endif
