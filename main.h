#ifndef _EXT_RAR_
#define _EXT_RAR_

typedef enum events_e{
	event_ra_period=0,

	event_stop,
	num_normal_event
}events_t;

typedef enum ue_state_e{
	idle=0,
	msg1,
	msg2,
	msg3,
	msg4
}ue_state_t;

typedef struct ue_s{
	ue_state_t state;
	//	follow exponetial, indicate next arrival time.
	float arrival_time;
	
	///	MSG1
	//	preamble index selected for random access procedure.
	int preamble_index;
	//	backoff triggered-> retransmit+1
	int retransmit_counter;
	int backoff_counter;
	
	/// MSG2
	//	timing advance value
	int ta;
	int ta_reg;
	
	/// MSG3
	//	msg3 HARQ round times
	int msg3_harq_round;
	
	/// MSG4
	
	/// GLOBAL
	//	used for msg1, msg3
	struct ue_s *next;
	struct ue_s *prev;
	//	NB-IoT UE fixed location
	float location_x;
	float location_y;
	float distance;
	
	//	acitve: during random access procedure(from msg1 to msg3 finish, including HARQ), inactive: failure or sucess, wait until next arrival time.
	//int is_active;
	
	///	statistic
	float access_delay;
	
}ue_t;

typedef struct msg3_s{
	int harq_round;
	int num_selected;
	ue_t *msg3_ue;
}msg3_t;

typedef struct preamble_s{
	int num_selected;
	ue_t *ue_list;
}preamble_t;

typedef struct rar_s{
	int num_selected;
	ue_t *ue_list;
}rar_t;

typedef struct ext_ra_inst_s{
    
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
    int ras;
    int failed;
    int attempt;
    int success;
    int collide;
    int once_attempt_success;
    int retransmit;
    int once_attempt_collide;
    int trial;
    int msg3_harq_round_max;
	float total_access_delay;
	int rar_success;
	int rar_failed;
	int rar_waste;
	float eNB_radius;
}ext_ra_inst_t;


double exponetial(double mean);
void ue_backoff_process(ext_ra_inst_t *inst);
void ra_procedure(ext_ra_inst_t *inst);
void ue_selected_preamble(ext_ra_inst_t *inst, ue_t *ue);
void ue_arrival(ext_ra_inst_t *inst, ue_t *ue);
void initialize(ext_ra_inst_t *inst);
void initialize_ue_preamble(ext_ra_inst_t *inst);
void timing(ext_ra_inst_t *inst); 
void report(ext_ra_inst_t *inst);

#endif
