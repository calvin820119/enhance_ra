#ifndef _EXT_RAR_
#define _EXT_RAR_

typedef enum events_e{
	event_ra_period=0,

	event_stop,
	num_normal_event
}events_t;

typedef struct ue_s{
	//	follow exponetial, indicate next arrival time.
	float arrival_time;
	//	preamble index selected for random access procedure.
	int preamble_index;
	//	acitve: during random access procedure(from msg1 to msg3 finish, including HARQ), inactive: failure or sucess, wait until next arrival time.
	int is_active;
	//	backoff triggered-> retransmit+1
	int retransmit_counter;
	int backoff_counter;
	//	statistic
	float access_delay;
	//	used for msg1, msg3
	struct ue_s *next;
	//	NB-IoT UE fixed location
	float location_x;
	float location_y;
	//	timing advance value(memorize)
	int ta;
	//	msg3 HARQ round times
	int msg3_harq_round;
}ue_t;

typedef struct msg3_s{
	int harq_round;
	int num_selected;
	ue_t *msg3_ue;
}msg3_t;

typedef struct preamble_s{
	int num_selected;
	ue_t *rar_ue;
}preamble_t;

typedef struct ext_ra_inst_s{
    
    int total_ras;
    int number_of_preamble;
    int num_ue;
    float mean_interarrival;
    float ra_period;
    int rar_type;
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
    
	float total_access_delay;
	int rar_success;
	int rar_failed;
	int rar_waste;
	float eNB_radius;
}ext_ra_inst_t;


float exponetial(float mean);
void ue_backoff_process(ext_ra_inst_t *inst);
void ra_procedure(ext_ra_inst_t *inst);
void ue_selected_preamble(ext_ra_inst_t *inst, int ue_id);
void ue_arrival(ext_ra_inst_t *inst, int next_event_type);
void initialize(ext_ra_inst_t *inst);
void initialize_ue_preamble(ext_ra_inst_t *inst);
void timing(ext_ra_inst_t *inst); 
void report(ext_ra_inst_t *inst);

#endif
