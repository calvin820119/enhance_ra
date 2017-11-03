#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "lcgrand.h"
#include <string.h>

//#define file_input

//  TODO: add multiple test for one-shot

static char cfg_always_tx_msg3;
static arrival_t cfg_ue_arrival;
static char cfg_algo_version;

#define TA_NORMAL_DISTRIBUTION
#define UE_ARRIVAL_ONE_SHOT
#define UE_ALGO_V1

#define ABS(a)		(((a)>0)?(a):(-a))
#define MAX(a, b)	((a)>(b))?(a):(b)
#define MIN(a, b)	((a)<(b))?(a):(b)

const float pi = 3.14159265;

static float sim_time;
static float stop_time;
static float time_next_event[num_normal_event];
static int next_event_type;
static FILE *fin;
static FILE *fout;

static char str_fin_name[] = "config.in";
static char str_fout_name[50];

static int cfg_print_output;
static distribution_t cfg_ta_distribution;

float normal(float mean, float std){
	float u = lcgrand(4);
    float v = lcgrand(5);
    return( (sqrt(-2 * log(u)) * cos(2 * pi * v) * std) + mean);
}

float exponetial(float mean){
    return -mean * log(lcgrand(1));
}

int algo_msg3_tx_v1(int ta, float ta_mean){
	float diff = (float)ta - ta_mean;
	
	if( 1 == cfg_always_tx_msg3 ){
        return 1;
    }
	
	return (ABS(diff) <= 1.0f );
}

int algo_msg3_tx_v2(int ta, float ta_mean, int ta_max, int ta_min, int harq_round){
	
	float diff;
    if( 1 == cfg_always_tx_msg3 ){
        return 1;
    }
	
	if((int)(ta_mean+0.5f) == ta){
		return 1;
	}
	
	if((float)ta > ta_mean){
		//	above ta_mean
		diff = (float)ta_max - ta_mean + 0.9f + (7>>harq_round);	//	considerate rounding problem
		return (ta < ((((int)diff)>>harq_round)+(int)(ta_mean+0.9f)));
	}else{
		//	below ta_mean
		diff = ta_mean - (float)ta_min + 0.9f + (7>>harq_round);
		return (ta > ((int)ta_mean - (((int)diff)>>harq_round)));
	}
}

//	11 bits in rar
int msg2_find_ta(simulation_t *inst, ue_t *head){
	ue_t *p = head, *min_ue;
	float min_distance;
	int ret;
	float ta;
	min_distance = p->distance;
	min_ue = p;
	while((ue_t *)0 != p){
		if(min_distance > p->distance){
			min_distance = p->distance;
			min_ue = p;
		}
		p = p->next;
	}
	min_ue->hit = 1;
	
	//	156m per bit
	ta = (int)(min_distance/156.0);	//	156m calculate by Ts x speed_of_light
	
	ret = MAX((int)normal(ta, inst->normal_std), 0);

	return ret;
}

void ue_backoff_process(simulation_t *inst){
    int i;
    ue_t *ue;
    int j;
    if(cfg_ue_arrival == one_shot){
    	j = inst->one_shot_ue;
    }else if(cfg_ue_arrival == poisson){
    	j = inst->num_ue;
	}
    
    for(i=0;i<j;++i){
   // for(i=0;i<inst->num_ue;++i){
    	ue = &inst->ue_list[i];
        if(ue->state == backoff){
            if(ue->backoff_counter == 0){
            	ue->state = state1;
                ue_selected_preamble(inst, ue);
            }else{
                ue->backoff_counter -= 1;
            }
        }
    }
}

//	after msg3 transmit
void process_dci(simulation_t *inst, ue_t *ue){
	//	step1 
	//	- determine receive ACK or NACK, must to do before any UE receive DCI.
	//	step2
	//	- ACK: success, TODO simulate the time to receive msg4
	//	- NACK: msg3 retransmission, msg3 tx decision, consider HARQ rounds, retransmission rounds.
	
	ue_t *iterator;
	float retransmit_grant = inst->mean_msg3_retransmit_latency;
	//	need to finish all the ack/nack decision before the UE receive DCI 
	
	if(1 == ue->eNB_process_msg3){ 
		ue->eNB_process_msg3 = 0;
		if((ue_t *)0 == ue->prev && (ue_t *)0 == ue->next){
			ue->msg3_ack = 1;
		}else{
			ue->msg3_ack = 0;
			ue->msg3_grant = retransmit_grant;
			
			iterator = ue->next;
			while((ue_t *)0 != iterator){
				iterator->msg3_grant = retransmit_grant;
				iterator->msg3_ack = 0;
				iterator->eNB_process_msg3 = 0;
				iterator = iterator->next;
			}
			iterator = ue->prev;
			while((ue_t *)0 != iterator){
				iterator->msg3_grant = retransmit_grant;
				iterator->msg3_ack = 0;
				iterator->eNB_process_msg3 = 0;
				iterator = iterator->prev;
			}
		}
	}
	
	if(ue->msg3_ack == 1){
		
		inst->success++;
		inst->total_access_delay += sim_time - ue->access_delay;
		if(ue->hit == 1){
			inst->num_hit++;
		}else{
			inst->num_hit_other++;
		}
		ue->hit = 0;
		
		if(ue->ta_count == 0){
			ue->ta_mean = ue->ta;
			ue->ta_max =  ue->ta;
			ue->ta_min =  ue->ta;
		}else{
			ue->ta_mean = ((((double)ue->ta/ue->ta_count)+ue->ta_mean) / (ue->ta_count+1))*ue->ta_count;
			ue->ta_min = MIN(ue->ta_min, ue->ta);
			ue->ta_max = MAX(ue->ta_max, ue->ta);
		}
			
		ue->ta_count++;

		ue->backoff_counter = 0;
		ue->msg3_harq_round = 0;
		ue->retransmit_counter = 0;
		ue->state = idle;

		ue->done = 1;

        if(cfg_ue_arrival == one_shot){
            inst->one_shot_ue--;
            //printf("%d\n", ue - inst->ue_list);
            //printf("%d finish, %d -> %d\n", ue - inst->ue_list, inst->one_shot_ue, ue - inst->ue_list);
            
			//	[remove ue]
			if(ue - inst->ue_list != inst->one_shot_ue){
            	
	            if((ue_t *)0 != inst->ue_list[inst->one_shot_ue].prev){
					inst->ue_list[inst->one_shot_ue].prev->next = &inst->ue_list[ue - inst->ue_list];
				}
				if((ue_t *)0 != inst->ue_list[inst->one_shot_ue].next){
					inst->ue_list[inst->one_shot_ue].next->prev = &inst->ue_list[ue - inst->ue_list];
				}
				
				inst->ue_list[ue - inst->ue_list] = inst->ue_list[inst->one_shot_ue];

			}
		 //ue->arrival_time = 10000.0f;
        }else if(cfg_ue_arrival == poisson){
            ue->arrival_time = sim_time + exponetial(inst->mean_interarrival);
        }

	}else{	//	nack
		
		//	collision happen, only count the first collision.
		if(ue->msg3_harq_round == 0){
			inst->collide += 1;
		}
		
		int decision;
        if(cfg_algo_version == 1){
            decision = algo_msg3_tx_v1(ue->ta, (int)(ue->distance/156.0));
        }else if(cfg_algo_version == 2){
            decision = (0 == ue->ta_count) || (1 == algo_msg3_tx_v2(ue->ta, ue->ta_mean, ue->ta_max, ue->ta_min, ue->msg3_harq_round));
        }
		
		if(ue->msg3_harq_round < inst->msg3_harq_round_max && ( 1 == decision) ){
			//[change time]
			ue->arrival_time = sim_time + ue->msg3_grant;
			ue->msg3_harq_round++;
			ue->eNB_process_msg3 = 1;
		}else{	//	give up
			
			if(ue->hit == 1 && ue->msg3_harq_round != inst->msg3_harq_round_max){
				inst->num_error_hit++;
			}
			ue->hit = 0;
			
			if((ue_t *)0 != ue->prev){
				ue->prev->next = ue->next;
			}
			if((ue_t *)0 != ue->next){
				ue->next->prev = ue->prev;
			}
			ue->next = (ue_t *)0;
			ue->prev = (ue_t *)0;
			ue->msg3_harq_round = 0;
			
			
			
			if(ue->retransmit_counter >= inst->max_retransmit){
				//	failed
				inst->failed += 1;
				ue->backoff_counter = 0;
				ue->retransmit_counter = 0;
				ue->state = idle;
ue->done = 1;
                if(cfg_ue_arrival == one_shot){
                	inst->one_shot_ue--;
                //	printf("%d %d", &inst->ue_list[ue - inst->ue_list], ue); getchar();
                	//printf("%d\n", ue - inst->ue_list);
                //	printf("%d finish, %d -> %d\n", ue - inst->ue_list, inst->one_shot_ue, ue - inst->ue_list);
                	//printf("%p %p", inst->ue_list[ue - inst->ue_list)], inst->ue_list[inst->one_shot_ue]);
                	if(ue - inst->ue_list != inst->one_shot_ue){
                		if((ue_t *)0 != inst->ue_list[inst->one_shot_ue].prev){
							inst->ue_list[inst->one_shot_ue].prev->next = &inst->ue_list[ue - inst->ue_list];
						}
						if((ue_t *)0 != inst->ue_list[inst->one_shot_ue].next){
							inst->ue_list[inst->one_shot_ue].next->prev = &inst->ue_list[ue - inst->ue_list];
						}
						inst->ue_list[ue - inst->ue_list] = inst->ue_list[inst->one_shot_ue];
					}
                	
				    //ue->arrival_time = 10000.0f;
				    
                }else if(cfg_ue_arrival == poisson){
                    ue->arrival_time = sim_time + exponetial(inst->mean_interarrival);
                }
		
			}else{
				//	backoff
				ue->retransmit_counter += 1;
		        //  uniform backoff
		        ue->backoff_counter = (int)(inst->back_off_window_size*lcgrand(4));
				ue->state = backoff;
			}
		}
	}
}

void debug_ues(simulation_t *inst){
	int i;
	ue_t *ue;
	printf("[%f]---------DEBUG UE---------\n", sim_time);
	for(i=0; i<inst->num_ue; ++i){
		printf("[%2d] state : %d \n", i, inst->ue_list[i].state);
	}
}

void debug_msg2(simulation_t *inst){
	int i;
	ue_t *ue;
	printf("[%f]---------DEBUG PREAMBLE---------\n", sim_time);
	for(i=0; i<inst->number_of_preamble; ++i){
		printf("[%2d] : %d UE ", i, inst->preamble_table[i].num_selected);
		
		if(inst->preamble_table[i].num_selected != 0){
			ue = inst->preamble_table[i].ue_list;
			while(ue != (ue_t *)0){
				printf("%d ", ue-inst->ue_list);
				ue = ue->next;
			}
		}
		printf("\n");
	}
}

void nprach_period_eNB(simulation_t *inst){
	int i, ta, num=0;
	float rar_time;
	float msg3_time;
	ue_t *iterator, *iterator1;
	inst->ras++;
	
	for(i=0; i<inst->number_of_preamble; ++i){
		
		if(0 != inst->preamble_table[i].num_selected){
		
			if(1 != inst->preamble_table[i].num_selected){
				inst->collide_preamble+=inst->preamble_table[i].num_selected;
			}
		
			iterator = inst->preamble_table[i].ue_list;
			
			ta = msg2_find_ta(inst, iterator);
			rar_time = inst->mean_rar_latency;
			msg3_time = inst->mean_msg3_latency;
			while((ue_t *)0 != iterator){
				num++;
				iterator->msg3_grant = msg3_time;
				iterator->msg3_ack = (inst->preamble_table[i].num_selected == 1);
				iterator->eNB_process_msg3 = 0;
				iterator->state = state2;
				//[change time]
				iterator->arrival_time = sim_time + rar_time;
				iterator->ta = ta;
				iterator = iterator->next;
			}
		}		
		
		//	clear for next NPRACH period
		inst->preamble_table[i].ue_list = (ue_t *)0;
		inst->preamble_table[i].num_selected = 0;
	}

	time_next_event[event_ra_period] = sim_time + inst->ra_period;
}

void ue_arrival(simulation_t *inst, ue_t *ue){
    inst->attempt+=1;
    ue->access_delay = sim_time;
    ue_selected_preamble(inst, ue);
    ue->state = state1;
}

void ue_decode_rar(simulation_t *inst, ue_t *ue){

    int decision;
    if(cfg_algo_version == 1){
        decision = algo_msg3_tx_v1(ue->ta, (int)(ue->distance/156.0));
    }else if(cfg_algo_version == 2){
        decision = (0 == ue->ta_count) || (1 == algo_msg3_tx_v2(ue->ta, ue->ta_mean, ue->ta_max, ue->ta_min, ue->msg3_harq_round));
    }

	if( 1 == decision ){

		ue->state = state3;
		//[change time]
		ue->arrival_time = sim_time + ue->msg3_grant;		//	wrong, all ue must be same value(fixed)
	}else{	//	give up at rar stage
		
		inst->give_up_rar++;
		
		if(ue->hit == 1){
			inst->num_error_hit++;
		}
		ue->hit = 0;
		
		//	give up(backoff)
		ue->retransmit_counter += 1;
        //  uniform backoff
        ue->backoff_counter = (int)(inst->back_off_window_size*lcgrand(4));
		ue->state = backoff;
		ue->msg3_harq_round = 0;
		
		if((ue_t *)0 != ue->prev){
			ue->prev->next = ue->next;
		}
		if((ue_t *)0 != ue->next){
			ue->next->prev = ue->prev;
		}

		ue->next = (ue_t *)0;
		ue->prev = (ue_t *)0;
	}
}

void ue_selected_preamble(simulation_t *inst, ue_t *ue){
    ue_t *iterator;
    int preamble_index;
    
    inst->trial+=1;
    
    preamble_index = (int)(inst->number_of_preamble*lcgrand(2));
    
    ue->preamble_index = preamble_index;
    
    //  choose RAR
    inst->preamble_table[preamble_index].num_selected += 1;
    
    if( (ue_t *)0 == inst->preamble_table[preamble_index].ue_list ){
        inst->preamble_table[preamble_index].ue_list = ue;
		ue->next = (ue_t *)0;
		ue->prev = (ue_t *)0;
    }else{
        iterator = inst->preamble_table[preamble_index].ue_list;
        inst->preamble_table[preamble_index].ue_list = ue;
        iterator->prev = ue;
        ue->next = iterator;
		ue->prev = (ue_t *)0;
    }
}

void initialize_simulation(simulation_t *inst){
    int i, j;
    float rand_radius;
    float rand_angle;
    inst->ue_list = (ue_t *)malloc(inst->num_ue*sizeof(ue_t));
	inst->preamble_table = (preamble_t *)malloc(inst->number_of_preamble*sizeof(preamble_t));
    inst->eNB_radius = 10000;	//	default 10,000 m, 10km
    
    for(i=0;i<inst->num_ue;++i){
        if(cfg_ue_arrival == one_shot){
        	//[change time]
        	inst->ue_list[i].arrival_time = inst->ra_period - 0.001f;    
        }else if(cfg_ue_arrival == poisson){
            inst->ue_list[i].arrival_time = sim_time + exponetial(inst->mean_interarrival);
        }
	
        inst->ue_list[i].state = idle;
        inst->ue_list[i].backoff_counter = 0;
        inst->ue_list[i].retransmit_counter = 0;
        inst->ue_list[i].preamble_index = 0;
        
        inst->ue_list[i].next = (ue_t *)0;
        inst->ue_list[i].access_delay = 0;
        rand_radius = inst->eNB_radius * lcgrand(3);
        rand_angle = 2* pi * lcgrand(4);
		inst->ue_list[i].location_x = rand_radius * cos( rand_angle );
		inst->ue_list[i].location_y = rand_radius * sin( rand_angle );
		inst->ue_list[i].ta = -1;
		inst->ue_list[i].ta_mean = 0;
		inst->ue_list[i].ta_max = 0;
		inst->ue_list[i].ta_min = 0;
		inst->ue_list[i].ta_count = 0;
		inst->ue_list[i].distance = rand_radius;
		inst->ue_list[i].eNB_process_msg3 = 0;
		inst->ue_list[i].hit = 0;
		inst->ue_list[i].done = 0;
    }
    
    for(i=0;i<inst->number_of_preamble;++i){
    	inst->preamble_table[i].num_selected = 0;
    	inst->preamble_table[i].ue_list = (ue_t *)0;
	}
	time_next_event[event_ra_period] = sim_time + inst->ra_period;
}

void initialize_structure(simulation_t *inst){
    
    //  simulator
    sim_time = 0.0f;
    
    time_next_event[event_stop] = stop_time;
    next_event_type = event_ra_period;
    
    //  statistic
    inst->failed=0;
    inst->attempt=0;
    inst->success=0;
    inst->collide=0;
    inst->retransmit=0;
    inst->trial=0;
    inst->ras = 0;
    inst->total_access_delay = 0.0f;
    inst->give_up_rar = 0;
    inst->collide_preamble = 0;
    inst->total_ras = 0;
    inst->number_of_preamble = 0;
    inst->num_ue = 0;
    inst->ra_period = 0.0f;
    inst->max_retransmit = 0;
    inst->back_off_window_size = 0;
    inst->mean_interarrival = 0.0f;
	inst->mean_rar_latency = 0.0f;	//	from NPRACH period to the time received rar(msg2)
	inst->mean_msg3_latency = 0.0f;	//	from the time received rar(msg2) to the time transimted msg3
	inst->mean_msg3_retransmit_latency = 00.0f;	//	from the time transimted msg3 to the time transimted msg3 again
	inst->msg3_harq_round_max = 0;
	
	inst->ue_list = (ue_t *)0;
	inst->preamble_table = (preamble_t *)0;
	
	inst->num_hit = 0;
	inst->num_error_hit = 0;
	inst->num_hit_other = 0;
	inst->normal_std = 0.0f;
	
	//	simulator
	cfg_print_output = 0;
	cfg_always_tx_msg3 = 0;
    cfg_ue_arrival = one_shot;  //  default setting
    cfg_algo_version = 1;       //  default setting
}

void timing(simulation_t *inst){ 
    int i;
    float min_time_next_event = time_next_event[event_ra_period];
    next_event_type = event_ra_period;
    
    int j;
    if(cfg_ue_arrival == one_shot){
    	j = inst->one_shot_ue;
    }else if(cfg_ue_arrival == poisson){
    	j = inst->num_ue;
	}
    
    
    
    #if 1
    //for(i=0;i<inst->num_ue;++i){
    for(i=0;i<j;++i){
        if( state1 > inst->ue_list[i].state){
            if(inst->ue_list[i].arrival_time < min_time_next_event){
                min_time_next_event = inst->ue_list[i].arrival_time;
                next_event_type = num_normal_event+i;
            }
        } 
    }
    #else
    if(inst->ue_list[0].arrival_time < min_time_next_event){
        min_time_next_event = inst->ue_list[0].arrival_time;
        next_event_type = num_normal_event+i;
    }
    #endif
    
    if(inst->ras >= inst->total_ras || inst->one_shot_ue == 0){
        next_event_type = event_stop;
        return;
    }
    
    sim_time = min_time_next_event;
}

void report(simulation_t *inst){ 
    int i, rest=0;
    int num_ras = inst->total_ras;
    float avg_num_attempt = (float)inst->attempt/num_ras;
    float avg_num_success = (float)inst->success/num_ras;
    float avg_num_collide = (float)inst->collide/num_ras;
    
    for(i=0;i<inst->num_ue;++i){
        if(1 != inst->ue_list[i].done){
            ++rest;
        }
    }
	if(cfg_print_output){
	    printf("total attemp  : %d\n", inst->attempt);  //attemp means try to connect to base station 
	    printf("total success : %d\n", inst->success);
	    printf("total failed  : %d\n", inst->failed);
	    printf("total residual: %d\n", rest);
	    printf("\n");
	    printf("total trial   : %d\n", inst->trial);	//	trial = numbers of ue select preamble
	    printf("total collide : %d\n", inst->collide);	//	collide = collision 
	    printf("\n");
	    printf("avg. success      : %f\n", avg_num_success);
	    printf("avg. prob. success: %f\n", (float)inst->success/inst->trial);
	    printf("avg. prob. collide: %f\n", (float)inst->collide/inst->trial);
	    printf("\n");
	    printf("avg. access delay : %f sec\n", inst->total_access_delay/inst->success);
	    printf("total correct hit : %d\n", inst->num_hit);
	    printf("total error hit   : %d\n", inst->num_error_hit);
	    printf("total others hit  : %d\n", inst->num_hit_other);
	    printf("correct rate      : %f %%\n", (float)(inst->num_hit*100)/(inst->num_hit+inst->num_hit_other));
	
	    printf("avg. attemp num   : %f\n", (float)inst->trial/inst->total_ras);
	    printf("avg. num giveup   : %f\n", (float)inst->give_up_rar/inst->total_ras);
	    printf("giveup/attemp     : %f\n", (float)inst->give_up_rar/inst->trial);
		printf("avg. prob. collide: %f\n", (float)inst->collide_preamble/inst->trial);
	    
	    printf("[one-shot] Ps     : %f %\n", 100.0*(float)inst->success/inst->attempt);
	    
	    printf("\n");
	}
    //fprintf(fout, "%f ", avg_num_success);	//	avg. success
    //fprintf(fout, "%f ", (float)inst->collide/inst->trial);	//	avg. prob. collide
    //fprintf(fout, "%f ", inst->total_access_delay/inst->success);	//	avg. access delay
    //fprintf(fout, "%f\n", (float)(inst->num_hit*100)/(inst->num_hit+inst->num_hit_other));
    
    //	performance index now we only consider Ps 
    fprintf(fout, "%f\n",  100.0*(float)inst->success/inst->attempt);
}

void get_global_config_parser(FILE *fin){
	char cmd[30] = { '\0' };
	char param[30] = { '\0' };
	while(!feof(fin)){
		fscanf(fin, "%s", cmd);
		
		if(cmd[0] != '-'){
		    continue;
        }
		
		if(strstr(cmd, "-PRINT_OUTPUT")){
			cfg_print_output = 1; 
		}else if(strstr(cmd, "-ALWAYS_TX_MSG3")){
			cfg_always_tx_msg3 = 1;
		}else if(strstr(cmd, "-TA_DISTRIBUTION")){
			fscanf(fin, " %s", param);
			if(strstr(param, "NORMAL")){
				cfg_ta_distribution = dist_normal;
			}else if(strstr(param, "DIST")){
				cfg_ta_distribution = dist_determine;
			}
		}else if(strstr(cmd, "-UE_ARRIVAL")){
		    fscanf(fin, " %s", param);
		    if(strstr(param, "ONE_SHOT")){
				cfg_ue_arrival = one_shot;
			}else if(strstr(param, "POISSON")){
				cfg_ue_arrival = poisson;
			}
        }else if(strstr(cmd, "-ALGO_VERSION")){
            fscanf(fin, " %d", &cfg_algo_version);
        }
	}
	fseek(fin, 0, SEEK_SET);
}

int set_system_config_parser(char *str, char *param, simulation_t *inst){
	if(strstr(str, "-NORMAL_STD")){
		sscanf(param, "%f", &inst->normal_std);
		return 2;
	}else if(strstr(str, "-BS_RADIUS")){
		sscanf(param, "%f", &inst->eNB_radius);
		return 2;
	}else if(strstr(str, "-TS")){
		//	haven't implement TODO
		return 2;
	}else if(strstr(str, "-TOTAL_RAS")){
		sscanf(param, "%d", &inst->total_ras);
		return 2;
	}else if(strstr(str, "-NUM_PREAMBLE")){
		sscanf(param, "%d", &inst->number_of_preamble);
		return 2;
	}else if(strstr(str, "-NUM_UE")){
		sscanf(param, "%d", &inst->num_ue);
		inst->one_shot_ue = inst->num_ue;
		return 2;
	}else if(strstr(str, "-RA_PERIOD")){
		sscanf(param, "%f", &inst->ra_period);
		return 2;
	}else if(strstr(str, "-MAX_RETRANS")){
		sscanf(param, "%d", &inst->max_retransmit);
		return 2;
	}else if(strstr(str, "-BACKOFF_SIZE")){
		sscanf(param, "%d", &inst->back_off_window_size);
		return 2;
	}else if(strstr(str, "-MEAN_INTER_ARR")){
		sscanf(param, "%f", &inst->mean_interarrival);
		return 2;
	}else if(strstr(str, "-MEAN_RAR_DELAY")){
		sscanf(param, "%f", &inst->mean_rar_latency);
		return 2;
	}else if(strstr(str, "-MEAN_MSG3_DELAY")){
		sscanf(param, "%f", &inst->mean_msg3_latency);
		return 2;
	}else if(strstr(str, "-MEAN_MSG3_RE_DELAY")){
		sscanf(param, "%f", &inst->mean_msg3_retransmit_latency);
		return 2;
	}else if(strstr(str, "-MAX_HARQ_ROUND")){
		sscanf(param, "%d", &inst->msg3_harq_round_max);
		return 2;
	}
	return 1;
}

void get_system_config_parser(FILE *fin, simulation_t *inst){
	char cmd[20] = { '\0' };
	
	while(!feof(fin)){
		
		fscanf(fin, "%s", cmd);
		
		if(cmd[0] != '-'){
		    continue;
        }
		
		if(strstr(cmd, "-NORMAL_STD")){
			fscanf(fin, "%f", &inst->normal_std);
		}else if(strstr(cmd, "-BS_RADIUS")){
			fscanf(fin, "%f", &inst->eNB_radius);
		}else if(strstr(cmd, "-TS")){
			//	haven't implement TODO
		}else if(strstr(cmd, "-TOTAL_RAS")){
			fscanf(fin, "%d", &inst->total_ras);
		}else if(strstr(cmd, "-NUM_PREAMBLE")){
			fscanf(fin, "%d", &inst->number_of_preamble);
		}else if(strstr(cmd, "-NUM_UE")){
			fscanf(fin, "%d", &inst->num_ue);
			inst->one_shot_ue = inst->num_ue;
		}else if(strstr(cmd, "-RA_PERIOD")){
			fscanf(fin, "%f", &inst->ra_period);
		}else if(strstr(cmd, "-MAX_RETRANS")){
			fscanf(fin, "%d", &inst->max_retransmit);
		}else if(strstr(cmd, "-BACKOFF_SIZE")){
			fscanf(fin, "%d", &inst->back_off_window_size);
		}else if(strstr(cmd, "-MEAN_INTER_ARR")){
			fscanf(fin, "%f", &inst->mean_interarrival);
		}else if(strstr(cmd, "-MEAN_RAR_DELAY")){
			fscanf(fin, "%f", &inst->mean_rar_latency);
		}else if(strstr(cmd, "-MEAN_MSG3_DELAY")){
			fscanf(fin, "%f", &inst->mean_msg3_latency);
		}else if(strstr(cmd, "-MEAN_MSG3_RE_DELAY")){
			fscanf(fin, "%f", &inst->mean_msg3_retransmit_latency);
		}else if(strstr(cmd, "-MAX_HARQ_ROUND")){
			fscanf(fin, "%d", &inst->msg3_harq_round_max);
		}
	}
	fseek(fin, 0, SEEK_SET);
}

int main(int argc, char *argv[]){

    simulation_t enhance_ra;
	int ue_id, argi;
	
    initialize_structure(&enhance_ra);

	if((FILE *)0 == (fin = fopen(str_fin_name, "r"))){
		printf(".in file access error!\n");
		return 1;
	}
	
    //if(13 != argc){
		//	use configuration file to set the structure
		get_global_config_parser(fin);		
		get_system_config_parser(fin, &enhance_ra);
	if(1 != argc){
		argi = 1;
		while(argi < argc){
			printf("overwrite %s\n", argv[argi]);
			argi += set_system_config_parser(argv[argi], argv[argi+1], &enhance_ra);
		}
	}
		//fscanf(fin, "%d %d %d %f %d %d %f %f %f %f %d %f", &enhance_ra.total_ras, &enhance_ra.number_of_preamble, &enhance_ra.num_ue, &enhance_ra.ra_period, &enhance_ra.max_retransmit, &enhance_ra.back_off_window_size, &enhance_ra.mean_interarrival, &enhance_ra.mean_rar_latency, &enhance_ra.mean_msg3_latency, &enhance_ra.mean_msg3_retransmit_latency, &enhance_ra.msg3_harq_round_max, &enhance_ra.normal_std);
		//printf("Usage: ./[exe] [total ras] [# of preamble] [# of ue] [max trans] [backoff window] [mean interarrival] [# of rar]\n");
		
	/*}else{
		get_global_config_parser(fin);
		
	    sscanf(*(argv+1), "%d", &enhance_ra.total_ras);
        sscanf(*(argv+2), "%d", &enhance_ra.number_of_preamble);
        sscanf(*(argv+3), "%d", &enhance_ra.num_ue);
        enhance_ra.one_shot_ue = enhance_ra.num_ue;
        sscanf(*(argv+4), "%f", &enhance_ra.ra_period);
        sscanf(*(argv+5), "%d", &enhance_ra.max_retransmit);
        sscanf(*(argv+6), "%d", &enhance_ra.back_off_window_size);
        sscanf(*(argv+7), "%f", &enhance_ra.mean_interarrival);
		sscanf(*(argv+8), "%f", &enhance_ra.mean_rar_latency);
		sscanf(*(argv+9), "%f", &enhance_ra.mean_msg3_latency);
		sscanf(*(argv+10), "%f", &enhance_ra.mean_msg3_retransmit_latency);
		sscanf(*(argv+11), "%d", &enhance_ra.msg3_harq_round_max);
		sscanf(*(argv+12), "%f", &enhance_ra.normal_std);
    }*/

	sprintf(str_fout_name, "out/result.out");

	if((FILE *)0 == (fout = fopen(str_fout_name, "ab+"))){
		printf(".out file access error!\n");
		return 1;
	}
	
    initialize_simulation(&enhance_ra);
    if(cfg_print_output){
    
        printf("-------------------------------------------------\n");
        printf("Number of preamble :           %d\n", enhance_ra.number_of_preamble);
        printf("Number of UE :                 %d\n", enhance_ra.num_ue);
        printf("Maximum retransmit times :     %d\n", enhance_ra.max_retransmit);
        printf("Uniform backoff window :       %d\n\n", enhance_ra.back_off_window_size);
        printf("RAs period :                   %f ms\n", enhance_ra.ra_period*1000);
        printf("Each UE mean inter-arrival :   %f ms\n", enhance_ra.mean_interarrival*1000);
        printf("Each UE mean rar latency :     %f ms\n", enhance_ra.mean_rar_latency*1000);
        printf("Each UE mean msg3 latency :    %f ms\n", enhance_ra.mean_msg3_latency*1000);
        printf("Each UE mean re-msg3 latency : %f ms\n", enhance_ra.mean_msg3_retransmit_latency*1000);
        printf("Maximum number of harq round : %d\n", enhance_ra.msg3_harq_round_max);
        printf("Total simulated RAs :          %d (=%f sec)\n", enhance_ra.total_ras, enhance_ra.total_ras*enhance_ra.ra_period);
        printf("-------------------------------------------------\n\nrun...\n");
    }
	do{
	//	printf("%f %d\n", sim_time, enhance_ra.one_shot_ue);

		
	    timing(&enhance_ra);
	    switch(next_event_type){
	        case event_ra_period:
					ue_backoff_process(&enhance_ra);
	                nprach_period_eNB(&enhance_ra);
	            break;
	        case event_stop:
                if(cfg_print_output){
	               printf("finished time:%f\nreport... \n\n", sim_time);	
                }
	            report(&enhance_ra);
		    break;
	        default:{
	        	ue_id = next_event_type - num_normal_event;
	        	switch(enhance_ra.ue_list[ue_id].state){
	        		case idle:
	        			ue_arrival(&enhance_ra, &enhance_ra.ue_list[ue_id]);
	        			break;
	        		case state2:
	        			ue_decode_rar(&enhance_ra, &enhance_ra.ue_list[ue_id]);
	        			break;
	        		case state3:
	        			process_dci(&enhance_ra, &enhance_ra.ue_list[ue_id]);
	        			break;
	        		default:
	        			break;
				}
			}
	        break;
	    }
	
	}while(event_stop != next_event_type);

#ifdef file_input
	fclose(fin);
#endif
	fclose(fout);
	return 0;
}




