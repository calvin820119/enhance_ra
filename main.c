#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "lcgrand.h"

#define file_input

const double pi = 3.14159265;

static float sim_time;
static float stop_time;
static float time_next_event[num_normal_event];
static int next_event_type;
static FILE *fin;
static FILE *fout;

static char str_fin_name[] = "config.in";
static char str_fout_name[50];

float exponetial(float mean){
    return -mean * log(lcgrand(1));
}

int algo_msg3_tx(int ta, int orig_ta){
	return 1;
}

//	11 bits
int msg2_find_ta(ue_t *list){
	ue_t *p = list;
	float max_distance, distance;
	max_distance = 0.0f;
	while((ue_t *)0 != p){
		distance = (float)sqrt(pow(p->location_x, 2) + pow(p->location_y, 2));
		if(max_distance < distance){
			max_distance = distance;
		}
		p = p->next;
	}
	
	//	70km per bit
	return (int)(max_distance/70.0);
}

void ue_backoff_process(ext_ra_inst_t *inst){
    int i;
    for(i=0;i<inst->num_ue;++i){
        if(inst->ue_list[i].is_active == 1){
            if(inst->ue_list[i].backoff_counter == 0){
                ue_selected_preamble(inst, i);
            }else{
                inst->ue_list[i].backoff_counter -= 1;
            }
        }
    }
}

void msg3_procedure(){
}

void msg2_procedure(ext_ra_inst_t *inst){
	inst->ras++;
	time_next_event[event_ra_period] = sim_time + inst->ra_period;
}

void ue_arrival(ext_ra_inst_t *inst, int next_event_type){
    int ue_id = next_event_type - num_normal_event;
    inst->attempt+=1;
    inst->ue_list[ue_id].access_delay = sim_time;
    inst->ue_list[ue_id].is_active = 0x1;
    //ue_selected_preamble(inst, ue_id);
}

void ue_selected_preamble(ext_ra_inst_t *inst, int ue_id){
    ue_t *iterator2;
    int preamble_index;
    //int rar_index;
    
    inst->trial+=1;
    
    preamble_index = (int)(inst->number_of_preamble*lcgrand(2));
    //rar_index = (int)(inst->rar_type*lcgrand(3));
    
    
    inst->ue_list[ue_id].preamble_index = preamble_index;
    
    //  choose RAR
    inst->preamble_table[preamble_index].num_selected += 1;
    
    if( (ue_t *)0 == inst->preamble_table[preamble_index].rar_ue ){
        inst->preamble_table[preamble_index].rar_ue = &inst->ue_list[ue_id];
		inst->ue_list[ue_id].next = (ue_t *)0;
    }else{
        iterator2 = inst->preamble_table[preamble_index].rar_ue;
        
        while( (ue_t *)0 != iterator2->next ){
            iterator2 = iterator2->next;
        }
        iterator2->next = &inst->ue_list[ue_id];
        inst->ue_list[ue_id].next = (ue_t *)0;
    }
}

void initialize_ue_preamble(ext_ra_inst_t *inst){
    int i, j;
    float rand_radius;
    float rand_angle;
    inst->ue_list = (ue_t *)malloc(inst->num_ue*sizeof(ue_t));
	inst->preamble_table = (preamble_t *)malloc(inst->number_of_preamble*sizeof(preamble_t));
    inst->eNB_radius = 10000;	//	default 10,000 m, 10km
    for(i=0;i<inst->num_ue;++i){
        inst->ue_list[i].is_active = 0;
        inst->ue_list[i].backoff_counter = 0;
        inst->ue_list[i].retransmit_counter = 0;
        inst->ue_list[i].preamble_index = 0;
        inst->ue_list[i].arrival_time = sim_time + exponetial(inst->mean_interarrival);
        inst->ue_list[i].next = (ue_t *)0;
        inst->ue_list[i].access_delay = 0;
        rand_radius = inst->eNB_radius * lcgrand(3);
        rand_angle = 2* pi * lcgrand(4);
		inst->ue_list[i].location_x = rand_radius * cos( rand_angle );
		inst->ue_list[i].location_y = rand_radius * sin( rand_angle );
		inst->ue_list[i].ta = -1;
    }
    
    for(i=0;i<inst->number_of_preamble;++i){
    		inst->preamble_table[i].num_selected = 0;
    		inst->preamble_table[i].rar_ue = (ue_t *)0;
	}
	time_next_event[event_ra_period] = sim_time + inst->ra_period;
}

void initialize(ext_ra_inst_t *inst){
    
    //  simulator
    sim_time = 0.0f;
    
    time_next_event[event_stop] = stop_time;
    next_event_type = event_ra_period;
    
    //  statistic
    inst->failed=0;
    inst->attempt=0;
    inst->success=0;
    inst->collide=0;
    // inst->once_attempt_success=0;
    inst->retransmit=0;
    // inst->once_attempt_collide=0;
    inst->trial=0;
    inst->ras = 0;
    inst->total_access_delay = 0.0f;
    inst->rar_success = 0;
	inst->rar_failed = 0;
	inst->rar_waste = 0;
    
    //  extended RA
    inst->total_ras = 0;
    inst->number_of_preamble = 0;
    inst->num_ue = 0;
    inst->ra_period = 0.0f;
    inst->max_retransmit = 0;
    inst->back_off_window_size = 0;
    inst->mean_interarrival = 0.0f;
	
	inst->ue_list = (ue_t *)0;
	inst->preamble_table = (preamble_t *)0;
}

void timing(ext_ra_inst_t *inst){ 
    int i;
    float min_time_next_event = time_next_event[event_ra_period];
    
    next_event_type = event_ra_period;
    
    for(i=0;i<inst->num_ue;++i){
        if( 0x0 == inst->ue_list[i].is_active){
            if(inst->ue_list[i].arrival_time < min_time_next_event){
                min_time_next_event = inst->ue_list[i].arrival_time;
                next_event_type = num_normal_event+i;
            }
        }
        
        //  TODO priority queue, using heap, implemented by array
        //	O(1), only need to check the root of heap 
    }
    if(inst->ras >= inst->total_ras){
        next_event_type = event_stop;
        return;
    }
    
    sim_time = min_time_next_event;
}

void report(ext_ra_inst_t *inst){ 
    int i, rest=0, rar_total=0;
    int num_ras = inst->total_ras;
    float avg_num_attempt = (float)inst->attempt/num_ras;
    float avg_num_success = (float)inst->success/num_ras;
    float avg_num_collide = (float)inst->collide/num_ras;
    
    for(i=0;i<inst->num_ue;++i){
        if(1 == inst->ue_list[i].is_active){
            ++rest;
        }
    }
    rar_total = inst->rar_failed + inst->rar_success + inst->rar_waste;
#ifdef print_output
    printf("total attemp  : %d\n", inst->attempt);
    printf("total success : %d\n", inst->success);
    printf("total failed  : %d\n", inst->failed);
    printf("total residual: %d\n", rest);
    printf("\n");
    printf("total trial   : %d\n", inst->trial);
    printf("total collide : %d\n", inst->collide);
    printf("\n");
    printf("avg. success      : %f\n", avg_num_success);
    printf("avg. prob. success: %f\n", (float)inst->success/inst->trial);
    printf("avg. prob. collide: %f\n", (float)inst->collide/inst->trial);
    printf("\n");
    printf("avg. access delay : %f\n", inst->total_access_delay/inst->success);
    printf("rar success rate  : %f\n", (float)inst->rar_success/rar_total);
    printf("rar failed  rate  : %f\n", (float)inst->rar_failed/rar_total);
    printf("rar waste   rate  : %f\n", (float)inst->rar_waste/rar_total);
    printf("\n");
#endif
    
    fprintf(fout, "%f ", avg_num_success);
    fprintf(fout, "%f ", (float)inst->collide/inst->trial);
    fprintf(fout, "%f ", inst->total_access_delay/inst->success);

    fprintf(fout, "%f ", (float)inst->rar_success/rar_total);
    fprintf(fout, "%f ", (float)inst->rar_failed/rar_total);
    fprintf(fout, "%f ", (float)inst->rar_waste/rar_total);
    fprintf(fout, "%d\n", inst->rar_waste);

}


int main(int argc, char *argv[]){

    ext_ra_inst_t ext_ra_inst;
	
    initialize(&ext_ra_inst);
#ifdef file_input
	if((FILE *)0 == (fin = fopen(str_fin_name, "r"))){
		printf(".in file access error!\n");
		return 1;
	}
	fscanf(fin, "%d %d %d %f %d %d %f %d", &ext_ra_inst.total_ras, &ext_ra_inst.number_of_preamble, &ext_ra_inst.num_ue, &ext_ra_inst.ra_period, &ext_ra_inst.max_retransmit, &ext_ra_inst.back_off_window_size, &ext_ra_inst.mean_interarrival, &ext_ra_inst.rar_type);
#else
    if(9 != argc){
		printf("Usage: ./[exe] [total ras] [# of preamble] [# of ue] [max trans] [backoff window] [mean interarrival] [# of rar]\n");
		return 1;
	}else{
	    sscanf(*(argv+1), "%d", &ext_ra_inst.total_ras);
        sscanf(*(argv+2), "%d", &ext_ra_inst.number_of_preamble);
        sscanf(*(argv+3), "%d", &ext_ra_inst.num_ue);
        sscanf(*(argv+4), "%f", &ext_ra_inst.ra_period);
        sscanf(*(argv+5), "%d", &ext_ra_inst.max_retransmit);
        sscanf(*(argv+6), "%d", &ext_ra_inst.back_off_window_size);
        sscanf(*(argv+7), "%f", &ext_ra_inst.mean_interarrival);
        sscanf(*(argv+8), "%d", &ext_ra_inst.rar_type);
    }
#endif

	sprintf(str_fout_name, "out/rar%d.out", ext_ra_inst.rar_type);

	if((FILE *)0 == (fout = fopen(str_fout_name, "ab+"))){
		printf(".out file access error!\n");
		return 1;
	}
	
    initialize_ue_preamble(&ext_ra_inst);
#ifdef print_output
    printf("-------------------------------------------------\n");
    printf("Number of preamble :         %d\n", ext_ra_inst.number_of_preamble);
    printf("Number of UE :               %d\n", ext_ra_inst.num_ue);
    printf("Number of RAR extended :     %d\n", ext_ra_inst.rar_type);
    printf("RAs period :                 %f sec\n", ext_ra_inst.ra_period);
    printf("Maximum retransmit times :   %d\n", ext_ra_inst.max_retransmit);
    printf("Uniform backoff window :     %d\n", ext_ra_inst.back_off_window_size);
    printf("Each UE mean inter-arrival : %f sec\n", ext_ra_inst.mean_interarrival);
    printf("Total simulated RAs :        %d\n", ext_ra_inst.total_ras);
    printf("-------------------------------------------------\n\nrun...\n");
#endif
	int i;
	for(i=0;i<ext_ra_inst.num_ue;++i){
        printf("UE%03d (%f, %f) arrival:%f\n", i, ext_ra_inst.ue_list[i].location_x, ext_ra_inst.ue_list[i].location_y, ext_ra_inst.ue_list[i].arrival_time);
	}

	do{
		
	    timing(&ext_ra_inst);
	    printf("%f %d\n", sim_time, next_event_type);
	    switch(next_event_type){
	        case event_ra_period:
	                msg2_procedure(&ext_ra_inst);
	                ue_backoff_process(&ext_ra_inst);
	            break;
	        case event_stop:
#ifdef print_output
	            printf("report...\n\n");
#endif
	            report(&ext_ra_inst);
		    break;
	        default:
	            ue_arrival(&ext_ra_inst, next_event_type);
	            break;
	    }
	
	}while(event_stop != next_event_type);

#ifdef file_input
	fclose(fin);
#endif
	fclose(fout);
	return 0;
}




