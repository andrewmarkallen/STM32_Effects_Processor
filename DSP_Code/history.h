#ifndef _HISTORY_H_
#define _HISTORY_H_

#include "global.h"

#define hist_idx(comp_buff, i) ((comp_buff->curr + i + comp_buff->size) % comp_buff->size)
#define hist(comp_buff, i) comp_buff->buff[hist_idx(comp_buff, i)]

typedef struct
{
int16_t buff[INPUT_CHANNELS][N_HISTORY];
uint32_t curr;

}adc_sample_history;

typedef struct
{
float* buff;
uint32_t curr;
uint32_t size;

}component_buffer;

void init_sample_history(adc_sample_history* history);
void init_component_buffer(component_buffer* buffer, q15_t* buff_param, uint32_t size_param);

#endif
