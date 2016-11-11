#include "history.h"

void init_sample_history(adc_sample_history* history)
{
	//Clear sample history, otherwise for large values of N_HISTORY, we'd get a few seconds of hiss for echo, reverb, etc.
	history->curr = 0;
	int i = 0;
	int j = 0;
	for(i = 0; i < INPUT_CHANNELS; i++)
	{
		for(j = 0; j < N_HISTORY; j++)
		{
			history->buff[i][j] = 0;
		}
	}
}

void init_component_buffer(component_buffer* buffer, q15_t* buff_param, uint32_t size_param)
{
	buffer->buff = buff_param;
	buffer->size = size_param;
	buffer->curr = 0;
	int i = 0;
	for(i = 0;  i < buffer->size; i++)
	{
		buffer->buff[i] = 0;
	}
}

