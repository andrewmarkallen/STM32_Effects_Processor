#include "test_components.h"

uint16_t mixer_exec(dsp_comp* comp)
{
	arm_add_f32(comp->in[0], comp->in[1], comp->pre_out[0], BLOCK_SIZE);

	finalise(comp);
	return 0;
}

uint16_t router_exec(dsp_comp* comp)
{
	uint8_t in, out;
	
	for(out = 0; out < comp->n_out; out++)
	{
		arm_fill_f32(0.0f, comp->pre_out[out], BLOCK_SIZE);

		for(in = 0; in < comp->n_in; in++)
		{
			if((comp->int_params[in] >> out) & 0x1)
			{
				arm_add_f32(comp->in[in], comp->pre_out[out], comp->pre_out[out], BLOCK_SIZE);
			}
		}
	}

	finalise(comp);
	return 0;
}

uint16_t amplifier_exec(dsp_comp* comp)
{
	arm_scale_f32(comp->in[0], comp->float_params[0], comp->pre_out[0], BLOCK_SIZE);

	finalise(comp);
	return 0;
}

uint16_t pass_through_exec(dsp_comp* comp)
{
	arm_copy_f32(comp->in[0], comp->pre_out[0], BLOCK_SIZE);

	finalise(comp);
	return 0;
}

uint16_t swap_exec(dsp_comp* comp)
{
	arm_copy_f32(comp->in[0], comp->pre_out[1], BLOCK_SIZE);
	arm_copy_f32(comp->in[1], comp->pre_out[0], BLOCK_SIZE);

	finalise(comp);
	return 0;
}

uint16_t splitter_exec(dsp_comp* comp)
{
	uint8_t j;
	for(j = 0; j < comp->n_out; j++)
	{
		arm_copy_f32(comp->in[0], comp->pre_out[j], BLOCK_SIZE);
	}

	finalise(comp);
	return 0;
}

uint16_t mult_exec(dsp_comp* comp)
{
	arm_mult_f32(comp->in[0], comp->in[1], comp->pre_out[0], BLOCK_SIZE);

	finalise(comp);
	return 0;
}

uint16_t noise_exec(dsp_comp* comp)
{
  //printf("exec called for %s\n", comp->name);
  
  //This implements a 16-bit linear feedback shift register in Galois configuration 
  //Each step returns the next element in a (2^16)-1 length unsigned integer sequence (all values except 0).
  
  //It works by shifting the current state (16 bit unsigned int) right by one, saving the LSB we shift off the end
  //Then certain bits (taps) of the shifted state are XOR'd with the stored LSB.
  //The taps are determined by algebraic concerns so as to ensure the entire field of numbers is computed (i.e. google it)
   
  int16_t curr_state = comp->int_params[0];  
   
  int i = 0;
  for(i = 0; i < BLOCK_SIZE; i++)
  {
    uint16_t lsb = curr_state & 1;  //Extract the least significant bit
    curr_state >>= 1;               //Shift curr state by 1
    curr_state ^= (-lsb) & 0xB400u; //bitwise XOR the curr_state with the bitmask corresponding to taps 
    
    int32_t int_output = 0;
    int_output = curr_state;
    
    float two_to_sixteen = 0x10000;
    float half_of_two_to_sixteen = 0x8000;
    
    comp->pre_out[0][i] = (int_output - half_of_two_to_sixteen) / two_to_sixteen;
  }  
  
  comp->int_params[0] = curr_state;
  finalise(comp);
  return 0; 
}

uint16_t diff_exec(dsp_comp* comp)
{
	arm_sub_f32(comp->in[0], comp->in[1], comp->pre_out[0], BLOCK_SIZE);

	finalise(comp);
	return 0;
}

uint16_t inv_exec(dsp_comp* comp)
{
	uint8_t j;
	for(j = 0; j < comp->n_out; j++)
	{
		arm_negate_f32(comp->in[j], comp->pre_out[j], BLOCK_SIZE);
	}

	finalise(comp);
	return 0;
}

uint16_t delay_exec(dsp_comp* comp)
{
	// NOTE: Size of history buffer needs to be twice as large as the maximum allowed delay

	component_buffer* buffer = &comp->buffer[0];

	float *buf = buffer->buff;
	uint32_t *rw_index = &buffer->curr;
	uint32_t M = buffer->size/2;
	if(*rw_index < M) *rw_index += M;
	if(*rw_index > 2*M-1) *rw_index -= M;

	// Store new samples in the history (double-buffer style)
	arm_copy_f32(comp->in[0], buf + *rw_index, BLOCK_SIZE);
	arm_copy_f32(comp->in[0], buf + *rw_index - M, BLOCK_SIZE);

	// Output delayed samples
	arm_copy_f32(buf + *rw_index - comp->int_params[0], comp->pre_out[0], BLOCK_SIZE);

	(*rw_index) += BLOCK_SIZE;

	finalise(comp);
	return 0;
}

uint16_t fir_exec(dsp_comp* comp)
{
	if(comp->int_params[0])
	{
		arm_fir_instance_f32 *S = (arm_fir_instance_f32 *)comp->pInstance;
		arm_fir_f32(S, comp->in[0], comp->pre_out[0], BLOCK_SIZE);
	}
	else
	{
		arm_fill_f32(0, comp->pre_out[0], BLOCK_SIZE);
	}

	finalise(comp);
	return 0;
}

uint16_t recorder_exec(dsp_comp* comp)
{
	// NOTE: Size of buffers are expected to be multiples of the block size

	uint8_t j;
	for(j = 0; j < comp->n_in; j++)
	{
		component_buffer* buffer = &comp->buffer[j];

		float *buf = buffer->buff;
		uint32_t *rw_index = &buffer->curr;

		arm_copy_f32(comp->in[j], buf + *rw_index, BLOCK_SIZE);

		(*rw_index) += BLOCK_SIZE;
		if(*rw_index > buffer->size-1) *rw_index -= buffer->size;
	}

	finalise(comp);
	return 0;
}

uint16_t osc_exec(dsp_comp* comp)
{
  //TODO: Finish this
  
  float phase_accum = comp->float_params[1];
  
  int i = 0;
  for(i = 0; i < BLOCK_SIZE; i++)
  {
  	comp->pre_out[0][i] = sinf(2 * PI * phase_accum / (float)comp->int_params[0]);  
    phase_accum ++;
    if(phase_accum > comp->int_params[0])
    {
      phase_accum = phase_accum - comp->int_params[0];
    }
  }
	comp->float_params[1] = phase_accum;

	finalise(comp);
	return 0;
}

uint16_t corr_exec(dsp_comp* comp)
{
	// comp->buffer[0] : left history buffer, size: 2*(M+BLOCK_SIZE-1)
	// comp->buffer[1] : correlation output vector, size: M
	// comp->buffer[2] : reserved buffer, size: M

	// comp->int_params[0] : update flag
	// comp->int_params[1] : reset flag
	// comp->int_params[2] : stop after this many samples (if nonzero)
	// comp->int_params[3] : reserved

	if(comp->int_params[0])
	{
		component_buffer* leftHist = &comp->buffer[0];
		component_buffer* corrVec = &comp->buffer[1];
		component_buffer* temp = &comp->buffer[2];

		float *leftBuf = leftHist->buff;
		uint32_t *rw_index = &leftHist->curr;
		uint32_t M = leftHist->size/2 + 1 - BLOCK_SIZE;
		uint32_t halfSize = leftHist->size/2;

		if(comp->int_params[1])
		{
			// Reset left input buffer and correlation vector
			arm_fill_f32(0.0f, leftHist->buff, leftHist->size);
			arm_fill_f32(0.0f, corrVec->buff, corrVec->size);
			(*rw_index) = 0;
			comp->int_params[1] = 0;
			comp->int_params[3] = 0;
		}

		if(*rw_index < halfSize) *rw_index += halfSize;
		if(*rw_index > 2*halfSize - BLOCK_SIZE) *rw_index -= halfSize;

		// Store new samples of the left signal in the history (double-buffer style)
		arm_copy_f32(comp->in[0], leftBuf + *rw_index, BLOCK_SIZE);
		arm_copy_f32(comp->in[0], leftBuf + *rw_index - halfSize, BLOCK_SIZE);

		uint8_t i;
		for(i = 0; i < BLOCK_SIZE; i++)
		{
			float in = comp->in[1][i];
			if(comp->int_params[2]) in /= comp->int_params[2];
			arm_scale_f32(leftBuf + *rw_index - M + i, in, temp->buff, M);
			arm_add_f32(corrVec->buff, temp->buff, corrVec->buff, M);
		}

		(*rw_index) += BLOCK_SIZE;
		comp->int_params[3] += BLOCK_SIZE;

		// If target number of samples is reached, stop updating
		if(comp->int_params[2] && comp->int_params[3] >= comp->int_params[2])
		{
			comp->int_params[0] = 0;
		}
	}

	finalise(comp);
	return 0;
}
