#include "nlms.h"

// comp->buffer[0] : input signal (M)
// comp->buffer[1] : filter taps (M)
// comp->int_params[0] : update
// comp->int_params[1] : reset
// comp->int_params[2] : apply
// comp->int_params[3] : err_in
// comp->int_params[4] : err_out
// comp->float_params[0] : mu <= 1
// comp->float_params[1] : gamma << 1
// comp->float_params[2] : (reserved) (must set to 0.0f)


uint16_t nlms_exec(dsp_comp* comp)
{
  // component_buffer* buffer = &comp->buffer[0];
//
//   // Store incoming sample in ring buffer
//   buffer->curr = hist_idx(buffer, 1);
//   hist(buffer, 0) = (float)comp->in[0]/32768.0f;
//   // Compute signal energy recursively
//   //comp->float_params[2] += hist(buffer, 0) * hist(buffer, 0) - hist(buffer, 1) * hist(buffer, 1);
//
//
//   float err, sum;
//   uint32_t j, n_order = buffer->size;
//
//   // Get filter tap array
//   float *w = comp->buffer[1].buff;
//
//   if(comp->int_params[1])
//   {
//     // Reset filter taps
//     memset(w, 0, n_order*sizeof(float));
//     comp->int_params[1] = 0;
//     //comp->float_params[2] = 0.0f;
//   }
//
//   if(comp->int_params[2] || comp->int_params[0])
//   {
//     // Apply filter
//     sum = 0.0f;
//     for(j=0; j<n_order; j++)
//     {
//     sum += w[j] * hist(buffer, -j);
//     }
//     // Compute the error, unless it is given as input
//     err = (float)comp->in[1]/32768.0f - sum * !(comp->int_params[3]);
//     if(comp->int_params[4])
//     {
//       // Output the error sample
//       comp->pre_out[0] = (int32_t)(32786.0f*err);
//     }
//     else
//     {
//       // Output the filtered sample
//       comp->pre_out[0] = (int32_t)(32768.0f*sum);
//     }
//   }
//   else
//   {
//     comp->pre_out[0] = 0;
//   }
//
//   if(comp->int_params[0])
//   {
//     // Update adaptive filter
//
//     // Get signal energy
//     //sum = comp->float_params[2];
//   sum = 0.0f;
//   for(j=0; j<n_order; j++)
//   {
//     sum += hist(buffer, -j) * hist(buffer, -j);
//   }
//     // Regularise in case of small energy
//     sum += comp->float_params[1] * comp->float_params[1] * (float)n_order;
//
//     // Normalize error
//     err *= comp->float_params[0] / sum;
//
//     if(err != err)
//     {
//       //DEBUG_PRINTF("NLMS: err = %f\n", err);
//       comp->int_params[0] = 0;
//     }
//     else
//     {
//       // Update filter taps
//       for(j=0; j<n_order; j++)
//       {
//         w[j] += hist(buffer, -j) * err;
//       }
//     }
//   }
  
  finalise(comp);
  return 0;
}
