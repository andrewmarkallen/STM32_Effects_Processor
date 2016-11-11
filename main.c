
#include <defines.h>
#include <global.h>

#include <stm32f4xx_rcc.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_i2c.h>
#include <arm_math.h>

#include <tm_stm32f4_gpio.h>
#include <tm_stm32f4_usart.h>
#include <tm_stm32f4_delay.h>
#include <tm_stm32f4_disco.h>
#include <tm_stm32f4_cpu_load.h>

#include <stdio.h>
#include <stdarg.h>
#include <codec.h>
#include <wolfson.h>
#include <not_main.c>

#include "history.h"
#include "global.h"
#include "component.h"
#include "controller.h"
#include "RLS.h"
#include "NLMS.h"
#include "test_components.h"
#include "ANC_noise.h"
#include "wav.h"
#include "echo.h"
#include "reverb.h"
#include "lftf.h"
#include "msmftf.h"

#define LED_BLUE_ON   GPIOD->BSRRL = GPIO_Pin_15;
#define LED_BLUE_OFF  GPIOD->BSRRH = GPIO_Pin_15;

#define ECHO_BUFF_SIZE 8000
#define REV_BUFF_SIZE 512

TM_CPULOAD_t CPU_LOAD;

//int16_t H_found = 0;
float32_t P_vals[NUMBER_OF_FIELDS] = {1.0,1.0,1.0,1.0,1.0,1.0};

typedef struct {
	float tabs[8];
	float params[8];
	uint8_t currIndex;
} fir_8;

// struct to initialize GPIO pins
GPIO_InitTypeDef GPIO_InitStructure;

float lastLoad = 0.0f;
uint32_t lastSamplesProcessed = 0;

volatile uint32_t sampleCounter = 0;
volatile int16_t sample = 0;

float sawWave = 0.0;

float filteredSaw = 0.0;

int16_t left_out_sample = 0;
int16_t right_out_sample = 0;
int16_t left_in_sample = 0;
int16_t right_in_sample = 0;

controller control;

#define STR_EXPAND(X) #X
#define STR(X) STR_EXPAND(X)

/*
#define CORR_N 40
component_buffer corr_comp_buffers[2];
float corr_hist[CORR_N]; // signal on left input will be recorded into this buffer
float corr_vec[CORR_N]; // this contains the output of the correlation

#define CORR_ITERS 128000
*/
#define DELAY_SIZE 20
component_buffer delay_comp_buffer;
float delay_buff[2*DELAY_SIZE];

#define FTF_SIZE 20
#define FTF_SIZE2 20

component_buffer ftf_comp_buffers1[5];
float ftf_buff1_x[2*(FTF_SIZE+1)];
float ftf_buff1_w[FTF_SIZE];
float ftf_buff1_a[FTF_SIZE];
float ftf_buff1_k[FTF_SIZE];
float ftf_buff1_k_ext[FTF_SIZE];

dsp_comp noise;
dsp_comp delay;
dsp_comp router;
dsp_comp ftf1;

/*component_buffer ftf_comp_buffers2[5];
float ftf_buff2_x[2*(FTF_SIZE2+1)];
float ftf_buff2_w[FTF_SIZE2];
float ftf_buff2_a[FTF_SIZE2];
float ftf_buff2_k[FTF_SIZE2];
float ftf_buff2_k_ext[FTF_SIZE2];

component_buffer fir_comp_buffers[2];
float fir_hist_buff[FTF_SIZE2];

dsp_comp noise;
dsp_comp osc;
//dsp_comp clicks1;
//dsp_comp clicks2;
dsp_comp delay;
dsp_comp router1;
dsp_comp router2;
dsp_comp corr;
//dsp_comp ftf;
dsp_comp ftf1;
dsp_comp ftf2;
dsp_comp amp;
dsp_comp fir;
//dsp_comp product;
*/

/*#define NUMTAPS 128
float arm_fir_pCoeffs[NUMTAPS];
float arm_fir_pState[NUMTAPS+BLOCK_SIZE-1];
arm_fir_instance_f32 arm_fir_S;
dsp_comp arm_fir;*/

/*component_buffer fir_comp_buffers[2];
float fir_buff_w[NUMTAPS];
float fir_buff_hist[NUMTAPS];
dsp_comp fir;*/

#define SERLCD_USART USART3


void correlate(component_buffer* buff1, component_buffer* buff2, component_buffer* outbuff)
{
	// NOTE: need outbuff->size == 2*max(buff1->size, buff2->size) - 1
	outbuff->curr = 0;

	arm_correlate_f32(buff1->buff, buff1->size, buff2->buff, buff2->size, outbuff->buff);
}

uint16_t findDelay(component_buffer* corr_buff)
{
	float maxval = 0.0f, val = 0.0f;
	uint16_t maxidx = 0, j = 0;

	DEBUG_PRINTF("CC:\n");
	for(j = 0; j < corr_buff->size; j++)
	{
		val = fabs(hist(corr_buff, j));
		if(val > maxval)
		{
			maxval = val;
			maxidx = j;
		}
		DEBUG_PRINTF("%.1e,\n", hist(corr_buff, j));
	}

	for(j = maxidx; hist(corr_buff, j)*hist(corr_buff, j-1) > 0; j--) ;

	return j-1;
}

void vserprintf(char* format, va_list args)
{
	char msg[33];
	vsprintf(msg, format, args);
	TM_USART_Puts(SERLCD_USART, msg);
}

void serprintf(char* format, ...)
{
	char msg[33];
	va_list args;
	va_start(args, format);
	vserprintf(format, args);
	va_end(args);
}

void Log(char* format, ...)
{
	TM_USART_Puts(SERLCD_USART, "Log:");
	va_list args;
	va_start(args, format);
	vserprintf(format, args);
	va_end(args);
	//TM_USART_Puts(SERLCD_USART, "\n");
}

void sendFloat(float x)
{
	union {
		float f;
		unsigned char bytes[4];
	} fu;

	fu.f = x;

	uint8_t j;
	for(j=0; j<4; j++)
	{
		TM_USART_Putc(SERLCD_USART, fu.bytes[j]);
	}
}

void sendWaveform(float* buf, uint16_t num)
{

	// Find max value in buffer for scaling
	uint16_t j;
	float maxval = 0.0f;
	for(j=0; j<num; j++)
	{
		maxval = fabs(buf[j]) > maxval ? fabs(buf[j]) : maxval;
	}
	if(maxval == 0.0f)
	{
		maxval = 1.0f;
	}

	// Send metadata
	serprintf("Wav:Afl%.1e\n1\n%d\n", maxval, num);

	// Send correlation vector
	for(j=0; j<num; j++)
	{
		sendFloat(buf[j]/maxval);
	}
}

int l_ready;
int r_ready; //Do we have our L and R samples from the ADC?

float updateFilter(fir_8* theFilter, float newValue);

void initFilter(fir_8* theFilter);

void process_samples();

extern int16_t pingIN[BUFSIZE], pingOUT[BUFSIZE], pongIN[BUFSIZE], pongOUT[BUFSIZE];
volatile int16_t rx_proc_buffer, tx_proc_buffer;
volatile int16_t RX_buffer_full = 0;
volatile int16_t TX_buffer_empty = 0;
float32_t x[BUFSIZE/2], y[BUFSIZE/2];

uint32_t samplesProcessed = 0;

int main(void)
{
	DEBUG_PRINTF("Starting\n");

	SystemInit();

	TM_DISCO_ButtonInit();

	init(&control);


	/*memset(corr_hist, 0, CORR_N*sizeof(float));
	corr_comp_buffers[0].buff = corr_hist;
	corr_comp_buffers[0].size = CORR_N;
	corr_comp_buffers[0].curr = 0;

	memset(corr_vec, 0, CORR_N*sizeof(float));
	corr_comp_buffers[1].buff = corr_vec;
	corr_comp_buffers[1].size = CORR_N;
	corr_comp_buffers[1].curr = 0;*/

	memset(delay_buff, 0, 2*DELAY_SIZE*sizeof(float));
	delay_comp_buffer.buff = delay_buff;
	delay_comp_buffer.size = 2*DELAY_SIZE;
	delay_comp_buffer.curr = 0;

	memset(ftf_buff1_x, 0, 2*(FTF_SIZE+1)*sizeof(float));
	ftf_comp_buffers1[0].buff = ftf_buff1_x;
	ftf_comp_buffers1[0].size = 2*(FTF_SIZE+1);
	ftf_comp_buffers1[0].curr = 0;

	memset(ftf_buff1_w, 0, FTF_SIZE*sizeof(float));
	ftf_comp_buffers1[1].buff = ftf_buff1_w;
	ftf_comp_buffers1[1].size = FTF_SIZE;

	memset(ftf_buff1_a, 0, FTF_SIZE*sizeof(float));
	ftf_comp_buffers1[2].buff = ftf_buff1_a;
	ftf_comp_buffers1[2].size = FTF_SIZE;

	memset(ftf_buff1_k, 0, FTF_SIZE*sizeof(float));
	ftf_comp_buffers1[3].buff = ftf_buff1_k;
	ftf_comp_buffers1[3].size = FTF_SIZE;

	memset(ftf_buff1_k_ext, 0, (FTF_SIZE+1)*sizeof(float));
	ftf_comp_buffers1[4].buff = ftf_buff1_k_ext;
	ftf_comp_buffers1[4].size = FTF_SIZE+1;

	/*memset(ftf_buff2_x, 0, 2*(FTF_SIZE2+1)*sizeof(float));
	ftf_comp_buffers2[0].buff = ftf_buff2_x;
	ftf_comp_buffers2[0].size = 2*(FTF_SIZE2+1);
	ftf_comp_buffers2[0].curr = 0;

	memset(ftf_buff2_w, 0, FTF_SIZE2*sizeof(float));
	ftf_comp_buffers2[1].buff = ftf_buff2_w;
	ftf_comp_buffers2[1].size = FTF_SIZE2;

	memset(ftf_buff2_a, 0, FTF_SIZE2*sizeof(float));
	ftf_comp_buffers2[2].buff = ftf_buff2_a;
	ftf_comp_buffers2[2].size = FTF_SIZE2;

	memset(ftf_buff2_k, 0, FTF_SIZE2*sizeof(float));
	ftf_comp_buffers2[3].buff = ftf_buff2_k;
	ftf_comp_buffers2[3].size = FTF_SIZE2;

	memset(ftf_buff2_k_ext, 0, (FTF_SIZE2+1)*sizeof(float));
	ftf_comp_buffers2[4].buff = ftf_buff2_k_ext;
	ftf_comp_buffers2[4].size = FTF_SIZE2+1;

	memset(fir_hist_buff, 0, FTF_SIZE2*sizeof(float));
	fir_comp_buffers[0].buff = fir_hist_buff;
	fir_comp_buffers[0].size = FTF_SIZE2;
	fir_comp_buffers[0].curr = 0;

	fir_comp_buffers[1].buff = ftf_buff2_w;
	fir_comp_buffers[1].size = FTF_SIZE2;*/

	setup_and_reg_comp(&control, &noise, 0, 1, ANC_Noise_Exec, "noise");
	//setup_and_reg_comp(&control, &osc, 0, 1, osc_exec, "osc");
	setup_and_reg_comp_with_buffer(&control, &delay, 1, 1, delay_exec, "delay", &delay_comp_buffer);
	setup_and_reg_comp(&control, &router, 1, 2, router_exec, "router");
	//setup_and_reg_comp(&control, &router1, 3, 7, router_exec, "router1");
	//setup_and_reg_comp(&control, &router2, 2, 2, router_exec, "router2");
	//setup_and_reg_comp(&control, &amp, 1, 1, amplifier_exec, "amp");
	//setup_and_reg_comp_with_buffer(&control, &corr, 2, 0, cross_corr_exec, "corr", corr_comp_buffers);
	//setup_and_reg_comp(&control, &clicks1, 0, 1, clicks_exec, "clicks1");
	//setup_and_reg_comp(&control, &clicks2, 0, 1, clicks_exec, "clicks2");
	setup_and_reg_comp_with_buffer(&control, &ftf1, 2, 1, msmftf_exec, "ftf1", ftf_comp_buffers1);
	//setup_and_reg_comp_with_buffer(&control, &ftf2, 2, 1, msmftf_exec, "ftf2", ftf_comp_buffers2);
	//setup_and_reg_comp(&control, &product, 2, 1, mult_exec, "mult");
	//setup_and_reg_comp_with_buffer(&control, &fir, 1, 1, fir_exec, "fir", fir_comp_buffers);

	router.int_params[0] = 0x1 | 0x2;
	delay.int_params[0] = DELAY_SIZE/2;

	/*router1.int_params[0] = 0x1 | 0x4; // route noise towards dac and to cross-correlation
	router1.int_params[2] = 0x8; // route adc(r) to cross-correlation

	router2.int_params[0] = 0x2; // route noise to dac(r)
	router2.int_params[1] = 0x0; // silence fir

	delay.int_params[0] = 0; // delay in samples

	corr.int_params[0] = 1; // update cross-correlation
	corr.int_params[1] = 1; // reset before first iteration
	corr.int_params[2] = CORR_ITERS; // finish after this many samples (can restart by setting above parameters to 1)*/

	/*clicks1.int_params[0] = 53;
	clicks1.int_params[1] = 0;

	clicks2.int_params[0] = 53;
	clicks2.int_params[1] = -16;*/

	//osc.int_params[0] = 50; // @8kHz -> 160Hz

	ftf1.float_params[0] = 0.997; // lambda
	ftf1.float_params[1] = 10;    // mu
	ftf1.float_params[2] = 0.995; // v1
	ftf1.float_params[3] = 1e-07; // c1

	ftf1.int_params[0] = 1; // update
	ftf1.int_params[1] = 1; // reset
	ftf1.int_params[2] = 0; // apply
	ftf1.int_params[3] = 0; // err_in
	ftf1.int_params[4] = 0; // err_out

	/*ftf2.float_params[0] = 0.997; // lambda
	ftf2.float_params[1] = 10;    // mu
	ftf2.float_params[2] = 0.995; // v1
	ftf2.float_params[3] = 1e-07; // c1

	ftf2.int_params[0] = 0; // update
	ftf2.int_params[1] = 0; // reset
	ftf2.int_params[2] = 0; // apply
	ftf2.int_params[3] = 0; // err_in
	ftf2.int_params[4] = 0; // err_out

	fir.int_params[0] = 0; // apply

	amp.int_params[0] = -1024;*/

	/*
	// adc(r) [for now]
	attach_ADC(&control, 0, &router1, 2);
	// adc(l) [for now]
	attach_ADC(&control, 1, &control.null_component, 1);

	set_target(&noise, 0, &router1, 0);
	set_target(&osc, 0, &router1, 1);

	set_target(&router1, 0, &router2, 0);

	set_target(&router1, 2, &corr, 0);
	set_target(&router1, 3, &corr, 1);

	set_target(&router1, 4, &delay, 0);
	set_target(&delay, 0, &ftf1, 0);
	set_target(&router1, 5, &ftf1, 1);

	set_target(&ftf1, 0, &ftf2, 0);
	set_target(&router1, 6, &ftf2, 1);

	set_target(&router1, 1, &fir, 0);
	set_target(&fir, 0, &amp, 0);
	set_target(&amp, 0, &router2, 1);

	set_target(&router2, 0, &control.dac_out, 0);
	set_target(&router2, 1, &control.dac_out, 1);*/

	//set_target(&router, 1, &corr, 0);
	//set_target(&router, 0, &ftf, 0);
	//set_target(&router, 1, &delay, 0);
	//set_target(&delay, 0, &ftf, 1);
	//set_target(&clicks1, 0, &ftf, 0);
	//set_target(&clicks2, 0, &ftf, 1);

	/*arm_fill_f32(0, arm_fir_pCoeffs, NUMTAPS);
	arm_fir_init_f32(&arm_fir_S, NUMTAPS, arm_fir_pCoeffs, arm_fir_pState, BLOCK_SIZE);

	setup_and_reg_comp(&control, &arm_fir, 1, 1, arm_fir_exec, "arm_fir");

	arm_fir.int_params[0] = 1;
	arm_fir.pInstance = &arm_fir_S;*/

	/*memset(fir_buff_hist, 0, NUMTAPS*sizeof(float));
	fir_comp_buffers[0].buff = fir_buff_hist;
	fir_comp_buffers[0].size = NUMTAPS;
	fir_comp_buffers[0].curr = 0;

	memset(fir_buff_w, 0, NUMTAPS*sizeof(float));
	fir_comp_buffers[1].buff = fir_buff_w;
	fir_comp_buffers[1].size = NUMTAPS;

	setup_and_reg_comp_with_buffer(&control, &fir, 1, 1, fir_exec, "fir", fir_comp_buffers);

	fir.int_params[0] = 1;*/

	//setup_and_reg_comp(&control, &noise, 0, 1, ANC_Noise_Exec, "noise");
	//setup_and_reg_comp(&control, &router, 1, 2, router_exec, "router");

	noise.float_params[0] = -1.0f;
	noise.float_params[1] = 1.0f;

	router.int_params[0] = 0x1 | 0x2;

	attach_ADC(&control, 0, &control.null_component, 0);
	attach_ADC(&control, 1, &control.null_component, 1);
	//attach_ADC(&control, 1, &arm_fir, 0);
	//attach_ADC(&control, 1, &fir, 0);
	//attach_ADC(&control, 0, &router, 0);
	//attach_ADC(&control, 1, &router, 1);

	set_target(&noise, 0, &router, 0);
	set_target(&router, 0, &ftf1, 0);
	set_target(&router, 1, &delay, 0);
	set_target(&delay, 0, &ftf1, 1);

	//set_target(&router, 0, &control.dac_out, 0);
	//set_target(&delay, 0, &control.dac_out, 1);

	//set_target(&fir, 0, &control.dac_out, 0);
	//set_target(&arm_fir, 0, &control.dac_out, 1);
	set_target(&control.null_component, 0, &control.dac_out, 0);
	set_target(&control.null_component, 1, &control.dac_out, 1);

	//attach_ADC(&control, 0, &control.dac_out, 0);
	//attach_ADC(&control, 1, &control.dac_out, 1);

	build_evaluation_order(&control);


	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	DEBUG_PRINTF("SYSCLK: %dMHz\n", clocks.SYSCLK_Frequency/1000000);

	/* Initialize delay */
	TM_DELAY_Init();
	//Initialize USART6 at some baud, TX: PC6, RX: PC7
	// Timer error with TM_USART requires a recalc of the baud rate by 231/127
	//TM_USART_Init(USART6, TM_USART_PinsPack_1, (231*230400)/127);
	/* Init pins PC8, 9, 14, & 15 for output, no pull-up, and high speed*/
	TM_GPIO_Init(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_14 | GPIO_Pin_15,
			 TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL,
			 TM_GPIO_Speed_High);
	//stm32_wm5102_init(FS_48000_HZ, WM5102_LINE_IN, IO_METHOD_INTR);

	TM_USART_Init(SERLCD_USART, TM_USART_PinsPack_2, 19200);

	stm32_wm5102_init(FS_48000_HZ, WM5102_LINE_IN, IO_METHOD_DMA);

	TM_RNG_Init();
	TM_CPULOAD_Init(&CPU_LOAD);

	char type = 0;

	while(1)
	{
		if(CPU_LOAD.Updated)
		{
			//printf("%3.1f%% (%d samples)\n", CPU_LOAD.Load, samplesProcessed);

			Log(" %3.1f%% (%d)\n", CPU_LOAD.Load, samplesProcessed);
			lastLoad = CPU_LOAD.Load;
			lastSamplesProcessed = samplesProcessed;
			samplesProcessed = 0;
		}

		while(!(RX_buffer_full && TX_buffer_empty))
		{

		}
		sampleCounter++;
		//step(&control, 1,1);
		if (sampleCounter==2400)
		{
			LED_BLUE_OFF;

		}
		else if (sampleCounter == 4800)
		{
			LED_BLUE_ON;
			sampleCounter = 0;
		}

		process_samples();

		TM_CPULOAD_GoToSleepMode(&CPU_LOAD, TM_LOWPOWERMODE_SleepUntilInterrupt);
	}

	/*

	//Everything below here is for on-board audio
	fir_8 filt;

	//enables GPIO clock for PortD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


	codec_init();
	codec_ctrl_init();

	I2S_Cmd(CODEC_I2S, ENABLE);

	initFilter(&filt);

    while(1)
    {

    	if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
    	{
    		SPI_I2S_SendData(CODEC_I2S, sample);

    		//only update on every second sample to insure that L & R ch. have the same sample value
    		if (sampleCounter & 0x00000001)
    		{
    			sawWave += NOTEFREQUENCY;
    			if (sawWave > 1.0)
    				sawWave -= 2.0;

    			filteredSaw = updateFilter(&filt, sawWave);

    			sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
    		}
    		sampleCounter++;
    	}

    	if (sampleCounter==48000)
    	{
    		LED_BLUE_OFF;

    	}
    	else if (sampleCounter == 96000)
    	{
    		LED_BLUE_ON;
    		sampleCounter = 0;
    	}

    }

    while(1)
    {
    	i++;
    }

    */
}



void DMA1_Stream3_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3))
	{
		DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream3))
		{
			rx_proc_buffer = PING;
		}
		else
		{
			rx_proc_buffer = PONG;
		}
		RX_buffer_full = 1;
	}
}

void DMA1_Stream4_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4))
	{
		DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream4))
		{
			tx_proc_buffer = PING;
		}
		else
		{
			tx_proc_buffer = PONG;
		}
		TX_buffer_empty = 1;
	}
}

void process_samples()
{
	int i;
	  int16_t *rxbuf, *txbuf;
	if (rx_proc_buffer == PING) rxbuf = pingIN; else rxbuf = pongIN;
	if (tx_proc_buffer == PING) txbuf = pingOUT; else txbuf = pongOUT;

	static unsigned char buttonPressed = 0;
	//static unsigned char calibrating = 1;
	static unsigned int debounce = 0;

	if(TM_DISCO_ButtonPressed())
	{
		if(!buttonPressed && !debounce)
		{
			/*switch(calibrating)
			{
			case 0:
				Log(" c%d load:%.0f%%(%d)\n", calibrating, lastLoad, lastSamplesProcessed);
				router1.int_params[0] = 0x1 | 0x4; // route noise towards dac and to cross-correlation
				router1.int_params[1] = 0x0;       // silence sinusoid
				router1.int_params[2] = 0x8;       // route adc(r) to cross-correlation
				router2.int_params[0] = 0x2;       // route noise to dac(r)
				router2.int_params[1] = 0x0;       // silence fir
				fir.int_params[0] = 0;             // stop applying fir
				corr.int_params[0] = 1;            // start updating cross-correlation
				corr.int_params[0] = 1;            // reset cross-correlation
				calibrating = 1;
				break;
			case 1:
				// cross-correlation mode will end automatically, ignore button input
				break;
			case 2:
				sendWaveform(ftf_buff1_w, FTF_SIZE);
				Log(" c%d load:%.0f%%(%d)\n", calibrating, lastLoad, lastSamplesProcessed);
				router1.int_params[0] = 0x1 | 0x2 | 0x10; // route noise towards dac, to ftf2 and to delay
				router1.int_params[1] = 0x0;              // silence sinusoid
				router1.int_params[2] = 0x40;             // route adc(r) to ftf2
				router2.int_params[0] = 0x1;              // route noise to dac(l)
				router2.int_params[1] = 0x0;              // silence fir
				ftf1.int_params[0] = 0;                   // stop updating ftf1
				ftf1.int_params[2] = 1;                   // start applying ftf1
				ftf2.int_params[0] = 1;                   // start updating ftf2
				ftf2.int_params[1] = 1;                   // reset ftf2
				calibrating = 3;
				break;
			case 3:
				sendWaveform(ftf_buff2_w, FTF_SIZE2);
				Log(" c%d load:%.0f%%(%d)\n", calibrating, lastLoad, lastSamplesProcessed);
				router1.int_params[0] = 0x0;       // silence noise
				router1.int_params[1] = 0x1 | 0x2; // route sinusoid towards dac and to fir
				router1.int_params[2] = 0x0;       // silence adc(r)
				router2.int_params[0] = 0x1;       // route sinusoid to dac(l)
				router2.int_params[1] = 0x2;       // route fir to dac(r)
				ftf1.int_params[0] = 0;            // stop updating ftf1
				ftf1.int_params[2] = 0;            // stop applying ftf1
				ftf2.int_params[0] = 0;            // stop updating ftf2
				ftf2.int_params[2] = 0;            // stop applying ftf2
				fir.int_params[0] = 1;             // start applying fir
				calibrating = 0;
				break;
			}*/

			sendWaveform(ftf_buff1_w, FTF_SIZE);

			buttonPressed = 1;
			debounce = 8000;
		}
	}
	else
	{
		if(buttonPressed && !debounce)
		{

			buttonPressed = 0;
			debounce = 8000;
		}
	}

	//char corr_busy_before = corr.int_params[0];

	uint8_t k;

	for (i=0 ; i<(BUFSIZE/2) ; i+=DOWNSAMPLING)
	{
		step(&control, conv_from_adc(rxbuf[2*i]), conv_from_adc(rxbuf[2*i+1]));
		for(k=0; k<DOWNSAMPLING; k++)
		{
			txbuf[2*i] = conv_to_dac(control.out[0]);
			txbuf[2*i+1] = conv_to_dac(control.out[1]);
		}
		samplesProcessed++;
		if(debounce > 0) debounce--;
	}

	/*if(calibrating == 1 && corr_busy_before && !corr.int_params[0])
	{
		sendWaveform(corr_vec, CORR_N);
		Log(" c%d load:%.0f%%(%d)\n", calibrating, lastLoad, lastSamplesProcessed);

		float maxval = 0.0f;
		uint16_t maxidx = 0;
		for(k=0; k<CORR_N; k++)
		{
			if(corr_vec[k] > maxval)
			{
				maxval = corr_vec[k];
				maxidx = k;
			}
		}
		delay.int_params[0] = maxidx - 3;
		if(delay.int_params[0] < 0) delay.int_params[0] = 0;

		router1.int_params[0] = 0x1 | 0x10; // route noise towards dac and to delay
		router1.int_params[1] = 0x0;        // silence sinusoid
		router1.int_params[2] = 0x20;       // route adc(r) to ftf1
		router2.int_params[0] = 0x2;        // route noise to dac(r)
		router2.int_params[1] = 0x0;        // silence fir

		ftf1.int_params[0] = 1; // start updating ftf1
		ftf1.int_params[1] = 1; // reset ftf1

		calibrating = 2;
	}*/

	TX_buffer_empty = 0;
	RX_buffer_full = 0;
}



/*void SPI2_IRQHandler()
{
	int16_t* lptr;
	int16_t* rptr;
	//Check for button press
	if(TM_DISCO_ButtonPressed())
	{
		lptr = &control.out[0];
		rptr = &control.out[1];

	}
	else
	{
		lptr = &left_in_sample;
		rptr = &right_in_sample;
	}

	TM_GPIO_SetPinHigh(GPIOC, GPIO_Pin_8);

  if (SPI_I2S_GetFlagStatus(I2Sx, I2S_FLAG_CHSIDE) == SET)
  {

	TM_GPIO_SetPinHigh(GPIOC, GPIO_Pin_9);
	left_in_sample = SPI_I2S_ReceiveData(I2Sx);
	l_ready = 1;
	left_out_sample = (int16_t) (P_vals[0]*left_in_sample);

	    while (SPI_I2S_GetFlagStatus(I2Sxext, SPI_I2S_FLAG_TXE ) != SET){}
	    //SPI_I2S_SendData(I2Sxext, left_out_sample);
	    //SPI_I2S_SendData(I2Sxext, control.out[0]);
	    SPI_I2S_SendData(I2Sxext, *lptr);
		TM_GPIO_SetPinLow(GPIOC, GPIO_Pin_9);
  }
  else
  {

	TM_GPIO_SetPinHigh(GPIOC, GPIO_Pin_14);
	right_in_sample = SPI_I2S_ReceiveData(I2Sx);
	r_ready = 1;
	right_out_sample = (int16_t)(P_vals[1]*right_in_sample);

    while (SPI_I2S_GetFlagStatus(I2Sxext, SPI_I2S_FLAG_TXE ) != SET){}
    //SPI_I2S_SendData(I2Sxext, right_out_sample);
    //SPI_I2S_SendData(I2Sxext, control.out[1]);
    SPI_I2S_SendData(I2Sxext, *rptr);
	TM_GPIO_SetPinLow(GPIOC, GPIO_Pin_14);
  }

  if(l_ready && r_ready)
  {
	  step(&control, left_in_sample, right_in_sample);
	  l_ready = 0;
	  r_ready = 0;

  }
	TM_GPIO_SetPinLow(GPIOC, GPIO_Pin_8);
}*/


// a very crude FIR lowpass filter
float updateFilter(fir_8* filt, float val)
{
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter)
{
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 0.01;
	theFilter->params[1] = 0.05;
	theFilter->params[2] = 0.12;
	theFilter->params[3] = 0.32;
	theFilter->params[4] = 0.32;
	theFilter->params[5] = 0.12;
	theFilter->params[6] = 0.05;
	theFilter->params[7] = 0.01;
}

void setup_sample_process(dsp_comp* noise, dsp_comp* mixer, dsp_comp* swap, dsp_comp* amp1, dsp_comp* amp2)
{
		setup_and_reg_comp(&control, noise, 0, 1, noise_exec, "noise");
		//setup_and_reg_comp(&control, mixer, 2, 1, mixer_exec, "mixer");
		//setup_and_reg_comp(&control, swap, 2, 2, swap_exec, "swap");
		/*setup_and_reg_comp(&control, amp1, 1, 1, amplifier_exec, "amp1");
		setup_and_reg_comp(&control, amp2, 1, 1, amplifier_exec, "amp2");

		amp1->int_params[0] = 1024;
		amp2->int_params[0] = 1024;
		noise->int_params[0] = 0xBEEF; //Random seed*/

		attach_ADC(&control, 0, &control.null_component, 0);
		attach_ADC(&control, 1, &control.null_component, 1);

		set_target(noise, 0, &control.dac_out, 0);
		//set_target(swap, 1, &control.dac_out, 0);

		/*set_target(noise, 0, mixer, 0);
		set_target(mixer, 0, swap, 0);
		set_target(swap, 0, amp1, 0);
		set_target(swap, 1, amp2, 0);
		set_target(amp1, 0, &control.dac_out, 0);
		set_target(amp2, 0, &control.dac_out, 1);*/
}
