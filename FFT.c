/**
  ******************************************************************************
  * @file     FFT.c
  * @author   Auto-generated by STM32CubeIDE
  * @version  V1.0
  * @date     09/09/2022 08:46:31
  * @brief    Default under dev library file.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "FFT.h"
/** Functions ----------------------------------------------------------------*/
/**
*@brief: get_fbin()  Calculates STM32L4,L5 series' ADC sampling frequency and frequency bin based on clock configuration
*@param: ADC_var				Store clock values
*@param: buf_len  				FFT input array length
*@param: FFT_var				Store FFT variables
*@retval: 0 if there's error, 1 if there's no error
*/
int get_fbin(struct ADC_param *ADC_var,uint32_t buf_len, struct FFT_res *FFT_var)
{
	//ensure the input values are valid
	if(ADC_var->speed<=0 || ADC_var->prescaler<=0 ||ADC_var->bit<=0 ||ADC_var->sampling_time <=0||buf_len<=0) return 0;
	//arm FFT can only receive certain input array length
	else if(!(buf_len==32||buf_len==64||buf_len==128||buf_len==256||buf_len==512||buf_len==1024||buf_len==2048||buf_len==4096))return 0;

	float clock_cycle;
	//clock cycles according to ADC bit resolution (more info, refer to stm32l4xx_hal_adc.h)
	if(ADC_var->bit==12) clock_cycle=12.5;
	else if(ADC_var->bit==10) clock_cycle=10.5;
	else if(ADC_var->bit==8) clock_cycle=8.5;
	else if(ADC_var->bit==6) clock_cycle=6.5;
	else return 0;

	float tconv=clock_cycle+ADC_var->sampling_time;
	FFT_var->fsampling=(float)ADC_var->speed/(float)ADC_var->prescaler/tconv;
	if(FFT_var->fsampling<3000)	//Since fmax =1500Hz, must at least 2 times of fmax
	{
		//Please change the clock configuration
		FFT_var->fsampling=0;
		return 0;
	}
	FFT_var->fbin=(float)ADC_var->speed/(float)ADC_var->prescaler/tconv/(float)buf_len;
	if(FFT_var->fbin>=50)		//because need to measure at least 50Hz signal
	{
		//Please change the clock configuration
		FFT_var->fbin=0;
		return 0;
	}

	return 1;
}

/*---------------------------------------------------------------*/
/**
*@brief: do_FFT() Calculates signal's dominant frequency and its magnitude using Fast Fourier Transform.
*@param: adc_buf  		The input array
*@param: buf_len  		The length of input array
*@param: FFT_var		Store FFT variables
*@retval: none
*/
void do_FFT(uint32_t adc_buf[],uint32_t buf_len,struct FFT_res *FFT_var)
{
	float float_input[buf_len];
	float output[buf_len];								//FFT output
	float result[buf_len/2];
	for(int i=0;i<buf_len;i++)
	{
		float_input[i]=(float)adc_buf[i]-FFT_var->offset;//convert to float
	}

	/*Execute Real FFT algorithm*/
	arm_rfft_fast_instance_f32 S;						//RFFT instance.
	arm_rfft_fast_init_f32(&S, buf_len);				//RFFT init
	arm_rfft_fast_f32(&S, float_input, output, 0);		//process (output will have magnitude)

	arm_cmplx_mag_f32(output, result, buf_len/2);		//magnitude
	result[0]=0;										//remove DC value

	arm_max_f32(result, buf_len/2, &FFT_var->magnitude, &FFT_var->index);//maxValue
	FFT_var->fdominant=FFT_var->index*FFT_var->fbin;	//return dominant frequency
	for(int i=0;i<buf_len;i++)							//clear old ADC values
	{
		adc_buf[i]=0;
	}
}

/*---------------------------------------------------------------*/
/**
*@brief: start_FFT()  	Start Fast Fourier Transform algorithm.
*@param: flag  			Check if complete callback interrupt is called
*@param: ADC_var		Store clock values
*@param: FFT_var		Store FFT variables
*@retval: 0 if there's error, 1 if there's no error
*/
int start_FFT(int *flag, struct ADC_param *ADC_var, struct FFT_res *FFT_var)
{
	//FFT process
	int result=get_fbin(ADC_var, ADC_var->adc_buf_len, FFT_var);
	if(result==1)		//no error
	{
		do_FFT(ADC_var->adc_buf,ADC_var->adc_buf_len,FFT_var);	//if result valid
		*flag=0;	//reset flag to 0
		return 1;
	}
	*flag=0;	//error
	return 0;
}

/*---------------------------------------------------------------*/
/**
 * Doppler radar sensor (DRS)
*@brief: get_velocity() calculates object velocity based on DRS signal.
*@param: fdominant 		DRS return  frequency;
*@param: fwave 			DRS transmitted frequency;
*@param: velocity 		Stores the calculation result
*@retval: none
*/

void get_velocity(float fdominant, float fwave, float * velocity)
{
	if(fwave>0)
	{
		*velocity=0.5*300000000*fdominant/fwave;
	}
}