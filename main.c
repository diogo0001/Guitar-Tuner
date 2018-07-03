/**********************************************************************************************
 *
 * 	Autor: Diogo Tavares
 *  Afinador digital para guitarra
 *
 **********************************************************************************************/

#include <stm32f4xx.h>
#include <arm_math.h>
#include <stm32f4_discovery.h>
#include <stm32f4_discovery_accelerometer.h>
#include <wolfson_pi_audio.h>
#include <diag/Trace.h>
#include <tests.h>
#include <dwt.h>
#include "filter.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rcc.h"
//#include "math_helper.h"

#define DEBUG_FFT_PEAKS
#define DEBUG_BUFFER_VALUES

// Resolução maxima da fft: 8000/512
#define BLOCK_SIZE (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE)/4
#define WOLFSON_PI_FS   8000		// 48000
#define FS				48000

// Notas para o afinador pelos indices da fft
#define E2	11
#define A2	14
#define D3	19
#define G3	25
#define B3	32
#define E4	42

//  Pinos dos leds
#define NOTE_PIN	GPIO_PIN_13
#define DW_PIN		GPIO_PIN_12
#define UP_PIN		GPIO_PIN_14

FilterTypeDef filterType=IIR_DF1_FLOAT32;
int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
float32_t dlyBuffer[BLOCK_SIZE];

GPIO_InitTypeDef GPIO_InitStructure;
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
__IO uint8_t Volume = 70;
uint32_t AcceleroTicks;
int16_t AcceleroAxis[3];

//-----------------------------------------------------------------------

// Detecta o tom diretamente pelo indice do buffer da fft.
// Os indices foram aferidos pelos tons de uma senoide pura,
// que foram tocados e detectados em que indice correspondiam no buffer
// utiizando o trace_printf()

uint32_t check_note(uint32_t note)
{

	if(note==E2)	return 0;  // esta na nota
	if(note==A2)	return 0;
	if(note==D3)	return 0;
	if(note==G3)	return 0;
	if(note==B3)	return 0;
	if(note==E4)	return 0;

	if((note<E2) & (note>=E2-2))	return 1; // esta abaixo
	if((note<=E2+1) & (note>E2)) 	return 2; // esta acima

	if((note<A2) & (note>=A2-1))	return 1;
	if((note<=A2+2) & (note>A2)) 	return 2;

	if((note<D3) & (note>=D3-2))	return 1;
	if((note<=D3+2) & (note>D3)) 	return 2;

	if((note<G3) & (note>=G3-3))	return 1;
	if((note<=G3+2) & (note>G3)) 	return 2;

	if((note<B3) & (note>=B3-3))	return 1;
	if((note<=B3+3) & (note>B3)) 	return 2;

	if((note<E4) & (note>=E4-4))	return 1;
	if((note<=E4+6) & (note>E4)) 	return 2;

	return -1;
}
//-----------------------------------------------------------------------

// Ha varios harmonicos no sinal da guitarra, alguns maiores que o tom fundamental,
// deseja-se adquirir o primeiro harmonico (fundamental)
// ... função nao traz resultados como esperado ...

uint32_t first_peak(float32_t *buffer, float32_t *mean)
{
	uint32_t ind = 0;
	uint32_t i;
	float32_t acc = 0;
	float32_t med;  // para substituir o mean

	for(i=0;i<256;i++){				// nao seria necessario varrer o buffer todo
		acc = buffer[i] + acc;		// resultado obtido incompreendido
	}

	acc = acc/256;

	arm_mean_f32(buffer,BLOCK_SIZE/2,&med);

	for(i=0;i<BLOCK_SIZE/2;i++){

		if(buffer[i]>med){
			ind = i;
			break;
		}
	}

	return ind;
}


//***********************************************************************
int main(int argc, char* argv[])
{
	UNUSED(argc);
	UNUSED(argv);

	uint32_t cycleCount;
	uint32_t i,aux,L,R;
	float32_t note_amp = 0;
	uint32_t note_ind = 0;

	float32_t inputF32Buffer_L[BLOCK_SIZE]; 		// buffers maiores ocasionam muitos problemas
	float32_t outputF32Buffer_L[BLOCK_SIZE];
	float32_t outFFTBuffer_L[BLOCK_SIZE];

/*	float32_t inputF32Buffer_R[BLOCK_SIZE];
	float32_t outputF32Buffer_R[BLOCK_SIZE];
	float32_t outFFTBuffer_R[BLOCK_SIZE];
*/

	float32_t  *inputF32_L, *outputF32_L;
	//float32_t  *inputF32_R, *outputF32_R;
	//arm_status status;

	inputF32_L = &inputF32Buffer_L[0];
	//inputF32_R = &inputF32Buffer_R[0];
	outputF32_L = &outFFTBuffer_L[0];
	//outputF32_R = &outFFTBuffer_R[0];


	HAL_Init();
	DWT_Enable();
	WOLFSON_PI_AUDIO_Init((INPUT_DEVICE_LINE_IN << 8) | OUTPUT_DEVICE_BOTH, 80,WOLFSON_PI_FS);// AUDIO_FREQUENCY_48K);
	WOLFSON_PI_AUDIO_SetInputMode(INPUT_DEVICE_LINE_IN);
	WOLFSON_PI_AUDIO_SetMute(AUDIO_MUTE_ON);
	WOLFSON_PI_AUDIO_Play(TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);
	WOLFSON_PI_AUDIO_SetVolume(Volume);
	BSP_ACCELERO_Init();
	TEST_Init();


	HAL_Init();
	// LEDS -----------------------------------------
    __GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = GPIO_PIN_12;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    __GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = GPIO_PIN_13;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    __GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = GPIO_PIN_14;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);


/*    HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_SET);
*/
	HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_RESET);


    // Inicializa FFT -------------------------------------------------------

	arm_rfft_fast_instance_f32 S_L, S_R;
	arm_status status;

	status = arm_rfft_fast_init_f32(&S_L, BLOCK_SIZE);
	if (status==ARM_MATH_ARGUMENT_ERROR) {
		trace_printf("PROBLEM at arm_rfft_fast_init_f32 (LEFT CHANNEL).\n");
		while(1);
	}

	status = arm_rfft_fast_init_f32(&S_R,BLOCK_SIZE);
	if (status==ARM_MATH_ARGUMENT_ERROR) {
		trace_printf("PROBLEM at arm_rfft_fast_init_f32 (RIGHT CHANNEL).\n");
		while(1);
	}

	trace_printf("End initialization.\n");

	// 	LOOP PRINCIPAL ------------------------------------------------------------------------------

	while (1) {
		// Add your code here.
		if(buffer_offset == BUFFER_OFFSET_HALF)
		{
			DWT_Reset();

			cycleCount = DWT_GetValue();

			//INPUT HALF
			for(i=0, L=0,R=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++) {

				if(i%2) {				 // aplicado uma pre amp para a guitarra ligada diretamente
					inputF32Buffer_L[L] = 2*(float32_t)(RxBuffer[i]/32768.0);//convert to float LEFT

					if(inputF32Buffer_L[L] > 0.98)	//tratamento para saturação
						inputF32Buffer_L[L]  = 0.987;

					//trace_printf("\nblock H Rx:\n L=%d  val=%f\n",i,inputF32Buffer_L[L]);

					L++;
				}
				else {
					//inputF32Buffer_R[R] = 2*(float32_t)(RxBuffer[i]/32768.0);//convert to float RIGHT

					//if(inputF32Buffer_R[R] > 0.98)	// aplicado um ganho, tratamento para saturação
					//		inputF32Buffer_R[R]  = 0.987;

					//R++;
				}
			}

			// PROCESSAMENTO -----------------------------------------------


			arm_rfft_fast_f32(&S_L, inputF32Buffer_L, outFFTBuffer_L, 0);
			//arm_rfft_fast_f32(&S_R, outputF32Buffer_R, outFFTBuffer_R, 0);

			// Varre o buffer do canal para separar a parte real, resulta em um buffer com metade do tamanho
			for(i=0,L=0; i < BLOCK_SIZE; i++){
				// pega a parte real
				if(i%2==0){

					outFFTBuffer_L[L] = outFFTBuffer_L[i]/BLOCK_SIZE;
					//trace_printf("\nblock H:\n k=%d  val=%f\n",i,outFFTBuffer_L[L]);
					//outFFTBuffer_R[L] = outFFTBuffer_L[i];
					L++;
				}
			}

			// Detecta o pico da harmonica fundamental

			arm_abs_f32(outFFTBuffer_L,outFFTBuffer_L,BLOCK_SIZE/2);
			arm_max_f32(outFFTBuffer_L,BLOCK_SIZE/2,&note_amp,&note_ind);		// retorna o valor maximo, que na guitarra sera sempre o tom errado
			//arm_mean_f32(outFFTBuffer_L,BLOCK_SIZE/2,&note_amp);				// retorna o valor medio em note_amp (nao funiona)
			//note_ind = first_peak(outFFTBuffer_L,&note_amp);					// nao funciona

			aux = check_note(note_ind);		// verifica que nota é para acender o respectivo led

			// Debug
			trace_printf("BLOCK H:	 amp: %f 	 ind: %d 	 sel: %d\n",note_amp,note_ind,aux);

			switch(aux){

				case 0:
					HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_RESET);
					break;

				case 1:
					HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_RESET);
					break;

				case 2:
					HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_RESET);
					break;

				default:
					HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_RESET);
					break;
			}


			//OUTPUT HALF  ----- mandando só um canal
			for(i=0, L=0,R=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++) {
				if(i%2)	{
					TxBuffer[i] = (int16_t)(outputF32Buffer_L[L]*32768/2);//back to 1.15
					L++;
				}
				else{
					//TxBuffer[i] = (int16_t)(outputF32Buffer_R[R]*32768/2);//back to 1.15
					//R++;
				}
			}

			buffer_offset = BUFFER_OFFSET_NONE;
		}

		if(buffer_offset == BUFFER_OFFSET_FULL)
		{
			DWT_Reset();

			cycleCount = DWT_GetValue();


			// INPUT FULL
			for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), L=0,R=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++) {

				if(i%2) {
					inputF32Buffer_L[L] = 2*(float32_t)(RxBuffer[i]/32768.0);//convert to float LEFT

					if(inputF32Buffer_L[L] > 0.98)	// aplicado um ganho, tratamento para saturação
						inputF32Buffer_L[L]  = 0.987;

					L++;
				}
				else {
					//inputF32Buffer_R[R] = 2*(float32_t)(RxBuffer[i]/32768.0);//convert to float RIGHT

					//if(inputF32Buffer_R[R] > 0.98)	// aplicado um ganho, tratamento para saturação
					//		inputF32Buffer_R[R]  = 0.987;

					//R++;
				}
			}


			// PROCESSAMENTO -----------------------------------------------------------

			arm_rfft_fast_f32(&S_L, inputF32Buffer_L, outFFTBuffer_L, 0);
			//arm_rfft_fast_f32(&S_R, outputF32Buffer_R, outFFTBuffer_R, 0);

			// Varre o buffer do canal para separar a parte real, resulta em um buffer com metade do tamanho
			for(i=0,L=0; i < BLOCK_SIZE; i++){
				// pega a parte real
				if(i%2==0){

					outFFTBuffer_L[L] = outFFTBuffer_L[i]/BLOCK_SIZE;
					//trace_printf("\nblock H:\n k=%d  val=%f\n",i,outFFTBuffer_L[L]);
					//outFFTBuffer_R[L] = outFFTBuffer_L[i];
					L++;
				}
			}

			// Detecta o pico da harmonica fundamental

			arm_abs_f32(outFFTBuffer_L,outFFTBuffer_L,BLOCK_SIZE/2);
			arm_max_f32(outFFTBuffer_L,BLOCK_SIZE/2,&note_amp,&note_ind);
			//arm_mean_f32(outFFTBuffer_L,BLOCK_SIZE/2,&note_amp);				// retorna o valor medio em note_amp
			//note_ind = first_peak(outFFTBuffer_L,&note_amp);

			aux = check_note(note_ind);		// verifica que nota é para acender o respectivo led

			// Debug
			trace_printf("BLOCK F:	 amp: %f 	 ind: %d 	 sel: %d\n",note_amp,note_ind,aux);

			switch(aux){

				case 0:
					HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_RESET);
					break;

				case 1:
					HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_RESET);
					break;

				case 2:
					HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_RESET);
					break;

				default:
					HAL_GPIO_WritePin(GPIOD, NOTE_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, DW_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, UP_PIN, GPIO_PIN_RESET);
					break;
			}


			//OUTPUT  FULL
			for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), L=0,R=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++) {
				if(i%2)	{
					TxBuffer[i] = (int16_t)(outputF32Buffer_L[L]*32768/2);//back to 1.15
					L++;
				}
				else{
					//TxBuffer[i] = (int16_t)(outputF32Buffer_R[R]*32768/2);//back to 1.15
					//R++;
				}
			}

			buffer_offset = BUFFER_OFFSET_NONE;
		}

		TEST_Main();
	}

	return 0;
}

//--------------------------------------------------------------------------------------------

/*--------------------------------
Callbacks implementation:
--------------------------------------------------------*/

/**
  * @brief  Manages the DMA full Transfer complete event.
  */
void WOLFSON_PI_AUDIO_TransferComplete_CallBack(void)
{
	buffer_offset = BUFFER_OFFSET_FULL;
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  */
void WOLFSON_PI_AUDIO_HalfTransfer_CallBack(void)
{
	  buffer_offset = BUFFER_OFFSET_HALF;
}

/**
  * @brief  Manages the DMA FIFO error interrupt.
  * @param  None
  * @retval None
  */
void WOLFSON_PI_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1);
}

//-----------------------------------------------------------------------------


