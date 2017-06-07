#include "ts_lib.h"
#include "thermistor.h"

static ADC_HandleTypeDef* hadc;

static SemaphoreHandle_t tempMtxHandle;

static uint32_t aggregate[TEMP_CHANNELS];
static uint32_t sampleCount[TEMP_CHANNELS];
static uint16_t dmaBuffer[8];

static uint8_t convInProg = 0;

static void switchChannel(uint8_t channel){
	HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, channel & 0x1);
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, channel & 0x2);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, channel & 0x4);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, channel & 0x8);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(xSemaphoreTakeFromISR(tempMtxHandle, NULL)){
		for(int i=0; 16*i+convInProg < TEMP_CHANNELS; i++){
			uint8_t channel = 16*i+convInProg;
			aggregate[channel] += dmaBuffer[i];
			sampleCount[channel]++;
			if(aggregate[channel] >= 0xFFFF0000){
				aggregate[channel] /= sampleCount[channel];
				sampleCount[channel] = 1;
			}
		}
		xSemaphoreGiveFromISR(tempMtxHandle, NULL);
		convInProg++;
		convInProg%=16;
	}
	switchChannel(convInProg);
	HAL_ADC_Stop_DMA(hadc);
	HAL_ADC_Start_DMA(hadc, (uint32_t*)dmaBuffer, TEMP_MUXES); //dw about ptr types. NEVER dma more than sequenced!
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
	Serial2_writeBuf("ADC ERROR!");
}

void Temp_begin(ADC_HandleTypeDef* hadc_in){
	for(uint8_t i=0; i<TEMP_CHANNELS; i++){
		aggregate[i]=0;
		sampleCount[i]=0;
	}

	hadc = hadc_in;

	tempMtxHandle = xSemaphoreCreateMutex();

	switchChannel(convInProg);
	HAL_ADC_Start_DMA(hadc, (uint32_t*)dmaBuffer, TEMP_MUXES); //dw about ptr types. NEVER dma more than sequenced!
}

uint16_t getReading(uint8_t channel){
	static uint32_t n, c, s;
	if(channel < TEMP_CHANNELS && sampleCount[channel]/TEMP_OVERSAMPLING){
		n = aggregate[channel];		//aggregated sum of samples
		c = sampleCount[channel];	//total number of samples aggreagated
		s = dmaBuffer[channel];		//latest one sample
		if(TEMP_OVERSAMPLING == 1){
			return n/c;
		}else{
			return (n-(s*(c%TEMP_OVERSAMPLING)))/(c/TEMP_OVERSAMPLING);
			//^(the sum minus last incomplete 16) divided by oversample chunks
		}
	}else{
		return 0;
	}
}

void resetReading(uint8_t channel){
	xSemaphoreTake(tempMtxHandle, portMAX_DELAY);
	aggregate[channel] = 0;
	sampleCount[channel] = 0;
	xSemaphoreGive(tempMtxHandle);
}

int32_t getMilliCelcius(uint8_t channel){
	static uint32_t n, c;
	if(channel < TEMP_CHANNELS && sampleCount[channel]/TEMP_OVERSAMPLING){
		n = aggregate[channel];		//aggregated sum of samples
		c = sampleCount[channel];	//total number of samples aggreagated
		return adc_to_milliCelcius(n,c);
	}else{
		return 0;
	}
}

int32_t getMicroCelcius(uint8_t channel){
	static uint32_t n, c;
	if(channel < TEMP_CHANNELS && sampleCount[channel]/TEMP_OVERSAMPLING){
		n = aggregate[channel];		//aggregated sum of samples
		c = sampleCount[channel];	//total number of samples aggreagated
		return adc_to_microCelcius(n,c);
	}else{
		return 0;
	}
}
