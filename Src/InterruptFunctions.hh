#include "InterfacesConf.hh"

using Drivers::BusStatus;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi == &hspi2){
		Drivers::spi2.handleFinish(nullptr);
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	if(hspi == &hspi2){
		Drivers::spi2.handleError(nullptr);
	}
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi == &hspi2){
		Drivers::spi2.handleAbort(nullptr);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c == &hi2c1){
		Drivers::i2c1.handleError(nullptr);
	}
	else{
		Drivers::i2c3.handleError(nullptr);
	}
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c == &hi2c1){
		Drivers::i2c1.handleAbort(nullptr);
	}
	else{
		Drivers::i2c3.handleAbort(nullptr);
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c == &hi2c1){
		Drivers::i2c1.handleFinish(nullptr);
	}
	else{
		Drivers::i2c3.handleFinish(nullptr);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c == &hi2c1){
		Drivers::i2c1.handleFinish(nullptr);
	}
	else{
		Drivers::i2c3.handleFinish(nullptr);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *) {
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == RADIO_IRQ_Pin){
		Drivers::gpio.setChangedPin(InterruptInputList::RadioIRQ);
		Drivers::gpio.handleFinish(nullptr);
	}else if(GPIO_Pin == MPU_IRQ_Pin){
		Drivers::gpio.setChangedPin(InterruptInputList::MPUIRQ);
		Drivers::gpio.handleFinish(nullptr);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim10){
		Drivers::time++;
		Drivers::gpio.handleTimeEvent(nullptr);
		Drivers::spi2.handleTimeEvent(nullptr);
		Drivers::i2c1.handleTimeEvent(nullptr);
		Drivers::i2c3.handleTimeEvent(nullptr);
	}
}

