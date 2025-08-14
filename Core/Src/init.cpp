/*
 * init.cpp
 *
 *  Created on: Aug 1, 2025
 *      Author: jeffr
 */

#include "init.h"


/* Initialize all configured peripherals */
void init(){
	LL_TIM_ClearFlag_UPDATE(TIM1);
	LL_TIM_EnableIT_UPDATE(TIM1);
	LL_TIM_EnableCounter(TIM1);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	LL_mDelay(1); // ★ 加入延遲穩定方向切換（必要！）
	uart4_dma_tx_start();
	usart6_dma_tx_start();
	LL_USART_EnableIT_RXNE(USART3);
	LL_USART_Enable(USART3);
	LL_USART_EnableIT_RXNE(UART5);
	LL_USART_Enable(UART5);
	NVIC_SetPriority(USART3_IRQn, 0);
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_SetPriority(UART5_IRQn, 0);
	NVIC_EnableIRQ(UART5_IRQn);

	printf("start\r\n");
	LL_mDelay(100);
	UART4_DMA_Config();
	USART6_DMA_Config();
	LL_USART_Enable(UART4);
	LL_USART_Enable(USART6);
	uint8_t ID_list[6] = { 1, 2, 3, 4, 5, 6 };
	SyncWrite_StatusReturnLevel(6, ID_list, 1);
	LL_mDelay(10);
	for (int i = 0;i <= 10; i++){
		SyncWrite_DisableDynamixels(6, ID_list);
    }

	for(int id = 1; id <= 6; id++) {
		TorqueEnable(id,0);
		while (dynamixel_Ready != 1);
        LL_mDelay(1);
    }

    for(int id = 1; id <= 6; id++) {
        OperatingMode(id, POSITION);
        while (dynamixel_Ready != 1);
        LL_mDelay(1);
    }

    for(int id = 1; id <= 2; id++) {
  	  TorqueEnable(id,1);
  	  while (dynamixel_Ready != 1);
        LL_mDelay(1);
    }
    SyncWrite_EnableDynamixels(6, ID_list);
    LL_mDelay(1);
    PositionWithVelocity(2,(RAD2DEG(0)+180)/0.088,20);
    LL_mDelay(1000);
    PositionWithVelocity(2,(RAD2DEG(0)+180)/0.088,20);
    LL_mDelay(1000);
    PositionWithVelocity(3,(RAD2DEG(0)+180)/0.088,30);
  	LL_mDelay(1000);
  	PositionWithVelocity(3,(RAD2DEG(0)+180)/0.088,30);
  	LL_mDelay(1000);
  	PositionWithVelocity(4,(RAD2DEG(0)+180)/0.088,30);
  	LL_mDelay(1000);
  	PositionWithVelocity(4,(RAD2DEG(0)+180)/0.088,30);
  	LL_mDelay(1000);
  	PositionWithVelocity(5,(RAD2DEG(0)+180)/0.088,40);
  	LL_mDelay(1000);
  	PositionWithVelocity(5,(RAD2DEG(0)+180)/0.088,40);
  	LL_mDelay(1000);
  	PositionWithVelocity(6,(RAD2DEG(0)+180)/0.088,40);
  	LL_mDelay(1000);
  	PositionWithVelocity(6,(RAD2DEG(0)+180)/0.088,40);
  	LL_mDelay(1000);
  	PositionWithVelocity(1,(RAD2DEG(0)+180)/0.088,20);
  	LL_mDelay(1000);
  	PositionWithVelocity(1,(RAD2DEG(0)+180)/0.088,20);
  	LL_mDelay(1000);
//  	for(int id = 1; id <= 2; id++) {
//  			TorqueEnable(id,0);
//  			while (dynamixel_Ready != 1);
//  	        LL_mDelay(1);
//  	}
//    for(int id = 1; id <= 2; id++) {
//        OperatingMode(id, VELOCITY);
//        while (dynamixel_Ready != 1);
//        LL_mDelay(1);
//    }
//    for(int id = 1; id <= 2; id++) {
//      	  TorqueEnable(id,1);
//      	  while (dynamixel_Ready != 1);
//          LL_mDelay(1);
//    }
}


