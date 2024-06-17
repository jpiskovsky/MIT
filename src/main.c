#include <main.h>
#include <utils.h>
#include <stm8s.h>
#include <stdbool.h>
#include <stdio.h>

#define SEGMENT_PORT GPIOC    
#define INPUT_PORT GPIOD
#define DOWN_PIN GPIO_PIN_1
#define UP_PIN GPIO_PIN_5
#define NEUTRAL_PIN GPIO_PIN_6
#define NEUTRAL_LED_PIN GPIO_PIN_7
#define NEUTRAL_LED_PORT GPIOD

uint8_t seg_vals[10] = {0b01000000,0b11110010,0b010001000,0b10100000, 0b00110010, 0b00100100, 0b00000100, 0b11110000,0b00000000, 0b00100000}; //Ještě se upraví

void rx_action(void) // will not compile without this event definition   
{
    char c = UART1_ReceiveData8();
}

// External interrupt handler
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6) {
}
int main(void)
{
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); //Set CLK
    GPIO_Init(SEGMENT_PORT, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(SEGMENT_PORT, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(SEGMENT_PORT, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(SEGMENT_PORT, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(SEGMENT_PORT, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(SEGMENT_PORT, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(SEGMENT_PORT, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(SEGMENT_PORT, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_SLOW);

    GPIO_Init(NEUTRAL_LED_PORT, NEUTRAL_LED_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);

    GPIO_Init(INPUT_PORT, DOWN_PIN, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(INPUT_PORT, UP_PIN, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(INPUT_PORT, NEUTRAL_PIN, GPIO_MODE_IN_PU_NO_IT);

    SEGMENT_PORT->ODR = seg_vals[0];

    init_milis();
    init_uart1();
    uint16_t last_time_stamp = 0;
    uint8_t old_down_state = 0;
    uint8_t old_up_state = 0;
    int8_t gear = 1; //NEUTRAL
    int i = 0;
    while(1) {
        if(GPIO_ReadInputPin(INPUT_PORT, DOWN_PIN) == 0){
            if(old_down_state == 0) gear++;
            old_down_state = 1;
        }else old_down_state = 0;
        if(GPIO_ReadInputPin(INPUT_PORT, UP_PIN) == 0){
            if(old_up_gear == 0) gear--;
            old_up_state = 1;
        }else old_up_state = 0;
        if(GPIO_ReadInputPin(INPUT_PORT, NEUTRAL_PIN) == 0) {
            gear = 1;
            GPIO_WriteHigh(NEUTRAL_LED_PORT, NEUTRAL_LED_PIN);
        }else GPIO_WriteLow(NEUTRAL_LED_PORT, NEUTRAL_LED_PIN);
        gear = gear < 0 ? 0 : gear > 6 ? 6 : gear;
        SEGMENT_PORT->ODR = seg_vals[gear];
        delay(5);
    }
}
/*-------------------------------  Assert -----------------------------------*/
#include "stm/__assert__.h"
