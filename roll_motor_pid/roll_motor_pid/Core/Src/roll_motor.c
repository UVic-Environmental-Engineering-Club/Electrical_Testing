/*
 * roll_motor.c
 *
 *  Created on: Aug 8, 2026
 *      Author: igorc
 */
#include "roll_motor.h"

//to do: add rate of change in pot and review that as it goes

// Global Variables		----------------------------------------------------------------------------
static float rm_current_duty_cycle;
static int rm_curr_direction = RM_START_DIRECTION; //cw == 0, ccw == 1
rm_degree_unit rm_current_positon;

static volatile rm_dma_type adc_buffer[1];

#define RM_DEGREE_STEPS 180
#define RM_MAX_ADC 3447
#define RM_MIN_ADC 550
#define RM_ONE_DEGREE_UNIT ((RM_MAX_ADC - RM_MIN_ADC) / RM_DEGREE_STEPS)



//fun. implementations	----------------------------------------------------------------------------

rm_return rm_stop(){
	return rm_set_duty_cycle(0);
}

rm_return rm_init_motor(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	rm_stop();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 1);
	rm_set_direction(RM_START_DIRECTION);
	HAL_Delay(50);
	if( rm_get_current_degree(&rm_current_positon) != OK ) return RUNTIME_ERROR; //cannot start in current position
	rm_current_duty_cycle = 0;
	return OK;

}

rm_return rm_set_duty_cycle(float duty_cycle){
	if(duty_cycle < 0 || duty_cycle > 1 ){
		return OUT_OF_BOUNDS;
	}
	__HAL_TIM_SET_COMPARE(&RM_PWM_TIMER, RM_PWM_TIMER_CHANNEL, (uint32_t)(RM_PWM_STEPS * (1.0f - duty_cycle)));
	rm_current_duty_cycle = duty_cycle;
	return OK;
}

rm_return rm_modify_duty_cycle(float duty_cycle_alteration){
	if( rm_current_duty_cycle + duty_cycle_alteration < 0 || rm_current_duty_cycle + duty_cycle_alteration > 1){
		return OUT_OF_BOUNDS;
	}
	rm_current_duty_cycle += duty_cycle_alteration;
	__HAL_TIM_SET_COMPARE(&RM_PWM_TIMER, RM_PWM_TIMER_CHANNEL, (uint32_t)(RM_PWM_STEPS * (1.0f - rm_current_duty_cycle)));
	return OK;
}

rm_return rm_set_direction(int new_direction){
	if(new_direction != 0 && new_direction != 1) return INVALID_INPUT;
	if(new_direction == 1) HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_SET);   //ccw == high (datasheet)
	if(new_direction == 0) HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_RESET); //cw  == low  (datasheet)
	rm_curr_direction = new_direction;
	return OK;
}

rm_return rm_flip_direction(){
	rm_stop();
	HAL_Delay(100);
	if(rm_curr_direction == 0){
		rm_set_direction(1);
		return OK;
	};
	if(rm_curr_direction == 1) rm_set_direction(0);
	return OK;
}

rm_return rm_read_pot_safe(rm_dma_type* result){
    rm_dma_type r[RM_POT_SAMPLE_SIZE];
    for(int i = 0; i < RM_POT_SAMPLE_SIZE; i++){
        r[i] = adc_buffer[0];
        HAL_Delay(2);
    }
    for(int i = 0; i < RM_POT_SAMPLE_SIZE - 1; i++){
        for(int j = 0; j < RM_POT_SAMPLE_SIZE - 1 - i; j++){
            if(r[j] > r[j+1]){
                rm_dma_type t = r[j]; r[j] = r[j+1]; r[j+1] = t;
            }
        }
    }
    rm_dma_type med = r[RM_POT_SAMPLE_SIZE / 2];
    if(med < 50 || med > 4400) return RUNTIME_ERROR;
    *result = med;
    return OK;
}

rm_return rm_get_current_degree(rm_degree_unit* result){
	rm_dma_type curr_pot;
	if(rm_read_pot_safe(&curr_pot) != OK) return RUNTIME_ERROR;
	*result = (rm_degree_unit)(((curr_pot - RM_MIN_ADC) * RM_DEGREE_STEPS) / (RM_MAX_ADC - RM_MIN_ADC));
	return OK;
}

rm_return rm_find_center(){
	return rm_rotate_to(90);
}

rm_return rm_rotate_degrees(int degrees){
	if(rm_get_current_degree(&rm_current_positon) != OK) return RUNTIME_ERROR;
	int desired_position_degrees = rm_current_positon + degrees;
	return rm_rotate_to(desired_position_degrees);
}

rm_return rm_rotate_to(int degrees){
	if(degrees >  RM_DEGREE_STEPS || degrees < 0) return INVALID_INPUT;
	if (rm_get_current_degree(&rm_current_positon) != OK) return RUNTIME_ERROR;
	int desired_position_degrees = degrees;
	int direction = (degrees > rm_current_positon) ? 0 : 1; //cw : ccw
	rm_set_direction(direction);
	rm_set_duty_cycle(RM_MAX_ROT_SPEED);
	while((direction == 0 && rm_current_positon < desired_position_degrees) ||
			(direction == 1 && rm_current_positon > desired_position_degrees)){
		if (rm_get_current_degree(&rm_current_positon) != OK){ rm_stop(); return RUNTIME_ERROR;}
		HAL_Delay(20);
	}
	rm_stop();
	return OK;
}
