/*
 * roll_motor.h
 *
 *  Created on: Aug 8, 2026
 *      Author: igorc
 */
#include "main.h"
#include <stdlib.h>

#ifndef INC_ROLL_MOTOR_H_
#define INC_ROLL_MOTOR_H_
// PWM inlines 			----------------------------------------------------------------------------
#define RM_PWM_TIMER htim2
#define RM_PWM_TIMER_CHANNEL TIM_CHANNEL_4
#define RM_PWM_STEPS (__HAL_TIM_GET_AUTORELOAD(&RM_PWM_TIMER) + 1) // ARR + 1 (resolution)

// Direction inlines	----------------------------------------------------------------------------
#define RM_START_DIRECTION 0

// Pot inlines			----------------------------------------------------------------------------
#define RM_POT_SAMPLE_SIZE 7 //how many samples for median

// Duty cycle inlines	----------------------------------------------------------------------------
#define RM_MAX_ROT_SPEED 0.5

// Structures and Types ----------------------------------------------------------------------------
typedef enum rm_return_type {
  INVALID_INPUT,
  OUT_OF_BOUNDS, //parameter or input is out of bounds
  RUNTIME_ERROR, //error occurred in execution
  OK
} rm_return;

typedef uint16_t rm_dma_type;
typedef int rm_degree_unit; // represents amount of degrees (0-180)


extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

// Functions			----------------------------------------------------------------------------

/*
 * @brief wrapper running rm_set_duty_cycle(0)
 * @param none
 * @retval rm_return enum of running rm_set_duty_cycle(0)
 */
rm_return rm_stop();

/*
 * @brief initialize motor (pwm, direction, pot)
 * @param none
 * @retval rm_return enum
 */
rm_return rm_init_motor();

/*
 * @brief set stm32 duty cycle
 * @param duty_cycle: desired duty cycle for pwm motor *RANGE: (0-1)*
 * @retval rm_return enum
 */
rm_return rm_set_duty_cycle(float duty_cycle);

/*
 * @brief modify duty cycle
 * @param duty_cycle_alteration: change added to current duty cycle
 * @retval rm_return enum
 */
rm_return rm_modify_duty_cycle(float duty_cycle_alteration);

/*
 * @brief set direction of motor cw/cww
 * @param new direction integer, must either be 0 == cw, 1 == ccw
 * @retval rm_return enum
 */
rm_return rm_set_direction(int new_direction);

/*
 * @brief flip direction of motor cw/cww
 * @param none
 * @retval rm_return enum
 */
rm_return rm_flip_direction();

/*
 * @brief read the pot DMA as a median of values
 * @param result int to store result
 * @retval rm_return enum
 */
rm_return rm_read_pot_safe(rm_dma_type* result);

/*
 * @brief runs fn rm_read_pot_safe and returns result as degrees
 * @param result int to store result
 * @retval rm_return enum
 */
rm_return rm_get_current_degree(rm_degree_unit* result);

/*
 * @brief spin to find center using raw movement (no degree units)
 * @param none
 * @retval rm_return enum
 */
rm_return rm_find_center();

/*
 * @brief read the pot DMA as a median of values
 * @param int degrees: turn that many degree units; *RANGE: (-90 - 90)*
 * @retval rm_return enum
 */
rm_return rm_rotate_degrees(int degrees);

/*
 * @brief read the pot DMA as a median of values
 * @param int degrees: turn to that many degree units; *RANGE: (0 - 180)cw*
 * @retval rm_return enum
 */
rm_return rm_rotate_to(int degrees);

#endif /* INC_ROLL_MOTOR_H_ */
