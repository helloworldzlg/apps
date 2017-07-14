#ifndef __BCAS_SYSTEMAPP_H__
#define __BCAS_SYSTEMAPP_H__

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <nuttx/config.h>

extern int32_t gpio_dev_init(void);
extern int32_t pwm_dev_init(void);


/*****************************************************************************
 * Description   : debug print info macro
 * Description   : debug print info macro
*****************************************************************************/
#define info_print(msg) \
    printf("%s %s Line %d msg: %s\n", __FILE__, __func__, __LINE__, msg);

#define warning_print() \
    printf("%s %s Line %d waring!!! ret : %d\n", __FILE__, __func__, __LINE__);

#define error_print(ret) \
    printf("%s %s Line %d error!!! ret : %d\n", __FILE__, __func__, __LINE__, ret);

#if 0
#define BCAS_ENCODE_RATE              (160 * 20)
#define PI                            (3.141592653f)
#define RADIUS                        (40)  /* mm */
#define BCAS_WHEEL_CIRCUMFERENCE      (2 * PI * RADIUS)

typedef enum
{
  BCAS_ENCODE_LEFT = 0,
  BCAS_ENCODE_RIGHT,
}BCASE_ENCODE_TYPE_E;

extern int32_t pwm_dev_initialize(void);
extern int32_t serial_dev_initialize(void);
extern int32_t gpio_dev_initialize(void);
extern void *serial_3_tx_function(void* thread_param);
extern void *serial_3_rx_function(void* thread_param);
extern void *serial_5_rx_function(void* thread_param);
extern void *pwm_ctrl_function(void* thread_param);

extern int32_t bcas_move_ctrl(int16_t speed, int16_t radius);
extern void get_speed_from_encode(int16_t* l_speed, int16_t* r_speed);
extern float PID_realize(uint16_t set_speed, uint16_t actual_speed, uint8_t wheel_type);
extern void cleanLeftPidIntegral();
extern void cleanRightPidIntegral();

extern uint16_t g_pid_integral_time;
#endif

#endif
