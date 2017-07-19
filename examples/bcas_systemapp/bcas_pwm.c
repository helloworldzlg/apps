/********************************************************************************

 **** Copyright (C), 2017, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : bcas_pwm.c
 * Author        : zhangligui
 * Date          : 2017-05-16
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2017-03-16
 *   Author      : zhangligui
 *   Modification: Created file

*************************************************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "bcas_systemapp.h"
#include <nuttx/drivers/pwm.h>


#define MAX_PWM_DUTY              (65535)

typedef enum
{
    PWM_CH1 = 1,
    PWM_CH3 = 3,

    LEFT_WHEEL  = PWM_CH3,
    RIGHT_WHEEL = PWM_CH1,
}PWM_CHANNEL_E;

/* this version wheel motor Low Level(full speed) HIGH Level : zero speed */
#define PWM_VALUE(val)            (MAX_PWM_DUTY - val)   

int32_t set_pwm_configs(int32_t pwm_fd, struct pwm_info_s* p_pwm_config);
int32_t start_pwm(int32_t pwm_fd);
int32_t stop_pwm(int32_t pwm_fd);



static bool g_bcas_pwm_init = false;


//typedef enum
//{
//    BOTTOM_BRASH_MOTOR = PWM_3,
//    LEFT_WHEEL_MOTOR = PWM_8_CH1,
//}BCAS_MOTOR_TYPE_E;

typedef struct
{
    int32_t pwm_id;
    int32_t pwm_fd;
    int32_t (*fp_set_pwm_configs)(int32_t pwm_fd, struct pwm_info_s* p_pwm_config);
    int32_t (*fp_start_pwm)(int32_t pwm_fd);
    int32_t (*fp_stop_pwm)(int32_t pwm_fd);
}BCAS_PWM_S;

BCAS_PWM_S g_bcas_pwm[] =
{
    /* wheel motor */
    {PWM_8, 0, set_pwm_configs, start_pwm, stop_pwm},
    
    /* bottom brash */
    {PWM_3, 0, set_pwm_configs, start_pwm, stop_pwm},

    
    /* front brash */
    //{0, },
};


/*****************************************************************************
 * Function      : set_pwm_configs
 * Description   : 
 * Input         : int32_t pwm_fd 
                   pwm_info_s* p_pwm_config
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170714
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int32_t set_pwm_configs(int32_t pwm_fd, struct pwm_info_s* p_pwm_config)
{
    int32_t ret;
    
    ret = ioctl(pwm_fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)p_pwm_config));
    if (ret < 0)  
    {    
        error_print(ret);
        return ret;
    }

    return BCAS_RETURN_SUCCESS;
}

/*****************************************************************************
 * Function      : start_pwm
 * Description   : 
 * Input         : int32_t pwm_fd  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170714
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int32_t start_pwm(int32_t pwm_fd)
{
    int ret;
    ret = ioctl(pwm_fd, PWMIOC_START, 0);
    if (ret < 0)
    {    
        error_print(ret);
        return ret;
    }
    return BCAS_RETURN_SUCCESS;
}

/*****************************************************************************
 * Function      : stop_pwm
 * Description   : 
 * Input         : int32_t pwm_fd  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170714
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int32_t stop_pwm(int32_t pwm_fd)
{
    int32_t ret;
    ret = ioctl(pwm_fd, PWMIOC_STOP, 0);
    if (ret < 0)  
    {    
        error_print(ret);
        return ret;  
    }
    return BCAS_RETURN_SUCCESS;
}

/*****************************************************************************
 * Function      : pwm_pause
 * Description   : pause wheel run because this version pwm default continuous run
 * Input         : int32_t pwm_fd  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170719
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int32_t pwm_pause(int32_t pwm_fd)
{
    struct pwm_info_s pwm_info;
    BCAS_PWM_S* p_wheel_pwm = &g_bcas_pwm[WHEEL_PWM];

    memset(&pwm_info, 0, sizeof(pwm_info));
    pwm_info.frequency = 100;
    pwm_info.channels[0].channel = LEFT_WHEEL;
    pwm_info.channels[0].duty    = PWM_VALUE(0);

    pwm_info.channels[1].channel = RIGHT_WHEEL;
    pwm_info.channels[1].duty    = PWM_VALUE(0);
    
    p_wheel_pwm->fp_set_pwm_configs(pwm_fd, &pwm_info);
    p_wheel_pwm->fp_start_pwm(pwm_fd);

    return BCAS_RETURN_SUCCESS;
}

void pwm_test()
{
    int32_t pwm_fd;
    struct pwm_info_s pwm_info;
    BCAS_PWM_S* p_wheel_pwm = &g_bcas_pwm[WHEEL_PWM];

    pwm_fd = p_wheel_pwm->pwm_fd;

    memset(&pwm_info, 0, sizeof(pwm_info));
    pwm_info.frequency = 100;
    pwm_info.channels[0].channel = LEFT_WHEEL;
    pwm_info.channels[0].duty    = PWM_VALUE(0);

    pwm_info.channels[1].channel = RIGHT_WHEEL;
    pwm_info.channels[1].duty    = PWM_VALUE(0);
    
    p_wheel_pwm->fp_set_pwm_configs(pwm_fd, &pwm_info);
    p_wheel_pwm->fp_start_pwm(pwm_fd);
    sleep(10);
    p_wheel_pwm->fp_stop_pwm(pwm_fd);
    p_wheel_pwm->fp_stop_pwm(pwm_fd);
    for (;;);
}

#if 0


#define BCAS_WHEEL_PWM_CH_NUM       (2)
#define BCAS_WHEELBASE_LENGTH       (302)
#define MAX_SPEED                   (850)
#define MAX_RADIUS                  (2000)
#define MAX_PWM_VALUE               (32768)

#define PWM_INCREASE_GRADIENT       (10)

typedef enum
{
    BCAS_RIGHT_WHEEL_TIM8_CH1 = 1, /* right wheel forward */
    BCAS_RIGHT_WHEEL_TIM8_CH2,     /* right wheel backward */
    BCAS_LEFT_WHEEL_TIM8_CH3,      /* left wheel forward */
    BCAS_LEFT_WHEEL_TIM8_CH4,      /* left wheel backward */
} BCAS_WHEEL_E;

typedef enum
{
    BCAS_WHEEL_LEFT = 0,
    BCAS_WHEEL_RIGHT,
} BCAS_WHEEL_TYPE_E;

typedef enum
{
    MOVE_FORWARD = 0,
    MOVE_BACKWARD,
} BCAS_ORIENTATION_E;

static int32_t g_bcas_pwm_id;
static bool g_bcas_pwm_initialize = false;

static uint16_t g_wheel_speed_l = 0;
static uint16_t g_wheel_speed_r = 0;

struct pwm_info_s g_pwm_info;

int32_t bcas_move_pwm_ctrl
(
    int16_t wheelspeed_l, BCAS_ORIENTATION_E wheelorien_l,
    int16_t wheelspeed_r, BCAS_ORIENTATION_E wheelorien_r
)
{
    int32_t result;
    uint8_t mChannal[2];

    (wheelorien_r == MOVE_FORWARD) ? (mChannal[0] = BCAS_RIGHT_WHEEL_TIM8_CH1) : (mChannal[0] = BCAS_RIGHT_WHEEL_TIM8_CH2);
    (wheelorien_l == MOVE_FORWARD) ? (mChannal[1] = BCAS_LEFT_WHEEL_TIM8_CH3) : (mChannal[1] = BCAS_LEFT_WHEEL_TIM8_CH4);

    memset((void*)&g_pwm_info, 0, sizeof(g_pwm_info));

    g_pwm_info.frequency = 100;

    g_pwm_info.channels[0].channel = mChannal[0];
    //g_pwm_info.channels[0].duty    = ((1.0f * wheelspeed_l)/MAX_SPEED) * MAX_PWM_VALUE;
    g_pwm_info.channels[0].duty    = MAX_PWM_VALUE;

    g_pwm_info.channels[1].channel = mChannal[1];
    //g_pwm_info.channels[1].duty    = ((1.0f * wheelspeed_r)/MAX_SPEED) * MAX_PWM_VALUE;
    g_pwm_info.channels[1].duty    = MAX_PWM_VALUE;

    result = ioctl(g_bcas_pwm_id, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&g_pwm_info));
    if (result < 0)
    {
        printf("ioctl(SETCHARACTERISTICS) failed!!!\n");
        return result;
    }

    result = ioctl(g_bcas_pwm_id, PWMIOC_START, 0);
    if (result < 0)
    {
        printf("ioctl(PWMIOC_START) failed!!!\n");
        return result;
    }

    return 0;
}

int32_t bcas_move_stop(void)
{
    int32_t result;
    result = ioctl(g_bcas_pwm_id, PWMIOC_STOP, 0);
    if (result < 0)
    {
        printf("pwm_main: ioctl(PWMIOC_STOP) failed!!!\n");
        return result;
    }
    return 0;
}


/**
 * @brief
 *
 * @param speed
 * @param radius
 *
 * @return
 */
int32_t bcas_move_ctrl(int16_t speed, int16_t radius)
{
    int16_t min_temp = 0;
    if ((speed > MAX_SPEED) || (speed < -MAX_SPEED))
    {
        return 0;
    }

    /* calc l_wheel & r_wheel move orien */
    if (speed >= 0)
    {
        set_bcas_wheel_orien_l(MOVE_FORWARD);
        set_bcas_wheel_orien_r(MOVE_FORWARD);

        if ((radius >= 1) && (radius < BCAS_WHEELBASE_LENGTH/2))
        {
            set_bcas_wheel_orien_l(MOVE_BACKWARD);
        }
    }
    else
    {
        set_bcas_wheel_orien_l(MOVE_BACKWARD);
        set_bcas_wheel_orien_r(MOVE_BACKWARD);

        if ((1 == radius) && (radius > -BCAS_WHEELBASE_LENGTH/2))
        {
            set_bcas_wheel_orien_l(MOVE_FORWARD);
        }
        speed = -speed;
    }

    /* translation */
    if (0 == radius)
    {
        g_wheel_speed_l = speed;
        g_wheel_speed_r = speed;
    }
    else if ((1 == radius) || (-1 == radius))
    {
        g_wheel_speed_l = speed;
        g_wheel_speed_r = speed;
    }
    else if (radius > 1)
    {
        if (radius <= BCAS_WHEELBASE_LENGTH/2)
        {
            g_wheel_speed_l = ( 1.0f * speed * (BCAS_WHEELBASE_LENGTH/2 - radius) / (radius + BCAS_WHEELBASE_LENGTH/2) );
        }
        else
        {
            g_wheel_speed_l = ( 1.0f * speed * (radius - BCAS_WHEELBASE_LENGTH/2) / (radius + BCAS_WHEELBASE_LENGTH/2) );
        }
        g_wheel_speed_r = speed;
    }
    else if (radius < -1)
    {
        if (radius >= -BCAS_WHEELBASE_LENGTH/2)
        {
            g_wheel_speed_r = ( 1.0f * speed * (BCAS_WHEELBASE_LENGTH/2 + radius) / (BCAS_WHEELBASE_LENGTH/2 - radius) );
        }
        else
        {
            g_wheel_speed_r = ( 1.0f * speed * (-radius - BCAS_WHEELBASE_LENGTH/2) / (BCAS_WHEELBASE_LENGTH/2 - radius) );
        }
        g_wheel_speed_l = speed;
    }

    return 0;
}

/**
 * @param
 * @return
 */
int16_t get_set_speed_from_soc(BCAS_WHEEL_TYPE_E wheel_type)
{
    if (BCAS_WHEEL_LEFT == wheel_type)
    {
        return g_wheel_speed_l;
    }
    else
    {
        return g_wheel_speed_r;
    }
}

BCAS_ORIENTATION_E get_set_orien_from_soc(BCAS_WHEEL_TYPE_E wheel_type)
{
    if (BCAS_WHEEL_LEFT == wheel_type)
    {
        return get_bcas_wheel_orien_l();
    }
    else
    {
        return get_bcas_wheel_orien_r();
    }
}

static uint16_t last_pwm_l = 0;
static uint16_t last_pwm_r = 0;

uint16_t get_expect_pwm(int16_t set_speed, int16_t actual_speed, BCAS_WHEEL_TYPE_E wheel_type)
{
    float increase_val;
    uint16_t expect_pwm;
    uint16_t last_pwm_val;

    (wheel_type == BCAS_WHEEL_LEFT) ? (last_pwm_val = last_pwm_l) : (last_pwm_val = last_pwm_r);

    increase_val = PID_realize(set_speed, actual_speed, wheel_type);
    expect_pwm   = last_pwm_val + increase_val * PWM_INCREASE_GRADIENT;

    (wheel_type == BCAS_WHEEL_LEFT) ? (last_pwm_l = expect_pwm) : (last_pwm_r = expect_pwm);

    return expect_pwm;
}

/*****************************************************************************
 * Function      : pwm_ctrl_function
 * Description   : robot wheel electric motor control
 * Input         : void* thread_param
 * Output        : None
 * Return        : void
 * Others        :
 * Record
 * 1.Date        : 20170316
 *   Author      : zhangligui
 *   Modification: Created function

*****************************************************************************/
void *pwm_ctrl_function(void* thread_param)
{
    int32_t result;
    uint32_t count = 0;
    uint8_t mChannal[2];
    time_t time_current, time_last;
    uint32_t time_diff;
    int16_t set_speed_l, set_speed_r;
    int16_t actual_speed_l, actual_speed_r;
    uint16_t expect_pwm_l, expect_pwm_r;
    float diff_speed_l, diff_speed_r;
    BCAS_ORIENTATION_E set_orien_l, set_orien_r;
    static bool move_stop_flag = false;
    static BCAS_ORIENTATION_E last_orien_l = MOVE_FORWARD;
    static BCAS_ORIENTATION_E last_orien_r = MOVE_FORWARD;

    PID_init();

    for (;;)
    {
        set_speed_l    = get_set_speed_from_soc(BCAS_WHEEL_LEFT);
        set_orien_l    = get_set_orien_from_soc(BCAS_WHEEL_LEFT);

        set_speed_r    = get_set_speed_from_soc(BCAS_WHEEL_RIGHT);
        set_orien_r    = get_set_orien_from_soc(BCAS_WHEEL_RIGHT);

        set_speed_l = 200;
        set_speed_r = 200;

        get_speed_from_encode(&actual_speed_l, &actual_speed_r);
        if (actual_speed_l >= set_speed_l)
        {
            cleanLeftPidIntegral();
        }

        if (actual_speed_r >= set_speed_l)
        {
            cleanRightPidIntegral();
        }
        //printf("a %d, %d\n", actual_speed_l, actual_speed_r);
        //printf("s %d, %d\n", set_speed_l, set_speed_r);

        if ( (last_orien_l != set_orien_l) || (last_orien_r != set_orien_r) )
        {
            bcas_move_stop();
            setPidIntegralZero();
            last_pwm_l = 0;
            last_pwm_r = 0;
            //printf("stop stop stop\n");

            last_orien_l = set_orien_l;
            last_orien_r = set_orien_r;
            continue;
        }

        if ( (0 == set_speed_l) && (0 == set_speed_r) )
        {
            if (!move_stop_flag)
            {
                bcas_move_stop();
                setPidIntegralZero();
                last_pwm_l = 0;
                last_pwm_r = 0;
                move_stop_flag = true;
                //printf("stop stop stop\n");
            }
            continue;
        }
        move_stop_flag = false;

        (set_orien_l == MOVE_FORWARD) ? (mChannal[0] = BCAS_LEFT_WHEEL_TIM8_CH3) : (mChannal[0] = BCAS_LEFT_WHEEL_TIM8_CH4);
        (set_orien_r == MOVE_FORWARD) ? (mChannal[1] = BCAS_RIGHT_WHEEL_TIM8_CH1) : (mChannal[1] = BCAS_RIGHT_WHEEL_TIM8_CH2);

        expect_pwm_l = get_expect_pwm(set_speed_l, actual_speed_l, BCAS_WHEEL_LEFT);
        expect_pwm_r = get_expect_pwm(set_speed_r, actual_speed_r, BCAS_WHEEL_RIGHT);

        memset((void*)&g_pwm_info, 0, sizeof(g_pwm_info));

        g_pwm_info.frequency = 100;

        g_pwm_info.channels[0].channel = mChannal[0];
        g_pwm_info.channels[0].duty    = (uint16_t)expect_pwm_l;
        g_pwm_info.channels[0].duty    = (uint16_t)1000;

        g_pwm_info.channels[1].channel = mChannal[1];
        g_pwm_info.channels[1].duty    = (uint16_t)expect_pwm_r;
        g_pwm_info.channels[1].duty    = (uint16_t)1000;

        //printf("%d, %d\n", (int32_t)expect_pwm_l, (int32_t)expect_pwm_r);

        result = ioctl(g_bcas_pwm_id, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&g_pwm_info));
        if (result < 0)
        {
            printf("ioctl(SETCHARACTERISTICS) failed!!!\n");
        }

        result = ioctl(g_bcas_pwm_id, PWMIOC_START, 0);
        if (result < 0)
        {
            printf("ioctl(PWMIOC_START) failed!!!\n");
        }
        usleep(50000);
    }
    return NULL;
}



#endif

/*****************************************************************************
 * Function      : pwm_dev_init
 * Description   :
 * Input         : void
 * Output        : None
 * Return        :
 * Others        :
 * Record
 * 1.Date        : 20170714
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int32_t pwm_dev_init(void)
{
    int32_t i;
    int32_t fd, ret;
    char dev_path[16];
    int32_t pwm_dev_num = sizeof(g_bcas_pwm)/sizeof(BCAS_PWM_S);
    BCAS_PWM_S* p_pwm_dev = &g_bcas_pwm[0];
    struct pwm_info_s pwm_info;

    if (!g_bcas_pwm_init)
    {
        for (i = 0; i < pwm_dev_num; i++)
        {
            memset(dev_path, '\0', sizeof(dev_path));
            sprintf(dev_path, "/dev/pwm%d", p_pwm_dev[i].pwm_id);

            fd = open(dev_path, O_RDONLY);
            if (fd < 0)
            {
                error_print(fd);
                return fd;
            }
            g_bcas_pwm[i].pwm_fd = fd;

            /* this version wheel pwm default continuous run */
            if (i == WHEEL_PWM)
            {
                ret = pwm_pause(fd);
                if (BCAS_RETURN_SUCCESS != ret)
                {
                    error_print(ret);
                    return ret;
                }
            }
        }

        g_bcas_pwm_init = true;
    }

    return 0;
}
