/********************************************************************************

 **** Copyright (C), 2017, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : bcas_gpio.c
 * Author        : zhanglg
 * Date          : 2017-07-13
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2017-07-13
 *   Author      : zhanglg
 *   Modification: Created file

*************************************************************************************************************/
#include <signal.h>
#include "bcas_systemapp.h"
#include <nuttx/ioexpander/gpio.h>

struct bcas_gpio_s
{
    uint8_t gpio_name[16];
    uint8_t gpio_fd;
    uint8_t gpio_type;
    uint8_t singal_no;
};

static struct bcas_gpio_s g_bcas_gpio_desc[] =
{
    {"gpout97", 0, GPIO_OUTPUT_PIN,    10},
    //{"gpint50", 0, GPIO_INTERRUPT_PIN, 11},
    //{"gpint53", 0, GPIO_INTERRUPT_PIN, 12},
    {"gpout99", 0, GPIO_OUTPUT_PIN,    11},
    //{"gpint54", 0, GPIO_INTERRUPT_PIN, 14},
    //{"gpint55", 0, GPIO_INTERRUPT_PIN, 15},
    {"gpint75", 0, GPIO_INTERRUPT_PIN, 12},
    {"gpint76", 0, GPIO_INTERRUPT_PIN, 13},
};

/*****************************************************************************
 * Function      : gpio_dev_initialize
 * Description   : 
 * Input         : void
 * Output        : None
 * Return        :
 * Others        :
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int32_t gpio_dev_init(void)
{
    int32_t index;
    int32_t result;
    char dev_path[16];
    
    uint8_t used_gpio_num = sizeof ( g_bcas_gpio_desc ) /sizeof ( struct bcas_gpio_s );
    struct bcas_gpio_s* p_bcas_gpio = NULL;

    for ( index = 0; index < used_gpio_num; index++ )
    {
        p_bcas_gpio = &g_bcas_gpio_desc[index];

        memset ( dev_path, '\0', sizeof ( dev_path ) );
        sprintf ( dev_path, "/dev/%s", &p_bcas_gpio->gpio_name[0] );

        result = open ( dev_path, O_RDWR );
        if ( result < 0 )
        {
            printf ( "open %s fail!!!\n", dev_path );
            return result;
        }
        p_bcas_gpio->gpio_fd = result;

        if ( GPIO_INTERRUPT_PIN == p_bcas_gpio->gpio_type )
        {
            result = ioctl ( p_bcas_gpio->gpio_fd, GPIOC_REGISTER, p_bcas_gpio->singal_no );
            if ( result < 0 )
            {
                printf ( "gpio GPIOC_REGISTER fail!!!\n" );
                return result;
            }
        }
    }

    return 0;
}

