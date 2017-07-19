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
#include <sys/ioctl.h>
#include "bcas_systemapp.h"
#include <nuttx/ioexpander/gpio.h>


struct bcas_gpio_s
{
    char gpio_name[16];
    int32_t gpio_fd;
    int32_t gpio_type;
    int32_t singal_no;
    int32_t (*fp_gpio_set_output)(int32_t gpio_fd, bool val);
};

static struct bcas_gpio_s g_bcas_gpio_desc[] =
{
    {"gpout99", 0, GPIO_OUTPUT_PIN,    10},
    {"gpint54", 0, GPIO_INTERRUPT_PIN, 11},
    {"gpint55", 0, GPIO_INTERRUPT_PIN, 12},
    {"gpout97", 0, GPIO_OUTPUT_PIN,    13},
    {"gpint50", 0, GPIO_INTERRUPT_PIN, 14},
    {"gpint53", 0, GPIO_INTERRUPT_PIN, 15},
    //{"gpint75", 0, GPIO_INTERRUPT_PIN, 16},
    //{"gpint76", 0, GPIO_INTERRUPT_PIN, 17},
};

/*****************************************************************************
 * Function      : gpio_set_output
 * Description   : 
 * Input         : int32_t gpio_fd  
                bool val         
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170715
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int32_t gpio_set_output(int32_t gpio_fd, bool val)
{
    int ret;
    bool read_val;
    ret = ioctl ( gpio_fd, GPIOC_READ, (unsigned long)((uintptr_t)&read_val));
    if ( ret < 0 )
    {
        error_print(ret);
        return ret;
    }
    printf("read_val = %d\n", read_val);
    
    ret = ioctl ( gpio_fd, GPIOC_WRITE, val );
    if ( ret < 0 )
    {
        error_print(ret);
        return ret;
    }
    
    ret = ioctl ( gpio_fd, GPIOC_READ, (unsigned long)((uintptr_t)&read_val ));
    if ( ret < 0 )
    {
        error_print(ret);
        return ret;
    }
    printf("read_val = %d\n", read_val);
    
    return BCAS_RETURN_SUCCESS;
}

void gpio_test()
{
    gpio_set_output(g_bcas_gpio_desc[0].gpio_fd, 1);
    gpio_set_output(g_bcas_gpio_desc[3].gpio_fd, 0);
}

/*****************************************************************************
 * Function      : gpio_dev_init
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
    int32_t fd, ret;
    char dev_path[16];
    
    uint8_t used_gpio_num = sizeof ( g_bcas_gpio_desc ) /sizeof ( struct bcas_gpio_s );
    struct bcas_gpio_s* p_bcas_gpio = NULL;

    for ( index = 0; index < used_gpio_num; index++ )
    {
        p_bcas_gpio = &g_bcas_gpio_desc[index];

        memset ( dev_path, '\0', sizeof ( dev_path ) );
        sprintf ( dev_path, "/dev/%s", &p_bcas_gpio->gpio_name[0] );

        fd = open ( dev_path, O_RDWR );
        if ( fd < 0 )
        {
            error_print(fd);
            return fd;
        }
        p_bcas_gpio->gpio_fd = fd;

        if ( GPIO_INTERRUPT_PIN == p_bcas_gpio->gpio_type )
        {
            ret = ioctl ( p_bcas_gpio->gpio_fd, GPIOC_REGISTER, p_bcas_gpio->singal_no );
            if ( ret < 0 )
            {
                error_print(ret);
                return ret;
            }
        }       
    }

    return 0;
}

