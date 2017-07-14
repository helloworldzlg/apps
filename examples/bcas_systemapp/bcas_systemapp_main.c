/****************************************************************************
 * examples/bcas_systemapp/bcas_systemapp_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcas_systemapp.h"




#if 0
/****************************************************************************
 * Global variables
 ****************************************************************************/
pthread_t g_serial3_TxPthreadId;
pthread_t g_serial3_RxPthreadId;
pthread_t g_serial5_RxPthreadId;
pthread_t g_pwmCtrlPthreadId;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#endif

/****************************************************************************
 * bcas_board_initialize
 ****************************************************************************/
int bcas_board_initialize(void)
{
    //serial_dev_initialize();

    //pwm_dev_initialize();

    gpio_dev_init();

    return 0;
}

/****************************************************************************
 * bcas_systemapp_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int bcas_systemapp_main(int argc, char *argv[])
#endif
{
    int32_t result;
    printf("system app start!!!\n");
    
    result = bcas_board_initialize();
    if (result != 0)
    {
        printf("bcas board initialize fail!!!, ret = %d\n", result);
        return result;
    }
#if 0


    result = pthread_create(&g_serial3_TxPthreadId, NULL, serial_3_tx_function, NULL);
    if (result != 0)
    {
        printf("serial tx thread create fail!!!\n");
        return result;
    }

    result = pthread_create(&g_serial3_RxPthreadId, NULL, serial_3_rx_function, NULL);
    if (result != 0)
    {
        printf("serial rx thread create fail!!!\n");
        return result;
    }

    result = pthread_create(&g_serial5_RxPthreadId, NULL, serial_5_rx_function, NULL);
    if (result != 0)
    {
        printf("serial rx thread create fail!!!\n");
        return result;
    }

    result = pthread_create(&g_pwmCtrlPthreadId, NULL, pwm_ctrl_function, NULL);
    if (result != 0)
    {
        printf("pwm ctrl thread create fail!!!\n");
        return result;
    }
#endif

    for (;;)
    {
        sleep(8000);
    }

    printf("bcas_systemapp_main process exit\n");
    return 0;
}
