/***********************************************************************************
 * ÎÄ ¼þ Ãû   : bcas_pid.c
 * ¸º Ôð ÈË   : zhangligui
 * ´´½¨ÈÕÆÚ   : 2017Äê4ÔÂ28ÈÕ
 * ÎÄ¼þÃèÊö   : bcas pid Ëã·¨¿ØÖÆ
 * °æÈ¨ËµÃ÷   : Copyright (c) 2008-2017   Î´À´»ï°é»úÆ÷ÈË
 * Æä    Ëû   :
 * ÐÞ¸ÄÈÕÖ¾   :
***********************************************************************************/

#include "bcas_systemapp.h"

typedef struct BcasPid_S {
  float SetSpeed;    //å®šä¹‰è®¾å®šå€¼
  float ActualSpeed; //å®šä¹‰å®žé™…å€¼
  float err;         //å®šä¹‰åå·®å€¼
  float integral;    //å®šä¹‰ä¸Šä¸€ä¸ªåå·®å€¼
  float err_last;    //å®šä¹‰æœ€ä¸Šå‰çš„åå·®å€¼
  float Kp,Ki,Kd;    //å®šä¹‰æ¯”ä¾‹ã€ç§¯åˆ†ã€å¾®åˆ†ç³»æ•°
}BcasPid_S;

BcasPid_S g_pid_left;
BcasPid_S g_pid_right;

uint16_t g_pid_integral_time = 2000000;

void PID_init()
{
  g_pid_left.SetSpeed    = 0.0;
  g_pid_left.ActualSpeed = 0.0;
  g_pid_left.err         = 0.0;
  g_pid_left.err_last    = 0.0;
  g_pid_left.integral    = 0.0;
  g_pid_left.Kp          = 8.0;
  g_pid_left.Ki          = 0.1;
  g_pid_left.Kd          = 0.00;

  g_pid_right.SetSpeed    = 0.0;
  g_pid_right.ActualSpeed = 0.0;
  g_pid_right.err         = 0.0;
  g_pid_right.err_last    = 0.0;
  g_pid_right.integral    = 0.0;
  g_pid_right.Kp          = 8.0;
  g_pid_right.Ki          = 0.1;
  g_pid_right.Kd          = 0.00;
}

void setPidIntegralZero()
{
  g_pid_left.integral = 0;
  g_pid_right.integral = 0;

  g_pid_left.err_last = 0;
  g_pid_right.err_last = 0;
}

void cleanLeftPidIntegral()
{
  //g_pid_left.integral  = 0;
  return;
}

void cleanRightPidIntegral()
{
  //g_pid_right.integral = 0;
  return;
}

/*****************************************************************************
 * º¯ Êý Ãû  : get_speed_from_encode
 * ¸º Ôð ÈË  : zhangligui
 * ´´½¨ÈÕÆÚ  : 2017Äê3ÔÂ28ÈÕ
 * º¯Êý¹¦ÄÜ  : ´Ó±àÂëÆ÷¶ÁÊý»ñÈ¡»úÆ÷µÄË²Ê±ËÙ¶È
 * ÊäÈë²ÎÊý  : int16_t* l_speed  ×óÂÖËÙ¶È
               int16_t* r_speed  ÓÒÂÖËÙ¶È
 * Êä³ö²ÎÊý  : ÎÞ
 * ·µ »Ø Öµ  :
 * µ÷ÓÃ¹ØÏµ  :
 * Æä    Ëü  :

*****************************************************************************/
void get_speed_from_encode(int16_t* l_speed, int16_t* r_speed)
{
  float diff;
  uint8_t l_orien, r_orien;
  uint16_t l_count[2];
  uint16_t r_count[2];

  l_orien = get_bcas_wheel_orien_l();
  r_orien = get_bcas_wheel_orien_r();

  l_count[0] = get_bcas_lencode_count();
  r_count[0] = get_bcas_rencode_count();
  usleep(10000);
  l_count[1] = get_bcas_lencode_count();
  r_count[1] = get_bcas_rencode_count();

  if (0 == l_orien)
  {
    (l_count[1] >= l_count[0]) ? (diff = l_count[1] - l_count[0]) : (diff = l_count[1] + 1 + 65536 - l_count[0]);
  }
  else
  {
    (l_count[1] <= l_count[0]) ? (diff = l_count[0] - l_count[1]) : (diff = l_count[0] + 1 + 65536 - l_count[1]);
  }
  *l_speed = (diff * 55 * BCAS_WHEEL_CIRCUMFERENCE)/BCAS_ENCODE_RATE;

  if (0 == r_orien)
  {
    (r_count[1] >= r_count[0]) ? (diff = r_count[1] - r_count[0]) : (diff = r_count[1] + 1 + 65536 - r_count[0]);
  }
  else
  {
    (r_count[1] <= r_count[0]) ? (diff = r_count[0] - r_count[1]) : (diff = r_count[0] + 1 + 65536 - r_count[1]);
  }
  *r_speed = (diff * 55 * BCAS_WHEEL_CIRCUMFERENCE)/BCAS_ENCODE_RATE;
  return;
}

float PID_realize(uint16_t set_speed, uint16_t actual_speed, uint8_t wheel_type)
{
  float incrementSpeed;

  if (0 == wheel_type)
  {
    g_pid_left.SetSpeed     = set_speed;
    g_pid_left.ActualSpeed  = actual_speed;
    g_pid_left.err          = g_pid_left.SetSpeed - g_pid_left.ActualSpeed;
    g_pid_left.integral    += g_pid_left.err;
    if (set_speed >= actual_speed)
    {
      g_pid_left.integral = 0;
    }
    incrementSpeed          = g_pid_left.Kp*g_pid_left.err
                            + g_pid_left.Ki*g_pid_left.integral
                            + g_pid_left.Kd*(g_pid_left.err-g_pid_left.err_last);
    g_pid_left.err_last = g_pid_left.err;
  }
  else
  {
    g_pid_right.SetSpeed     = set_speed;
    g_pid_right.ActualSpeed  = actual_speed;
    g_pid_right.err          = g_pid_right.SetSpeed - g_pid_right.ActualSpeed;
    g_pid_right.integral    += g_pid_right.err;
    if (set_speed >= actual_speed)
    {
      g_pid_right.integral = 0;
    }
    incrementSpeed          = g_pid_right.Kp*g_pid_right.err
                            + g_pid_right.Ki*g_pid_right.integral
                            + g_pid_right.Kd*(g_pid_right.err-g_pid_right.err_last);
    g_pid_right.err_last = g_pid_right.err;
  }
  return incrementSpeed;
}
