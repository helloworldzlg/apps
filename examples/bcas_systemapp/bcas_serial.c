#include <stdlib.h>
#include <string.h>
#include "bcas_systemapp.h"

#define TO_SOC_PACKAGE_MAX_LEN        (240)

#define PACKAGE_HEADER_0              (0xAA)
#define PACKAGE_HEADER_1              (0x55)

#define TO_SOC_SERIAL_PATH            ("/dev/ttyS1")
#define TO_IMU_SERIAL_PATH            ("/dev/ttyS2")

#define RAD_TO_DEGREE                 ((180 * 1.0f) / PI)

static int32_t g_serial_3_devId;
static int32_t g_serial_5_devId;
static uint8_t g_rxBuffer[512];
static int32_t g_rxDataHead;
static int32_t g_rxDataTail;
static uint8_t g_complete_pkg[64];
static uint8_t g_version_type;

struct Bcas_base_control_s
{
  uint8_t header_0;
  uint8_t header_1;
  uint8_t package_len;
  uint8_t sub_package_id;
  uint8_t subpackage_len;
  int16_t speed;
  int16_t radius;
  uint8_t checksum;
};

struct Bcas_request_extra_s
{
  uint8_t header_0;
  uint8_t header_1;
  uint8_t package_len;
  uint8_t sub_package_id;
  uint8_t subpackage_len;
  uint16_t request_flag;
  uint8_t checksum;
};

typedef struct
{
  uint8_t data[TO_SOC_PACKAGE_MAX_LEN];
  uint8_t size;
}SERIAL_TO_SOC_PKG_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint16_t time_stamp;
  uint8_t bumper;
  uint8_t wheel_drop;
  uint8_t cliff;
  uint16_t left_encoder;
  uint16_t right_encoder;
  uint8_t left_pwm;
  uint8_t right_pwm;
  uint8_t buttons;
  uint8_t charger;
  uint8_t battery;
  uint8_t over_current;
}BasicSensorData_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint8_t right_signal;
  uint8_t central_signal;
  uint8_t left_signal;
}DOCKING_IR_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  int16_t Angle;
  int16_t Angle_rate;
  uint8_t Unused1;
  uint8_t Unused2;
  uint8_t Unused3;
}INERTIAL_SENSOR_DATA_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint16_t right_cliff_sensor;
  uint16_t central_cliff_sensor;
  uint16_t left_cliff_sensor;
}CLIFF_SENSOR_DATA_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint8_t left_motor;
  uint8_t right_motor;
}CURRENT_S;

typedef struct
{
  int16_t x_axis;
  int16_t y_axis;
  int16_t z_axis;
}GYRO_SENSOR_DATA_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint8_t frame_id;
  uint8_t data_length;
  GYRO_SENSOR_DATA_S gyro_data[16];
}RAW_DATA_GYRO_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint8_t patch;
  uint8_t minor;
  uint8_t major;
  uint8_t unused;
}HARDWARE_VERSION_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint8_t patch;
  uint8_t minor;
  uint8_t major;
  uint8_t unused;
}FIRMWARE_VERSION_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint32_t udio_0;
  uint32_t udio_1;
  uint32_t udio_2;
}UNIQUE_DEVICE_IDENTIFIER_S;

typedef struct
{
  uint8_t Header;
  uint8_t Length;
  uint8_t type;
  uint32_t p_gain;
  uint32_t i_gain;
  uint32_t d_gain;
}CONTROLLER_INFO_S;

typedef struct
{
  int16_t angle;
  int16_t angle_rate;
  int16_t x_axis;
  int16_t y_axis;
  int16_t z_axis;
}BCAS_IMU_PKG_S;

typedef enum
{
  BCAS_HARDWARE_VERSION = 0x1,
  BCAS_FIRMWARE_VERSION = 0x2,
  BCAS_UNIQUE_VERSION = 0x8,
}BCAS_VERSION_TYPE_E;

typedef enum
{
  BCAS_BUMPER_NONE    = 0x0,
  BCAS_BUMPER_RIGHT   = 0x1,
  BCAS_BUMPER_CENTRAL = 0x2,
  BCAS_BUMPER_LEFT    = 0x4,
}BCAS_BUMPER_DEF_E;

SERIAL_TO_SOC_PKG_S g_serial_to_soc_pkg;
BasicSensorData_S g_basic_sersor_data = {0x1, 0xF, 0x0};
DOCKING_IR_S g_docking_ir = {0x3, 0x3, 0x0};
INERTIAL_SENSOR_DATA_S g_inertial_sensor_data = {0x4, 0x7, 0x0};
CLIFF_SENSOR_DATA_S g_cliff_sensor_data = {0x5, 0x6, 0x0};
CURRENT_S g_current = {0x6, 0x2};
RAW_DATA_GYRO_S g_raw_data_gyro = {0xD, 0x8, 0x0};
HARDWARE_VERSION_S g_hardware_version = {0xA, 0x4, 0x4, 0x0, 0x1, 0x0};
FIRMWARE_VERSION_S g_firmware_version = {0xB, 0x4, 0x9, 0x2, 0x1, 0x0};
UNIQUE_DEVICE_IDENTIFIER_S g_unique_device_identifier = {0x13, 0xC, 736034612, 892621875, 1125147971};
CONTROLLER_INFO_S g_controller_info = {0x15, 0xD, 0x1, 0x0, 0x0, 0x0};

uint8_t g_hardware_ver_flag = 0;
uint8_t g_firmware_ver_flag = 0;
uint8_t g_unique_device_identifier_flag = 0;
uint8_t g_controller_info_flag = 0;
uint8_t g_imu_package_count = 0;
bool g_imu_package_status = false;

uint8_t g_serial_5_rxbuff[512];
uint8_t g_imu_indata[256];
int32_t g_imu_rxbuff_head;
int32_t g_imu_rxbuff_tail;
BCAS_IMU_PKG_S g_imu_pkg_buffer[64];
int32_t g_imu_pkg_head = -1;
int32_t g_imu_pkg_tail = 0;

uint8_t get_bcas_bumper_status()
{
  bool l_status, r_status;

  l_status = get_bcas_lbumper_status();
  r_status = get_bcas_rbumper_status();

  if ( (l_status == true) && (r_status == true) )
  {
    return BCAS_BUMPER_CENTRAL;
  }
  else if ( (l_status == true) && (r_status == false) )
  {
    return BCAS_BUMPER_LEFT;
  }
  else if ( (l_status == false) && (r_status == true) )
  {
    return BCAS_BUMPER_RIGHT;
  }
  else
  {
    return BCAS_BUMPER_NONE;
  }
}

void buildBytes8_t(const uint8_t val, SERIAL_TO_SOC_PKG_S* buffer)
{
  buffer->data[buffer->size++] = (uint8_t)(((val) >>  0) & 0xFF);
}

void buildBytes16_t(const uint16_t val, SERIAL_TO_SOC_PKG_S* buffer)
{
  buffer->data[buffer->size++] = (uint8_t)(((val) >>  0) & 0xFF);
  buffer->data[buffer->size++] = (uint8_t)(((val) >>  8) & 0xFF);
}

void buildBytes32_t(const uint32_t val, SERIAL_TO_SOC_PKG_S* buffer)
{
  buffer->data[buffer->size++] = (uint8_t)(((val) >>  0) & 0xFF);
  buffer->data[buffer->size++] = (uint8_t)(((val) >>  8) & 0xFF);
  buffer->data[buffer->size++] = (uint8_t)(((val) >>  16) & 0xFF);
  buffer->data[buffer->size++] = (uint8_t)(((val) >>  24) & 0xFF);
}

void buildCheckSum(SERIAL_TO_SOC_PKG_S* buffer)
{
  char checksum = 0;
  uint16_t i = 0;

  buffer->data[2] = buffer->size - 3;

  for (i = 2; i < buffer->size; i++)
  {
    checksum ^= (buffer->data[i]);
  }

  buffer->data[buffer->size++] = checksum;
}

/**
 * @brief 
 *
 * @param buffer
 */
void serial_to_soc_pkg_init(SERIAL_TO_SOC_PKG_S* buffer)
{
  buffer->size = 0;

  buffer->data[buffer->size++] = PACKAGE_HEADER_0;
  buffer->data[buffer->size++] = PACKAGE_HEADER_1;
  buffer->data[buffer->size++] = 0;

  //memset(g_imu_pkg_buffer, 0, sizeof(g_imu_pkg_buffer));

  return;
}

/**
 * 设置基本传感器的包内容
 */
uint32_t set_basic_sensor_data_pkg(void)
{
  g_basic_sersor_data.time_stamp += 20;

  buildBytes8_t(g_basic_sersor_data.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.Length, &g_serial_to_soc_pkg);
  buildBytes16_t(g_basic_sersor_data.time_stamp, &g_serial_to_soc_pkg);
  g_basic_sersor_data.bumper = get_bcas_bumper_status();
  buildBytes8_t(g_basic_sersor_data.bumper, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.wheel_drop, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.cliff, &g_serial_to_soc_pkg);
  g_basic_sersor_data.left_encoder = get_bcas_lencode_count();
  buildBytes16_t(g_basic_sersor_data.left_encoder, &g_serial_to_soc_pkg);
  g_basic_sersor_data.right_encoder = get_bcas_rencode_count();
  buildBytes16_t(g_basic_sersor_data.right_encoder, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.left_pwm, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.right_pwm, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.buttons, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.charger, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.battery, &g_serial_to_soc_pkg);
  buildBytes8_t(g_basic_sersor_data.over_current, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t set_docking_ir_pkg(void)
{
  buildBytes8_t(g_docking_ir.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_docking_ir.Length, &g_serial_to_soc_pkg);
  buildBytes8_t(g_docking_ir.right_signal, &g_serial_to_soc_pkg);
  buildBytes8_t(g_docking_ir.central_signal, &g_serial_to_soc_pkg);
  buildBytes8_t(g_docking_ir.left_signal, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t set_inertial_sensor_data(void)
{
  buildBytes8_t(g_inertial_sensor_data.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_inertial_sensor_data.Length, &g_serial_to_soc_pkg);
  g_inertial_sensor_data.Angle = g_imu_pkg_buffer[g_imu_pkg_head].angle;
  buildBytes16_t(g_inertial_sensor_data.Angle, &g_serial_to_soc_pkg);
  g_inertial_sensor_data.Angle_rate = g_imu_pkg_buffer[g_imu_pkg_head].angle_rate;
  buildBytes16_t(g_inertial_sensor_data.Angle_rate, &g_serial_to_soc_pkg);
  // printf("angle = %d, angle_rate = %d, ", g_inertial_sensor_data.Angle, g_inertial_sensor_data.Angle_rate);

  buildBytes8_t(g_inertial_sensor_data.Unused1, &g_serial_to_soc_pkg);
  buildBytes8_t(g_inertial_sensor_data.Unused2, &g_serial_to_soc_pkg);
  buildBytes8_t(g_inertial_sensor_data.Unused3, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t set_cliff_sensor_data(void)
{
  buildBytes8_t(g_cliff_sensor_data.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_cliff_sensor_data.Length, &g_serial_to_soc_pkg);
  buildBytes16_t(g_cliff_sensor_data.right_cliff_sensor, &g_serial_to_soc_pkg);
  buildBytes16_t(g_cliff_sensor_data.central_cliff_sensor, &g_serial_to_soc_pkg);
  buildBytes16_t(g_cliff_sensor_data.left_cliff_sensor, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t set_current(void)
{
  buildBytes8_t(g_current.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_current.Length, &g_serial_to_soc_pkg);
  buildBytes8_t(g_current.left_motor, &g_serial_to_soc_pkg);
  buildBytes8_t(g_current.right_motor, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t set_raw_data_gyro(void)
{
  buildBytes8_t(g_raw_data_gyro.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_raw_data_gyro.Length, &g_serial_to_soc_pkg);
  buildBytes8_t(g_raw_data_gyro.frame_id++, &g_serial_to_soc_pkg);

  g_raw_data_gyro.data_length = 0x3;
  buildBytes8_t(g_raw_data_gyro.data_length, &g_serial_to_soc_pkg);

  (g_imu_pkg_head >= 0) ? (g_raw_data_gyro.gyro_data[0].x_axis = g_imu_pkg_buffer[g_imu_pkg_head].x_axis)
    : (g_raw_data_gyro.gyro_data[0].x_axis = 0);
  buildBytes16_t(g_raw_data_gyro.gyro_data[0].x_axis, &g_serial_to_soc_pkg);

  (g_imu_pkg_head >= 0) ? (g_raw_data_gyro.gyro_data[0].y_axis = g_imu_pkg_buffer[g_imu_pkg_head].y_axis)
    : (g_raw_data_gyro.gyro_data[0].y_axis = 0);
  buildBytes16_t(g_raw_data_gyro.gyro_data[0].y_axis, &g_serial_to_soc_pkg);

  (g_imu_pkg_head >= 0) ? (g_raw_data_gyro.gyro_data[0].z_axis = g_imu_pkg_buffer[g_imu_pkg_head].z_axis)
    : (g_raw_data_gyro.gyro_data[0].z_axis = 0);
  buildBytes16_t(g_raw_data_gyro.gyro_data[0].z_axis, &g_serial_to_soc_pkg);

  // printf("x_axis = %d, y_axis = %d, z_axis = %d\n", g_raw_data_gyro.gyro_data[0].x_axis,
  //   g_raw_data_gyro.gyro_data[0].y_axis, g_raw_data_gyro.gyro_data[0].z_axis);
  return 0;
}

uint32_t set_hardware_version(void)
{
  buildBytes8_t(g_hardware_version.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_hardware_version.Length, &g_serial_to_soc_pkg);
  buildBytes8_t(g_hardware_version.patch, &g_serial_to_soc_pkg);
  buildBytes8_t(g_hardware_version.minor, &g_serial_to_soc_pkg);
  buildBytes8_t(g_hardware_version.major, &g_serial_to_soc_pkg);
  buildBytes8_t(g_hardware_version.unused, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t set_firmware_version(void)
{
  buildBytes8_t(g_firmware_version.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_firmware_version.Length, &g_serial_to_soc_pkg);
  buildBytes8_t(g_firmware_version.patch, &g_serial_to_soc_pkg);
  buildBytes8_t(g_firmware_version.minor, &g_serial_to_soc_pkg);
  buildBytes8_t(g_firmware_version.major, &g_serial_to_soc_pkg);
  buildBytes8_t(g_firmware_version.unused, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t set_unique_device_identifier(void)
{
  buildBytes8_t(g_unique_device_identifier.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_unique_device_identifier.Length, &g_serial_to_soc_pkg);
  buildBytes32_t(g_unique_device_identifier.udio_0, &g_serial_to_soc_pkg);
  buildBytes32_t(g_unique_device_identifier.udio_1, &g_serial_to_soc_pkg);
  buildBytes32_t(g_unique_device_identifier.udio_2, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t set_controller_info(void)
{
  buildBytes8_t(g_controller_info.Header, &g_serial_to_soc_pkg);
  buildBytes8_t(g_controller_info.Length, &g_serial_to_soc_pkg);
  buildBytes8_t(g_controller_info.type, &g_serial_to_soc_pkg);

  buildBytes32_t(g_controller_info.p_gain, &g_serial_to_soc_pkg);
  buildBytes32_t(g_controller_info.i_gain, &g_serial_to_soc_pkg);
  buildBytes32_t(g_controller_info.d_gain, &g_serial_to_soc_pkg);
  return 0;
}

uint32_t serial_to_soc_pkg_create(void)
{
  static uint32_t count = 0;

  serial_to_soc_pkg_init(&g_serial_to_soc_pkg);

  /* 打包传感器数据 */
  (void)set_basic_sensor_data_pkg();

  (void)set_docking_ir_pkg();

  (void)set_inertial_sensor_data();

  (void)set_cliff_sensor_data();

  (void)set_current();

  (void)set_raw_data_gyro();

  if (BCAS_HARDWARE_VERSION == (g_version_type & 0x1))
  {
    (void)set_hardware_version();
  }
  else if (BCAS_FIRMWARE_VERSION == (g_version_type & 0x2))
  {
    (void)set_firmware_version();
  }
  else if (BCAS_UNIQUE_VERSION == (g_version_type & 0x8))
  {
    (void)set_unique_device_identifier();
  }
  else {}
  g_version_type = 0;

  (void)set_controller_info();

  buildCheckSum(&g_serial_to_soc_pkg);

  return 0;
}

typedef enum {
  BASE_CONTROL_CMD = 1,
  SOUND_CMD = 3,
  SOUND_SEQUENCE = 4,
  REQUEST_EXTRA = 9,
  GENERAL_PURPOSE_OUTPUT = 12,
  SET_CONTROLLER_GAIN = 13,
  GET_CONTROLLER_GAIN = 14,
  INVALID_CMD,
}PACKAGE_ID_FROM_SOC_E;

uint32_t get_complete_protocol_pkg(void)
{
  uint32_t i;
  uint32_t index;
  uint32_t complete_pkg_len;
  int32_t current_buff_len;
  uint32_t rxDataHead = g_rxDataHead;
  uint32_t rxDataTail = g_rxDataTail;
  uint32_t rxBuffSize = sizeof(g_rxBuffer)/sizeof(g_rxBuffer[0]);

  if (rxDataHead <= rxDataTail)
  {
    current_buff_len = rxDataTail - rxDataHead + 1;
  }
  else
  {
    current_buff_len = rxDataTail + rxBuffSize - rxDataHead + 1;
  }

  if (current_buff_len >= 4)
  {
    for (index = 0; index < current_buff_len; index++)
    {
      /* 确保校验位和长度位存在 */
      if (index + 2 < current_buff_len)
      {
        if ( (0xAA == g_rxBuffer[(rxDataHead + index)%rxBuffSize]) &&
            			(0x55 == g_rxBuffer[(rxDataHead + index + 1)%rxBuffSize]) )
        {
          complete_pkg_len = g_rxBuffer[(rxDataHead + index + 2)%rxBuffSize];

          if ( ( ((rxDataHead + index + 2) + complete_pkg_len + 1)%rxBuffSize ) <= rxDataTail )
          {
            memset(g_complete_pkg, 0, sizeof(g_complete_pkg));

            for (i = 0; i < complete_pkg_len+4; i++)
            {
              g_complete_pkg[i] = g_rxBuffer[(rxDataHead + index + i)%rxBuffSize];
              //printf("%x ", g_complete_pkg[i]);
            }
            //printf("\n");
            g_rxDataHead = (rxDataHead + index + i)%rxBuffSize;
            return 1;
          }
        }
      }
    }
  }

  return 0;
}

uint8_t *get_command_pkg(void)
{
  return (uint8_t*)&g_complete_pkg[0];
}

uint8_t get_command_pkg_len(void)
{
  return g_complete_pkg[2];
}

uint8_t get_command_pkg_id(void)
{
  return g_complete_pkg[3];
}

//uint8_t deal_command_package(uint16_t data_len)
//{
//  uint16_t i, j;
//  uint16_t pkg_len;
//  uint8_t cmdId;
//  int16_t speed;
//  int16_t radius;
//  uint8_t checksum = 0;
//  uint8_t* pdata = &g_rxBuffer[g_rxDataTail];

//  for (i = 0; i < data_len; i++)
//  {
//    printf("%x ", pdata[i]);
//  }
//  printf("\n");

//  for (i = 0; i <= data_len-10; i++)
//  {
//    if ((0xAA == pdata[i]) && (0x55 == pdata[i+1]))
//    {
//      checksum = 0;
//      pkg_len = pdata[i+2];
//      for (j = 0; j <= pkg_len; j++)
//      {
//        checksum ^= pdata[i+2+j];
//      }

//      if (checksum == pdata[i+2+pkg_len+1])
//      {
//        cmdId = pdata[i+2+1];
//        switch (cmdId)
//        {
//          case BASE_CONTROL_CMD:
//            speed  = pdata[i+2+3] | (pdata[i+2+4] << 8);
//            radius = pdata[i+2+5] | (pdata[i+2+6] << 8);
//            printf("s = %d, r = %d\n", speed, radius);
//            bcas_move_ctrl(speed, radius);
//          break;

//          case REQUEST_EXTRA:
//            g_version_type = pdata[i+2+3] | (pdata[i+2+4] << 8);
//          break;
//        }
//        return i+2+pkg_len+1+1;
//      }
//      else
//      {
//        return i+2;
//      }
//    }
//  }
//  return i+1;
//}

uint8_t deal_command_package(uint16_t data_len)
{
  uint16_t i, j;
  uint16_t index = 0;
  uint16_t pkg_len;
  uint8_t cmdId;
  int16_t speed;
  int16_t radius;
  uint8_t checksum = 0;
  uint8_t* pdata = &g_rxBuffer[g_rxDataTail];

#if 0
  for (i = 0; i < data_len; i++)
  {
    printf("%x ", pdata[i]);
  }
  printf("\n");
#endif

  while (index < data_len - 10)
  {
    if ((0xAA == pdata[index]) && (0x55 == pdata[index+1]))
    {
      checksum = 0;
      pkg_len = pdata[index+2];
      for (j = 0; j <= pkg_len; j++)
      {
        checksum ^= pdata[index+2+j];
      }

      if (checksum == pdata[index+2+pkg_len+1])
      {
        cmdId = pdata[index+2+1];
        switch (cmdId)
        {
          case BASE_CONTROL_CMD:
            speed  = pdata[index+2+3] | (pdata[index+2+4] << 8);
            radius = pdata[index+2+5] | (pdata[index+2+6] << 8);
            printf("%d, %d \n", speed, radius);
            bcas_move_ctrl(speed, radius);
          break;

          case REQUEST_EXTRA:
            g_version_type = pdata[index+2+3] | (pdata[index+2+4] << 8);
          break;
        }
        index += (pkg_len+3)+1;
      }
      else
      {
        index += 2;
      }
    }
    else
    {
      index++;
    }
  }

  return index;
}

void *serial_3_tx_function(void* thread_param)
{
  uint32_t result;

  for (;;)
  {
    serial_to_soc_pkg_create();
    result = write(g_serial_3_devId, &g_serial_to_soc_pkg.data[0], g_serial_to_soc_pkg.size);
    if (result < 0)
    {
      printf("write to soc serial fail!!!\n");
    }
    usleep(8000);
  }
  return NULL;
}

void *serial_3_rx_function(void* thread_param)
{
  int32_t i;
  int32_t result;
  uint8_t rxBuff[64];
  uint16_t move_bytes;
  uint8_t unuse_data_len;
  uint32_t rxBuffSize = sizeof(g_rxBuffer)/sizeof(g_rxBuffer[0]);

  g_rxDataHead = -1;
  g_rxDataTail = 0;

  for (;;)
  {
    memset(rxBuff, 0, sizeof(rxBuff));

    result = read(g_serial_3_devId, rxBuff, sizeof(rxBuff));
    if (result < 0)
    {
      printf("read from soc serial fail!!!\n");
    }
    else if (result > 0)
    {
      for (i = 0; i < result; i++)
      {
        g_rxBuffer[(g_rxDataHead+1+i)%rxBuffSize] = rxBuff[i];
      }
      g_rxDataHead = ((g_rxDataHead + result)%rxBuffSize);

      if (g_rxDataHead >= g_rxDataTail)
      {
        unuse_data_len = g_rxDataHead - g_rxDataTail + 1;
      }
      else
      {
        unuse_data_len = (g_rxDataHead + 1) + (rxBuffSize - g_rxDataTail);
      }

      if (unuse_data_len > 16)
      {
        move_bytes = deal_command_package(unuse_data_len);
        g_rxDataTail = ((g_rxDataTail + move_bytes)%rxBuffSize);
      }
    }
  }
  return NULL;
}

/*
 * IMU data: "#YPRAG=110.93,57.35,-94.99,-0.99,-0.39,0.07,-0.00,0.00,0.00
              #YPRAG=110.93,57.34,-94.99,-0.99,-0.39,0.07,0.00,0.00,0.01"
**/
// void deal_imu_data()
// {
//   uint8_t index;
//   uint8_t* phead = NULL;
//   uint8_t* ptail = NULL;
//   uint8_t* pcurrent = NULL;
//   uint8_t temp[8];
//   uint8_t* pimudata = &g_serial_5_rxbuff[0];
//   float imu_data[9];

//   memset(imu_data, 0, sizeof(imu_data));

//   phead = strstr(pimudata, "#YPRAG=");
//   if (phead == NULL)
//     return;
//   ptail = strstr(phead+1,  "#YPRAG=");
//   if (ptail == NULL)
//     return;

//   //printf("%s\n", phead);

//   pimudata = phead + 7;

//   for (index = 0; index < 9; index++)
//   {
//     if (index < 8)
//     {
//       pcurrent = strchr(pimudata, ',');
//     }
//     else
//     {
//       pcurrent = strchr(pimudata, '\n');
//     }

//     if ((pcurrent >= ptail) || (pcurrent == NULL))
//     {
//       return;
//     }
//     memset(temp, '\0', sizeof(temp));
//     memcpy(temp, pimudata, pcurrent-pimudata);
//     imu_data[index] = atof(temp);
//     pimudata = pcurrent+1;
//   }

//   // for (index = 0; index < 9; index++)
//   // {
//   //   if (index > 5)
//   //   {
//   //     imu_data[index] = imu_data[index] * 100;
//   //   }
//   //   printf(" %d ", (int16_t)imu_data[index]);
//   // }
//   // printf("\n");

//   g_inertial_sensor_data.Angle      = (int16_t)(imu_data[8] * 100);
//   g_inertial_sensor_data.Angle_rate = (int16_t)((imu_data[2] * 100 * RAD_TO_DEGREE) / 0.00875);
//   g_raw_data_gyro.x_axis = (int16_t)((imu_data[0] * RAD_TO_DEGREE) / 0.00875);
//   g_raw_data_gyro.y_axis = (int16_t)((imu_data[1] * RAD_TO_DEGREE) / 0.00875);
//   g_raw_data_gyro.z_axis = (int16_t)((imu_data[2] * RAD_TO_DEGREE) / 0.00875);

//   // printf("yaw = %d, x = %d, y = %d, z = %d\n", g_inertial_sensor_data.Angle, g_raw_data_gyro.x_axis,
//   //      g_raw_data_gyro.y_axis, g_raw_data_gyro.z_axis);

//   return;
// }

// void set_imudata_buffer()
// {
//     uint16_t i;
//     static uint32_t count = 0;
//     uint16_t imudata_len;
//     uint32_t rxBuffSize = sizeof(g_serial_5_rxbuff)/sizeof(g_serial_5_rxbuff[0]);
//     memset(g_imu_indata, '\0', sizeof(g_imu_indata));

//     if (g_imu_rxbuff_head >= g_imu_rxbuff_tail)
//     {
//         imudata_len = (g_imu_rxbuff_head - g_imu_rxbuff_tail + 1);
//     }
//     else
//     {
//         imudata_len = (g_imu_rxbuff_head - 0 + 1) + (rxBuffSize - g_imu_rxbuff_tail + 1);
//     }

//     for (i = 0; i < imudata_len; i++)
//     {
//       g_imu_indata[i] = g_serial_5_rxbuff[(g_imu_rxbuff_tail + i) % rxBuffSize];
//     }
//     return;
// }

// uint8_t set_imudata_package()
// {
//   uint8_t i;
//   uint8_t* phead = NULL;
//   uint8_t* ptail = NULL;
//   uint8_t* ptemp = NULL;
//   uint8_t* pdata = &g_imu_indata[0];
//   uint8_t temp[8];
//   float imu_data[9];
//   uint8_t pkgcount = 0;
//   uint16_t move_bytes;
//   uint8_t test[128];

//   for (;;)
//   {
//     phead = strstr(pdata, "#YPRAG=");
//     if ((phead == NULL) || (pdata > &g_imu_indata[sizeof(g_imu_indata)-1]))
//     {
//       move_bytes = pdata - &g_imu_indata[0];
//       return move_bytes;
//     }

//     ptail = strstr(pdata+1, "#YPRAG=");
//     if ((ptail == NULL) || (ptail > &g_imu_indata[sizeof(g_imu_indata)-1]))
//     {
//       move_bytes = pdata - &g_imu_indata[0];
//       return move_bytes;
//     }

//     memset(test, '\0', sizeof(test));
//     memcpy(test, phead, ptail - phead);
//     printf("%s", test);

//     pdata = phead + 7;
//     for (i = 0; i < 9; i++)
//     {
//       if (i < 8)
//       {
//         ptemp = strchr(pdata, ',');
//       }
//       else
//       {
//         ptemp = strchr(pdata, '\n');
//       }

//       if ((ptemp == NULL) || (ptemp >= ptail))
//       {
//         move_bytes = ptail - &g_imu_indata[0];
//         printf("3 move_bytes = %d\n", move_bytes);
//         return move_bytes;
//       }

//       memset(temp, '\0', sizeof(temp));
//       memcpy(temp, pdata, ptemp - pdata);
//       imu_data[i] = atof(temp);
//       pdata = ptemp + 1;
//     }

//     if (0 == pkgcount)
//     {
//       g_inertial_sensor_data.Angle      = (int16_t)(imu_data[8] * 100);
//       g_inertial_sensor_data.Angle_rate = (int16_t)((imu_data[2] * 100 * RAD_TO_DEGREE) / 0.00875);
//     }
//     g_raw_data_gyro.gyro_data[pkgcount].x_axis = (int16_t)((imu_data[0] * RAD_TO_DEGREE) / 0.00875);
//     g_raw_data_gyro.gyro_data[pkgcount].y_axis = (int16_t)((imu_data[1] * RAD_TO_DEGREE) / 0.00875);
//     g_raw_data_gyro.gyro_data[pkgcount].z_axis = (int16_t)((imu_data[2] * RAD_TO_DEGREE) / 0.00875);

//     // printf("yaw = %d, x = %d, y = %d, z = %d, ", g_inertial_sensor_data.Angle, g_raw_data_gyro.gyro_data[pkgcount].x_axis,
//     //                 g_raw_data_gyro.gyro_data[pkgcount].y_axis, g_raw_data_gyro.gyro_data[pkgcount].z_axis);
//     pkgcount++;
//     g_imu_package_count = pkgcount;
//     printf("pkg = %d\n", g_imu_package_count);
//   }
//   return 0;
// }

bool is_imu_data_valid(int16_t current_angle)
{
  uint8_t i;
  int16_t average;
  int16_t sum = 0;
  static uint8_t count = 0;
  static int16_t imu_angle[10] = {0};
  uint8_t array_size = sizeof(imu_angle)/sizeof(int16_t);

  for (i = 0; i < array_size; i++)
  {
    sum += imu_angle[i];
  }
  average = sum/array_size;
  printf("c = %d, a = %d\n", current_angle, average);

  if (abs(current_angle - average) > 1000)
  {
    return false;
  }
  else
  {
    imu_angle[count%array_size] = current_angle;
    count++;
    return true;
  }
}

int32_t search_full_package(uint8_t* pdata, uint32_t data_len)
{
  uint8_t i,j;
  uint8_t length;
  uint8_t* pcurrent = pdata;
  uint8_t* phead = NULL;
  uint8_t* ptail = NULL;
  uint8_t* ptemp = NULL;
  uint8_t temp_data[16];
  int16_t angle_current;
  uint16_t diff, diff_1, diff_2;
  static int16_t angle_last = 0;
  int32_t head_temp;
  float imu_data[9];
  uint32_t imu_pkg_buffer_size = sizeof(g_imu_pkg_buffer)/sizeof(g_imu_pkg_buffer[0]);
  static bool imu_reboot_flag = true;
  static float imu_init_value = 0;

  for (;;)
  {
    phead = strstr(pcurrent, "#YPRAG=");
    if (phead == NULL)
    {
      return pcurrent - pdata;
    }
    pcurrent = phead + 7;

    ptail = strchr(pcurrent, '\n');
    if (ptail == NULL)
    {
      return phead - pdata;
    }

    memset(imu_data, 0, sizeof(imu_data));
    for (i = 0; i < 4; i++)
    {
      if (i < 3)
      {
        ptemp = strchr(pcurrent, ',');
      }
      else
      {
        ptemp = strchr(pcurrent, '\n');
      }

      if ((ptemp == NULL) || (ptemp > ptail))
      {
        pcurrent = ptail + 1;
      }
      memset(temp_data, '\0', sizeof(temp_data));
      imu_data[i] = atof(pcurrent);
      pcurrent = ptemp + 1;
    }

    if (imu_reboot_flag)
    {
      imu_init_value = imu_data[3];
      imu_reboot_flag = false;
      /*printf("init = %d\n", (int32_t)imu_init_value);*/
    }

    if (imu_data[3] - imu_init_value < -180)
    {
      imu_data[3] = (360 + imu_data[3] - imu_init_value);
    }
    else if (imu_data[3] - imu_init_value > 180)
    {
      imu_data[3] = (-360 + imu_data[3] - imu_init_value);
    }
    else
    {
      imu_data[3] = imu_data[3] - imu_init_value;
    }
    angle_current = (int16_t)(imu_data[3] * 100);

    if ((angle_last >= 0) && (angle_last < 18000) && (angle_current <= 0) && (angle_current > -18000))
    {
      diff_1 = (18000 - angle_last) + (angle_current + 18000);
      diff_2 = angle_last - angle_current;
      (diff_1 <= diff_2) ? (diff  = diff_1) : (diff = diff_2);
    }
    else if ((angle_current >= 0) && (angle_current < 18000) && (angle_last <= 0) && (angle_last > -18000))
    {
      diff_1 = (18000 - angle_current) + (angle_last + 18000);
      diff_2 = angle_current - angle_last;
      (diff_1 <= diff_2) ? (diff  = diff_1) : (diff = diff_2);
    }
    else
    {
      (angle_current >= angle_last) ? (diff = angle_current - angle_last) : (diff = angle_last - angle_current);
    }

    //printf("diff = %d, c = %d, l = %d\n", diff, angle_current, angle_last);
    if (diff > 1000)
    {
      continue;
    }
    else
    {
      angle_last = angle_current;
    }

    head_temp = (g_imu_pkg_head + 1)%imu_pkg_buffer_size;
    g_imu_pkg_buffer[head_temp].angle  = angle_current;
    g_imu_pkg_buffer[head_temp].angle_rate = (int16_t)((imu_data[2] * 100 * RAD_TO_DEGREE) / 0.00875);
    g_imu_pkg_buffer[head_temp].x_axis = (int16_t)((imu_data[0] * RAD_TO_DEGREE) / 0.00875);
    g_imu_pkg_buffer[head_temp].y_axis = (int16_t)((imu_data[1] * RAD_TO_DEGREE) / 0.00875);
    g_imu_pkg_buffer[head_temp].z_axis = (int16_t)((imu_data[2] * RAD_TO_DEGREE) / 0.00875);
    // g_imu_pkg_buffer[head_temp].angle_rate = 0;
    // g_imu_pkg_buffer[head_temp].x_axis = 0;
    // g_imu_pkg_buffer[head_temp].y_axis = 0;
    // g_imu_pkg_buffer[head_temp].z_axis = 0;
    g_imu_pkg_head = head_temp;
    // printf("angle = %6d, x = %6d, y = %6d, z = %6d\n", g_imu_pkg_buffer[g_imu_pkg_head].angle,
    //   g_imu_pkg_buffer[g_imu_pkg_head].x_axis, g_imu_pkg_buffer[g_imu_pkg_head].y_axis,
    //   g_imu_pkg_buffer[g_imu_pkg_head].z_axis);
  }
  return data_len;
}

int32_t update_imu_package_buffer(int32_t data_head, int32_t data_tail, int32_t data_buffer_size)
{
  uint16_t move_bytes;

  uint16_t data_len;
  uint8_t temp_data[512];

  memset(temp_data, '\0', sizeof(temp_data));
  if (data_head >= data_tail)
  {
    data_len = data_head - data_tail + 1;
    memcpy(temp_data, &g_serial_5_rxbuff[data_tail], data_len);
  }
  else
  {
    data_len = (data_head - 0 + 1) + (data_buffer_size - data_tail);
    memcpy(temp_data, &g_serial_5_rxbuff[data_tail], (data_buffer_size - data_tail));
    memcpy(&temp_data[(data_buffer_size - data_tail)], &g_serial_5_rxbuff[0], (data_head - 0 + 1));
  }
  //printf("%s\n", temp_data);
  move_bytes = search_full_package(temp_data, data_len);
  return move_bytes;
}

/******************************************************
 * 接收IMU数据线程
******************************************************/
void *serial_5_rx_function(void* thread_param)
{
  uint8_t i;
  int32_t result;
  uint8_t rxBuff[128];
  uint8_t count = 0;
  uint32_t move_bytes = 0;
  uint32_t rxBuffSize = sizeof(g_serial_5_rxbuff)/sizeof(g_serial_5_rxbuff[0]);
  static bool complete_flag = false;

  g_imu_rxbuff_head = -1;
  g_imu_rxbuff_tail = 0;

  memset(g_serial_5_rxbuff, '\0', sizeof(g_serial_5_rxbuff));

  for (;;)
  {
    memset(rxBuff, '\0', sizeof(rxBuff));

    result = read(g_serial_5_devId, rxBuff, sizeof(rxBuff));
    if (result < 0)
    {
      printf("read from imu serial fail!!!\n");
    }
    else
    {
      for (i = 0; i < result; i++)
      {
        g_serial_5_rxbuff[(g_imu_rxbuff_head + 1 + i)%rxBuffSize] = rxBuff[i];
      }
      g_imu_rxbuff_head = ((g_imu_rxbuff_head + result)%rxBuffSize);

      /* over 128bytes update imu_data_buffer */
      if (abs(g_imu_rxbuff_head-g_imu_rxbuff_tail) >= 64)
      {
        move_bytes = update_imu_package_buffer(g_imu_rxbuff_head, g_imu_rxbuff_tail, rxBuffSize);
        g_imu_rxbuff_tail = ((g_imu_rxbuff_tail + move_bytes) % rxBuffSize);
      }
    }
  }
  return NULL;
}

/**
 * @ 串口设备初始化
 *
 * @ return 0: Success anything: Fail
 */
int32_t serial_dev_initialize(void)
{
  int32_t result;

  result = open(TO_SOC_SERIAL_PATH, O_RDWR);
  if (result < 0)
  {
    printf("open to soc serial fail!!!\n");
    return result;
  }
  g_serial_3_devId = result;

  result = open(TO_IMU_SERIAL_PATH, O_RDWR);
  if (result < 0)
  {
    printf("open to imu serial fail!!!\n");
    return result;
  }
  g_serial_5_devId = result;

  return 0;
}
