#ifndef MOTOR_H
#define MOTOR_H
#include "struct_typedef.h"
#include "PID.h"
#include "remote_control.h"

//rm motor data
//接收电机的返回值
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;     // 电机转速
    int16_t given_current; // 给定电流
    uint8_t temperate;     
    int16_t last_ecd;
} motor_measure_t;

//dart motor data
//单个电机数据结构体
typedef struct
{
  motor_measure_t motor_measure; // 电机的基本数据
  fp32 accel;     // 加速度
  fp32 speed;	  // 速度
  fp32 target_speed; // 设定速度
  int16_t give_voltage;  // 电压赋值 
} dart_motor_t;

//dart total data
//飞镖整体控制结构体
typedef struct 
{
    dart_motor_t motor[5];     //四个发射电机3508,一个装填电机2006
    pid_struct_t motor_speed_pid[5];    //四个电机的pid
}dart_control;

dart_control   dart;

extern void dart_shoot_reset(dart_control *_dart);
extern void dart_reload_reset(dart_control *_dart);

#endif
