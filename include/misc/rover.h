/*
 * rover.h
 * 
 * Rover microcontroller 0 (wheels, distance sensor) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * Copyright (C) 2016 Tomasz Chadzynski
 */

#ifndef _ROVER_H_
#define _ROVER_H_

/*
 * Mesaages going to/from device are in the form:
 * 1. Command [command:int16_t, param:int16_t]
 * 2. Return data [command:int16_t, data:int16_t] 
 */

enum command {
  NOP = 0x0,
  REQ_LEFT_SPEED = 0x01,
  REQ_RIGHT_SPEED = 0x02,
  SET_LEFT_SPEED = 0x03,
  SET_RIGHT_SPEED = 0x04,
  SET_TOTAL_SPEED = 0x05,
  WHEELS_STOP = 0x06,
  REQ_DISTANCE = 0x07,
  INIT_CONTROL_STATE = 0xF0,
  ERR = 0xFF
}; 

struct uc0_wheel_state {
    int16_t left_wheel_speed;
    int16_t right_wheel_speed;
    int16_t wheel_max_speed;
    int16_t wheel_min_speed;
};
/*
 * to se the wheel speed pass int16_t[2] as an argument (0=left 1=right)
 */
enum ioctl_command {
    SET_WHEEL_SPEED = 0xF1, 
    STOP = 0xF2
};

#endif
