// calibration.h
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "can_protocol.h"
#include "mcp2515.h"

#define BOOT_IDLE      0
#define BOOT_ANNOUNCE  1
#define BOOT_DONE      2

#define CALIB_IDLE           0
#define CALIB_START          1
#define CALIB_BKG_WAIT       2
#define CALIB_WAIT_REPLY_BKG 3
#define CALIB_LED_ON         4
#define CALIB_LED_WAIT       5
#define CALIB_REQ_LUX        6
#define CALIB_WAIT_REPLY_LUX 7
#define CALIB_DONE           8

struct NodeInfo {
    uint32_t uid;
    uint8_t addr;
};

extern uint8_t boot_state;
extern uint8_t calib_state;
extern uint32_t my_uid;
extern uint8_t my_addr;
extern bool is_hub;

extern uint8_t num_nodes;
extern NodeInfo network[ADDR_MAX_NODES];
extern float gain_matrix[ADDR_MAX_NODES][ADDR_MAX_NODES];

extern float gain_static;
extern float lux_background;

void calibration_setup();
void boot_task();
void calib_task();
void process_calib_message(const struct can_frame &f);

void can_send_frame(struct can_frame &f);
void set_led_duty(float duty);
float get_measured_lux();

#endif