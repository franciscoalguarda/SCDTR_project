// calibration.cpp
#include "calibration.h"
#include "hardware/flash.h"

uint8_t boot_state = BOOT_IDLE;
uint8_t calib_state = CALIB_IDLE;
uint32_t my_uid = 0;
uint8_t my_addr = 0;
bool is_hub = false;

uint8_t num_nodes = 0;
NodeInfo network[ADDR_MAX_NODES];
float gain_matrix[ADDR_MAX_NODES][ADDR_MAX_NODES];
float lux_off[ADDR_MAX_NODES];

static unsigned long last_hello_ms = 0;
static const unsigned long HELLO_PERIOD_MS = 500;

static uint8_t calib_led_node = 1;
static uint8_t calib_req_node = 1;
static unsigned long calib_timer = 0;
static const unsigned long CALIB_LED_SETTLE_MS = 2000;

bool register_node(uint32_t uid) {
    for (int i = 0; i < num_nodes; i++) {
        if (network[i].uid == uid) return false;
    }
    if (num_nodes < ADDR_MAX_NODES) {
        network[num_nodes++].uid = uid;
        return true;
    }
    return false;
}

void assign_addresses() {
    for (int i = 1; i < num_nodes; i++) {
        NodeInfo key = network[i];
        int j = i - 1;
        while (j >= 0 && network[j].uid > key.uid) {
            network[j+1] = network[j];
            j--;
        }
        network[j+1] = key;
    }
    for (int i = 0; i < num_nodes; i++) {
        network[i].addr = i + 1;
        if (network[i].uid == my_uid) my_addr = i + 1;
    }
}

uint32_t get_unique_id() {
    uint8_t id[8];
    flash_get_unique_id(id);
    return ((uint32_t)id[4] << 24) | ((uint32_t)id[5] << 16) | ((uint32_t)id[6] << 8) | ((uint32_t)id[7]);
}

void can_broadcast_cmd(uint8_t type) {
    struct can_frame tx_frame;
    tx_frame.can_id = MAKE_CAN_ID(type, ADDR_BROADCAST);
    tx_frame.can_dlc = 1;
    tx_frame.data[0] = my_addr;
    can_send_frame(tx_frame);
}

void send_hello() {
    struct can_frame tx_frame;
    uint8_t buf[4];
    buf[0] = (my_uid >> 24) & 0xFF;
    buf[1] = (my_uid >> 16) & 0xFF;
    buf[2] = (my_uid >> 8) & 0xFF;
    buf[3] = (my_uid) & 0xFF;
    
    tx_frame.can_id = MAKE_CAN_ID(MSG_HELLO, ADDR_BROADCAST);
    tx_frame.can_dlc = 5;
    tx_frame.data[0] = 0;
    memcpy(&tx_frame.data[1], buf, 4);
    can_send_frame(tx_frame);
}

void calibration_setup() {
    my_uid = get_unique_id();
    memset(gain_matrix, 0, sizeof(gain_matrix));
    memset(lux_off, 0, sizeof(lux_off));
    boot_state = BOOT_IDLE;
    calib_state = CALIB_IDLE;
}

void boot_task() {
    unsigned long now = millis();
    switch (boot_state) {
        case BOOT_IDLE:
            register_node(my_uid);
            boot_state = BOOT_ANNOUNCE;
            send_hello();
            last_hello_ms = now;
            break;
        case BOOT_ANNOUNCE:
            if (now - last_hello_ms >= HELLO_PERIOD_MS) {
                send_hello();
                last_hello_ms = now;
            }
            if (num_nodes == ADDR_MAX_NODES) {
                assign_addresses();
                boot_state = BOOT_DONE;
                if (is_hub) {
                    calib_state = CALIB_START;
                }
            }
            break;
    }
}

void calib_task() {
    if (!is_hub) return;
    unsigned long now = millis();

    switch (calib_state) {
        case CALIB_START:
            can_broadcast_cmd(MSG_CALIB_START);
            set_led_duty(0.0f);
            calib_timer = now;
            calib_req_node = 1;
            calib_state = CALIB_BKG_WAIT;
            break;
            
        case CALIB_BKG_WAIT:
            if (now - calib_timer >= CALIB_LED_SETTLE_MS) {
                struct can_frame tx_frame;
                tx_frame.can_id = MAKE_CAN_ID(MSG_GET_LUX, calib_req_node);
                tx_frame.can_dlc = 1;
                tx_frame.data[0] = my_addr;
                can_send_frame(tx_frame);
                calib_state = CALIB_WAIT_REPLY_BKG;
            }
            break;

        case CALIB_LED_ON:
            if (calib_led_node == my_addr) {
                set_led_duty(1.0f);
            } else {
                struct can_frame tx_frame;
                uint8_t buf[4];
                float duty = 1.0f;
                memcpy(buf, &duty, 4);
                tx_frame.can_id = MAKE_CAN_ID(MSG_CALIB_LED_ON, calib_led_node);
                tx_frame.can_dlc = 5;
                tx_frame.data[0] = my_addr;
                memcpy(&tx_frame.data[1], buf, 4);
                can_send_frame(tx_frame);
            }
            calib_timer = now;
            calib_req_node = 1;
            calib_state = CALIB_LED_WAIT;
            break;

        case CALIB_LED_WAIT:
            if (now - calib_timer >= CALIB_LED_SETTLE_MS) {
                calib_state = CALIB_REQ_LUX;
            }
            break;

        case CALIB_REQ_LUX:
            {
                struct can_frame tx_frame;
                tx_frame.can_id = MAKE_CAN_ID(MSG_GET_LUX, calib_req_node);
                tx_frame.can_dlc = 1;
                tx_frame.data[0] = my_addr;
                can_send_frame(tx_frame);
                calib_state = CALIB_WAIT_REPLY_LUX;
            }
            break;

        case CALIB_DONE:
            can_broadcast_cmd(MSG_CALIB_DONE);
            calib_state = CALIB_IDLE;
            break;
    }
}

void process_calib_message(const struct can_frame &f) {
    uint8_t type = GET_MSG_TYPE(f.can_id);
    uint8_t dest = GET_DEST(f.can_id);
    uint8_t src  = f.data[0];

    if (dest != ADDR_BROADCAST && dest != my_addr) return;

    switch (type) {
        case MSG_HELLO: {
            uint32_t uid = ((uint32_t)f.data[1] << 24) | ((uint32_t)f.data[2] << 16) | ((uint32_t)f.data[3] << 8) | ((uint32_t)f.data[4]);
            register_node(uid);
            break;
        }
        case MSG_GET_LUX: {
            uint8_t buf[4];
            float lux = get_measured_lux();
            memcpy(buf, &lux, 4);
            struct can_frame tx_frame;
            tx_frame.can_id = MAKE_CAN_ID(MSG_LUX, src);
            tx_frame.can_dlc = 5;
            tx_frame.data[0] = my_addr;
            memcpy(&tx_frame.data[1], buf, 4);
            can_send_frame(tx_frame);
            break;
        }
        case MSG_LUX: {
            float lux;
            memcpy(&lux, &f.data[1], 4);
            int idx = src - 1;
            
            if (calib_state == CALIB_WAIT_REPLY_BKG) {
                if (calib_req_node == src) {
                    lux_off[idx] = lux;
                    calib_req_node++;
                    if (calib_req_node > num_nodes) {
                        calib_led_node = 1;
                        calib_state = CALIB_LED_ON;
                    } else {
                        calib_timer = millis();
                        calib_state = CALIB_BKG_WAIT;
                    }
                }
            } else if (calib_state == CALIB_WAIT_REPLY_LUX) {
                if (calib_req_node == src) {
                    int led_idx = calib_led_node - 1;
                    gain_matrix[led_idx][idx] = lux - lux_off[idx];
                    if (gain_matrix[led_idx][idx] < 0) gain_matrix[led_idx][idx] = 0;
                    calib_req_node++;
                    
                    if (calib_req_node > num_nodes) {
                        if (calib_led_node == my_addr) {
                            set_led_duty(0.0f);
                        } else {
                            struct can_frame tx_frame;
                            tx_frame.can_id = MAKE_CAN_ID(MSG_CALIB_LED_OFF, calib_led_node);
                            tx_frame.can_dlc = 1;
                            tx_frame.data[0] = my_addr;
                            can_send_frame(tx_frame);
                        }
                        calib_led_node++;
                        if (calib_led_node > num_nodes) {
                            gain_static = gain_matrix[my_addr - 1][my_addr - 1];
                            lux_background = lux_off[my_addr - 1];
                            calib_state = CALIB_DONE;
                        } else {
                            calib_state = CALIB_LED_ON;
                        }
                    } else {
                        calib_state = CALIB_REQ_LUX;
                    }
                }
            }
            break;
        }
        case MSG_CALIB_START:
            if (!is_hub) set_led_duty(0.0f);
            break;
        case MSG_CALIB_LED_ON:
            if (!is_hub && f.data[0] == my_addr) {
                float duty;
                memcpy(&duty, &f.data[1], 4);
                set_led_duty(duty);
            }
            break;
        case MSG_CALIB_LED_OFF:
            if (!is_hub) set_led_duty(0.0f);
            break;
        case MSG_CALIB_DONE:
            if (!is_hub) {
                gain_static = gain_matrix[my_addr - 1][my_addr - 1];
                lux_background = lux_off[my_addr - 1];
            }
            break;
    }
}