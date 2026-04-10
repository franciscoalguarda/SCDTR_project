#include <SPI.h>
#include <mcp2515.h>
#include "pico/unique_id.h" 
#include "pico/mutex.h"     

// ======================================================================
// --- CAN-BUS / NETWORK CONFIGURATION ---
// ======================================================================
const int CAN_CS_PIN = 17; 
MCP2515* mcp2515; 

enum BootState { BOOT_IDLE, BOOT_ANNOUNCE, BOOT_WAIT_STABLE, BOOT_DONE };
enum CalibState { CALIB_IDLE, CALIB_START, CALIB_BKG_WAIT, CALIB_LED_ON, CALIB_LED_WAIT, CALIB_REQ_LUX, CALIB_WAIT_REPLY_LUX, CALIB_WAIT_REPLY_BKG, CALIB_DONE };

volatile BootState boot_state = BOOT_IDLE;
volatile CalibState calib_state = CALIB_IDLE;
volatile bool system_ready = false; 

uint32_t my_uid = 0;
int my_addr = 0;     
bool is_hub = false; 

// Limite máximo de memória (Plug & Play dita o número real de nós)
const int MAX_NODES = 10;
int num_nodes = 0;
uint32_t network_uids[MAX_NODES];

float gain_matrix[MAX_NODES][MAX_NODES]; 
float lux_off[MAX_NODES];                
float network_lux[MAX_NODES];            
float network_u[MAX_NODES] = {0}; 

unsigned long last_hello_ms = 0;
const unsigned long HELLO_PERIOD_MS = 500;
unsigned long boot_start_ms = 0;
unsigned long last_discovery_ms = 0; // Cronómetro do Plug & Play

int calib_led_node = 1;
int calib_req_node = 1;
unsigned long calib_timer = 0;
const unsigned long CALIB_LED_SETTLE_MS = 2000;

int send_bkg_idx = 0;
int send_mat_row = 0;
int send_mat_col = 0;

mutex_t data_mutex;

// Watchdog de Falhas de Rede
unsigned long last_rx_time[MAX_NODES] = {0}; 
const unsigned long WATCHDOG_TIMEOUT_MS = 2000;

// ======================================================================
// --- STREAMING & CONTROL STATE ---
// ======================================================================
bool stream_active_y[MAX_NODES + 1] = {false}; 
bool stream_active_u[MAX_NODES + 1] = {false};

volatile bool distributed_ctrl = true; 
float current_lambda = 0.0;             
float network_lambda[MAX_NODES] = {0}; 

volatile float pd_rho = 0.0001;         
volatile float pd_alpha = 0.005;        
volatile float pd_decay = 0.001;        

// ======================================================================
// --- LOCAL LUMINAIRE CONFIGURATION (PHASE 1) ---
// ======================================================================
const int LED_PIN = 15;
const int DAC_RANGE = 4096;

float m = 0.0; 
float b = 0.0;  

float background_lux = 0;
volatile float system_gain = 0.0;
const float h = 0.01; 

volatile bool feedback_enabled = false; 
volatile bool anti_windup_enabled = true; 
volatile float manual_pwm = 0;             
float current_lux = 0;           
float current_u = 0;       
char current_occupancy = 'h';    

float lower_bound_low = 10.0;  
float lower_bound_high = 50.0; 

// --- RAMPA DE REFERENCIA (Soft Start) ---
volatile float current_setpoint = lower_bound_high; // O que a matemática vê (Muda devagar)
volatile float target_setpoint = lower_bound_high;  // O objetivo final (Muda rápido pelo comando)
// ----------------------------------------------

volatile float energy_cost = 1.0;       

// Variável global para o Filtro IIR
volatile float filtered_lux_val = -1.0;

const int minute_buffer_size = 600; 
float history_y[minute_buffer_size];
float history_u[minute_buffer_size];
int history_idx = 0;

float E_energy = 0;          
float V_visibility_sum = 0;  
float F_flicker_sum = 0;     
long metrics_count = 0;      
float last_u_1 = 0, last_u_2 = 0;          
float P_max = 0.05;          

void printHelpMenu();
float getFilteredLux(float new_sample);
float get_R_LDR(); 
float getLux();
void updateMetrics(float u, float y, float ref);
void process_calib_and_network(struct can_frame &canMsg);
void executeCommand(String input); 

// ======================================================================
// --- PID CONTROLLER CLASS ---
// ======================================================================
class PIDController {
  private: 
    float Kp, Ki, Kd, b_weight, i_term, last_error, h;
  public:
    PIDController(float p, float i, float d, float bw) {
      Kp = p; Ki = i; Kd = d; b_weight = bw; i_term = 0; last_error = 0; h = ::h; 
    }
    float compute(float ref, float y, bool antiwindup_control, float ff_pwm = 0.0) {
      float error_p = b_weight * ref - y; 
      float error = ref - y; 
      float p_term = Kp * error_p;
      i_term += Ki * error * h; 
      float d_term = Kd * (error - last_error) / h;
      last_error = error;
      
      float u = p_term + i_term + d_term + ff_pwm; 
      
      if (u > 4095) { u = 4095; if (antiwindup_control && error > 0) i_term -= Ki * error * h; } 
      else if (u < 0) { u = 0; if (antiwindup_control && error < 0) i_term -= Ki * error * h; }
      return u;
    }
    void setKp(float p) { Kp = p; }
    void setKi(float i) { Ki = i; }
    void setKd(float d) { Kd = d; }
    void reset() { i_term = 0; last_error = 0; }
};
PIDController myPID(80.0, 400.0, 0.0, 0.5); 

// ======================================================================
// --- SENSOR READING & METRICS ---
// ======================================================================

// Filtro Passa-Baixo Exponencial (IIR)
float getFilteredLux(float new_sample) {
    if (filtered_lux_val < 0.0) {
        filtered_lux_val = new_sample;
        return filtered_lux_val;
    }
    filtered_lux_val = (0.05 * new_sample) + (0.95 * filtered_lux_val);
    return filtered_lux_val;
}

float get_R_LDR() {
  float adc_sum = 0;
  for(int j = 0; j < 10; j++) { adc_sum += analogRead(A0); delayMicroseconds(500); }
  float read_adc = adc_sum / 10.0; if (read_adc == 0) read_adc = 1;
  return 40950000.0 / (float)read_adc - 10000.0;
}

float getLux() {
  float R_LDR = get_R_LDR();
  return pow(10, (log10(R_LDR) - b) / m);
}

void updateMetrics(float u, float y, float ref) {
    mutex_enter_blocking(&data_mutex); 
    float d_k = u / 4095.0; 
    E_energy += P_max * d_k * h;
    if (y < ref) V_visibility_sum += (ref - y);
    float diff1 = d_k - last_u_1;
    float diff2 = last_u_1 - last_u_2;
    if (diff1 * diff2 < 0) F_flicker_sum += (abs(diff1) + abs(diff2));
    last_u_2 = last_u_1;
    last_u_1 = d_k;
    metrics_count++;
    mutex_exit(&data_mutex);
}

// ======================================================================
// --- AUTO-ADDRESSING LOGIC ---
// ======================================================================
uint32_t get_unique_id() {
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    return board_id.id[6]; 
}

bool register_node(uint32_t uid) {
    for (int i = 0; i < num_nodes; i++) {
        if (network_uids[i] == uid) return false;
    }
    if (num_nodes < MAX_NODES) {
        network_uids[num_nodes++] = uid;
        last_discovery_ms = millis(); 
        return true;
    }
    return false;
}

void assign_addresses() {
    for (int i = 0; i < num_nodes - 1; i++) {
        for (int j = 0; j < num_nodes - i - 1; j++) {
            if (network_uids[j] > network_uids[j + 1]) {
                uint32_t temp = network_uids[j];
                network_uids[j] = network_uids[j + 1];
                network_uids[j + 1] = temp;
            }
        }
    }
    for (int i = 0; i < num_nodes; i++) {
        if (network_uids[i] == my_uid) my_addr = i + 1;
    }
    if (my_addr == 1) is_hub = true; 
    
    Serial.print("\n=== BOOT DONE ===\nMy UID: "); Serial.println(my_uid);
    Serial.print("Assigned Address: NODE "); Serial.println(my_addr);
    Serial.print("Role: "); Serial.println(is_hub ? "Hub" : "Node");
}

// ======================================================================
// --- CAN COMMUNICATION & PROTOCOL ---
// ======================================================================
void sendCANNetworkMsg(char action, char variable, int dest_node, float value) {
  struct can_frame canMsg;
  canMsg.can_id  = my_addr > 0 ? my_addr : 99; 
  canMsg.can_dlc = 7; 
  canMsg.data[0] = action;    
  canMsg.data[1] = variable;  
  canMsg.data[2] = dest_node; 
  
  unsigned char* p = (unsigned char*)&value;
  for(int i = 0; i < 4; i++) canMsg.data[3 + i] = p[i];
  
  mcp2515->sendMessage(&canMsg);

  // Loopback Local 
  if (dest_node == my_addr || dest_node == 0) {
      process_calib_and_network(canMsg);
  }
}

void sendCANUidMsg(int dest_node, uint32_t uid_val) {
  struct can_frame canMsg;
  canMsg.can_id  = my_addr > 0 ? my_addr : 99;
  canMsg.can_dlc = 7;
  canMsg.data[0] = 'd';   
  canMsg.data[1] = 'I';   
  canMsg.data[2] = dest_node;
  unsigned char* p = (unsigned char*)&uid_val;
  for(int i = 0; i < 4; i++) canMsg.data[3 + i] = p[i];
  mcp2515->sendMessage(&canMsg);
}

void send_hello() {
  struct can_frame canMsg;
  canMsg.can_id  = my_uid; 
  canMsg.can_dlc = 7;
  canMsg.data[0] = 'w'; 
  canMsg.data[1] = 'h'; 
  canMsg.data[2] = 0;   
  unsigned char* p = (unsigned char*)&my_uid;
  for(int i = 0; i < 4; i++) canMsg.data[3 + i] = p[i];
  mcp2515->sendMessage(&canMsg);
}

void process_calib_and_network(struct can_frame &canMsg) {
    int sender_id = canMsg.can_id;
    char action = canMsg.data[0];
    char variable = canMsg.data[1];
    int dest_node = canMsg.data[2];

    float value;
    unsigned char* p = (unsigned char*)&value;
    for(int i = 0; i < 4; i++) p[i] = canMsg.data[3 + i];

    if (action == 's') {
        Serial.print("s "); Serial.print(variable); Serial.print(" "); Serial.print(sender_id);
        Serial.print(" "); Serial.print(value, 4);
        Serial.print(" "); Serial.println(millis());
        return;
    }

    if (action == 'R') {
        Serial.println("\n[SYSTEM] Restart command received. Rebooting and recalibrating...");
        mutex_enter_blocking(&data_mutex);
        myPID.reset(); E_energy = 0; V_visibility_sum = 0; F_flicker_sum = 0; metrics_count = 0;
        distributed_ctrl = false; 
        current_lambda = 0.0;
        filtered_lux_val = -1.0; 
        for(int i=0; i<MAX_NODES; i++) last_rx_time[i] = 0; 
        mutex_exit(&data_mutex);
        
        boot_state = BOOT_IDLE; 
        calib_state = CALIB_IDLE;
        system_ready = false;
        num_nodes = 0; 
        is_hub = false;
        background_lux = 0;
        // Streams não são mortos aqui para podermos fazer o plot do arranque!
        return;
    }

    if (action == 'w' && variable == 'h') {
        uint32_t received_uid;
        unsigned char* up = (unsigned char*)&received_uid;
        for(int i = 0; i < 4; i++) up[i] = canMsg.data[3 + i];
        
        register_node(received_uid); 
        
        if (system_ready && is_hub) {
            Serial.println("\n[HUB] Reboot ou Novo no detetado! A forcar recalibracao global...");
            sendCANNetworkMsg('R', 'R', 0, 0); 
        }
        return;
    }

    if (action == 'M') {
        int row = variable - '0';
        int col = dest_node - 1;
        gain_matrix[row][col] = value;
        return;
    }
    if (action == 'B') {
        int idx = dest_node - 1;
        lux_off[idx] = value;
        return;
    }

    if (action == 'l' && sender_id >= 1 && sender_id <= MAX_NODES) {
        mutex_enter_blocking(&data_mutex);
        network_lambda[sender_id - 1] = value;
        last_rx_time[sender_id - 1] = millis(); 
        mutex_exit(&data_mutex);
        return; 
    }

    if (dest_node != my_addr && dest_node != 0) return; 

    if (action == 'c') {
        if (variable == 's') { if (!is_hub) analogWrite(LED_PIN, 0); } 
        else if (variable == 'l') { if (!is_hub) analogWrite(LED_PIN, 4095); } 
        else if (variable == 'o') { if (!is_hub) analogWrite(LED_PIN, 0); } 
        else if (variable == 'g') { 
            float meas = getLux();
            if (calib_led_node == 1 && !is_hub && background_lux == 0) background_lux = meas;
            sendCANNetworkMsg('c', 'x', sender_id, meas); 
        } 
        else if (variable == 'x' && is_hub) { 
            int idx = sender_id - 1;
            if (calib_state == CALIB_WAIT_REPLY_BKG) {
                if (calib_req_node == sender_id) {
                    lux_off[idx] = value;
                    calib_req_node++;
                    if (calib_req_node > num_nodes) {
                        calib_led_node = 1; calib_state = CALIB_LED_ON;
                    } else {
                        calib_timer = millis(); calib_state = CALIB_BKG_WAIT;
                    }
                }
            } else if (calib_state == CALIB_WAIT_REPLY_LUX) {
                if (calib_req_node == sender_id) {
                    int led_idx = calib_led_node - 1;
                    gain_matrix[led_idx][idx] = (value - lux_off[idx]) / 4095.0; 
                    if (gain_matrix[led_idx][idx] < 0) gain_matrix[led_idx][idx] = 0;
                    
                    calib_req_node++;
                    if (calib_req_node > num_nodes) {
                        if (calib_led_node == my_addr) analogWrite(LED_PIN, 0);
                        else sendCANNetworkMsg('c', 'o', calib_led_node, 0); 
                        
                        calib_led_node++;
                        if (calib_led_node > num_nodes) {
                            calib_state = CALIB_DONE;
                            send_bkg_idx = 0; send_mat_row = 0; send_mat_col = 0;
                            calib_timer = millis();
                        }
                        else calib_state = CALIB_LED_ON;
                    } else calib_state = CALIB_REQ_LUX;
                }
            }
        } 
        else if (variable == 'd') { 
            system_gain = gain_matrix[my_addr - 1][my_addr - 1];
            background_lux = lux_off[my_addr - 1];
            
            Serial.println("\n=== CALIBRATION MATRIX (K) ===");
            for(int i=0; i<num_nodes; i++) {
                for(int j=0; j<num_nodes; j++) {
                    Serial.print("K"); Serial.print(i+1); Serial.print(j+1); Serial.print(": ");
                    Serial.print(gain_matrix[i][j], 6); Serial.print("\t");
                }
                Serial.println();
            }
            Serial.println("==============================");
            printHelpMenu();
            
            system_ready = true;
            mutex_enter_blocking(&data_mutex);
            feedback_enabled = false; 
            distributed_ctrl = true; 
            current_lambda = 0.0;
            
            if (system_gain > 0.0001) {
                current_u = (current_setpoint - background_lux) / system_gain;
                if (current_u > 4095.0) current_u = 4095.0; 
                if (current_u < 0.0) current_u = 0.0;       
            } else {
                current_u = 0.0;
            }
            
            filtered_lux_val = background_lux; 
            for(int k=0; k<MAX_NODES; k++) last_rx_time[k] = millis();
            
            mutex_exit(&data_mutex);
        }
        return;
    }

    if (sender_id >= 1 && sender_id <= MAX_NODES) {
        if (variable == 'y') {
            network_lux[sender_id - 1] = value;
        }
        if (variable == 'u') {
            mutex_enter_blocking(&data_mutex);
            network_u[sender_id - 1] = value;
            last_rx_time[sender_id - 1] = millis(); 
            mutex_exit(&data_mutex);
        }
    }

    if (action == 'g') {
        if (variable == 'I') {
            sendCANNetworkMsg('d', 'N', sender_id, (float)my_addr); 
            sendCANUidMsg(sender_id, my_uid);                        
            return;
        }

        float reply_val = 0;
        switch(variable) {
            case 'y': mutex_enter_blocking(&data_mutex); reply_val = current_lux; mutex_exit(&data_mutex); break;
            case 'u': mutex_enter_blocking(&data_mutex); reply_val = current_u; mutex_exit(&data_mutex); break;
            case 'l': mutex_enter_blocking(&data_mutex); reply_val = current_lambda; mutex_exit(&data_mutex); break; 
            case 'r': reply_val = target_setpoint; break; 
            case 'v': reply_val = (analogRead(A0) / 4095.0) * 3.3; break;
            case 'R': reply_val = get_R_LDR(); break; 
            case 't': reply_val = millis() / 1000.0; break;
            case 'd': reply_val = background_lux; break;
            case 'p': mutex_enter_blocking(&data_mutex); reply_val = P_max * (current_u / 4095.0); mutex_exit(&data_mutex); break;
            case 'E': mutex_enter_blocking(&data_mutex); reply_val = E_energy; mutex_exit(&data_mutex); break;
            case 'V': mutex_enter_blocking(&data_mutex); reply_val = (metrics_count > 0) ? (V_visibility_sum / metrics_count) : 0; mutex_exit(&data_mutex); break;
            case 'F': mutex_enter_blocking(&data_mutex); reply_val = (metrics_count > 0) ? (F_flicker_sum / metrics_count) : 0; mutex_exit(&data_mutex); break;
            case 'o': reply_val = (float)current_occupancy; break; 
            case 'a': reply_val = (float)anti_windup_enabled; break;
            case 'f': reply_val = (float)feedback_enabled; break;
            case 'U': reply_val = lower_bound_low; break;
            case 'O': reply_val = lower_bound_high; break;
            case 'C': reply_val = energy_cost; break;
            case 'D': reply_val = (float)distributed_ctrl; break; 
            case 'P': reply_val = pd_rho; break;
            case 'A': reply_val = pd_alpha; break;
            case 'w': reply_val = pd_decay; break; 
            case 'L': 
                if (current_occupancy == 'h') reply_val = lower_bound_high;
                else if (current_occupancy == 'l') reply_val = lower_bound_low;
                else reply_val = 0.0;
                break;
        }
        sendCANNetworkMsg('d', variable, sender_id, reply_val);
    }
    else if (action == 'Y') { 
        if (dest_node == my_addr) {
            if (variable == 'y') stream_active_y[my_addr] = true;
            else if (variable == 'u') stream_active_u[my_addr] = true;
        }
    }
    else if (action == 'Z') { 
        if (dest_node == my_addr) {
            if (variable == 'y') stream_active_y[my_addr] = false;
            else if (variable == 'u') stream_active_u[my_addr] = false;
        }
    }
    else if (action == 'W') { 
        mutex_enter_blocking(&data_mutex);
        if (variable == 'r') target_setpoint = value; 
        else if (variable == 'u') { manual_pwm = value; feedback_enabled = false; distributed_ctrl = false; }
        else if (variable == 'f') { feedback_enabled = (value > 0); if (feedback_enabled) distributed_ctrl = false; }
        else if (variable == 'a') anti_windup_enabled = (value > 0);
        else if (variable == 'U') lower_bound_low = value;
        else if (variable == 'O') lower_bound_high = value;
        else if (variable == 'C') energy_cost = value;
        else if (variable == 'D') { 
            distributed_ctrl = (value > 0); 
            if (distributed_ctrl) feedback_enabled = false; 
            current_lambda = 0.0; 
        }
        else if (variable == 'P') pd_rho = value;
        else if (variable == 'A') pd_alpha = value;
        else if (variable == 'w') pd_decay = value; 
        else if (variable == 'o') { 
            current_occupancy = (char)value; 
            if (current_occupancy == 'h') target_setpoint = lower_bound_high;
            else if (current_occupancy == 'l') target_setpoint = lower_bound_low;
            else target_setpoint = 0.0;
        }
        mutex_exit(&data_mutex);
    }
    else if (action == 'd') { 
        if (variable == 'N') {
            Serial.print("id addr "); Serial.print(sender_id); Serial.print(" -> NODE "); Serial.println((int)value);
            return;
        }
        if (variable == 'I') {
            uint32_t uid_recv;
            unsigned char* up = (unsigned char*)&uid_recv;
            for(int i = 0; i < 4; i++) up[i] = canMsg.data[3 + i];
            Serial.print("id uid  "); Serial.print(sender_id); Serial.print(" -> "); Serial.println(uid_recv);
            return;
        }
        Serial.print(variable); Serial.print(" "); Serial.print(sender_id); Serial.print(" "); 
        if (variable == 'o') Serial.println((char)value);
        else Serial.println(value, 4);
    }
}

void readCANMessages() {
  struct can_frame canMsg;
  if (mcp2515->readMessage(&canMsg) == MCP2515::ERROR_OK) process_calib_and_network(canMsg);
}

// ======================================================================
// --- STATE MACHINE TASKS (BOOT & CALIB) ---
// ======================================================================
void boot_task() {
    unsigned long now = millis();
    switch (boot_state) {
        case BOOT_IDLE:
            my_uid = get_unique_id();
            
            if (my_uid == 97) { m = -1.0; b = 6.75; } 
            else if (my_uid == 107) { m = -1.0; b = 6.75; } 
            else if (my_uid == 122) { m = -1.0; b = 6.75; } 
            else { m = -1.0; b = 6.75; }

            last_discovery_ms = now; 
            register_node(my_uid);
            boot_state = BOOT_ANNOUNCE;
            send_hello();
            last_hello_ms = now;
            Serial.println("Waiting for nodes to join (Plug & Play)...");
            break;
            
        case BOOT_ANNOUNCE:
            if (now - last_hello_ms >= 500) {
                send_hello(); 
                last_hello_ms = now;
            }
            if (now - last_discovery_ms >= 8000) {
                send_hello(); 
                boot_start_ms = now; 
                boot_state = BOOT_WAIT_STABLE;
            }
            break;

        case BOOT_WAIT_STABLE: 
            if (now - boot_start_ms >= 500) {
                assign_addresses(); 
                
                energy_cost = 1.0; 
                if (my_addr == 2) {
                    energy_cost = 1.0;  
                }
                
                boot_state = BOOT_DONE;
                if (is_hub) calib_state = CALIB_START; 
            }
            break;
    }
}

void calib_task() {
    if (!is_hub) return;
    unsigned long now = millis();

    switch (calib_state) {
        case CALIB_START:
            Serial.println("\n[HUB] Starting Network Calibration...");
            calib_timer = now;
            calib_req_node = 1;
            calib_state = CALIB_BKG_WAIT; 
            sendCANNetworkMsg('c', 's', 0, 0); 
            analogWrite(LED_PIN, 0);
            break;
            
        case CALIB_BKG_WAIT:
            if (now - calib_timer >= CALIB_LED_SETTLE_MS) {
                calib_state = CALIB_WAIT_REPLY_BKG; 
                sendCANNetworkMsg('c', 'g', calib_req_node, 0); 
            }
            break;

        case CALIB_LED_ON:
            calib_timer = now;
            calib_req_node = 1;
            calib_state = CALIB_LED_WAIT; 
            if (calib_led_node == my_addr) analogWrite(LED_PIN, 4095);
            else sendCANNetworkMsg('c', 'l', calib_led_node, 0); 
            break;

        case CALIB_LED_WAIT:
            if (now - calib_timer >= CALIB_LED_SETTLE_MS) {
                calib_state = CALIB_REQ_LUX;
            }
            break;

        case CALIB_REQ_LUX:
            calib_state = CALIB_WAIT_REPLY_LUX; 
            sendCANNetworkMsg('c', 'g', calib_req_node, 0); 
            break;

        case CALIB_DONE: 
            if (now - calib_timer >= 10) { 
                calib_timer = now;
                
                if (send_bkg_idx < num_nodes) {
                    sendCANNetworkMsg('B', '0', send_bkg_idx + 1, lux_off[send_bkg_idx]);
                    send_bkg_idx++;
                } 
                else if (send_mat_row < num_nodes) {
                    sendCANNetworkMsg('M', '0' + send_mat_row, send_mat_col + 1, gain_matrix[send_mat_row][send_mat_col]);
                    send_mat_col++;
                    if (send_mat_col >= num_nodes) {
                        send_mat_col = 0;
                        send_mat_row++;
                    }
                } 
                else {
                    sendCANNetworkMsg('c', 'd', 0, 0); 
                    calib_state = CALIB_IDLE;
                }
            }
            break;
    }
}

// ======================================================================
// --- USER INTERFACE (HUB) ---
// ======================================================================
void printHelpMenu() {
  Serial.println("\n==================================================");
  Serial.println("         COMMAND MENU (PHASE 1 & 2 HUB)         ");
  Serial.println("==================================================");
  Serial.println("--- SET COMMANDS ---");
  Serial.println("  r <node> <val> : Set Setpoint (Ref) in LUX");
  Serial.println("  u <node> <val> : Set manual PWM (0-4095) [Needs f 0]"); 
  Serial.println("  f <node> <0/1> : Turn Feedback (PID) Off (0) or On (1)");
  Serial.println("  D <node> <0/1> : Turn Primal-Dual Algorithm Off (0) or On (1)");
  Serial.println("  a <node> <0/1> : Turn Anti-Windup Off (0) or On (1)");
  Serial.println("  o <node> <val> : Set occupancy ('o'=off, 'l'=low, 'h'=high)");
  Serial.println("  U <node> <val> : Set LOWER BOUND for LOW state (U)");
  Serial.println("  O <node> <val> : Set LOWER BOUND for HIGH state (O)");
  Serial.println("  C <node> <val> : Set Energy Cost (C)");
  Serial.println("  P <node> <val> : Set Primal Step (Rho) [Tuning]");
  Serial.println("  A <node> <val> : Set Dual Step (Alpha) [Tuning]");
  Serial.println("  w <node> <val> : Set Lambda Weight Decay [Tuning]");
  Serial.println("\n--- GET COMMANDS ---");
  Serial.println("  g r <node>     : Get current Setpoint");
  Serial.println("  g y <node>     : Get measured LUX");
  Serial.println("  g u <node>     : Get Duty Cycle / PWM");
  Serial.println("  g D <node>     : Get Primal-Dual State");
  Serial.println("  g U <node>     : Get LOW state Lower Bound"); 
  Serial.println("  g O <node>     : Get HIGH state Lower Bound"); 
  Serial.println("  g C <node>     : Get Energy Cost");
  Serial.println("  g E <node>     : Get Accumulated Energy (J)");
  Serial.println("  g V <node>     : Get Mean Visibility Error");
  Serial.println("  g F <node>     : Get Mean Flicker Error");
  Serial.println("  g w <node>     : Get Weight Decay");
  Serial.println("  g l <node>     : Get current Lambda (Panic Balloon)"); 
  Serial.println("  g o <node>     : Get Occupancy state");
  Serial.println("  g v <node>     : Get LDR Voltage");
  Serial.println("  g R <node>     : Get LDR Resistance (Ohms) [CALIBRATION]");
  Serial.println("  g d <node>     : Get External Illuminance");
  Serial.println("  g L <node>     : Get Current Lower Bound");
  Serial.println("  g id <node>    : Get UID and Address of node"); 
  Serial.println("\n--- STREAM COMMANDS ---");                      
  Serial.println("  s y <node>     : Start streaming LUX of node");   
  Serial.println("  s u <node>     : Start streaming PWM of node");   
  Serial.println("  S y <node>     : Stop streaming LUX of node");    
  Serial.println("  S u <node>     : Stop streaming PWM of node");    
  Serial.println("\n--- LOCAL COMMANDS ---");
  Serial.println("  p <val>        : Set Kp (Local)");
  Serial.println("  i <val>        : Set Ki (Local)");
  Serial.println("  d <val>        : Set Kd (Local)");
  Serial.println("  b y 1 / b u 1  : Get last minute buffer");
  Serial.println("  R              : Restart System & Recalibrate");
  Serial.println("==================================================\n");
}

void executeCommand(String input) {
    char cmd = input.charAt(0);

    if (cmd == 'h' || cmd == '?') { printHelpMenu(); return; }

    if (cmd == 'R') { 
      sendCANNetworkMsg('R', 'R', 0, 0); 
      Serial.println("ack"); return;
    }

    if (cmd == 'g') {
      char subcmd = input.charAt(2);
      
      if (subcmd == 'i' && input.charAt(3) == 'd') {
        int target_node = input.substring(5).toInt();
        if (target_node == my_addr) {
          Serial.print("id addr "); Serial.print(my_addr); Serial.print(" -> NODE "); Serial.println(my_addr);
          Serial.print("id uid  "); Serial.print(my_addr); Serial.print(" -> "); Serial.println(my_uid);
        } else {
          sendCANNetworkMsg('g', 'I', target_node, 0.0);
        }
        return;
      }

      int target_node = input.substring(4).toInt(); 
      if (target_node == my_addr) {
          switch (subcmd) {
            case 'u': { mutex_enter_blocking(&data_mutex); float snap_u = current_u; mutex_exit(&data_mutex); Serial.print("u "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_u); break;}
            case 'r': Serial.print("r "); Serial.print(my_addr); Serial.print(" "); Serial.println(target_setpoint); break;
            case 'l': { mutex_enter_blocking(&data_mutex); float snap_lam = current_lambda; mutex_exit(&data_mutex); Serial.print("l "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_lam, 4); break;} 
            case 'y': { mutex_enter_blocking(&data_mutex); float snap_y = current_lux; mutex_exit(&data_mutex); Serial.print("y "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_y); break;}
            case 'a': Serial.print("a "); Serial.print(my_addr); Serial.print(" "); Serial.println(anti_windup_enabled); break;
            case 'f': Serial.print("f "); Serial.print(my_addr); Serial.print(" "); Serial.println(feedback_enabled); break;
            case 'D': Serial.print("D "); Serial.print(my_addr); Serial.print(" "); Serial.println(distributed_ctrl); break;
            case 'P': Serial.print("P "); Serial.print(my_addr); Serial.print(" "); Serial.println(pd_rho, 4); break;
            case 'A': Serial.print("A "); Serial.print(my_addr); Serial.print(" "); Serial.println(pd_alpha, 4); break;
            case 'w': Serial.print("w "); Serial.print(my_addr); Serial.print(" "); Serial.println(pd_decay, 4); break;
            case 'o': Serial.print("o "); Serial.print(my_addr); Serial.print(" "); Serial.println(current_occupancy); break;
            case 'v': Serial.print("v "); Serial.print(my_addr); Serial.print(" "); Serial.println((analogRead(A0) / 4095.0) * 3.3, 3); break;
            case 'R': Serial.print("R "); Serial.print(my_addr); Serial.print(" "); Serial.println(get_R_LDR()); break; 
            case 't': Serial.print("t "); Serial.print(my_addr); Serial.print(" "); Serial.println(millis() / 1000.0); break;
            case 'd': Serial.print("d "); Serial.print(my_addr); Serial.print(" "); Serial.println(background_lux); break;
            case 'p': { mutex_enter_blocking(&data_mutex); float snap_p = P_max * (current_u / 4095.0); mutex_exit(&data_mutex); Serial.print("p "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_p, 4); break;}
            case 'E': { mutex_enter_blocking(&data_mutex); float snap_e = E_energy; mutex_exit(&data_mutex); Serial.print("E "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_e, 4); break;}
            case 'V': { mutex_enter_blocking(&data_mutex); float snap_v = (metrics_count > 0) ? (V_visibility_sum / metrics_count) : 0; mutex_exit(&data_mutex); Serial.print("V "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_v, 4); break;}
            case 'F': { mutex_enter_blocking(&data_mutex); float snap_f = (metrics_count > 0) ? (F_flicker_sum / metrics_count) : 0; mutex_exit(&data_mutex); Serial.print("F "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_f, 4); break;}
            case 'U': Serial.print("U "); Serial.print(my_addr); Serial.print(" "); Serial.println(lower_bound_low); break;
            case 'O': Serial.print("O "); Serial.print(my_addr); Serial.print(" "); Serial.println(lower_bound_high); break;
            case 'C': Serial.print("C "); Serial.print(my_addr); Serial.print(" "); Serial.println(energy_cost); break;
            case 'L': {
                float L_val = 0;
                if (current_occupancy == 'h') L_val = lower_bound_high;
                else if (current_occupancy == 'l') L_val = lower_bound_low;
                Serial.print("L "); Serial.print(my_addr); Serial.print(" "); Serial.println(L_val); 
                break;
            }
            case 'b': {
                char var = input.charAt(6); 
                Serial.print("b "); Serial.print(var); Serial.print(" "); Serial.print(my_addr); Serial.print(" ");
                for (int j = 0; j < minute_buffer_size; j++) {
                    int read_idx = (history_idx + j) % minute_buffer_size;
                    if (var == 'y') Serial.print(history_y[read_idx]);
                    else if (var == 'u') Serial.print(history_u[read_idx]);
                    if (j < minute_buffer_size - 1) Serial.print(", ");
                }
                Serial.println();
                break;
            }
            default: Serial.println("err"); break;
          }
      } else {
          if (subcmd == 'b') Serial.println("err: buffer too large for CAN");
          else sendCANNetworkMsg('g', subcmd, target_node, 0.0);
      }
      return;
    }

    if ((cmd == 's' || cmd == 'S') && input.length() > 2 && (input.charAt(2) == 'y' || input.charAt(2) == 'u')) {
      char var = input.charAt(2);
      int target_node = input.substring(4).toInt();
      
      char can_action = (cmd == 's') ? 'Y' : 'Z';

      if (target_node == my_addr) {
        if (cmd == 's') {
            if (var == 'y') stream_active_y[my_addr] = true;
            else            stream_active_u[my_addr] = true;
        } else {
            if (var == 'y') stream_active_y[my_addr] = false;
            else            stream_active_u[my_addr] = false;
        }
        Serial.println("ack");
      } else {
        sendCANNetworkMsg(can_action, var, target_node, 0.0); 
        Serial.println("ack");
      }
      return;
    }

    int fS = input.indexOf(' '); int sS = input.indexOf(' ', fS + 1);
    
    if (sS == -1 && fS != -1) {
      float val = input.substring(fS + 1).toFloat();
      switch(cmd) {
        case 'p': mutex_enter_blocking(&data_mutex); myPID.setKp(val); mutex_exit(&data_mutex); Serial.println("ack"); break;
        case 'i': mutex_enter_blocking(&data_mutex); myPID.setKi(val); mutex_exit(&data_mutex); Serial.println("ack"); break;
        case 'd': mutex_enter_blocking(&data_mutex); myPID.setKd(val); mutex_exit(&data_mutex); Serial.println("ack"); break;
      }
      return;
    } 

    if (sS != -1) {
      int target_node = input.substring(fS + 1, sS).toInt();
      float val = input.substring(sS + 1).toFloat();

      if (target_node == my_addr || target_node == 0) {
          mutex_enter_blocking(&data_mutex);
          switch (cmd) {
            case 'r': target_setpoint = val; break; 
            case 'u': manual_pwm = val; feedback_enabled = false; distributed_ctrl = false; break;
            case 'f': feedback_enabled = (val > 0); if(feedback_enabled) distributed_ctrl = false; break;
            case 'D': 
                distributed_ctrl = (val > 0); 
                if (distributed_ctrl) feedback_enabled = false; 
                current_lambda = 0.0; 
                break;
            case 'P': pd_rho = val; break;
            case 'A': pd_alpha = val; break;
            case 'w': pd_decay = val; break; 
            case 'a': anti_windup_enabled = (val > 0); break;
            case 'o': 
              current_occupancy = input.charAt(sS + 1); 
              target_setpoint = (current_occupancy == 'h') ? lower_bound_high : (current_occupancy == 'l' ? lower_bound_low : 0.0); 
              break;
            case 'U': lower_bound_low = val; break;
            case 'O': lower_bound_high = val; break;
            case 'C': energy_cost = val; break;
            default: break;
          }
          mutex_exit(&data_mutex);
          if (target_node == my_addr) Serial.println("ack");
      } 
      
      if (target_node != my_addr) {
          if (cmd == 'o') val = (float)input.charAt(sS + 1); 
          sendCANNetworkMsg('W', cmd, target_node, val);
          if (target_node != 0) Serial.println("ack");
      }
    }
}

String serialBuffer = "";
unsigned long last_serial_rx_time = 0;

void processUI() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        last_serial_rx_time = millis(); 
        
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                String input = serialBuffer;
                serialBuffer = ""; 
                input.trim();
                if (input.length() > 0) {
                    executeCommand(input); 
                }
            }
        } else {
            serialBuffer += c;
        }
    }

    if (serialBuffer.length() > 0 && (millis() - last_serial_rx_time > 500)) {
        String input = serialBuffer;
        serialBuffer = ""; 
        input.trim();
        if (input.length() > 0) {
            executeCommand(input); 
        }
    }
}

// ======================================================================
// --- CORE 0: SETUP & MAIN LOOP (UI, CAN, STREAMING) ---
// ======================================================================
void setup() {
  mutex_init(&data_mutex); 
  Serial.begin(115200);
  SPI.begin();
  mcp2515 = new MCP2515(CAN_CS_PIN);
  mcp2515->reset();
  mcp2515->setBitrate(CAN_500KBPS, MCP_8MHZ); 
  mcp2515->setNormalMode();

  analogReadResolution(12);
  analogWriteFreq(60000);      
  analogWriteRange(DAC_RANGE); 
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0); 

  boot_state = BOOT_IDLE; 
}

void loop() {
  readCANMessages(); 
  processUI();       

  if (boot_state != BOOT_DONE) {
      boot_task();
  } 
  else if (!system_ready) {
      calib_task(); 
  } 
  else {
      static unsigned long last_bc = 0;
      static int hist_cnt = 0;
      
      if (millis() - last_bc >= 100) { 
        last_bc += 100;
        
        mutex_enter_blocking(&data_mutex);
        float lux_snap = current_lux;
        float u_snap = current_u;
        float lambda_snap = current_lambda; 
        bool dist_snap = distributed_ctrl;
        mutex_exit(&data_mutex);

        if (dist_snap) {
            sendCANNetworkMsg('l', '0', 0, lambda_snap); 
        }

        history_y[history_idx] = lux_snap;
        history_u[history_idx] = u_snap;
        history_idx = (history_idx + 1) % minute_buffer_size;
        hist_cnt++;

        if (stream_active_y[my_addr]) {
            sendCANNetworkMsg('s', 'y', 0, lux_snap); 
        }
        if (stream_active_u[my_addr]) {
            sendCANNetworkMsg('s', 'u', 0, u_snap);
        }

        if (hist_cnt >= 10) {
          hist_cnt = 0;
          sendCANNetworkMsg('b', 'y', 0, lux_snap);
          sendCANNetworkMsg('b', 'u', 0, u_snap); 
        }
      }
  }
}

// ======================================================================
// --- CORE 1: CONTROLO PID & PRIMAL-DUAL MATEMÁTICA (100 Hz PURA) ---
// ======================================================================
void setup1() {
    delay(500); 
    while (boot_state == BOOT_IDLE) delay(10);
}

void loop1() {
    if (!system_ready) return;

    static unsigned long last_t = 0;
    unsigned long now = millis();
    if (now - last_t < 10) return;  
    last_t += 10;

    // --- RAMPA DE REFERENCIA (SOFT START) ---
    float ramp_speed = 0.04; 
    if (current_setpoint < target_setpoint) {
        current_setpoint += ramp_speed;
        if (current_setpoint > target_setpoint) current_setpoint = target_setpoint;
    } else if (current_setpoint > target_setpoint) {
        current_setpoint -= ramp_speed;
        if (current_setpoint < target_setpoint) current_setpoint = target_setpoint;
    }

    float lux = getFilteredLux(getLux());

    mutex_enter_blocking(&data_mutex);
    bool snap_fb = feedback_enabled;
    bool snap_dist = distributed_ctrl; 
    bool snap_aw = anti_windup_enabled;
    float snap_sp = current_setpoint; 
    float snap_man = manual_pwm;
    float snap_cost = energy_cost;
    float snap_rho = pd_rho;
    float snap_alpha = pd_alpha;
    float snap_decay = pd_decay;
    float snap_net_u[MAX_NODES];
    float snap_net_lam[MAX_NODES];
    
    for (int i = 0; i < num_nodes; i++) {
        if (i != (my_addr - 1) && (now - last_rx_time[i] > WATCHDOG_TIMEOUT_MS)) {
            snap_net_u[i] = 0.0;
            snap_net_lam[i] = 0.0;
        } else {
            snap_net_u[i] = network_u[i];
            snap_net_lam[i] = network_lambda[i];
        }
    }
    mutex_exit(&data_mutex);

    float u_out = 0;

    if (snap_dist) {
        // 1. AVALIAÇÃO DE REDE (GRADIENTE) PRIMEIRO
        // Precisamos calcular o gradiente antes do travão, para saber o que a rede quer!
        float grad_d = snap_cost; 
        for (int j = 0; j < num_nodes; j++) {
            float G_ji = gain_matrix[my_addr - 1][j] * 4095.0; 
            float lam_j = (j == (my_addr - 1)) ? current_lambda : snap_net_lam[j];
            grad_d -= lam_j * G_ji; 
        }

        // --- TRAVÃO INTELIGENTE (Network-Aware) ---
        float active_decay = snap_decay;
        float active_rho = snap_rho;

        if (lux > snap_sp + 2.0) { 
            // Fura sempre o balão para matar o overshoot local
            active_decay = 0.2;            
            
            // SÓ trava o PWM violentamente SE a rede não precisar de luz (grad_d > 0)
            if (grad_d > 0.0) {
                active_rho = snap_rho * 10.0;  
            }
        }
        
        // 2. UPDATE DUAL
        current_lambda = current_lambda * (1.0 - active_decay) + snap_alpha * (snap_sp - lux);
        if (current_lambda < 0) current_lambda = 0; 
        if (current_lambda > 500.0) current_lambda = 500.0; 

        // 3. UPDATE PRIMAL
        float d_i = current_u / 4095.0; 
        d_i -= active_rho * grad_d;
        
        if (d_i < 0) d_i = 0;
        if (d_i > 1) d_i = 1;

        u_out = d_i * 4095.0; 
    }
    else if (snap_fb) {
        float instant_external_illuminance = 0.0;
        static float filtered_external_illuminance = 0.0; 
        
        for (int j = 0; j < num_nodes; j++) {
            if (j != (my_addr - 1)) {
                instant_external_illuminance += gain_matrix[j][my_addr - 1] * snap_net_u[j];
            }
        }
        
        filtered_external_illuminance = (0.95 * filtered_external_illuminance) + (0.05 * instant_external_illuminance);
        
        float ff_pwm = 0.0;
        if (system_gain > 0.0001) { 
            ff_pwm = -(filtered_external_illuminance / system_gain);
        }

        u_out = myPID.compute(snap_sp, lux, snap_aw, ff_pwm);
    } 
    else {
        u_out = snap_man;
    }

    analogWrite(LED_PIN, (int)u_out);
    
    updateMetrics(u_out, lux, snap_sp);

    mutex_enter_blocking(&data_mutex);
    current_lux = lux;
    current_u   = u_out;
    mutex_exit(&data_mutex);
}
