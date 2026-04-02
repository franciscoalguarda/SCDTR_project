#include <SPI.h>
#include <mcp2515.h>
#include "pico/unique_id.h" // Biblioteca segura para ler o ID do Pico em multicore
#include "pico/mutex.h"     // Biblioteca de concorrência (Mutex)

// ======================================================================
// --- CAN-BUS / NETWORK CONFIGURATION ---
// ======================================================================
const int CAN_CS_PIN = 17; 
MCP2515* mcp2515; 

// --- AUTO-ADDRESSING & STATE MACHINE (WAKE-UP) ---
enum BootState { BOOT_IDLE, BOOT_ANNOUNCE, BOOT_DONE };
enum CalibState { CALIB_IDLE, CALIB_START, CALIB_BKG_WAIT, CALIB_LED_ON, CALIB_LED_WAIT, CALIB_REQ_LUX, CALIB_WAIT_REPLY_LUX, CALIB_WAIT_REPLY_BKG, CALIB_DONE };

volatile BootState boot_state = BOOT_IDLE;
volatile CalibState calib_state = CALIB_IDLE;
volatile bool system_ready = false; 

uint32_t my_uid = 0;
int my_addr = 0;     
bool is_hub = false; 

const int MAX_NODES = 3;
int num_nodes = 0;
uint32_t network_uids[MAX_NODES];

float gain_matrix[MAX_NODES][MAX_NODES]; 
float lux_off[MAX_NODES];                
float network_lux[MAX_NODES];            
float network_u[MAX_NODES] = {0.0, 0.0, 0.0}; 

// Timers
unsigned long last_hello_ms = 0;
const unsigned long HELLO_PERIOD_MS = 500;
const unsigned long BOOT_TIMEOUT_MS = 3000;
unsigned long boot_start_ms = 0;

int calib_led_node = 1;
int calib_req_node = 1;
unsigned long calib_timer = 0;
const unsigned long CALIB_LED_SETTLE_MS = 2000;

// O Mutex para proteger o acesso a memória partilhada entre o Core 0 e o Core 1
mutex_t data_mutex;

// ======================================================================
// --- STREAMING STATE ---
// ======================================================================
bool stream_active_y[MAX_NODES + 1] = {false}; 
bool stream_active_u[MAX_NODES + 1] = {false};

// ======================================================================
// --- LOCAL LUMINAIRE CONFIGURATION (PHASE 1) ---
// ======================================================================
const int LED_PIN = 15;
const int DAC_RANGE = 4096;
float m = 0.0; 
float b = 0.0;  
float background_lux = 0;
volatile float system_gain = 0.0;
const float h = 0.01; // 100Hz

// State Variables (Interface)
volatile bool feedback_enabled = false; 
volatile bool anti_windup_enabled = true; 
volatile float manual_pwm = 0;             
float current_lux = 0;           
float current_u = 0;       
char current_occupancy = 'h';    

// PHASE 2 Variables (Table 3)
float lower_bound_low = 10.0;  
float lower_bound_high = 50.0; 
volatile float setpoint = lower_bound_high;
volatile float energy_cost = 1.0;       

// Buffers and Metrics
const int buffer_size = 10;
float lux_buffer[buffer_size];
int buffer_idx = 0;

const int minute_buffer_size = 600; 
float history_y[minute_buffer_size];
float history_u[minute_buffer_size];
int history_idx = 0;
int history_counter = 0; 

float E_energy = 0;          
float V_visibility_sum = 0;  
float F_flicker_sum = 0;     
long metrics_count = 0;      
float last_u_1 = 0, last_u_2 = 0;          
float P_max = 0.05;          

// Prototypes
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
float getFilteredLux(float new_sample) {
  lux_buffer[buffer_idx] = new_sample;
  buffer_idx = (buffer_idx + 1) % buffer_size; 
  float sum = 0;
  for(int j = 0; j < buffer_size; j++) sum += lux_buffer[j];
  return sum / (float)buffer_size;
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
    Serial.print("Role: "); Serial.println(is_hub ? "HUB (Master)" : "SLAVE");
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
  canMsg.can_id  = 99; 
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

    if (action == 'R') {
        mutex_enter_blocking(&data_mutex);
        myPID.reset(); E_energy = 0; V_visibility_sum = 0; F_flicker_sum = 0; metrics_count = 0;
        mutex_exit(&data_mutex);
        background_lux = 0;
        system_ready = false;
        for(int k = 0; k <= MAX_NODES; k++) { stream_active_y[k] = false; stream_active_u[k] = false; }
        if (is_hub) calib_state = CALIB_START; 
        return;
    }

    if (action == 'w' && variable == 'h') {
        uint32_t received_uid;
        unsigned char* up = (unsigned char*)&received_uid;
        for(int i = 0; i < 4; i++) up[i] = canMsg.data[3 + i];
        register_node(received_uid);
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
                        if (calib_led_node > num_nodes) calib_state = CALIB_DONE;
                        else calib_state = CALIB_LED_ON;
                    } else calib_state = CALIB_REQ_LUX;
                }
            }
        } 
        else if (variable == 'd') { 
            if (!is_hub) {
                for(int k=0; k<buffer_size; k++) lux_buffer[k] = background_lux;
                system_ready = true;
                mutex_enter_blocking(&data_mutex);
                feedback_enabled = true; 
                mutex_exit(&data_mutex);
            }
        }
        return;
    }

    if (sender_id >= 1 && sender_id <= MAX_NODES) {
        if (variable == 'y') network_lux[sender_id - 1] = value;
        if (variable == 'u') {
            mutex_enter_blocking(&data_mutex);
            network_u[sender_id - 1] = value;
            mutex_exit(&data_mutex);
        }
    }

    if (action == 'g') {
        if (variable == 'I') {
            sendCANNetworkMsg('d', 'A', sender_id, (float)my_addr); 
            sendCANUidMsg(sender_id, my_uid);                        
            return;
        }

        float reply_val = 0;
        switch(variable) {
            case 'y': mutex_enter_blocking(&data_mutex); reply_val = current_lux; mutex_exit(&data_mutex); break;
            case 'u': mutex_enter_blocking(&data_mutex); reply_val = current_u; mutex_exit(&data_mutex); break;
            case 'r': reply_val = setpoint; break;
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
            case 'L': 
                if (current_occupancy == 'h') reply_val = lower_bound_high;
                else if (current_occupancy == 'l') reply_val = lower_bound_low;
                else reply_val = 0.0;
                break;
        }
        sendCANNetworkMsg('d', variable, sender_id, reply_val);
    }
    else if (action == 's') {
        mutex_enter_blocking(&data_mutex);
        if (variable == 'y' && dest_node == my_addr) { stream_active_y[my_addr] = true; }
        else if (variable == 'u' && dest_node == my_addr) { stream_active_u[my_addr] = true; }
        else if (variable == 'r') setpoint = value;
        else if (variable == 'u') { manual_pwm = (value / 100.0) * 4095.0; feedback_enabled = false; }
        else if (variable == 'f') feedback_enabled = (value > 0);
        else if (variable == 'a') anti_windup_enabled = (value > 0);
        else if (variable == 'U') lower_bound_low = value;
        else if (variable == 'O') lower_bound_high = value;
        else if (variable == 'C') energy_cost = value;
        else if (variable == 'o') { 
            current_occupancy = (char)value; 
            if (current_occupancy == 'h') setpoint = lower_bound_high;
            else if (current_occupancy == 'l') setpoint = lower_bound_low;
            else setpoint = 0.0;
        }
        mutex_exit(&data_mutex);
    }
    else if (action == 'S') {
        if (variable == 'y' && dest_node == my_addr) { stream_active_y[my_addr] = false; return; }
        if (variable == 'u' && dest_node == my_addr) { stream_active_u[my_addr] = false; return; }
    }
    else if (action == 'd') { 
        if (variable == 'A') {
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

            register_node(my_uid);
            boot_state = BOOT_ANNOUNCE;
            send_hello();
            last_hello_ms = now;
            Serial.println("Waiting for all nodes to join...");
            break;
            
        case BOOT_ANNOUNCE:
            if (now - last_hello_ms >= 500) {
                send_hello(); 
                last_hello_ms = now;
            }
            if (num_nodes == MAX_NODES) {
                send_hello(); 
                delay(500);   
                assign_addresses(); 
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
            calib_state = CALIB_IDLE; 
            sendCANNetworkMsg('c', 'd', 0, 0); 
            
            system_gain = gain_matrix[my_addr - 1][my_addr - 1];
            background_lux = lux_off[my_addr - 1];
            for(int k=0; k<buffer_size; k++) lux_buffer[k] = background_lux;
            
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
            feedback_enabled = true; 
            mutex_exit(&data_mutex);
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
  Serial.println("  a <node> <0/1> : Turn Anti-Windup Off (0) or On (1)");
  Serial.println("  o <node> <val> : Set occupancy ('o'=off, 'l'=low, 'h'=high)");
  Serial.println("  U <node> <val> : Set LOWER BOUND for LOW state (U)");
  Serial.println("  O <node> <val> : Set LOWER BOUND for HIGH state (O)");
  Serial.println("  C <node> <val> : Set Energy Cost (C)");
  Serial.println("\n--- GET COMMANDS ---");
  Serial.println("  g r <node>     : Get current Setpoint");
  Serial.println("  g y <node>     : Get measured LUX");
  Serial.println("  g u <node>     : Get Duty Cycle / PWM");
  Serial.println("  g o <node>     : Get Occupancy state");
  Serial.println("  g v <node>     : Get LDR Voltage");
  Serial.println("  g R <node>     : Get LDR Resistance (Ohms) [CALIBRATION]");
  Serial.println("  g d <node>     : Get External Illuminance");
  Serial.println("  g E <node>     : Get Accumulated Energy (J)");
  Serial.println("  g V <node>     : Get Mean Visibility Error");
  Serial.println("  g F <node>     : Get Mean Flicker Error");
  Serial.println("  g L <node>     : Get Current Lower Bound");
  Serial.println("  g C <node>     : Get Energy Cost");
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
            case 'r': Serial.print("r "); Serial.print(my_addr); Serial.print(" "); Serial.println(setpoint); break;
            case 'y': { mutex_enter_blocking(&data_mutex); float snap_y = current_lux; mutex_exit(&data_mutex); Serial.print("y "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_y); break;}
            case 'a': Serial.print("a "); Serial.print(my_addr); Serial.print(" "); Serial.println(anti_windup_enabled); break;
            case 'f': Serial.print("f "); Serial.print(my_addr); Serial.print(" "); Serial.println(feedback_enabled); break;
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

    if (cmd == 's' && (input.charAt(2) == 'y' || input.charAt(2) == 'u')) {
      char var = input.charAt(2);
      int target_node = input.substring(4).toInt();
      if (target_node == my_addr) {
        if (var == 'y') stream_active_y[my_addr] = true;
        else            stream_active_u[my_addr] = true;
        Serial.println("ack");
      } else {
        sendCANNetworkMsg('s', var, target_node, 0.0); 
        if (var == 'y') stream_active_y[target_node] = true;
        else            stream_active_u[target_node] = true;
        Serial.println("ack");
      }
      return;
    }

    if (cmd == 'S') {
      char var = input.charAt(2);
      int target_node = input.substring(4).toInt();
      if (target_node == my_addr) {
        if (var == 'y') stream_active_y[my_addr] = false;
        else            stream_active_u[my_addr] = false;
        Serial.println("ack");
      } else {
        sendCANNetworkMsg('S', var, target_node, 0.0);
        if (var == 'y') stream_active_y[target_node] = false;
        else            stream_active_u[target_node] = false;
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

      if (target_node == my_addr) {
          mutex_enter_blocking(&data_mutex);
          switch (cmd) {
            case 'r': setpoint = val; Serial.println("ack"); break;
            case 'u': manual_pwm = (val / 100.0) * 4095.0; feedback_enabled = false; Serial.println("ack"); break;
            case 'f': feedback_enabled = (val > 0); if(!feedback_enabled) manual_pwm = current_u; Serial.println("ack"); break;
            case 'a': anti_windup_enabled = (val > 0); Serial.println("ack"); break;
            case 'o': 
              current_occupancy = input.charAt(sS + 1); 
              setpoint = (current_occupancy == 'h') ? lower_bound_high : (current_occupancy == 'l' ? lower_bound_low : 0.0);
              Serial.println("ack"); break;
            case 'U': lower_bound_low = val; Serial.println("ack"); break;
            case 'O': lower_bound_high = val; Serial.println("ack"); break;
            case 'C': energy_cost = val; Serial.println("ack"); break;
            default: Serial.println("err"); break;
          }
          mutex_exit(&data_mutex);
      } else {
          if (cmd == 'o') val = (float)input.charAt(sS + 1); 
          sendCANNetworkMsg('s', cmd, target_node, val);
          Serial.println("ack");
      }
    }
}

String serialBuffer = "";

void processUI() {
    while (Serial.available() > 0) {
        char c = Serial.read();
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
        mutex_exit(&data_mutex);

        history_y[history_idx] = lux_snap;
        history_u[history_idx] = u_snap;
        history_idx = (history_idx + 1) % minute_buffer_size;
        hist_cnt++;

        if (stream_active_y[my_addr]) {
          Serial.print("s y "); Serial.print(my_addr); 
          Serial.print(" "); Serial.print(lux_snap, 4);
          Serial.print(" "); Serial.println(millis());
        }
        if (stream_active_u[my_addr]) {
          Serial.print("s u "); Serial.print(my_addr); 
          Serial.print(" "); Serial.print(u_snap, 4);
          Serial.print(" "); Serial.println(millis());
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
// --- CORE 1: CONTROLO PID & MATEMÁTICA (100 Hz PURA) ---
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

    float lux = getFilteredLux(getLux());

    mutex_enter_blocking(&data_mutex);
    bool snap_fb = feedback_enabled;
    bool snap_aw = anti_windup_enabled;
    float snap_sp = setpoint;
    float snap_man = manual_pwm;
    float snap_net_u[MAX_NODES];
    for (int i = 0; i < num_nodes; i++) {
        snap_net_u[i] = network_u[i];
    }
    mutex_exit(&data_mutex);

    float u_out = 0;

    if (snap_fb) {
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

    } else {
        u_out = snap_man;
    }

    analogWrite(LED_PIN, (int)u_out);
    
    updateMetrics(u_out, lux, snap_sp);

    mutex_enter_blocking(&data_mutex);
    current_lux = lux;
    current_u   = u_out;
    mutex_exit(&data_mutex);
}
