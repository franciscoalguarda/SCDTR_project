#include <SPI.h>
#include "mcp2515.h"
#include "hardware/flash.h"
#include "hardware/spi.h"
#include "pico/mutex.h"

// ======================================================================
// --- CAN-BUS / NETWORK CONFIGURATION ---
// ======================================================================
const int CAN_CS_PIN = 17; 
MCP2515* mcp2515; 

// --- AUTO-ADDRESSING & STATE MACHINE (WAKE-UP) ---
enum BootState { BOOT_IDLE, BOOT_ANNOUNCE, BOOT_DONE };
enum CalibState { CALIB_IDLE, CALIB_START, CALIB_BKG_WAIT, CALIB_LED_ON, CALIB_LED_WAIT, CALIB_REQ_LUX, CALIB_WAIT_REPLY_LUX, CALIB_WAIT_REPLY_BKG, CALIB_DONE, CALIB_POST_DELAY };

volatile BootState boot_state = BOOT_IDLE;
volatile CalibState calib_state = CALIB_IDLE;
volatile bool system_ready = false; // Only becomes true when calibration finishes

uint32_t my_uid = 0;
int my_addr = 0;     // Replaces the old MY_NODE_ID (Assigned automatically!)
bool is_hub = false; // The board with my_addr == 1 automatically becomes the hub

const int MAX_NODES = 3;
int num_nodes = 0;
uint32_t network_uids[MAX_NODES];

float gain_matrix[MAX_NODES][MAX_NODES]; // Our K matrix (Coupling Gains)
float lux_off[MAX_NODES];                // Background Lux
float network_lux[MAX_NODES];            // Real-time Lux state of neighbors
float network_u[MAX_NODES] = {0.0, 0.0, 0.0};

// Timers for the State Machine
unsigned long last_hello_ms = 0;
const unsigned long HELLO_PERIOD_MS = 500;
const unsigned long BOOT_TIMEOUT_MS = 3000;
unsigned long boot_start_ms = 0;
unsigned long boot_done_time = 0;

int calib_led_node = 1;
int calib_req_node = 1;
unsigned long calib_timer = 0;
const unsigned long CALIB_LED_SETTLE_MS = 2000;

mutex_t data_mutex;
mutex_t metrics_mutex;

// ======================================================================
// --- LOCAL LUMINAIRE CONFIGURATION (FASE 1) ---
// ======================================================================
const int LED_PIN = 15;
const int DAC_RANGE = 4096;
float m = 0.0; 
float b = 0.0;  
float background_lux = 0;
float system_gain = 0.0;
const float h = 0.01; // 100Hz

// State Variables (Interface)
bool feedback_enabled = false;  // Starts disabled. Only turns on after calibration
bool anti_windup_enabled = true; 
float manual_pwm = 0;             
float current_lux = 0;           
float current_u = 0;       
char current_occupancy = 'o';     

// PHASE 2 Variables (Table 3)
float lower_bound_low = 10.0;  // Command 'U'
float lower_bound_high = 50.0; // Command 'O'
float setpoint = lower_bound_high;
float energy_cost = 1.0;       // Command 'C'

// Phase 2 Optimization Variables
float lambda_i = 0.0;
float network_lambda[MAX_NODES] = {0.0, 0.0, 0.0};
const float alpha = 0.01; 
bool stream_y = false;
bool stream_u = false;

// Buffers and Metrics
const int buffer_size = 10;
float lux_buffer[buffer_size];
int buffer_idx = 0;

const int minute_buffer_size = 600; // 60s * 10Hz
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
float getLux();
void updateMetrics(float u, float y, float ref);

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
    float compute(float ref, float y, bool antiwindup_control, float ff) {
      float error_p = b_weight * ref - y; 
      float error = ref - y; 
      float p_term = Kp * error_p;
      i_term += Ki * error * h; 
      float d_term = Kd * (error - last_error) / h;
      last_error = error;
      float u = p_term + i_term + d_term + ff; // Apply feedforward before saturation
      
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

float getLux() {
  float adc_sum = 0;
  for (int j = 0; j < 10; j++) adc_sum += analogRead(A0);
  float read_adc = adc_sum / 10.0;
  if (read_adc == 0) read_adc = 1;
  float R_LDR = 40950000.0 / read_adc - 10000.0;
  return pow(10, (log10(R_LDR) - b) / m);
}

void updateMetrics(float u, float y, float ref) {
    mutex_enter_blocking(&metrics_mutex);
    float d_k = u / 4095.0;
    E_energy += P_max * d_k * h;
    if (y < ref) V_visibility_sum += (ref - y);
    float diff1 = d_k - last_u_1;
    float diff2 = last_u_1 - last_u_2;
    if (diff1 * diff2 < 0) F_flicker_sum += (abs(diff1) + abs(diff2));
    last_u_2 = last_u_1;
    last_u_1 = d_k;
    metrics_count++;
    mutex_exit(&metrics_mutex);
}

// ======================================================================
// --- AUTO-ADDRESSING LOGIC ---
// ======================================================================
uint32_t get_unique_id() {
    uint8_t id[8];
    flash_get_unique_id(id);
    return ((uint32_t)id[4] << 24) | ((uint32_t)id[5] << 16) | ((uint32_t)id[6] << 8) | ((uint32_t)id[7]);
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
    // Sort UIDs from lowest to highest
    for (int i = 0; i < num_nodes - 1; i++) {
        for (int j = 0; j < num_nodes - i - 1; j++) {
            if (network_uids[j] > network_uids[j + 1]) {
                uint32_t temp = network_uids[j];
                network_uids[j] = network_uids[j + 1];
                network_uids[j + 1] = temp;
            }
        }
    }
    // My address is my position in the list + 1
    for (int i = 0; i < num_nodes; i++) {
        if (network_uids[i] == my_uid) my_addr = i + 1;
    }
    is_hub = (my_addr == 1); // Node 1 becomes the Hub
    
    Serial.print("\n=== BOOT DONE ===\nMy UID: "); Serial.println(my_uid);
    Serial.print("Assigned Address: NODE "); Serial.println(my_addr);
    Serial.print("Role: "); Serial.println(is_hub ? "HUB (Master)" : "SLAVE");
}

// ======================================================================
// --- CAN COMMUNICATION & PROTOCOL ---
// ======================================================================
void sendCANNetworkMsg(char action, char variable, int dest_node, float value) {
  struct can_frame canMsg;
  canMsg.can_id  = my_addr > 0 ? my_addr : 99; // 99 if no address yet
  canMsg.can_dlc = 7; 
  canMsg.data[0] = action;    
  canMsg.data[1] = variable;  
  canMsg.data[2] = dest_node; 
  
  unsigned char* p = (unsigned char*)&value;
  for(int i = 0; i < 4; i++) canMsg.data[3 + i] = p[i];
  
  // Sends the message physically to the network
  mcp2515->sendMessage(&canMsg);

  // --- SOFTWARE LOOPBACK (THE MAGIC!) ---
  // Since the CAN transceiver doesn't hear what it itself transmits,
  // if the message is destined for ourselves (or broadcast),
  // we inject it directly into our processor!
  if (dest_node == my_addr || dest_node == 0) {
      process_calib_and_network(canMsg);
  }
}

void send_hello() {
  struct can_frame canMsg;
  canMsg.can_id  = 99; 
  canMsg.can_dlc = 7;
  canMsg.data[0] = 'w'; // Wake
  canMsg.data[1] = 'h'; // Hello
  canMsg.data[2] = 0;   // Broadcast
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

    // --- 0. RESTART GLOBAL ('R') ---
    if (action == 'R') {
        mutex_enter_blocking(&data_mutex);
        myPID.reset();
        mutex_exit(&data_mutex);
        
        mutex_enter_blocking(&metrics_mutex);
        E_energy = 0; 
        V_visibility_sum = 0; 
        F_flicker_sum = 0; 
        metrics_count = 0;
        mutex_exit(&metrics_mutex);
        
        background_lux = 0;
        system_ready = false;
        if (is_hub) calib_state = CALIB_START;
        return;
    }

        // --- 1. BOOT MESSAGES ('w') ---
    if (action == 'w' && variable == 'h') {
        uint32_t received_uid;
        unsigned char* up = (unsigned char*)&received_uid;
        for(int i = 0; i < 4; i++) up[i] = canMsg.data[3 + i];
        
        // Se as placas já terminaram o BOOT há mais de 2s e virem alguem novo a iniciar, recomeçam.
        // Ignora ecos e resets imediatamente a seguir a BOOT_DONE. Protege ambos casos e deadlocks!
        if (boot_state == BOOT_DONE && (millis() - boot_done_time > 2000) && received_uid != my_uid) {
            system_ready = false;
            boot_state = BOOT_IDLE;
            calib_state = CALIB_IDLE;
            num_nodes = 0;
            is_hub = false;
            mutex_enter_blocking(&data_mutex);
            myPID.reset();
            mutex_exit(&data_mutex);
            return;
        }
        
        register_node(received_uid);
        return;
    }

    // --- HUB MATRIX DISTRIBUTION TO SLAVES ---
    if (action == 'M') { // M for Matrix
        int row = variable - '0'; // Convert char '0','1','2' to integer index
        int col = dest_node - 1; 
        gain_matrix[row][col] = value;
        if (row == col && my_addr == dest_node) system_gain = value;
        return;
    }
    if (action == 'B') { // B for Background
        int idx = dest_node - 1;
        lux_off[idx] = value;
        if (my_addr == dest_node) background_lux = value;
        return;
    }
    if (action == 'l' && variable == 'l') {
        network_lambda[sender_id - 1] = value;
        return;
    }

    if (action == 'Y' || action == 'Z') {
        if (dest_node == my_addr) {
            if (variable == 'y') stream_y = (action == 'Y');
            else if (variable == 'u') stream_u = (action == 'Y');
        }
        return;
    }
    if (action == 'q' && is_hub) {
        Serial.print("s "); Serial.print(variable); Serial.print(" "); Serial.print(sender_id); Serial.print(" "); Serial.print(value, 4); Serial.print(" "); Serial.println(millis());
        return;
    }

    if (dest_node != my_addr && dest_node != 0) return; // Ignore if not for me or broadcast

    // --- 2. CALIBRATION MESSAGES ('c') ---
    if (action == 'c') {
        if (variable == 's') { if (!is_hub) analogWrite(LED_PIN, 0); } 
        else if (variable == 'l') { if (!is_hub) analogWrite(LED_PIN, 4095); } 
        else if (variable == 'o') { if (!is_hub) analogWrite(LED_PIN, 0); } 
        else if (variable == 'g') { 
            float meas = getLux();
            if (calib_led_node == 1 && !is_hub && background_lux == 0) background_lux = meas;
            sendCANNetworkMsg('c', 'x', sender_id, meas); 
        } 
        else if (variable == 'x' && is_hub) { // Lux response arrives at the Hub
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
        else if (variable == 'd') { // Calibration End received by Slaves
            if (!is_hub) {
                for(int k=0; k<buffer_size; k++) lux_buffer[k] = background_lux;
                system_ready = true;
                mutex_enter_blocking(&data_mutex);
                feedback_enabled = true; // Turn on local PID
                mutex_exit(&data_mutex);
            }
        }
        return;
    }

    // --- 3. REGULAR NETWORK COMMANDS (HUB ROUTING) ---
    if (sender_id >= 1 && sender_id <= MAX_NODES) {
        if (variable == 'y') network_lux[sender_id - 1] = value;
        if (variable == 'u') {
            mutex_enter_blocking(&data_mutex);
            network_u[sender_id - 1] = value;
            mutex_exit(&data_mutex);
        }
    }

    if (action == 'g') {
        float reply_val = 0;
        switch(variable) {
            case 'y': reply_val = current_lux; break;
            case 'u': reply_val = current_u; break;
            case 'r': reply_val = setpoint; break;
            case 'v': reply_val = (analogRead(A0) / 4095.0) * 3.3; break;
            case 't': reply_val = millis() / 1000.0; break;
            case 'd': reply_val = background_lux; break;
            case 'p': reply_val = P_max * (current_u / 4095.0); break;
            case 'E': reply_val = E_energy; break;
            case 'V': reply_val = (metrics_count > 0) ? (V_visibility_sum / metrics_count) : 0; break;
            case 'F': reply_val = (metrics_count > 0) ? (F_flicker_sum / metrics_count) : 0; break;
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
        if (variable == 'r') setpoint = value;
        else if (variable == 'u') { manual_pwm = value; feedback_enabled = false; }
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
    // ANY board that receives a command response prints it to the screen!
    else if (action == 'd') { 
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
            boot_start_ms = millis();
            
            Serial.print("\n[SYSTEM] Power ON! My Hardware UID is: ");
            Serial.println(my_uid);
            
            if (my_uid == 97) {
                m = -1.0;
                b = 6.75;
            } else if (my_uid == 107) {
                m = -0.87;
                b = 6.5193;
            } else if (my_uid == 122) {
                m = -0.75;
                b = 7.4;
            } else {
                m = -1.0;
                b = 6.75;
            }

            register_node(my_uid);
            boot_state = BOOT_ANNOUNCE;
            send_hello();
            last_hello_ms = now;
            Serial.println("Waiting for all 3 nodes to join...");
            break;
            
        case BOOT_ANNOUNCE:
            if (now - last_hello_ms >= 500) {
                send_hello(); 
                last_hello_ms = now;
            }
            
            // THE FIX: Advance after BOOT_TIMEOUT_MS, supporting a variable number of nodes!
            if (now - boot_start_ms >= BOOT_TIMEOUT_MS && num_nodes > 0) {
                send_hello(); // Give a last shout for the last board that joined to hear
                assign_addresses(); // Distribute addresses (1 to num_nodes)
                boot_state = BOOT_DONE;
                boot_done_time = millis(); // Guarda a hora a que terminou para bloquear resets ecoados
                if (is_hub) calib_state = CALIB_START; // Node 1 starts calibration
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
            calib_state = CALIB_BKG_WAIT; // <-- UPDATE STATE FIRST
            sendCANNetworkMsg('c', 's', 0, 0); // Broadcast START (Turn off LEDs)
            analogWrite(LED_PIN, 0);
            break;
            
        case CALIB_BKG_WAIT:
            if (now - calib_timer >= CALIB_LED_SETTLE_MS) {
                calib_state = CALIB_WAIT_REPLY_BKG; // <-- UPDATE STATE FIRST
                sendCANNetworkMsg('c', 'g', calib_req_node, 0); // Request Background Lux
                calib_timer = now;
            }
            break;
            
        case CALIB_WAIT_REPLY_BKG:
            if (now - calib_timer >= 1000) { // SAFETY TIMEOUT: Se a board 2/3 não acusa CAN, pergunta de novo!
                sendCANNetworkMsg('c', 'g', calib_req_node, 0); 
                calib_timer = now;
            }
            break;

        case CALIB_LED_ON:
            calib_timer = now;
            calib_req_node = 1;
            calib_state = CALIB_LED_WAIT; // <-- UPDATE STATE FIRST
            if (calib_led_node == my_addr) analogWrite(LED_PIN, 4095);
            else sendCANNetworkMsg('c', 'l', calib_led_node, 0); 
            break;

        case CALIB_LED_WAIT:
            if (now - calib_timer >= CALIB_LED_SETTLE_MS) {
                calib_state = CALIB_REQ_LUX;
            }
            break;

        case CALIB_REQ_LUX:
            calib_state = CALIB_WAIT_REPLY_LUX; // <-- UPDATE STATE FIRST
            // Hub saves its own values
            system_gain = gain_matrix[my_addr - 1][my_addr - 1];
            background_lux = lux_off[my_addr - 1];
            for(int k=0; k<buffer_size; k++) lux_buffer[k] = background_lux;
            
            Serial.println("\n=== CALIBRATION MATRIX (K) ===");
            for(int i=0; i<num_nodes; i++) {
                // Transmits background lux to the network (Node i+1)
                sendCANNetworkMsg('B', '0', i+1, lux_off[i]);
                
                for(int j=0; j<num_nodes; j++) {
                    Serial.print("K");
                    Serial.print(i+1); Serial.print(j+1); Serial.print(": ");
                    Serial.print(gain_matrix[i][j], 6); Serial.print("\t");
                    
                    // Transmits cross-gain to the correct destination
                    sendCANNetworkMsg('M', '0' + i, j+1, gain_matrix[i][j]);
                }
                Serial.println();
            }
            Serial.println("==============================");
            printHelpMenu();
            
            calib_timer = now;
            calib_state = CALIB_POST_DELAY;
            break;

        case CALIB_POST_DELAY:
            if (now - calib_timer >= 200) {
                sendCANNetworkMsg('c', 'd', 0, 0); // Broadcast DONE (Now Slaves can start)
                system_ready = true;
                mutex_enter_blocking(&data_mutex);
                feedback_enabled = true; // Starts Hub PID
                mutex_exit(&data_mutex);
                calib_state = CALIB_IDLE; 
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
  Serial.println("  g d <node>     : Get External Illuminance");
  Serial.println("  g E <node>     : Get Accumulated Energy (J)");
  Serial.println("  g V <node>     : Get Mean Visibility Error");
  Serial.println("  g F <node>     : Get Mean Flicker Error");
  Serial.println("  g L <node>     : Get Current Lower Bound");
  Serial.println("  g C <node>     : Get Energy Cost");
  Serial.println("\n--- LOCAL COMMANDS ---");
  Serial.println("  p <val>        : Set Kp (Local)");
  Serial.println("  i <val>        : Set Ki (Local)");
  Serial.println("  d <val>        : Set Kd (Local)");
  Serial.println("  b y 1 / b u 1  : Get last minute buffer");
  Serial.println("  R              : Restart System & Recalibrate");
  Serial.println("==================================================\n");
}

void processUI() {
  static String input_buffer = "";
  while (Serial.available() > 0) {
    char in_char = Serial.read();
    if (in_char == '\n' || in_char == '\r') {
      String input = input_buffer;
      input_buffer = "";
      input.trim(); 
      if (input.length() == 0) continue;
      char cmd = input.charAt(0);

    if (cmd == 'h' || cmd == '?') { printHelpMenu(); return; }

    if (cmd == 'R') { 
      sendCANNetworkMsg('R', 'R', 0, 0); // Shout to the whole network to restart
      Serial.println("ack"); return;
    }

    if (cmd == 's' || cmd == 'S') {
        char subcmd = input.charAt(2);
        int target_node = input.substring(4).toInt();
        if (target_node == my_addr) {
            if (subcmd == 'y') stream_y = (cmd == 's');
            if (subcmd == 'u') stream_u = (cmd == 's');
            Serial.println("ack");
        } else {
            char can_action = (cmd == 's') ? 'Y' : 'Z';
            sendCANNetworkMsg(can_action, subcmd, target_node, 0.0);
            Serial.println("ack");
        }
        return;
    }
    if (cmd == 'g') {
      char subcmd = input.charAt(2);
      int target_node = input.substring(4).toInt(); 
      if (target_node == my_addr) {
          switch (subcmd) {
            case 'u': {
                mutex_enter_blocking(&data_mutex);
                float snap_u = current_u;
                mutex_exit(&data_mutex);
                Serial.print("u "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_u); 
                break;
            }
            case 'r': Serial.print("r "); Serial.print(my_addr); Serial.print(" "); Serial.println(setpoint); break;
            case 'y': {
                mutex_enter_blocking(&data_mutex);
                float snap_y = current_lux;
                mutex_exit(&data_mutex);
                Serial.print("y "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_y); 
                break;
            }
            case 'a': Serial.print("a "); Serial.print(my_addr); Serial.print(" "); Serial.println(anti_windup_enabled); break;
            case 'f': Serial.print("f "); Serial.print(my_addr); Serial.print(" "); Serial.println(feedback_enabled); break;
            case 'o': Serial.print("o "); Serial.print(my_addr); Serial.print(" "); Serial.println(current_occupancy); break;
            case 'v': Serial.print("v "); Serial.print(my_addr); Serial.print(" "); Serial.println((analogRead(A0) / 4095.0) * 3.3, 3); break;
            case 't': Serial.print("t "); Serial.print(my_addr); Serial.print(" "); Serial.println(millis() / 1000.0); break;
            case 'd': Serial.print("d "); Serial.print(my_addr); Serial.print(" "); Serial.println(background_lux); break;
            case 'p': Serial.print("p "); Serial.print(my_addr); Serial.print(" "); Serial.println(P_max * (current_u / 4095.0), 4); break;
            case 'E': {
                mutex_enter_blocking(&metrics_mutex);
                float snap_E = E_energy;
                mutex_exit(&metrics_mutex);
                Serial.print("E "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_E, 4); 
                break;
            }
            case 'V': {
                mutex_enter_blocking(&metrics_mutex);
                float snap_V = metrics_count > 0 ? (V_visibility_sum / metrics_count) : 0;
                mutex_exit(&metrics_mutex);
                Serial.print("V "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_V, 4); 
                break;
            }
            case 'F': {
                mutex_enter_blocking(&metrics_mutex);
                float snap_F = metrics_count > 0 ? (F_flicker_sum / metrics_count) : 0;
                mutex_exit(&metrics_mutex);
                Serial.print("F "); Serial.print(my_addr); Serial.print(" "); Serial.println(snap_F, 4); 
                break;
            }
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

    int fS = input.indexOf(' '); int sS = input.indexOf(' ', fS + 1);
    
    // GAIN TUNING LOCAL
    if (sS == -1 && fS != -1) {
      float val = input.substring(fS + 1).toFloat();
      switch(cmd) {
        case 'p': mutex_enter_blocking(&data_mutex); myPID.setKp(val); mutex_exit(&data_mutex); Serial.println("ack"); break;
        case 'i': mutex_enter_blocking(&data_mutex); myPID.setKi(val); mutex_exit(&data_mutex); Serial.println("ack"); break;
        case 'd': mutex_enter_blocking(&data_mutex); myPID.setKd(val); mutex_exit(&data_mutex); Serial.println("ack"); break;
      }
      return;
    } 

    // SET COMMANDS
    if (sS != -1) {
      int target_node = input.substring(fS + 1, sS).toInt();
      float val = input.substring(sS + 1).toFloat();

      if (target_node == my_addr) {
          mutex_enter_blocking(&data_mutex);
          switch (cmd) {
            case 'r': setpoint = val; Serial.println("ack"); break;
                    case 'u': manual_pwm = val; feedback_enabled = false; Serial.println("ack"); break;
                    case 'f': feedback_enabled = (val > 0); if(!feedback_enabled) manual_pwm = current_u; Serial.println("ack"); break;
                    case 'a': anti_windup_enabled = (val > 0); Serial.println("ack"); break;
                    case 'o': current_occupancy = input.charAt(sS + 1); setpoint = (current_occupancy == 'h') ? lower_bound_high : (current_occupancy == 'l' ? lower_bound_low : 0.0); Serial.println("ack"); break;
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
    } else {
        input_buffer += in_char;
    }
  }
}

void setup() {
    mutex_init(&data_mutex);
    mutex_init(&metrics_mutex);
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

    boot_state = BOOT_IDLE; // Starts the state machine
}

void loop() {
    readCANMessages();
    processUI();
    if (boot_state != BOOT_DONE) {
        boot_task();
    } else if (!system_ready) {
        calib_task();
    } else {
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
            
            sendCANNetworkMsg('l', 'l', 0, lambda_i);
            
            if (stream_y) {
                if (is_hub) {
                    Serial.print("s y "); Serial.print(my_addr); Serial.print(" "); Serial.print(lux_snap); Serial.print(" "); Serial.println(millis());
                } else {
                    sendCANNetworkMsg('q', 'y', 1, lux_snap);
                }
            }
            if (stream_u) {
                if (is_hub) {
                    Serial.print("s u "); Serial.print(my_addr); Serial.print(" "); Serial.print(u_snap); Serial.print(" "); Serial.println(millis());
                } else {
                    sendCANNetworkMsg('q', 'u', 1, u_snap);
                }
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
// --- CORE 1: CONTROL LOOP (100 Hz) ---
// =====================================================================
void setup1() {
    while (boot_state == BOOT_IDLE) delay(10);
}

void loop1() {
    if (!system_ready) return;

    static unsigned long last_t = 0;
    unsigned long now = millis();
    if (now - last_t < 10) return;  // 100 Hz
    last_t += 10;

    // Measure LUX
    float lux = getFilteredLux(getLux());

    // --- SNAPSHOT SHARED DATA --- //
    mutex_enter_blocking(&data_mutex);
    bool snap_fb = feedback_enabled;
    bool snap_aw = anti_windup_enabled;
    float snap_sp = setpoint;
    float snap_man = manual_pwm;
    float snap_net_u[MAX_NODES];
    float snap_lambda[MAX_NODES];
    float snap_gain[MAX_NODES][MAX_NODES];
    for (int i = 0; i < num_nodes; i++) {
        snap_net_u[i] = network_u[i];
        snap_lambda[i] = network_lambda[i];
        for (int j = 0; j < num_nodes; j++) {
            snap_gain[i][j] = gain_matrix[i][j];
        }
    }
    mutex_exit(&data_mutex);

    // Calculate external illuminance (Feedforward base)
    float external_illuminance = 0.0;
    for (int j = 0; j < num_nodes; j++) {
        if (j != (my_addr - 1)) {
            external_illuminance += snap_gain[j][my_addr - 1] * snap_net_u[j];
        }
    }

    // Calculate control
    float u = 0.0f;
    float y_est = background_lux;

    if (snap_fb) {
        float sum_lambda_k = 0.0;
        for (int j = 0; j < num_nodes; j++) {
            sum_lambda_k += snap_lambda[j] * snap_gain[my_addr - 1][j];
        }

        float u_opt = sum_lambda_k / (2.0 * energy_cost);
        
        u = myPID.compute(snap_sp, lux, snap_aw, u_opt);

        for (int j = 0; j < num_nodes; j++) {
            if (j == my_addr - 1) {
                y_est += snap_gain[my_addr - 1][my_addr - 1] * u;
            } else {
                y_est += snap_gain[j][my_addr - 1] * snap_net_u[j];
            }
        }

        mutex_enter_blocking(&data_mutex);
        lambda_i = lambda_i + alpha * (snap_sp - y_est);
        if (lambda_i < 0.0) lambda_i = 0.0;
        mutex_exit(&data_mutex);
        
    } else {
        u = snap_man;
        y_est = background_lux + external_illuminance + (system_gain * u);
    }

    analogWrite(LED_PIN, (int)u);

    // Compute estimated illuminance for visibility error metric
    updateMetrics(u, y_est, snap_sp);

    // Update shared variables with protection
    mutex_enter_blocking(&data_mutex);
    current_lux = lux;
    current_u   = u;
    mutex_exit(&data_mutex);
}
