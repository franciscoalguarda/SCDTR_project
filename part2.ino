#include <SPI.h>
#include <mcp2515.h>
#include "hardware/flash.h" // Necessário para ler o UID da Pico para o Auto-Addressing

// ======================================================================
// --- CAN-BUS / NETWORK CONFIGURATION ---
// ======================================================================
const int CAN_CS_PIN = 17; 
MCP2515* mcp2515; 

// --- AUTO-ADDRESSING & MÁQUINA DE ESTADOS (WAKE-UP) ---
enum BootState { BOOT_IDLE, BOOT_ANNOUNCE, BOOT_DONE };
enum CalibState { CALIB_IDLE, CALIB_START, CALIB_BKG_WAIT, CALIB_LED_ON, CALIB_LED_WAIT, CALIB_REQ_LUX, CALIB_WAIT_REPLY_LUX, CALIB_WAIT_REPLY_BKG, CALIB_DONE };

BootState boot_state = BOOT_IDLE;
CalibState calib_state = CALIB_IDLE;
bool system_ready = false; // Só fica true quando a calibração acaba

uint32_t my_uid = 0;
int my_addr = 0;     // Substitui o antigo MY_NODE_ID (É atribuído sozinho!)
bool is_hub = false; // A placa que ficar com my_addr == 1 vira hub automaticamente

const int MAX_NODES = 3;
int num_nodes = 0;
uint32_t network_uids[MAX_NODES];

float gain_matrix[MAX_NODES][MAX_NODES]; // A nossa matriz K (Ganhos de Acoplamento)
float lux_off[MAX_NODES];                // Background Lux
float network_lux[MAX_NODES];            // Estado do Lux dos vizinhos em tempo real

// Timers para a Máquina de Estados
unsigned long last_hello_ms = 0;
const unsigned long HELLO_PERIOD_MS = 500;
const unsigned long BOOT_TIMEOUT_MS = 3000;
unsigned long boot_start_ms = 0;

int calib_led_node = 1;
int calib_req_node = 1;
unsigned long calib_timer = 0;
const unsigned long CALIB_LED_SETTLE_MS = 2000;

// ======================================================================
// --- LOCAL LUMINAIRE CONFIGURATION (FASE 1) ---
// ======================================================================
const int LED_PIN = 15;
const int DAC_RANGE = 4096;
float m = -1.0; 
float b = 6.75;  
float background_lux = 0;
float system_gain = 0.0;  
const float h = 0.01; // 100Hz

// Variáveis de Estado (Interface)
bool feedback_enabled = false; // Arranca desligado. Só liga depois da calibração    
bool anti_windup_enabled = true; 
float manual_pwm = 0;             
float current_lux = 0;           
float current_u = 0;       
char current_occupancy = 'o';     

// Variáveis da FASE 2 (Tabela 3)
float lower_bound_low = 10.0;  // Comando 'U'
float lower_bound_high = 50.0; // Comando 'O'
float setpoint = lower_bound_high;
float energy_cost = 1.0;       // Comando 'C'

// Buffers e Métricas
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

// Protótipos
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
    float compute(float ref, float y, bool antiwindup_control) {
      float error_p = b_weight * ref - y; 
      float error = ref - y; 
      float p_term = Kp * error_p;
      i_term += Ki * error * h; 
      float d_term = Kd * (error - last_error) / h;
      last_error = error;
      float u = p_term + i_term + d_term;
      
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
  for(int j = 0; j < 10; j++) { adc_sum += analogRead(A0); delayMicroseconds(500); }
  float read_adc = adc_sum / 10.0; if (read_adc == 0) read_adc = 1;
  float R_LDR = 40950000.0 / (float)read_adc - 10000.0;
  return pow(10, (log10(R_LDR) - b) / m);
}

void updateMetrics(float u, float y, float ref) {
    float d_k = u / 4095.0; 
    E_energy += P_max * d_k * h;
    if (y < ref) V_visibility_sum += (ref - y);
    float diff1 = d_k - last_u_1;
    float diff2 = last_u_1 - last_u_2;
    if (diff1 * diff2 < 0) F_flicker_sum += (abs(diff1) + abs(diff2));
    last_u_2 = last_u_1;
    last_u_1 = d_k;
    metrics_count++;
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
    // Ordenar UIDs do menor para o maior
    for (int i = 0; i < num_nodes - 1; i++) {
        for (int j = 0; j < num_nodes - i - 1; j++) {
            if (network_uids[j] > network_uids[j + 1]) {
                uint32_t temp = network_uids[j];
                network_uids[j] = network_uids[j + 1];
                network_uids[j + 1] = temp;
            }
        }
    }
    // O meu endereço é a minha posição na lista + 1
    for (int i = 0; i < num_nodes; i++) {
        if (network_uids[i] == my_uid) my_addr = i + 1;
    }
    if (my_addr == 1) is_hub = true; // O Nó 1 vira Hub
    
    Serial.print("\n=== BOOT DONE ===\nMy UID: "); Serial.println(my_uid);
    Serial.print("Assigned Address: NODE "); Serial.println(my_addr);
    Serial.print("Role: "); Serial.println(is_hub ? "HUB (Master)" : "SLAVE");
}

// ======================================================================
// --- CAN COMMUNICATION & PROTOCOL ---
// ======================================================================
void sendCANNetworkMsg(char action, char variable, int dest_node, float value) {
  struct can_frame canMsg;
  canMsg.can_id  = my_addr > 0 ? my_addr : 99; // 99 se ainda não tiver morada
  canMsg.can_dlc = 7; 
  canMsg.data[0] = action;    
  canMsg.data[1] = variable;  
  canMsg.data[2] = dest_node; 
  
  unsigned char* p = (unsigned char*)&value;
  for(int i = 0; i < 4; i++) canMsg.data[3 + i] = p[i];
  
  // Envia a mensagem fisicamente para a rede
  mcp2515->sendMessage(&canMsg);

  // --- LOOPBACK DE SOFTWARE (A MAGIA!) ---
  // Como o transceptor CAN não ouve o que ele próprio transmite,
  // se a mensagem for destinada a nós mesmos (ou a todos),
  // injetamo-la diretamente no nosso processador!
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
        myPID.reset(); E_energy = 0; V_visibility_sum = 0; F_flicker_sum = 0; metrics_count = 0;
        background_lux = 0;
        system_ready = false;
        if (is_hub) calib_state = CALIB_START; // Só o Maestro arranca a nova onda
        return;
    }

    // --- 1. BOOT MESSAGES ('w') ---
    if (action == 'w' && variable == 'h') {
        uint32_t received_uid;
        unsigned char* up = (unsigned char*)&received_uid;
        for(int i = 0; i < 4; i++) up[i] = canMsg.data[3 + i];
        register_node(received_uid);
        return;
    }

    if (dest_node != my_addr && dest_node != 0) return; // Ignorar se não for para mim ou broadcast

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
        else if (variable == 'x' && is_hub) { // Resposta de Lux chega ao Hub
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
        else if (variable == 'd') { // Fim da Calibração recebido pelos Slaves
            if (!is_hub) {
                for(int k=0; k<buffer_size; k++) lux_buffer[k] = background_lux;
                system_ready = true;
                feedback_enabled = true; // Liga o PID local
            }
        }
        return;
    }

    // --- 3. REGULAR NETWORK COMMANDS (HUB ROUTING) ---
    if (sender_id >= 1 && sender_id <= MAX_NODES && variable == 'y') {
        network_lux[sender_id - 1] = value; 
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
    }
    // QUALQUER placa que receba a resposta de um comando, imprime no ecrã!
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
            register_node(my_uid); // Regista-se a si própria (num_nodes passa a 1)
            boot_state = BOOT_ANNOUNCE;
            send_hello();
            last_hello_ms = now;
            Serial.println("Waiting for all 3 nodes to join...");
            break;
            
        case BOOT_ANNOUNCE:
            // Continua a gritar "Olá" a cada 500ms para sempre, até ver as outras
            if (now - last_hello_ms >= 500) {
                send_hello(); 
                last_hello_ms = now;
            }
            
            // A CORREÇÃO: Só avança quando tiver encontrado exatamente 3 Picos!
            if (num_nodes == MAX_NODES) {
                send_hello(); // Dá um último grito para a última placa que entrou a ouvir
                delay(500);   // Espera meio segundo para estabilizar a rede
                assign_addresses(); // Distribui as moradas (1, 2 e 3)
                boot_state = BOOT_DONE;
                if (is_hub) calib_state = CALIB_START; // O Nó 1 arranca a calibração
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
            calib_state = CALIB_BKG_WAIT; // <-- ATUALIZAR ESTADO PRIMEIRO
            sendCANNetworkMsg('c', 's', 0, 0); // Broadcast START (Apagar Leds)
            analogWrite(LED_PIN, 0);
            break;
            
        case CALIB_BKG_WAIT:
            if (now - calib_timer >= CALIB_LED_SETTLE_MS) {
                calib_state = CALIB_WAIT_REPLY_BKG; // <-- ATUALIZAR ESTADO PRIMEIRO
                sendCANNetworkMsg('c', 'g', calib_req_node, 0); // Pede Background Lux
            }
            break;

        case CALIB_LED_ON:
            calib_timer = now;
            calib_req_node = 1;
            calib_state = CALIB_LED_WAIT; // <-- ATUALIZAR ESTADO PRIMEIRO
            if (calib_led_node == my_addr) analogWrite(LED_PIN, 4095);
            else sendCANNetworkMsg('c', 'l', calib_led_node, 0); 
            break;

        case CALIB_LED_WAIT:
            if (now - calib_timer >= CALIB_LED_SETTLE_MS) {
                calib_state = CALIB_REQ_LUX;
            }
            break;

        case CALIB_REQ_LUX:
            calib_state = CALIB_WAIT_REPLY_LUX; // <-- ATUALIZAR ESTADO PRIMEIRO
            sendCANNetworkMsg('c', 'g', calib_req_node, 0); 
            break;

        case CALIB_DONE:
            calib_state = CALIB_IDLE; // <-- ATUALIZAR ESTADO PRIMEIRO
            sendCANNetworkMsg('c', 'd', 0, 0); // Broadcast DONE
            
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
            feedback_enabled = true; // Inicia PID do Hub
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
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); if (input.length() == 0) return;
    char cmd = input.charAt(0);

    if (cmd == 'h' || cmd == '?') { printHelpMenu(); return; }

    if (cmd == 'R') { 
      sendCANNetworkMsg('R', 'R', 0, 0); // Grita para a rede toda recomeçar
      Serial.println("ack"); return;
    }

    // GET COMMANDS
    if (cmd == 'g') {
      char subcmd = input.charAt(2);
      int target_node = input.substring(4).toInt(); 
      if (target_node == my_addr) {
          switch (subcmd) {
            case 'u': Serial.print("u "); Serial.print(my_addr); Serial.print(" "); Serial.println(current_u); break;
            case 'r': Serial.print("r "); Serial.print(my_addr); Serial.print(" "); Serial.println(setpoint); break;
            case 'y': Serial.print("y "); Serial.print(my_addr); Serial.print(" "); Serial.println(current_lux); break;
            case 'a': Serial.print("a "); Serial.print(my_addr); Serial.print(" "); Serial.println(anti_windup_enabled); break;
            case 'f': Serial.print("f "); Serial.print(my_addr); Serial.print(" "); Serial.println(feedback_enabled); break;
            case 'o': Serial.print("o "); Serial.print(my_addr); Serial.print(" "); Serial.println(current_occupancy); break;
            case 'v': Serial.print("v "); Serial.print(my_addr); Serial.print(" "); Serial.println((analogRead(A0) / 4095.0) * 3.3, 3); break;
            case 't': Serial.print("t "); Serial.print(my_addr); Serial.print(" "); Serial.println(millis() / 1000.0); break;
            case 'd': Serial.print("d "); Serial.print(my_addr); Serial.print(" "); Serial.println(background_lux); break;
            case 'p': Serial.print("p "); Serial.print(my_addr); Serial.print(" "); Serial.println(P_max * (current_u / 4095.0), 4); break;
            case 'E': Serial.print("E "); Serial.print(my_addr); Serial.print(" "); Serial.println(E_energy, 4); break;
            case 'V': Serial.print("V "); Serial.print(my_addr); Serial.print(" "); Serial.println(metrics_count > 0 ? (V_visibility_sum / metrics_count) : 0, 4); break;
            case 'F': Serial.print("F "); Serial.print(my_addr); Serial.print(" "); Serial.println(metrics_count > 0 ? (F_flicker_sum / metrics_count) : 0, 4); break;
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
        case 'p': myPID.setKp(val); Serial.println("ack"); break;
        case 'i': myPID.setKi(val); Serial.println("ack"); break;
        case 'd': myPID.setKd(val); Serial.println("ack"); break;
      }
      return;
    } 

    // SET COMMANDS
    if (sS != -1) {
      int target_node = input.substring(fS + 1, sS).toInt();
      float val = input.substring(sS + 1).toFloat();

      if (target_node == my_addr) {
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
      } else {
          if (cmd == 'o') val = (float)input.charAt(sS + 1); 
          sendCANNetworkMsg('s', cmd, target_node, val);
          Serial.println("ack");
      }
    }
  }
}

// ======================================================================
// --- SETUP & MAIN LOOP ---
// ======================================================================
void setup() {
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

  boot_state = BOOT_IDLE; // Inicia a máquina de estados
}

void loop() {
  static unsigned long last_t = millis();
  
  readCANMessages(); 
  processUI();       

  // 1. Boot & Auto-Addressing (Os 3 segundos iniciais)
  if (boot_state != BOOT_DONE) {
      boot_task();
  } 
  // 2. Calibração da Matriz de Ganhos (A "Onda")
  else if (!system_ready) {
      calib_task(); 
  } 
  // 3. Execução Normal (PID a 100Hz)
  else {
      if (millis() - last_t >= 10) { 
        last_t += 10;
        
        current_lux = getFilteredLux(getLux());
        network_lux[my_addr - 1] = current_lux; 

        if (feedback_enabled) current_u = myPID.compute(setpoint, current_lux, anti_windup_enabled);
        else current_u = manual_pwm;

        analogWrite(LED_PIN, (int)current_u);
        updateMetrics(current_u, current_lux, setpoint);

        // Broadcast e Gravação de Histórico (10Hz)
        history_counter++;
        if (history_counter >= 10) {
          history_y[history_idx] = current_lux;
          history_u[history_idx] = current_u;
          history_idx = (history_idx + 1) % minute_buffer_size;
          
          sendCANNetworkMsg('b', 'y', 0, current_lux);
          history_counter = 0;
        }
      }
  }
}