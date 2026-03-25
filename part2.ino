#include <SPI.h>
#include <mcp2515.h>

// ======================================================================
// --- CAN-BUS / NETWORK CONFIGURATION ---
// ======================================================================
const int CAN_CS_PIN = 17; 
MCP2515* mcp2515; // Pointer to avoid USB crash at startup

const int MY_NODE_ID = 1; // CHANGE TO 2 OR 3 ON THE OTHER LUMINAIRES
const int MAX_NODES = 3;
float network_lux[MAX_NODES] = {0.0, 0.0, 0.0}; // Stores the Lux of [Node 1, Node 2, Node 3]

// ======================================================================
// --- LOCAL LUMINAIRE CONFIGURATION ---
// ======================================================================
const int LED_PIN = 15;
const int DAC_RANGE = 4096;
float m = -1.0; 
float b = 6.75;  

float background_lux = 0;
float system_gain = 0.0; 
float setpoint = 0.0; // Current reference in lux

const float h = 0.01; // Sampling period of 10ms (100Hz) 

// --- STATE VARIABLES (INTERFACE) ---
bool feedback_enabled = true;    
bool anti_windup_enabled = true; 
bool stream_y = false;           
bool stream_u = false;           
float manual_pwm = 0;             
float current_lux = 0;           
float current_u = 0;       
char current_occupancy = 'o'; // 'o' (off), 'l' (low) or 'h' (high)       

// --- PHASE 2 VARIABLES (TABLE 3) ---
float lower_bound_low = 10.0;  // U
float lower_bound_high = 50.0; // O
float energy_cost = 1.0;       // C

bool stream_excel = false;

// Circular Buffer
const int buffer_size = 10;
float lux_buffer[buffer_size];
int buffer_idx = 0;

// 1-Minute history variables 
const int minute_buffer_size = 600; 
float history_y[minute_buffer_size];
float history_u[minute_buffer_size];
int history_idx = 0;
int history_counter = 0; 

// Performance metrics variables
float E_energy = 0;          
float V_visibility_sum = 0;  
float F_flicker_sum = 0;     
long metrics_count = 0;      
float last_u_1 = 0;          
float last_u_2 = 0;          
float P_max = 0.05;          

// Prototypes
float getLux();
float getFilteredLux(float new_sample);
void calibrateGain();
void processUI();
void updateMetrics(float u, float y, float ref);
void sendCANNetworkMsg(char action, char variable, int dest_node, float value);
void readCANMessages();
void printHelpMenu();

// ======================================================================
// --- SETUP ---
// ======================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize CAN-bus safely
  SPI.begin();
  mcp2515 = new MCP2515(CAN_CS_PIN);
  mcp2515->reset();
  mcp2515->setBitrate(CAN_500KBPS, MCP_8MHZ); // Change to MCP_16MHZ if needed
  mcp2515->setNormalMode();

  Serial.println("CAN-bus Initialized!");
  
  // while (!Serial); // <--- KEEP COMMENTED OUT

  Serial.setTimeout(2); 

  analogReadResolution(12);
  analogWriteFreq(60000);      
  analogWriteRange(DAC_RANGE); 

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0); 

  delay(2000); 
  calibrateGain(); // Run initial calibration

  printHelpMenu();
}

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
  Serial.println("  g v <node>     : Get LDR sensor Voltage");
  Serial.println("  g d <node>     : Get External Illuminance (Background)");
  Serial.println("  g E <node>     : Get Accumulated Energy (J)");
  Serial.println("  g V <node>     : Get Mean Visibility Error (LUX)");
  Serial.println("  g F <node>     : Get Mean Flicker Error (s^-1)");
  Serial.println("  g L <node>     : Get Current Lower Bound (L)");
  Serial.println("  g C <node>     : Get Energy Cost (C)");
  Serial.println("\n--- LOCAL COMMANDS ---");
  Serial.println("  b y 1 / b u 1  : Get last minute buffer");
  Serial.println("  R              : Restart System & Recalibrate");
  Serial.println("==================================================\n");
}

// ======================================================================
// --- PID CONTROLLER CLASS ---
// ======================================================================
class PIDController {
  private:
    float Kp, Ki, Kd;
    float b_weight;
    float i_term, last_error;
    float h;

  public:
    PIDController(float p, float i, float d, float bw) {
      Kp = p; Ki = i; Kd = d; b_weight = bw; 
      i_term = 0; last_error = 0;
      h = ::h; 
    }

    float compute(float ref, float y, bool antiwindup_control) {
      float error_p = b_weight * ref - y; 
      float error = ref - y; 
      
      float p_term = Kp * error_p;
      i_term += Ki * error * h; 
      float d_term = Kd * (error - last_error) / h;
      last_error = error;

      float u = p_term + i_term + d_term;

      // Anti-Windup
      if (u > 4095) {
          u = 4095;
          if (error > 0) i_term -= Ki * error * h; 
      } 
      else if (u < 0) {
          u = 0;
          if (error < 0) i_term -= Ki * error * h; 
      }
      return u;
    }

    void setKp(float p) { Kp = p; }
    void setKi(float i) { Ki = i; }
    void setKd(float d) { Kd = d; }
    void reset() { i_term = 0; last_error = 0; }
};

PIDController myPID(80.0, 400.0, 0.0, 0.5); 

// ======================================================================
// --- SENSOR READING & MATH ---
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
  for(int j = 0; j < 10; j++) {
    adc_sum += analogRead(A0);
    delayMicroseconds(500);
  }
  float read_adc = adc_sum / 10.0;
  if (read_adc == 0) read_adc = 1;

  float R_LDR = 40950000.0 / (float)read_adc - 10000.0;
  float logR = log10(R_LDR);
  return pow(10, (logR - b) / m);
}

void calibrateGain() {
  analogWrite(LED_PIN, 0);
  delay(1000);
  
  float sum = 0;
  for(int j = 0; j < 50; j++) {
    float val = getLux();
    sum += val;
    for(int k=0; k<buffer_size; k++) lux_buffer[k] = val; 
    delay(5);
  }
  background_lux = sum / 50.0;
  Serial.print("Background LUX Calibrated: ");
  Serial.println(background_lux); 
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
// --- HUB / CAN NETWORK ROUTING ---
// ======================================================================
void sendCANNetworkMsg(char action, char variable, int dest_node, float value) {
  struct can_frame canMsg;
  canMsg.can_id  = MY_NODE_ID; 
  canMsg.can_dlc = 7; 

  canMsg.data[0] = action;    
  canMsg.data[1] = variable;  
  canMsg.data[2] = dest_node; 

  unsigned char* bytes = (unsigned char*)&value;
  for(int i = 0; i < 4; i++) {
    canMsg.data[3 + i] = bytes[i];
  }
  mcp2515->sendMessage(&canMsg);
}

void readCANMessages() {
  struct can_frame canMsg;
  
  if (mcp2515->readMessage(&canMsg) == MCP2515::ERROR_OK) {
    int sender_id = canMsg.can_id;
    char action = canMsg.data[0];
    char variable = canMsg.data[1];
    int dest_node = canMsg.data[2];

    float value;
    unsigned char* p = (unsigned char*)&value;
    for(int i = 0; i < 4; i++) {
      p[i] = canMsg.data[3 + i];
    }

    // Keep track of neighbors' light levels (even from background broadcasts 'b')
    if (sender_id >= 1 && sender_id <= MAX_NODES && variable == 'y') {
        network_lux[sender_id - 1] = value;
    }

    if (dest_node == MY_NODE_ID || dest_node == 0) {
        
        if (action == 'g') {
            float reply_val = 0;
            switch(variable) {
                // Phase 1 Get
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
                
                // Phase 2 Get (Table 3)
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
            // Phase 1 Set
            if (variable == 'r') setpoint = value;
            else if (variable == 'u') { manual_pwm = value; feedback_enabled = false; }
            else if (variable == 'f') feedback_enabled = (value > 0);
            else if (variable == 'a') anti_windup_enabled = (value > 0);
            else if (variable == 'o') { 
              current_occupancy = (char)value; 
              // Update local setpoint based on new occupancy
              if (current_occupancy == 'h') setpoint = lower_bound_high;
              else if (current_occupancy == 'l') setpoint = lower_bound_low;
              else setpoint = 0.0;
            }
            // Phase 2 Set
            else if (variable == 'U') lower_bound_low = value;
            else if (variable == 'O') lower_bound_high = value;
            else if (variable == 'C') energy_cost = value;
        }
        else if (action == 'd') {
            Serial.print(variable); 
            Serial.print(" "); 
            Serial.print(sender_id); 
            Serial.print(" "); 
            if (variable == 'o') Serial.println((char)value);
            else Serial.println(value, 4);
        }
    }
  }
}

// ======================================================================
// --- USER INTERFACE (HUB LOGIC) ---
// ======================================================================
void processUI() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    if (input.length() == 0) return;

    char cmd = input.charAt(0);

    if (cmd == 'h' || cmd == '?') { printHelpMenu(); return; }

    // Phase 2 Restart & Recalibrate (Command R)
    if (cmd == 'R') {
      myPID.reset(); E_energy = 0; V_visibility_sum = 0; F_flicker_sum = 0; metrics_count = 0;
      calibrateGain(); 
      Serial.println("ack"); return;
    }

    // GET COMMANDS 
    if (cmd == 'g') {
      char subcmd = input.charAt(2);
      int target_node = input.substring(4).toInt(); 

      if (target_node == MY_NODE_ID) {
          switch (subcmd) {
            case 'u': Serial.print("u "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(current_u); break;
            case 'r': Serial.print("r "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(setpoint); break;
            case 'y': Serial.print("y "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(current_lux); break;
            case 'a': Serial.print("a "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(anti_windup_enabled); break;
            case 'f': Serial.print("f "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(feedback_enabled); break;
            case 'o': Serial.print("o "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(current_occupancy); break;
            case 'v': Serial.print("v "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println((analogRead(A0) / 4095.0) * 3.3, 3); break;
            case 't': Serial.print("t "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(millis() / 1000.0); break;
            case 'd': Serial.print("d "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(background_lux); break;
            case 'p': Serial.print("p "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(P_max * (current_u / 4095.0), 4); break;
            case 'E': Serial.print("E "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(E_energy, 4); break;
            case 'V': Serial.print("V "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(metrics_count > 0 ? (V_visibility_sum / metrics_count) : 0, 4); break;
            case 'F': Serial.print("F "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(metrics_count > 0 ? (F_flicker_sum / metrics_count) : 0, 4); break;
            
            // Phase 2 Gets
            case 'U': Serial.print("U "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(lower_bound_low); break;
            case 'O': Serial.print("O "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(lower_bound_high); break;
            case 'C': Serial.print("C "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(energy_cost); break;
            case 'L': {
                float L_val = 0;
                if (current_occupancy == 'h') L_val = lower_bound_high;
                else if (current_occupancy == 'l') L_val = lower_bound_low;
                Serial.print("L "); Serial.print(MY_NODE_ID); Serial.print(" "); Serial.println(L_val); 
                break;
            }
            case 'b': {
                char var = input.charAt(6); 
                Serial.print("b "); Serial.print(var); Serial.print(" "); Serial.print(MY_NODE_ID); Serial.print(" ");
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
          if (subcmd == 'b') Serial.println("err: buffer too large for CAN-BUS");
          else sendCANNetworkMsg('g', subcmd, target_node, 0.0);
      }
      return;
    }

    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);

    // SET COMMANDS 
    if (secondSpace != -1) {
      int target_node = input.substring(firstSpace + 1, secondSpace).toInt();
      float val = input.substring(secondSpace + 1).toFloat();

      if (target_node == MY_NODE_ID) {
          switch (cmd) {
            case 'r': setpoint = val; Serial.println("ack"); break;
            case 'u': manual_pwm = (val / 100.0) * 4095.0; feedback_enabled = false; Serial.println("ack"); break;
            case 'f': feedback_enabled = (val > 0); if(!feedback_enabled) manual_pwm = current_u; Serial.println("ack"); break;
            case 'a': anti_windup_enabled = (val > 0); Serial.println("ack"); break;
            case 'o': 
              current_occupancy = input.charAt(secondSpace + 1); 
              if (current_occupancy == 'h') setpoint = lower_bound_high;
              else if (current_occupancy == 'l') setpoint = lower_bound_low;
              else setpoint = 0.0;
              Serial.println("ack"); 
              break;
            // Phase 2 Sets
            case 'U': lower_bound_low = val; Serial.println("ack"); break;
            case 'O': lower_bound_high = val; Serial.println("ack"); break;
            case 'C': energy_cost = val; Serial.println("ack"); break;
            default: Serial.println("err"); break;
          }
      } else {
          if (cmd == 'o') val = (float)input.charAt(secondSpace + 1); // Pass char as float payload
          sendCANNetworkMsg('s', cmd, target_node, val);
          Serial.println("ack");
      }
    }
  }
}

// ======================================================================
// --- MAIN LOOP (100Hz) ---
// ======================================================================
void loop() {
  static unsigned long last_t = millis();
  processUI(); 

  if (millis() - last_t >= 10) { 
    readCANMessages(); 
    last_t += 10;

    current_lux = getFilteredLux(getLux());
    network_lux[MY_NODE_ID - 1] = current_lux; 

    if (feedback_enabled) {
      current_u = myPID.compute(setpoint, current_lux, anti_windup_enabled);
    } else {
      current_u = manual_pwm;
    }

    analogWrite(LED_PIN, (int)current_u);
    updateMetrics(current_u, current_lux, setpoint);

    // History recording and Broadcast (10Hz)
    history_counter++;
    if (history_counter >= 10) {
      history_y[history_idx] = current_lux;
      history_u[history_idx] = current_u;
      history_idx = (history_idx + 1) % minute_buffer_size;
      
      // BROADCAST my Lux using 'b' (background) so it doesn't print on Serial
      sendCANNetworkMsg('b', 'y', 0, current_lux);
      
      history_counter = 0;
    }
  }
}