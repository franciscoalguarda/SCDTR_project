#include "mcp2515.h"


enum inter_core_cmds {
  //From core1 to core0: contains data read (16 bit)
  ICC_READ_DATA = 1,
  // From core0 to core1: contains data to write (16 bit)
  ICC_WRITE_DATA = 2,
  // From core1 to core0: contains regs CANINTF, EFLG
  ICC_ERROR_DATA = 3
};

//Packs 4 bytes into an unsigned int
uint32_t bytes_to_msg(uint8_t * b) {
  uint32_t b0 {b[0]}, b1 {b[1]},b2 {b[2]},b3 {b[3]};
  return b0 + (b1 << 8) + (b2 << 16) + (b3 << 24);
}

//Unpacks an unsigned int into its constituting 4 bytes
void msg_to_bytes(uint32_t msg, uint8_t * bytes) {
  bytes[0] = msg; bytes[1] = (msg >> 8);
  bytes[2] = (msg >> 16); bytes[3] = (msg >> 24);
}

//Packs the CAN frame contents into an unsigned int
uint32_t can_frame_to_msg(can_frame * frm) {
  uint8_t b[4];
  b[3] = ICC_READ_DATA; b[2] = frm->can_id;
  b[1] = frm->data[1]; b[0] = frm->data[0];
  return bytes_to_msg(b);
}

//Packs the CAN error flags into an unsigned int
uint32_t error_flags_to_msg(uint8_t canintf, uint8_t eflg) {
  uint8_t b[4];
  b[3] = ICC_ERROR_DATA; b[2] = 0;
  b[1] = canintf; b[0] = eflg;
  return bytes_to_msg(b);
}

void print_message(int number, int node, int id, int val) {
  Serial.print( number ); Serial.print(" at node " ); Serial.print( node, HEX );
  Serial.print(" with id "); Serial.print( id, HEX );
  Serial.print(" : "); Serial.println( val );
} 

char canintf_str[] {"| MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | "};
char eflg_str [] {"| RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | "};

void print_can_errors(uint8_t canintf, uint8_t eflg) {
  Serial.println("-----------------------------------------------------------------");
  Serial.println( canintf_str );
  Serial.print("| ");
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print(" "); Serial.write( bitRead( canintf, bit ) ? '1' : '0' ); Serial.print(" | ");
  }
  Serial.println("");
  Serial.println("-----------------------------------------------------------------");
  Serial.println(eflg_str);
  Serial.print("| ");
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print(" "); Serial.write(bitRead(eflg, bit) ? '1' : '0'); Serial.print(" | ");
  }
  Serial.println("");
  Serial.println("-----------------------------------------------------------------");
}

//Initializing Core 0 resources
pico_unique_board_id_t pico_board_id ;
uint8_t node_address;
unsigned long counterTx{0}, counterRx{0};
unsigned long time_to_write;
unsigned long write_delay {1000};
bool detected_errors {false};
uint8_t canintf_save {0};
uint8_t eflg_save {0};

void setup() {
  pico_get_unique_board_id( &pico_board_id );
  node_address = pico_board_id.id[7];
  Serial.begin();
  time_to_write = millis() + write_delay;
}

//Initializing Core 1 resources 
const uint8_t interruptPin {20};
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
volatile bool got_irq {false};
//the interrupt service routine for core 1
void read_interrupt(uint gpio, uint32_t events) {
  got_irq = true;
}

void setup1(){
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback( interruptPin,
  GPIO_IRQ_EDGE_FALL,
  true,
  &read_interrupt );
}

//Main loop in core0
void loop() {
  can_frame frm;
  uint32_t msg;
  uint8_t b[4];
  if(Serial.available()) {
    int val = Serial.parseInt();
    if(val > 0) write_delay = val;
  }
  if(millis() >= time_to_write ) {
    b[3] = ICC_WRITE_DATA;
    b[2] = node_address;
    b[0] = counterTx;
    b[1] = (counterTx >> 8);
    rp2040.fifo.push(bytes_to_msg(b));
    Serial.print(">>>>>> Sending ");
    print_message(counterTx, b[2], b[2], counterTx++);
    time_to_write = millis() + write_delay;
  }
  if( rp2040.fifo.pop_nb(& msg) ) {
    msg_to_bytes(msg, b);
    if(b[3] == ICC_READ_DATA) {
      uint16_t val = msg;
      Serial.print("<<<<<< Received ");
      print_message(counterRx++, node_address, b[2], val );
    }
    
    else if(b[3] == ICC_ERROR_DATA) {
      print_can_errors(b[1],b[0]);
      if(b[0] & 0b11111000) { //RX0OV | RX1OV | TXBO | TXEP | RXEP
        eflg_save = b[0]; canintf_save = b[1];
        can0.clearRXnOVRFlags(); 
        can0.clearInterrupts();
        detected_errors = true;
      }
    }
    if(detected_errors) {
      Serial.println(" ***** Detected errors in past operations *****");
      print_can_errors(canintf_save, eflg_save);
    }
  }
}


void loop1() {
  can_frame frm;
  uint32_t msg;
  uint8_t b[4];
  //reading the can-bus and writing the fifo
  if(got_irq) {
    got_irq = false;
    uint8_t irq = can0.getInterrupts();
    if(irq & MCP2515::CANINTF_RX0IF) {
      can0.readMessage( MCP2515::RXB0, &frm );
      rp2040.fifo.push_nb(can_frame_to_msg( &frm ) );
    }
    if(irq & MCP2515::CANINTF_RX1IF) {
      can0.readMessage(MCP2515::RXB1, &frm);
      rp2040.fifo.push_nb(can_frame_to_msg( &frm ) );
    }
    uint8_t err = can0.getErrorFlags();
    rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
  }

  //read fifo write bus
  if( rp2040.fifo.pop_nb( &msg ) ) {
    msg_to_bytes( msg , b );
    if( b[3] == ICC_WRITE_DATA ) {
      frm.can_id = b[2];
      frm.can_dlc = 2;
      frm.data[1] = b[1];
      frm.data[0] = b[0];
      can0.sendMessage(&frm);
    }

    uint8_t irq = can0.getInterrupts();
    uint8_t err = can0.getErrorFlags();
    rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
  }
}









