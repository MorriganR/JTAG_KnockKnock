#include <EEPROM.h>

#define MAX4xR 4096
#define MAXREG8 400

#define TCK_PIN 2
#define TMS_PIN 3
#define TDI_PIN 4
#define TDO_PIN 5
#define PROBE_PIN 6
#define PROBE_ANALOG_PIN A0
#define JTAG_ANALOG_PIN A1
#define LED 13

// Use macros from avr/sfr_defs.h #define _BV(bit) (1 << (bit))
// A - must be defined as uint8_t*
#define setBit(A,k)   ( A[(k/8)] |= _BV(k%8) )
#define clrBit(A,k)   ( A[(k/8)] &= ~_BV(k%8) )            
#define getBit(A,k)   ( (A[(k/8)] & _BV(k%8)) >> (k%8) )

#define j_Clock(x, y) j_Clock_fast(x, y)
//#define j_Clock(x, y) j_Clock_slow(x, y)

struct j_EepromObject {
  uint32_t BOUNDLN;
  uint32_t INSTRLN;
  uint32_t SAMPLE;
};

int BDRlen, IRlen, nDevices;
char inByte = 0;
char regState = 0;
char preChar[4] = {'-', '\\', '|', '/'};
uint8_t preCharCount = 0;
uint32_t BOUNDLN = 1488; // BOUNDARY_LENGTH 
uint32_t INSTRLN = 10;   // INSTRUCTION_LENGTH
uint32_t SAMPLE = 0x00000005;
j_EepromObject j_Eeprom;
uint32_t idcode[10];     // IDCODE_REGISTER = 4-bit Version + 16-bit Part Number + 11-bit Manufacturer's ID + "1" Mandatory LSB
uint8_t dr0[MAXREG8];
uint8_t dr1[MAXREG8];
uint8_t dr2[MAXREG8];

uint8_t j_Clock_slow(uint8_t TDI, uint8_t TMS) {
  uint8_t result;
  digitalWrite(TDI_PIN, TDI);
  digitalWrite(TMS_PIN, TMS);
  result = digitalRead(TDO_PIN);
  digitalWrite(TCK_PIN, HIGH);
  digitalWrite(TCK_PIN, LOW);
  return result;
}

// Only if ALL pins map to PORTD
uint8_t j_Clock_fast(uint8_t TDI, uint8_t TMS) {
  uint8_t result, tempPORTD;
  tempPORTD = PORTD;
  if (TDI == 0) tempPORTD &= ~_BV(TDI_PIN); else tempPORTD |= _BV(TDI_PIN);
  if (TMS == 0) tempPORTD &= ~_BV(TMS_PIN); else tempPORTD |= _BV(TMS_PIN);
  PORTD = tempPORTD;
  result = (PIND >> TDO_PIN) & 0x01;  // result = digitalRead(TDO_PIN)
  tempPORTD |= _BV(TCK_PIN);          //digitalWrite(TCK_PIN, HIGH)
  PORTD = tempPORTD;
  tempPORTD &= ~_BV(TCK_PIN);         //digitalWrite(TCK_PIN, LOW);
  PORTD = tempPORTD;
  return result;
}

void j_RunIdle_ShiftIR() {
  j_Clock(0, 1); j_Clock(0, 1); j_Clock(0, 0); j_Clock(0, 0);
}
void j_RunIdle_ShiftDR() {
  j_Clock(0, 1); j_Clock(0, 0); j_Clock(0, 0);
}
void j_Exit1xx_RunIdle() {
  j_Clock(0, 1); j_Clock(0, 0);
}
void j_Shiftxx_RunIdle() {
  j_Clock(0, 1);
  j_Exit1xx_RunIdle();
}
void j_Any_Reset_RunIdle() {
  for(int i = 0; i < 10; i++) j_Clock(0, 1);
  j_Clock(0, 0);
}

// From ShiftXR => ?RegLen? =>  return to RunIdle
int j_GetChainLength() {
  int i;
  for(i = 0; i < MAX4xR; i++)    j_Clock(0, 0);        // empty the chain (fill it with 0's)
  for(i = 0; i < MAX4xR; i++) if(j_Clock(1, 0)) break; // feed the chain with 1's
  j_Shiftxx_RunIdle();
  return (i == MAX4xR) ? 0 : i;
}
// From ShiftXR => Send <len> bits Data from p[] => return to RunIdle
void j_SendData(uint8_t* p, int len){
  for (int i = 0; i < len; i++){
    j_Clock(getBit(p, i), (len - i) == 1); // at last bit set TMS=1
  }
  j_Exit1xx_RunIdle();
}
// From ShiftXR => Read <len> bits Data into p[] => return to RunIdle
void j_ReadData(uint8_t* p, int len) {
  for (int i = 0; i < len; i++){
    //clrBit(p, i);
    if (j_Clock(0, ((len - i) == 1))) setBit(p, i); // at last bit set TMS=1
  }
  j_Exit1xx_RunIdle();
}

// From RunIdle => ShiftIR => put data to IR => return to RunIdle
void j_SetIR(uint32_t ir, uint8_t len) {
  j_RunIdle_ShiftIR();
  j_SendData((uint8_t*) &ir, len);
}
// From RunIdle => ShiftDR => put data to DR => return to RunIdle
void j_SetDR(uint8_t* p, int len) {
  j_RunIdle_ShiftDR();
  j_SendData(p, len);
}
// From RunIdle => ShiftDR => get data from DR => return to RunIdle
void j_GetDR(uint8_t* p, int len) {
  j_RunIdle_ShiftDR();
  j_ReadData(p, len);
}

void jInfoScan() {
  j_Any_Reset_RunIdle();
  j_RunIdle_ShiftIR();
  IRlen = j_GetChainLength();
  // we are in BYPASS mode since j_GetChainLength filled the IR chain full of 1's
  // now we can easily determine the number of devices (= DR chain length when all the devices are in BYPASS mode)
  j_RunIdle_ShiftDR();
  nDevices = j_GetChainLength();
  // read the IDCODEs (assume all devices support IDCODE, so read 32 bits per device)
  j_Any_Reset_RunIdle(); // equal JTAG Instruction: IDCODE
  j_GetDR((uint8_t*) idcode, (32 * nDevices));
  j_Any_Reset_RunIdle();
  j_SetIR(SAMPLE, INSTRLN);
  j_RunIdle_ShiftDR();
  BDRlen = j_GetChainLength();
  j_Any_Reset_RunIdle();
  EEPROM.get(0x00, j_Eeprom);
  Serial.println("");
  Serial.print("JTAG Voltage = "); Serial.print(analogRead(JTAG_ANALOG_PIN) * 5.0 / 1023); Serial.println("V");
  Serial.print("SEMPLE inst code = 0x"); Serial.print(SAMPLE, HEX);
    Serial.print("/0x"); Serial.print(SAMPLE, HEX);
    Serial.print("/0x"); Serial.println(j_Eeprom.SAMPLE, HEX);
  Serial.print("IR chain length = "); Serial.print(IRlen);
    Serial.print("/"); Serial.print(INSTRLN);
    Serial.print("/"); Serial.println(j_Eeprom.INSTRLN);
  Serial.print("DR bound ch len = "); Serial.print(BDRlen);
    Serial.print("/"); Serial.print(BOUNDLN);
    Serial.print("/"); Serial.println(j_Eeprom.BOUNDLN);
  Serial.print("Number of device(s) = "); Serial.println(nDevices);
  for (int k = 0; k < nDevices; k++){
    Serial.print(" device #");
    Serial.print(k);
    Serial.print(" IDCODE: 0x");
    Serial.println(idcode[k], HEX);
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(PROBE_PIN, INPUT);
  pinMode(TCK_PIN, OUTPUT);
  pinMode(TMS_PIN, OUTPUT);
  pinMode(TDI_PIN, OUTPUT);
  pinMode(TDO_PIN, INPUT);
  Serial.begin(115200);
  //while (!Serial) { ;} // wait for serial port to connect. Needed for native USB port only
  for (uint32_t i = 0; i < (BOUNDLN / 8 + 1); i++) {dr2[i] = 0xFF;}
}

void loop() {
  pinMode(PROBE_PIN, INPUT);
  digitalWrite(LED, digitalRead(PROBE_PIN));   // turn the LED on|off
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if (inByte == '1') {
      regState = '1';
      Serial.println("Start Slow j_Clock...");
    }
    else if (inByte == '2') {
      regState = '2';
      Serial.println("Start Fast j_Clock...");
    }
    else if (inByte == 'w') {
      regState = 'w';
      Serial.println("Write some data to EEPROM.");
      j_Eeprom.SAMPLE = SAMPLE;
      j_Eeprom.INSTRLN = IRlen;
      j_Eeprom.BOUNDLN = BDRlen;
      EEPROM.put(0x00, j_Eeprom);
    }
    else if (inByte == 'r') {
      regState = 'r';
      Serial.println("Read data from EEPROM.");
      EEPROM.get(0x00, j_Eeprom);
      SAMPLE = j_Eeprom.SAMPLE;
      INSTRLN = j_Eeprom.INSTRLN;
      BOUNDLN = j_Eeprom.BOUNDLN;
    }
    else if (inByte == 'h') {
      regState = 'h';
      Serial.println("");
      Serial.println("Write HELP block here...");
    }
    else if (inByte == 's') {
      regState = 's';
      Serial.println("Enter new value for SAMPLE instruction:");
      Serial.setTimeout(10000);
      SAMPLE = strtoul((Serial.readStringUntil('\r')).c_str(),NULL,16);
      Serial.print("SAMPLE = 0x"); Serial.println(SAMPLE, HEX);
    }
    else if (inByte == 'i') {
      regState = 'i';
      jInfoScan();
    }
    else if (inByte == 'm') {
      regState = 'm';
      Serial.println("Set Monitor mode");
      j_Any_Reset_RunIdle();
      j_SetIR(SAMPLE, INSTRLN);
    }
    else if (inByte == 'b') {
      regState = 'b';
      Serial.println("Set Block some pins");
      for (uint32_t i = 0; i < (BOUNDLN / 8 + 1); i++) {dr2[i] = 0xFF;}
      j_Any_Reset_RunIdle();
      j_SetIR(SAMPLE, INSTRLN);
    }
    else if (inByte == 'l') {
      regState = 'l';
      Serial.println("List all Blocked pins:");
      int j = 0;
      for (uint32_t i = 0; i < BOUNDLN; i++) {
        if (getBit(dr2, i) == 0) {
          Serial.print(i);
          Serial.print(" ");
          j++;
        }
      }
      if (j != 0) Serial.println("");
    }
  }
  if (regState == 'm') {
    for (uint32_t i = 0; i < (BOUNDLN / 8 + 1); i++) {dr0[i] = 0x00; dr1[i] = 0x00;}
    pinMode(PROBE_PIN, OUTPUT);
    digitalWrite(PROBE_PIN, 0);
    j_GetDR(dr0, BOUNDLN);
    digitalWrite(PROBE_PIN, 1);
    j_GetDR(dr1, BOUNDLN);
    pinMode(PROBE_PIN, INPUT);
    int j = 0;
    for (uint32_t i = 0; i < BOUNDLN; i++) {
      if ((getBit(dr0, i) != getBit(dr1, i)) && getBit(dr2, i)) {
        if (j == 0){
          preCharCount++;
          Serial.write(preChar[preCharCount & 0x03]);
          Serial.print(" ");
        }
        Serial.print(i);
        Serial.print(" ");
        j++;
      }
    }
    if (j != 0) Serial.println("");
  }
  else if (regState == 'b') {
    for (uint32_t i = 0; i < (BOUNDLN / 8 + 1); i++) {dr0[i] = 0x00; dr1[i] = 0x00;}
    j_GetDR(dr0, BOUNDLN);
    j_GetDR(dr1, BOUNDLN);
    int j = 0;
    for (uint32_t i = 0; i < BOUNDLN; i++) {
      if ((getBit(dr0, i) != getBit(dr1, i)) && getBit(dr2, i)) {
        clrBit(dr2, i);
        Serial.print(i);
        Serial.print(" ");
        j++;
      }
    }
    if (j != 0) Serial.println("");
  }
  else if (regState == '1') {
    int i;
    for(i = 0; i < MAX4xR; i++) j_Clock_slow(((i >> 1) & 0x01), (i & 0x01));
  }
  else if (regState == '2') {
    int i;
    for(i = 0; i < MAX4xR; i++) j_Clock_fast(((i >> 1) & 0x01), (i & 0x01));
  }
}
