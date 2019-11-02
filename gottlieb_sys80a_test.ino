/**
 * Test fixture for Gottlieb System 80/80A MPU boards
 * 
 * Remove 6502 CPU (U1) and connect to Arduino Mega digital pins:
 *         +------------------+
 *    x* --| 1(VSS)  (/RES)40 |-- x
 *     x --| 2(RDY)  (Phi2)39 |-- 23
 *     x --| 3(Phi1)  (/SO)38 |-- x
 *     x --| 4(/IRQ) (Phi0)37 |-- x
 *     x --| 5(nc)     (nc)36 |-- x
 *     x --| 6(/NMI)   (nc)35 |-- x
 *     x --| 7(SYNC)  (R/W)34 |-- 27
 *     x --| 8(VCC)    (D0)33 |-- 29
 *    30 --| 9(A0)     (D1)32 |-- 31
 *    32 --| 10(A1)    (D2)31 |-- 33
 *    34 --| 11(A2)    (D3)30 |-- 35
 *    36 --| 12(A3)    (D4)29 |-- 37
 *    38 --| 13(A4)    (D5)28 |-- 39
 *    40 --| 14(A5)    (D6)27 |-- 41
 *    42 --| 15(A6)    (D7)26 |-- 43
 *    44 --| 16(A7)   (A15)25 |-- 45
 *    46 --| 17(A8)   (A14)24 |-- 47
 *    48 --| 18(A9)   (A13)23 |-- 49
 *    50 --| 19(A10)  (A12)22 |-- 51
 *    52 --| 20(A11)  (VSS)21 |-- x*
 *         +------------------+
 *    x = no connect
 *    * connections from VSS pins to GND are not necessary if
 *      the Arduino and MPU board share the same power supply
 */

#include <avr/pgmspace.h>

/* Mega2560 fast I/O */
#define PORT_ARDUINODIGITAL_0  E
#define PORT_ARDUINODIGITAL_1  E
#define PORT_ARDUINODIGITAL_2  E
#define PORT_ARDUINODIGITAL_3  E
#define PORT_ARDUINODIGITAL_4  G
#define PORT_ARDUINODIGITAL_5  E
#define PORT_ARDUINODIGITAL_6  H
#define PORT_ARDUINODIGITAL_7  H
#define PORT_ARDUINODIGITAL_8  H
#define PORT_ARDUINODIGITAL_9  H
#define PORT_ARDUINODIGITAL_10 B
#define PORT_ARDUINODIGITAL_11 B
#define PORT_ARDUINODIGITAL_12 B
#define PORT_ARDUINODIGITAL_13 B
#define PORT_ARDUINODIGITAL_14 J
#define PORT_ARDUINODIGITAL_15 J
#define PORT_ARDUINODIGITAL_16 H
#define PORT_ARDUINODIGITAL_17 H
#define PORT_ARDUINODIGITAL_18 D
#define PORT_ARDUINODIGITAL_19 D
#define PORT_ARDUINODIGITAL_20 D
#define PORT_ARDUINODIGITAL_21 D
#define PORT_ARDUINODIGITAL_22 A
#define PORT_ARDUINODIGITAL_23 A
#define PORT_ARDUINODIGITAL_24 A
#define PORT_ARDUINODIGITAL_25 A
#define PORT_ARDUINODIGITAL_26 A
#define PORT_ARDUINODIGITAL_27 A
#define PORT_ARDUINODIGITAL_28 A
#define PORT_ARDUINODIGITAL_29 A
#define PORT_ARDUINODIGITAL_30 C
#define PORT_ARDUINODIGITAL_31 C
#define PORT_ARDUINODIGITAL_32 C
#define PORT_ARDUINODIGITAL_33 C
#define PORT_ARDUINODIGITAL_34 C
#define PORT_ARDUINODIGITAL_35 C
#define PORT_ARDUINODIGITAL_36 C
#define PORT_ARDUINODIGITAL_37 C
#define PORT_ARDUINODIGITAL_38 D
#define PORT_ARDUINODIGITAL_39 G
#define PORT_ARDUINODIGITAL_40 G
#define PORT_ARDUINODIGITAL_41 G
#define PORT_ARDUINODIGITAL_42 L
#define PORT_ARDUINODIGITAL_43 L
#define PORT_ARDUINODIGITAL_44 L
#define PORT_ARDUINODIGITAL_45 L
#define PORT_ARDUINODIGITAL_46 L
#define PORT_ARDUINODIGITAL_47 L
#define PORT_ARDUINODIGITAL_48 L
#define PORT_ARDUINODIGITAL_49 L
#define PORT_ARDUINODIGITAL_50 B
#define PORT_ARDUINODIGITAL_51 B
#define PORT_ARDUINODIGITAL_52 B
#define PORT_ARDUINODIGITAL_53 B
#define PNUM_ARDUINODIGITAL_0  0
#define PNUM_ARDUINODIGITAL_1  1
#define PNUM_ARDUINODIGITAL_2  4
#define PNUM_ARDUINODIGITAL_3  5
#define PNUM_ARDUINODIGITAL_4  5
#define PNUM_ARDUINODIGITAL_5  3
#define PNUM_ARDUINODIGITAL_6  3
#define PNUM_ARDUINODIGITAL_7  4
#define PNUM_ARDUINODIGITAL_8  5
#define PNUM_ARDUINODIGITAL_9  6
#define PNUM_ARDUINODIGITAL_10 4
#define PNUM_ARDUINODIGITAL_11 5
#define PNUM_ARDUINODIGITAL_12 6
#define PNUM_ARDUINODIGITAL_13 7
#define PNUM_ARDUINODIGITAL_14 1
#define PNUM_ARDUINODIGITAL_15 0
#define PNUM_ARDUINODIGITAL_16 1
#define PNUM_ARDUINODIGITAL_17 0
#define PNUM_ARDUINODIGITAL_18 3
#define PNUM_ARDUINODIGITAL_19 2
#define PNUM_ARDUINODIGITAL_20 1
#define PNUM_ARDUINODIGITAL_21 0
#define PNUM_ARDUINODIGITAL_22 0
#define PNUM_ARDUINODIGITAL_23 1
#define PNUM_ARDUINODIGITAL_24 2
#define PNUM_ARDUINODIGITAL_25 3
#define PNUM_ARDUINODIGITAL_26 4
#define PNUM_ARDUINODIGITAL_27 5
#define PNUM_ARDUINODIGITAL_28 6
#define PNUM_ARDUINODIGITAL_29 7
#define PNUM_ARDUINODIGITAL_30 7
#define PNUM_ARDUINODIGITAL_31 6
#define PNUM_ARDUINODIGITAL_32 5
#define PNUM_ARDUINODIGITAL_33 4
#define PNUM_ARDUINODIGITAL_34 3
#define PNUM_ARDUINODIGITAL_35 2
#define PNUM_ARDUINODIGITAL_36 1
#define PNUM_ARDUINODIGITAL_37 0
#define PNUM_ARDUINODIGITAL_38 7
#define PNUM_ARDUINODIGITAL_39 2
#define PNUM_ARDUINODIGITAL_40 1
#define PNUM_ARDUINODIGITAL_41 0
#define PNUM_ARDUINODIGITAL_42 7
#define PNUM_ARDUINODIGITAL_43 6
#define PNUM_ARDUINODIGITAL_44 5
#define PNUM_ARDUINODIGITAL_45 4
#define PNUM_ARDUINODIGITAL_46 3
#define PNUM_ARDUINODIGITAL_47 2
#define PNUM_ARDUINODIGITAL_48 1
#define PNUM_ARDUINODIGITAL_49 0
#define PNUM_ARDUINODIGITAL_50 3
#define PNUM_ARDUINODIGITAL_51 2
#define PNUM_ARDUINODIGITAL_52 1
#define PNUM_ARDUINODIGITAL_53 0

#define _PASTE2(x,y)      x##y
#define _PASTE2X(x,y)     _PASTE2(x,y)
#define _PLET(p)          _PASTE2X(PORT_ARDUINODIGITAL_, p)
#define _PNUM(p)          _PASTE2X(PNUM_ARDUINODIGITAL_, p)

#define _DDR(p)           _PASTE2X(DDR, _PLET(p))
#define _PORT(p)          _PASTE2X(PORT,_PLET(p))
#define _PIN(p)           _PASTE2X(PIN, _PLET(p))

#define CLR_BIT(p,b)      (p) &= (~(_BV(b)))
#define SET_BIT(p,b)      (p) |= _BV(b)
#define BIT_IS_SET(n,b)   (!!((n) & _BV(b)))
#define setPinInput(p)    CLR_BIT(_DDR(p),  _PNUM(p))
#define setPinOutput(p)   SET_BIT(_DDR(p),  _PNUM(p))
#define setPinLow(p)      CLR_BIT(_PORT(p), _PNUM(p))
#define setPinHigh(p)     SET_BIT(_PORT(p), _PNUM(p))
#define setPin(p,v)       if (v) { setPinHigh(p); } else { setPinLow(p); }
#define pinIsHigh(p)      BIT_IS_SET(_PIN(p), _PNUM(p))

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define ROUND_UP_TO_POWER_OF_2(n,p) ((((n)-1)|((p)-1))+1)

/* Mapping to Arduino pins */
#define FLIPPED_LAYOUT 1

#if !FLIPPED_LAYOUT
#define BUS_A0  30
#define BUS_A1  32
#define BUS_A2  34
#define BUS_A3  36
#define BUS_A4  38
#define BUS_A5  40
#define BUS_A6  42
#define BUS_A7  44
#define BUS_A8  46
#define BUS_A9  48
#define BUS_A10 50
#define BUS_A11 52
#define BUS_A12 51
#define BUS_A13 49
#define BUS_A14 47
#define BUS_A15 45
#define BUS_D0  29
#define BUS_D1  31
#define BUS_D2  33
#define BUS_D3  35
#define BUS_D4  37
#define BUS_D5  39
#define BUS_D6  41
#define BUS_D7  43
#define BUS_RW  34
#else
#define BUS_A0  31
#define BUS_A1  33
#define BUS_A2  35
#define BUS_A3  37
#define BUS_A4  39
#define BUS_A5  41
#define BUS_A6  43
#define BUS_A7  45
#define BUS_A8  47
#define BUS_A9  49
#define BUS_A10 51
#define BUS_A11 53
#define BUS_A12 52
#define BUS_A13 50
#define BUS_A14 48
#define BUS_A15 46
#define BUS_D7  44
#define BUS_D6  42
#define BUS_D5  40
#define BUS_D4  38
#define BUS_D3  36
#define BUS_D2  34
#define BUS_D1  32
#define BUS_D0  30
#define BUS_RW  28
#define PHI2    26
#endif

#define PRINTF_BUF 80
void serialPrintf(const __FlashStringHelper *fmt, ...) {
  char buf[PRINTF_BUF];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf_P(buf, sizeof(buf), (const char *)fmt, ap);
  Serial.print(buf);
}


static void setBusAddress(uint16_t addr) {
  setPin(BUS_A0,  BIT_IS_SET(addr, 0))
  setPin(BUS_A1,  BIT_IS_SET(addr, 1))
  setPin(BUS_A2,  BIT_IS_SET(addr, 2))
  setPin(BUS_A3,  BIT_IS_SET(addr, 3))
  setPin(BUS_A4,  BIT_IS_SET(addr, 4))
  setPin(BUS_A5,  BIT_IS_SET(addr, 5))
  setPin(BUS_A6,  BIT_IS_SET(addr, 6))
  setPin(BUS_A7,  BIT_IS_SET(addr, 7))
  setPin(BUS_A8,  BIT_IS_SET(addr, 8))
  setPin(BUS_A9,  BIT_IS_SET(addr, 9))
  setPin(BUS_A10, BIT_IS_SET(addr, 10))
  setPin(BUS_A11, BIT_IS_SET(addr, 11))
  setPin(BUS_A12, BIT_IS_SET(addr, 12))
  setPin(BUS_A13, BIT_IS_SET(addr, 13))
  setPin(BUS_A14, BIT_IS_SET(addr, 14))
  setPin(BUS_A15, BIT_IS_SET(addr, 15))
}


/* must call acquireBus() first */
static void setBusData(uint8_t data) {
  setPin(BUS_D0,  BIT_IS_SET(data, 0))
  setPin(BUS_D1,  BIT_IS_SET(data, 1))
  setPin(BUS_D2,  BIT_IS_SET(data, 2))
  setPin(BUS_D3,  BIT_IS_SET(data, 3))
  setPin(BUS_D4,  BIT_IS_SET(data, 4))
  setPin(BUS_D5,  BIT_IS_SET(data, 5))
  setPin(BUS_D6,  BIT_IS_SET(data, 6))
  setPin(BUS_D7,  BIT_IS_SET(data, 7))
}


static uint8_t getBusData() {
  uint8_t value;
  value |= pinIsHigh(BUS_D7); value <<= 1;
  value |= pinIsHigh(BUS_D6); value <<= 1;
  value |= pinIsHigh(BUS_D5); value <<= 1;
  value |= pinIsHigh(BUS_D4); value <<= 1;
  value |= pinIsHigh(BUS_D3); value <<= 1;
  value |= pinIsHigh(BUS_D2); value <<= 1;
  value |= pinIsHigh(BUS_D1); value <<= 1;
  value |= pinIsHigh(BUS_D0);
  return value;
}


static void releaseDataBus() {
  setPinInput(BUS_D0);
  setPinInput(BUS_D1);
  setPinInput(BUS_D2);
  setPinInput(BUS_D3);
  setPinInput(BUS_D4);
  setPinInput(BUS_D5);
  setPinInput(BUS_D6);
  setPinInput(BUS_D7);
  setPinHigh(BUS_D0);
  setPinHigh(BUS_D1);
  setPinHigh(BUS_D2);
  setPinHigh(BUS_D3);
  setPinHigh(BUS_D4);
  setPinHigh(BUS_D5);
  setPinHigh(BUS_D6);
  setPinHigh(BUS_D7);
}


static void acquireDataBus() {
  setPinOutput(BUS_D0);
  setPinOutput(BUS_D1);
  setPinOutput(BUS_D2);
  setPinOutput(BUS_D3);
  setPinOutput(BUS_D4);
  setPinOutput(BUS_D5);
  setPinOutput(BUS_D6);
  setPinOutput(BUS_D7);
}



static uint8_t busReadByte(uint16_t addr) {
  noInterrupts();
  setBusAddress(addr);
  setPinHigh(BUS_RW);
  _delay_us(0.5);
  setPinHigh(PHI2);
  _delay_us(0.75);
  uint8_t data = getBusData();
  setPinLow(PHI2);
  interrupts();
  return data;
}


static void busWriteByte(uint16_t addr, uint8_t data) {
  noInterrupts();
  setBusAddress(addr);
  acquireDataBus();
  setBusData(data);
  setPinLow(BUS_RW);
  _delay_us(0.5);
  setPinHigh(PHI2);
  _delay_us(0.75);
  setPinLow(PHI2);
  setPinHigh(BUS_RW);
  releaseDataBus();
  interrupts();
}


static void dumpROM(uint16_t start, uint16_t len)
{
  /* very rudimentary, writes fixed-length records of 16 bytes each */
  uint16_t end = ROUND_UP_TO_POWER_OF_2(start+len, 5);
  uint8_t checksum;
  for (uint16_t addr = start; addr < end; addr++) {
    uint16_t outAddr = addr-start;
    if ((addr & 15) == 0) {
      serialPrintf(F("S113%04X"), outAddr);
      checksum = 0x13 + (outAddr & 0xFF) + ((outAddr >> 8) & 0xFF);
    }
    uint8_t b = busReadByte(addr);
    serialPrintf(F("%02X"), b);
    checksum += b;
    if ((addr & 15) == 15) {
      checksum = ~checksum;
      serialPrintf(F("%02X\n"), checksum);
    }
  }
}


static void testRAM(uint16_t start, uint16_t len, uint8_t mask=0xFF)
{
  /* Phase 1: check all possible values for each byte */
  /* Will identify bad SRAM cells and/or data lines */
  Serial.println(F("PHASE 1..."));
  uint16_t end = start+len;
  bool pass = true;
  for (uint16_t addr = start; addr < end; addr++) {
    uint8_t expected = 0;
    do {
      busWriteByte(addr, expected);
      uint8_t readback = busReadByte(addr);
      if ((readback & mask) != (expected & mask)) {
        serialPrintf(F("FAILED AT %04X: READ %02X, EXPECTED %02X\n"),
          addr, readback&mask, expected&mask);
          pass = false;
        break;
      }
    } while (++expected);
  }

  /* Phase 2: write a pseudorandom sequence of values at once, then read back. */
  /* Will identify address line issues. */
  Serial.println(F("PHASE 2..."));
  uint8_t n = 0;
  for (uint16_t addr = start; addr < end; addr++) {
    busWriteByte(addr, n);// ^ ((addr >> 8) & 0xFF));
    n += 137;
  }
  n = 0;
  for (uint16_t addr = start; addr < end; addr++) {
    uint8_t expected = n;// ^ ((addr >> 8) & 0xFF);
    uint8_t readback = busReadByte(addr);
    if ((readback & mask) != (expected & mask)) {
        serialPrintf(F("FAILED AT %04X: READ %02X, EXPECTED %02X\n"),
          addr, readback&mask, expected&mask);
          pass = false;      
    }
    n += 137;
  }
  Serial.println((pass) ? F("--- PASSED ---") : F("*** FAILED ***"));
}


static void dumpU2ROM()      { dumpROM(0x2000, 4096); }
static void dumpU3ROM()      { dumpROM(0x3000, 4096); }
static void dumpGameROM()    { dumpROM(0x1000, 2048); }
static void testU4RiotRAM()  { testRAM(0x0000, 128);  }
static void testU5RiotRAM()  { testRAM(0x0080, 128);  }
static void testU6RiotRAM()  { testRAM(0x0100, 128);  }
static void testAllRiotRAM() { testRAM(0x0000, 384);  }
static void testCmosRAM()    { testRAM(0x1800, 256, 0x0F); } /* CMOS RAM is 4-bit */


struct Test {
  const char * PROGMEM name;
  void (*testfn)();
} __attribute__((packed));
static_assert(sizeof(struct Test)==4, "wrong size");

static const char test1label[] PROGMEM = ": DUMP U2 ROM";
static const char test2label[] PROGMEM = ": DUMP U3 ROM";
static const char test3label[] PROGMEM = ": DUMP GAME PROM";
static const char test4label[] PROGMEM = ": TEST U4 RIOT RAM";
static const char test5label[] PROGMEM = ": TEST U5 RIOT RAM";
static const char test6label[] PROGMEM = ": TEST U6 RIOT RAM";
static const char test7label[] PROGMEM = ": TEST ALL RIOT RAMS";
static const char test8label[] PROGMEM = ": TEST CMOS RAM";

static const struct Test tests[] PROGMEM = {
  {test1label, dumpU2ROM},
  {test2label, dumpU3ROM},
  {test3label, dumpGameROM},
  {test4label, testU4RiotRAM},
  {test5label, testU5RiotRAM},
  {test6label, testU6RiotRAM},
  {test7label, testAllRiotRAM},
  {test8label, testCmosRAM},
};


static struct Test getTest(uint8_t idx) {
  Test t = {0};
  memcpy_P(&t, tests+idx, sizeof(Test));
  return t;
}


static void menu() {
  Serial.println(F("----------------------------------"));
  for (uint8_t i = 0; i < COUNT_OF(tests); i++) {
    Test t = getTest(i);
    Serial.write('1'+i);
    Serial.println((const __FlashStringHelper *)t.name);
  }
  Serial.write('?');
  while (!Serial.available()) {}
  char choice = Serial.read();
  Serial.write('\n');
  if (choice < '1' || choice >= '1'+COUNT_OF(tests)) {
    return;
  }
  Test t = getTest(choice-'1');
  Serial.print(F("RUNNING"));
  Serial.println((const __FlashStringHelper *)t.name);
  t.testfn();
}


void setup() {
  /* address bus is all outputs */
  setPinOutput(BUS_A0);
  setPinOutput(BUS_A1);
  setPinOutput(BUS_A2);
  setPinOutput(BUS_A3);
  setPinOutput(BUS_A4);
  setPinOutput(BUS_A5);
  setPinOutput(BUS_A6);
  setPinOutput(BUS_A7);
  setPinOutput(BUS_A8);
  setPinOutput(BUS_A9);
  setPinOutput(BUS_A10);
  setPinOutput(BUS_A11);
  setPinOutput(BUS_A12);
  setPinOutput(BUS_A13);
  setPinOutput(BUS_A14);
  setPinOutput(BUS_A15);
  setPinOutput(BUS_RW);
  setPinHigh(BUS_RW);
  setBusAddress(0x0000);
  /* data bus is initially inputs */
  releaseDataBus();

  /* Phi2 clock output */
  setPinOutput(PHI2);
  setPinLow(PHI2);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("GOTTLIEB SYSTEM 80/80A MPU TESTER"));
}


void loop() {
  menu();
}
