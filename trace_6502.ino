/**
   6502 Bus Tracer for Arduino Mega
   

   PA0 (Digital 22) -> A0
   PA1 (Digital 23) -> A1
   PA2 (Digital 24) -> A2
   PA3 (Digital 25) -> A3
   PA4 (Digital 26) -> A4
   PA5 (Digital 27) -> A5
   PA6 (Digital 28) -> A6
   PA7 (Digital 29) -> A7
   PC0 (Digital 37) -> A8
   PC1 (Digital 36) -> A9
   PC2 (Digital 35) -> A10
   PC3 (Digital 34) -> A11
   PC4 (Digital 33) -> A12
   PC5 (Digital 32) -> A13
   PC6 (Digital 31) -> A14 or R/W
   PC7 (Digital 30) -> A15 or SYNC
   PB0 (Digital 53) -> D0
   PB1 (Digital 52) -> D1
   PB2 (Digital 51) -> D2
   PB3 (Digital 50) -> D3
   PB4 (Digital 10) -> D4
   PB5 (Digital 11) -> D5
   PB6 (Digital 12) -> D6
   PB7 (Digital 13) -> D7
   PD0 (Digital 21) -> Phi2
   PD1/INT1 (Digital 20) -> /RESET
*/

#define MAX_CAPTURE_SAMPLES 2048

#define BYTES_PER_SAMPLE    3
#define MAX_CAPTURE_BYTES   (BYTES_PER_SAMPLE)*(MAX_CAPTURE_SAMPLES)

#define STR_(x) #x
#define STR(x) STR_(x)


static void __attribute__((noinline)) capture(bool immediate);
static void dump();


#define PRINTF_BUF 80
void serialPrintf(const __FlashStringHelper *fmt, ...) {
  static char buf[PRINTF_BUF];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf_P(buf, sizeof(buf), (const char *)fmt, ap);
  Serial.print(buf);
}


//void serialPrintBinary8(uint8_t n) {
//  for (uint8_t b = 0; b < 4; b++) {
//    Serial.write('0'+((int8_t)n < 0)); /* check high bit */
//    n <<= 1;
//  }
//  Serial.write('_');
//  for (uint8_t b = 0; b < 4; b++) {
//    Serial.write('0'+((int8_t)n < 0)); /* check high bit */
//    n <<= 1;
//  }
//}
//
//
//void serialPrintBinary16(uint16_t n) {
//  serialPrintBinary8(n >> 8);
//  Serial.write('_');
//  serialPrintBinary8(n & 0xFF);
//}
//

struct Sample {
  uint8_t data;
  uint16_t address;
} __attribute__((packed));
static_assert(sizeof(struct Sample)==3, "wrong size");

static Sample captureData[MAX_CAPTURE_SAMPLES];


static void clearCaptureData() {
  memset(captureData, 0, sizeof(captureData));
}


void loop() {
  clearCaptureData();
  Serial.println(F("1: TRIGGER ON RESET VECTOR FETCH"));
  Serial.println(F("2: TRIGGER IMMEDIATELY"));
  Serial.write('?');
  while (!Serial.available()) {}
  char ch = Serial.read();
  Serial.write(ch);
  Serial.write('\n');

  if (ch == '1') {
    capture(0);
  } else if (ch == '2') {
    capture(1);
  } else {
    return;
  }
  dump();
}


static void __attribute__((noinline)) capture(bool immediate) {
  register byte al = 0, ah = 0, d = 0;
  void *addr = captureData;

  /* Don't want interrupts to mess with the timing */
  noInterrupts();
  PORTD &= ~_BV(7);

  /* Capture loop is fully unrolled. */
  /* In order to keep up with the Phi2 clock, we have to */
  /* read and store three bytes in less than 16 cycles. */
  /* Pin change interrupts add too much latency, */
  /* so we manually wait for the Phi2 signal to go high. */
  /* Unrolling also avoids the overhead of maintaining a */
  /* loop counter, or comparing the destination pointer */
  /* to the end of the buffer. */
  /* Detailed operation and timing: */
  /* - Wait for the rising edge of Phi2. */
  /*   Because each loop iteration takes 3 cycles, */
  /*   there can be 2, 3, or 4 cycles (125-250ns) of latency after the rising edge. */
  /* - Store two address bytes from the previous iteration to RAM. */
  /*   This takes 6 cycles. (375ns) */
  /* - Read the two address bytes from the bus. */
  /*   This takes 2 cycles. */
  /* - By now, the falling edge of Phi2 has passed. */
  /*   (We are between 625-750ns after the rising edge. and */
  /*   can assume the data lines are valid. */
  /*   Read the data byte from the bus. (1 cycle) */
  /* - Store the data byte to RAM. (2 cycles) */
  /* - Total time: 812.5-937.5ns */

  /* Loop until we see a read from 0xFFFC */
  if (!immediate) {
    asm volatile("0: nop \n" \
                 "   nop \n" \
                 "1: sbic %[clk],0\n" \
                 "   rjmp 1b      \n" \
                 "   in   %[al],%[pal]\n" \
                 "   in   %[ah],%[pah]\n" \
                 "   cpi  %[ah],0x7F  \n" /* SYNC should be low */ \
                 "   brne 0b\n" \
                 "   cpi  %[al],0xFC  \n" \
                 "   brne 1b\n" \
                 "   in   %[d],%[pd]  \n" /* read data byte */
               : [al]  "+r" (al),
                 [ah]  "+r" (ah),
                 [d]   "+r" (d)
               : [clk] "M"  (_SFR_IO_ADDR(PIND)),
                 [pal] "M"  (_SFR_IO_ADDR(PINA)),
                 [pah] "M"  (_SFR_IO_ADDR(PINC)),
                 [pd]  "M"  (_SFR_IO_ADDR(PINB))
              );
  }

  /* Run the capture loop */
  PORTD |= _BV(7);
  asm volatile("\n" \
               "st   %a[ptr]+,%[d]  \n" /* dummy */ \
               ".rept " STR(MAX_CAPTURE_SAMPLES) "\n" \
               "sbic %[clk],0       \n" \
               "rjmp .-4            \n" \
               "st   %a[ptr]+,%[al] \n" /* low address byte of prev sample */ \
               "st   %a[ptr]+,%[ah] \n" /* high address byte of prev sample */ \
               "in   %[al],%[pal]   \n" \
               "in   %[ah],%[pah]   \n" \
               "in   %[d],%[pd]     \n" \
               "st   %a[ptr]+,%[d]  \n" /* data byte of current sample */ \
               ".endr               \n" \
               "st   %a[ptr]+,%[al] \n" /* low address byte of last sample */ \
               "st   %a[ptr]+,%[ah] \n" /* high address byte of last sample */ \
               : [al]  "+r" (al),
                 [ah]  "+r" (ah),
                 [d]   "+r" (d)
               : [clk] "M"  (_SFR_IO_ADDR(PIND)),
                 [pal] "M"  (_SFR_IO_ADDR(PINA)),
                 [pah] "M"  (_SFR_IO_ADDR(PINC)),
                 [pd]  "M"  (_SFR_IO_ADDR(PINB)),
                 [ptr] "e"  (addr)
               : "memory", "r30", "r31"
              );

  PORTD &= ~_BV(7);
  interrupts();
}


#define DISASSEMBLE 1

static void dump() {
#if !DISASSEMBLE
  Serial.println(F("CYCLE || ADDR | DATA\n"
                   "------++------+-----"));
  for (int i = 0; i < MAX_CAPTURE_SAMPLES; i++) {
    Sample s = captureData[i];
    serialPrintf(F("% 5d || %04X |  %02X\n"), i, s.address, s.data);
  }
#else
  Serial.println(F("CYCLE ||  ADDR  | DATA | DISASSEMBLY\n"
                   "------++--------+------+------------------\n"));

  /* for disassembly */
  uint8_t instruction[3] = {0};
  uint8_t instructionLength = 0;
  uint8_t instructionByte = 0; 
  uint16_t instructionLocation = 0;
  
  for (int i = 0; i < MAX_CAPTURE_SAMPLES; i++) {
    Sample s = captureData[i];
    bool sync = (s.address & _BV(15));
    bool isRead = (s.address & _BV(14));
    uint16_t address = s.address & 0x3FFF;
    
    /* first byte of instruction? */
    if (sync) {
      instructionLocation = address;
      instructionLength = getInstructionLength(s.data);
      instructionByte = 0;
      serialPrintf(F("------++--------+------+------------------\n"));
    }

    serialPrintf(F("% 5d || %c %04X |  %02X  | "),
      i, (isRead)?'R':'W', address, s.data);

    /* are we in an instruction fetch? */
    if (instructionLength) {
      instruction[instructionByte] = s.data;
      instructionByte++;
      /* after last byte, print the disassembly */
      if (instructionByte == instructionLength) {
        serialPrintInstruction(instructionLocation,
          instruction[0],
          instruction[1]|(instruction[2]<<8));
        /* no longer in an instruction fetch */
        instructionLength = 0;
        instruction[2] = 0;
        /* did the CPU crash? */
        if (isKillInstruction(instruction[0])) {
          Serial.println(F("\n(TARGET CRASHED)"));
          break;
        }
      }
    }
    Serial.write('\n');    
  }
#endif
}

void setup() {
  /* all ports are inputs */
  DDRA = 0;
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;
  DDRD |= _BV(7);

  EIMSK = 0;

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("6502 BUS TRACER"));
}




/* ------------------ Disassembler ------------------ */
enum AddressingMode : uint8_t {
  Acc,  /* accumulator:        OPC A */
  Abs,  /* absolute:           OPC $NNNN */ 
  AbsX, /* absolute X-indexed: OPC $NNNN,X */
  AbsY, /* absolute Y-indexed: OPC $NNNN,Y */
  Imm,  /* immediate:          OPC #$NN */
  Impl, /* implied:            OPC */
  Ind,  /* indirect:           OPC ($NNNN) */
  XInd, /* X-indexed indirect: OPC ($NN,X) */
  IndY, /* indirect Y-indexed: OPC ($NN),Y */
  Rel,  /* relative:           OPC $RR */
  Zpg,  /* zeropage:           OPC $NN */
  ZpgX, /* zeropage X-indexed: OPC $NN,X */
  ZpgY, /* zeropage Y-indexed: OPC $NN,Y */
};

static const uint8_t instructionLengths[16] PROGMEM = {
  [Acc]  = 1,
  [Abs]  = 3,
  [AbsX] = 3,
  [AbsY] = 3,
  [Imm]  = 2,
  [Impl] = 1,
  [Ind]  = 2,
  [XInd] = 2,
  [IndY] = 2,
  [Rel]  = 2,
  [Zpg]  = 2,
  [ZpgX] = 2,
  [ZpgY] = 2,
  [13]   = 1,
  [14]   = 1,
  [15]   = 1
};

static const char fmtAcc[]  PROGMEM = " A";
static const char fmtAbs[]  PROGMEM = " $%04hX";
static const char fmtAbsX[] PROGMEM = " $%04hX,X";
static const char fmtAbsY[] PROGMEM = " $%04hX,Y";
static const char fmtImm[]  PROGMEM = " #$%02hhX";
static const char fmtImpl[] PROGMEM = "";
static const char fmtInd[]  PROGMEM = " ($%04hX)";
static const char fmtXInd[] PROGMEM = " ($%02hhX,X)";
static const char fmtIndY[] PROGMEM = " ($%02hhX),Y";
static const char fmtRel[]  PROGMEM = " $%02hhX <$%04hX>";
static const char fmtZpg[]  PROGMEM = " $%02hhX";
static const char fmtZpgX[] PROGMEM = " $%02hhX,X";
static const char fmtZpgY[] PROGMEM = " $%02hhX,Y";

static const char * const PROGMEM instructionFormatStrings[16] PROGMEM = {
  [Acc]  = fmtAcc,
  [Abs]  = fmtAbs,
  [AbsX] = fmtAbsX,
  [AbsY] = fmtAbsY,
  [Imm]  = fmtImm,
  [Impl] = fmtImpl,
  [Ind]  = fmtInd,
  [XInd] = fmtXInd,
  [IndY] = fmtIndY,
  [Rel]  = fmtRel,
  [Zpg]  = fmtZpg,
  [ZpgX] = fmtZpgX,
  [ZpgY] = fmtZpgY,
  [13]   = fmtImpl,
  [14]   = fmtImpl,
  [15]   = fmtImpl
 };

struct Opcode {
  char mnem1, mnem2, mnem3;
  AddressingMode amode;
} __attribute__((packed));
static_assert(sizeof(struct Opcode)==4, "wrong size");

#undef ADC
#undef DEC
#define ADC 'A','D','C' 
#define AND 'A','N','D' 
#define ASL 'A','S','L' 
#define BCC 'B','C','C' 
#define BCS 'B','C','S' 
#define BEQ 'B','E','Q' 
#define BIT 'B','I','T' 
#define BMI 'B','M','I' 
#define BNE 'B','N','E' 
#define BPL 'B','P','L' 
#define BRK 'B','R','K' 
#define BVC 'B','V','C' 
#define BVS 'B','V','S' 
#define CLC 'C','L','C' 
#define CLD 'C','L','D' 
#define CLI 'C','L','I' 
#define CLV 'C','L','V' 
#define CMP 'C','M','P' 
#define CPX 'C','P','X' 
#define CPY 'C','P','Y' 
#define DEC 'D','E','C' 
#define DEX 'D','E','X' 
#define DEY 'D','E','Y' 
#define EOR 'E','O','R' 
#define INC 'I','N','C' 
#define INX 'I','N','X' 
#define INY 'I','N','Y' 
#define JMP 'J','M','P'
#define JSR 'J','S','R' 
#define LDA 'L','D','A' 
#define LDX 'L','D','X' 
#define LDY 'L','D','Y' 
#define LSR 'L','S','R'
#define NOP 'N','O','P' 
#define ORA 'O','R','A' 
#define PHA 'P','H','A' 
#define PHP 'P','H','P' 
#define PLA 'P','L','A' 
#define PLP 'P','L','P' 
#define ROL 'R','O','L' 
#define ROR 'R','O','R' 
#define RTI 'R','T','I' 
#define RTS 'R','T','S' 
#define SBC 'S','B','C' 
#define SEC 'S','E','C' 
#define SED 'S','E','D' 
#define SEI 'S','E','I' 
#define STA 'S','T','A' 
#define STX 'S','T','X' 
#define STY 'S','T','Y' 
#define TAX 'T','A','X' 
#define TAY 'T','A','Y' 
#define TSX 'T','S','X' 
#define TXA 'T','X','A' 
#define TXS 'T','X','S' 
#define TYA 'T','Y','A' 
#define KIL 'K','I','L' /* illegal opcode that halts the CPU */
#define ILL '?','?','?' /* other illegal opcodes */

static const Opcode opcodeTable[256] PROGMEM = {
  /*           x0          x1          x2          x3          x4          x5          x6          x7          x8          x9          xA          xB          xC          xD          xE          xF*     */
  /* 0x */ {BRK,Impl}, {ORA,XInd}, {KIL,Impl}, {ILL,XInd}, {ILL, Zpg}, {ORA, Zpg}, {ASL, Zpg}, {ILL, Zpg}, {PHP,Impl}, {ORA, Imm}, {ASL, Acc}, {ILL, Imm}, {ILL, Abs}, {ORA, Abs}, {ASL, Abs}, {ILL, Abs},
  /* 1x */ {BPL, Rel}, {ORA,IndY}, {KIL,Impl}, {ILL,IndY}, {ILL,ZpgX}, {ORA,ZpgX}, {ASL,ZpgX}, {ILL,ZpgX}, {CLC,Impl}, {ORA,AbsY}, {ILL,Impl}, {ILL,AbsY}, {ILL,AbsX}, {ORA,AbsX}, {ASL,AbsX}, {ILL,AbsX},
  /* 2x */ {JSR, Abs}, {AND,XInd}, {KIL,Impl}, {ILL,XInd}, {BIT, Zpg}, {AND, Zpg}, {ROL, Zpg}, {ILL, Zpg}, {PLP,Impl}, {AND, Imm}, {ROL, Acc}, {ILL, Imm}, {BIT, Abs}, {AND, Abs}, {ROL, Abs}, {ILL, Abs},
  /* 3x */ {BMI, Rel}, {AND,IndY}, {KIL,Impl}, {ILL,IndY}, {ILL,ZpgX}, {AND,ZpgX}, {ROL,ZpgX}, {ILL,ZpgX}, {SEC,Impl}, {AND,AbsY}, {ILL,Impl}, {ILL,AbsY}, {ILL,AbsX}, {AND,AbsX}, {ROL,AbsX}, {ILL,AbsX},
  /* 4x */ {RTI,Impl}, {EOR,XInd}, {KIL,Impl}, {ILL,XInd}, {ILL, Zpg}, {EOR, Zpg}, {LSR, Zpg}, {ILL, Zpg}, {PHA,Impl}, {EOR, Imm}, {LSR, Acc}, {ILL, Imm}, {JMP, Abs}, {EOR, Abs}, {LSR, Abs}, {ILL, Abs},
  /* 5x */ {BVC, Rel}, {EOR,IndY}, {KIL,Impl}, {ILL,IndY}, {ILL,ZpgX}, {EOR,ZpgX}, {LSR,ZpgX}, {ILL,ZpgX}, {CLI,Impl}, {EOR,AbsY}, {ILL,Impl}, {ILL,AbsY}, {ILL,AbsX}, {EOR,AbsX}, {LSR,AbsX}, {ILL,AbsX},
  /* 6x */ {RTS,Impl}, {ADC,XInd}, {KIL,Impl}, {ILL,XInd}, {ILL, Zpg}, {ADC, Zpg}, {ROR, Zpg}, {ILL, Zpg}, {PLA,Impl}, {ADC, Imm}, {ROR, Acc}, {ILL, Imm}, {JMP, Ind}, {ADC, Abs}, {ROR, Abs}, {ILL, Abs},
  /* 7x */ {BVS, Rel}, {ADC,IndY}, {KIL,Impl}, {ILL,IndY}, {ILL,ZpgX}, {ADC,ZpgX}, {ROR,ZpgX}, {ILL,ZpgX}, {SEI,Impl}, {ADC,AbsY}, {ILL,Impl}, {ILL,AbsY}, {ILL,AbsX}, {ADC,AbsX}, {ROR,AbsX}, {ILL,AbsX},
  /* 8x */ {ILL, Imm}, {STA,XInd}, {ILL, Imm}, {ILL,XInd}, {STY, Zpg}, {STA, Zpg}, {STX, Zpg}, {ILL, Zpg}, {DEY,Impl}, {ILL, Imm}, {TXA,Impl}, {ILL, Imm}, {STY, Abs}, {STA, Abs}, {STX, Abs}, {ILL, Abs},
  /* 9x */ {BCC, Rel}, {STA,IndY}, {KIL,Impl}, {ILL,IndY}, {STY,ZpgX}, {STA,ZpgX}, {STX,ZpgX}, {ILL,ZpgY}, {TYA,Impl}, {STA,AbsY}, {TXS,Impl}, {ILL,AbsY}, {ILL,AbsX}, {STA,AbsX}, {ILL,AbsY}, {ILL,AbsY},
  /* Ax */ {LDY, Imm}, {LDA,XInd}, {LDX, Imm}, {ILL,XInd}, {LDY, Zpg}, {LDA, Zpg}, {LDX, Zpg}, {ILL, Zpg}, {TAY,Impl}, {LDA, Imm}, {TAX,Impl}, {ILL, Imm}, {LDY, Abs}, {LDA, Abs}, {LDX, Abs}, {ILL, Abs},
  /* Bx */ {BCS, Rel}, {LDA,IndY}, {KIL,Impl}, {ILL,IndY}, {LDY,ZpgX}, {LDA,ZpgX}, {LDX,ZpgX}, {ILL,ZpgX}, {CLV,Impl}, {LDA,AbsY}, {TSX,Impl}, {ILL,AbsY}, {LDY,AbsX}, {LDA,AbsX}, {LDX,AbsY}, {ILL,AbsX},
  /* Cx */ {CPY, Imm}, {CMP,XInd}, {ILL, Imm}, {ILL,XInd}, {CPY, Zpg}, {CMP, Zpg}, {DEC, Zpg}, {ILL, Zpg}, {INY,Impl}, {CMP, Imm}, {DEX,Impl}, {ILL, Imm}, {CPY, Abs}, {CMP, Abs}, {DEC, Abs}, {ILL, Abs},
  /* Dx */ {BNE, Rel}, {CMP,IndY}, {KIL,Impl}, {ILL,IndY}, {ILL,ZpgX}, {CMP,ZpgX}, {DEC,ZpgX}, {ILL,ZpgX}, {CLD,Impl}, {CMP,AbsY}, {ILL,Impl}, {ILL,AbsY}, {ILL,AbsX}, {CMP,AbsX}, {DEC,AbsX}, {ILL,AbsX},
  /* Ex */ {CPX, Imm}, {SBC,XInd}, {ILL, Imm}, {ILL,XInd}, {CPX, Zpg}, {SBC, Zpg}, {INC, Zpg}, {ILL, Zpg}, {INX,Impl}, {SBC, Imm}, {NOP,Impl}, {ILL, Imm}, {CPX, Abs}, {SBC, Abs}, {INC, Abs}, {ILL, Abs},
  /* Fx */ {BEQ, Rel}, {SBC,IndY}, {KIL,Impl}, {ILL,IndY}, {ILL,ZpgX}, {SBC,ZpgX}, {INC,ZpgX}, {ILL,ZpgX}, {SED,Impl}, {SBC,AbsY}, {ILL,Impl}, {ILL,AbsY}, {ILL,AbsX}, {SBC,AbsX}, {INC,AbsX}, {ILL,AbsX}
};


uint8_t getInstructionLength(uint8_t code) {
  Opcode op;
  memcpy_P(&op, opcodeTable+code, sizeof(op));
  return pgm_read_byte(instructionLengths+(op.amode & 0xF));
}


bool isKillInstruction(uint8_t code) {
  Opcode op;
  memcpy_P(&op, opcodeTable+code, sizeof(op));
  return op.mnem1=='K' && op.mnem2=='I' && op.mnem3=='L';
}

void serialPrintInstruction(uint16_t pc, uint8_t opc, uint16_t args) {
  Opcode op;
  memcpy_P(&op, opcodeTable+opc, sizeof(op));
  serialPrintf(F("%04X: "), pc);
  Serial.write(op.mnem1);
  Serial.write(op.mnem2);
  Serial.write(op.mnem3);
  const char *fmt PROGMEM = pgm_read_word(instructionFormatStrings+(op.amode & 0xF));
  serialPrintf((const __FlashStringHelper *)fmt, args, 2+pc+(int8_t)(args&0xFF));
}





