#include <avr/pgmspace.h>

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

struct Input {
  const char * const PROGMEM label;
  const uint8_t inpin;
  const uint8_t unused;
} __attribute__((packed));
static_assert(sizeof(struct Input)==4, "wrong size");

union InputRaw {
  struct Input input;
  uint32_t raw;
};

static const char label_U31_nCE[] PROGMEM = "U31 ROM   /CE";
static const char label_U30_nCE[] PROGMEM = "U30 ROM   /CE";
static const char label_Z5_nCE[] PROGMEM  = " Z5 RAM   /CE";
static const char label_U4_nRS[] PROGMEM  = " U4 RIOT  /RS";
static const char label_U4_CS1[] PROGMEM  = " U4 RIOT  CS1";
static const char label_U4_nCS2[] PROGMEM = " U4 RIOT /CS2";
static const char label_U5_nRS[] PROGMEM  = " U5 RIOT  /RS";
static const char label_U5_CS1[] PROGMEM  = " U5 RIOT  CS1";
static const char label_U5_nCS2[] PROGMEM = " U5 RIOT /CS2";
static const char label_U6_nRS[] PROGMEM  = " U6 RIOT  /RS";
static const char label_U6_CS1[] PROGMEM  = " U6 RIOT  CS1";
static const char label_U6_nCS2[] PROGMEM = " U6 RIOT /CS2";

/**
 * Connect the numbered Arduino Mega digital pins to their respective
 * pins on the ICs.
 */
static const struct Input inputs[] PROGMEM = {
  {label_U31_nCE, 23},
  {label_U30_nCE, 25},
  {label_Z5_nCE,  27},
  {label_U4_nRS,  29},
  {label_U4_nCS2, 31},
  {label_U4_CS1,  33},
  {label_U5_nRS,  35},
  {label_U5_nCS2, 37},
  {label_U5_CS1,  39},
  {label_U6_nRS,  41},
  {label_U6_nCS2, 43},
  {label_U6_CS1,  45},
};
static const int NUM_INPUTS = COUNT_OF(inputs);


void setup() {
  for (uint8_t i = 0; i < NUM_INPUTS; i++) {
    union InputRaw input = { .raw = pgm_read_dword_near(&inputs[i]) };
    pinMode(input.input.inpin, INPUT);
    digitalWrite(input.input.inpin, HIGH); // enable pullup resistor
  }
  
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("OK");
}

void loop() {
  Serial.print("\033[H\033[2J");
  for (uint8_t i = 0; i < NUM_INPUTS; i++) {
    union InputRaw input = { .raw = pgm_read_dword_near(&inputs[i]) };
    bool value = digitalRead(input.input.inpin);
    Serial.print((__FlashStringHelper*)input.input.label);
    if (value) {
      Serial.println(F(" \033[1;7;32m 1 \033[0m"));
    } else {
      Serial.println(F(" \033[1;7;31m 0 \033[0m"));
    }
  }
  delay(50);
}
