#define NUM_BTN_COLUMNS (4)
#define NUM_BTN_ROWS (4)
#define MAX_DEBOUNCE (5)
#include <MIDI.h>
#include "Button.h"


MIDI_CREATE_DEFAULT_INSTANCE();

int SWITCH1 = 2;
int SWITCH2 = 3;
int SWITCH3 = 4;
int SWITCH4 = 5;

int GROUND1 = 6;
int GROUND2 = 7;
int GROUND3 = 8;
int GROUND4 = 9;

static const Button buttons[NUM_BTN_COLUMNS] = {Button(SWITCH1, 500), Button(SWITCH2, 500), Button(SWITCH3, 500), Button(SWITCH4, 500) };
static const uint8_t btnInputs[NUM_BTN_COLUMNS] = {SWITCH1, SWITCH2, SWITCH3, SWITCH4}; // rows
static const uint8_t btnGrounds[NUM_BTN_COLUMNS] = {GROUND1, GROUND2, GROUND3, GROUND4}; // columns
static int8_t debounceCount[NUM_BTN_COLUMNS][NUM_BTN_ROWS];
static int8_t notes[NUM_BTN_COLUMNS * NUM_BTN_ROWS] = { 104, 110, 117, 123, 131, 139, 147, 156, 156, 175, 185, 196, 208, 220, 233, 247 };

void setup() {
  uint8_t i;
  for (i = 0; i < NUM_BTN_COLUMNS; i++) {
    pinMode(btnGrounds[i], OUTPUT);
    // with nothing selected by default
    digitalWrite(btnGrounds[i], HIGH);
  }

  for (i = 0; i < NUM_BTN_ROWS; i++) {
    pinMode(btnInputs[i], INPUT_PULLUP);
  }

  for (uint8_t i = 0; i < NUM_BTN_COLUMNS; i++) {
    for (uint8_t j = 0; j < NUM_BTN_ROWS; j++) {
      debounceCount[i][j] = 0;
    }
  }

  MIDI.begin(MIDI_CHANNEL_OFF);
  Serial.begin(31250);
//  Serial.begin(9600);
}

static uint8_t current = 0;
//void scan() {
//  uint8_t val;
//  uint8_t i, j;
//
//  digitalWrite(btnGrounds[current], LOW);
//  delay(1);
//
//  for (j = 0; j < NUM_BTN_ROWS; j++) {
//    //val = digitalRead(btnInputs[j]);
//    val = buttons[i].read();
//    Serial.println(val);
//    if (val == 22) return;
//
//    int index = (current * NUM_BTN_ROWS) + j;
//    if (val == LOW) {
//
//      if ( debounceCount[current][j] < MAX_DEBOUNCE) {
//        debounceCount[current][j]++;
//        if ( debounceCount[current][j] == MAX_DEBOUNCE ) {
//           MIDI.sendNoteOn(notes[index], 127, 15);
//           //Serial.println("ON");
//          }
//      }
//    } else {
//      if (debounceCount[current][j] > 0) {
//        debounceCount[current][j]--;
//        if ( debounceCount[current][j] == 0 ) {
//            MIDI.sendNoteOff(notes[index], 127, 15);
//            //Serial.println("OFF");
//        }
//      }
//    }
//  }
//
//  delay(1);
//  digitalWrite(btnGrounds[current], HIGH);
//
//  current++;
//  if (current >= NUM_BTN_COLUMNS) {
//    current = 0;
//  }
//}

void scan() {
  uint8_t val;
  uint8_t i, j;

  digitalWrite(btnGrounds[current], LOW);
  delay(1);

  for (j = 0; j < NUM_BTN_ROWS; j++) {
    val = digitalRead(btnInputs[j]);
    if (val == 22) return;

    int index = (current * NUM_BTN_ROWS) + j;
    if (val == LOW) {

      if ( debounceCount[current][j] < MAX_DEBOUNCE) {
        debounceCount[current][j]++;
        if ( debounceCount[current][j] == MAX_DEBOUNCE ) {
           MIDI.sendNoteOn(notes[index], 127, 15);
           //Serial.println("ON");
          }
      }
    } else {
      if (debounceCount[current][j] > 0) {
        debounceCount[current][j]--;
        if ( debounceCount[current][j] == 0 ) {
            MIDI.sendNoteOff(notes[index], 127, 15);
            //Serial.println("OFF");
        }
      }
    }
  }

  delay(1);
  digitalWrite(btnGrounds[current], HIGH);

  current++;
  if (current >= NUM_BTN_COLUMNS) {
    current = 0;
  }
}

void loop()
{
  scan();
}





