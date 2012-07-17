#include <SoftwareSerial.h>
#define DEBUG 0

// Command variables
#define BUFFER_SIZE 20
char buffer[BUFFER_SIZE];
char input_char = -1;
byte index = 0;
SoftwareSerial serial1(10, 11);

// Timing variables
unsigned long start, loop_time;

// Encoder variables
#define ENCODER0_PINA 2
#define ENCODER0_PINB 3
volatile int count_a = LOW;
volatile int state_b = LOW;
volatile int encoder_count = 0;

// Control variables
#define LOOP_PERIOD 10000 // ms
byte desired_motor_command = 127;

void setup() {
  pinMode (ENCODER0_PINA, INPUT); 
  pinMode (ENCODER0_PINB, INPUT);
  Serial.begin(115200);
  serial1.begin(9600);
  attachInterrupt(1, onCountA, CHANGE);
  attachInterrupt(0, onStateB, FALLING);
  Serial.println("System Ready");
  serial1.write(desired_motor_command);
}

void parse_buffer() {
  int power = atoi(buffer);
  power = constrain(power, -127, 127);
  desired_motor_command = (power < 0 ? 127 : 128) + power;
}

void loop() {
  /* Start timing */
  start = micros();
  /* Do work */
  // Report the encoder count
  Serial.println(encoder_count/5.0);
  // Check for new data
  while (Serial.available() > 0) {
    // Something to read, buffer it
    if (index == BUFFER_SIZE - 1) {
      index = 0;
    }
    buffer[index] = Serial.read();
    if (buffer[index] == '\n') {
      buffer[index] = '\0';
      parse_buffer();
      serial1.write(desired_motor_command);
      index = 0;
      break;
    } 
    else {
      index++;
    }
  }
  /* Stop Timing */
  loop_time = micros() - start;
  if (loop_time < LOOP_PERIOD) {
    unsigned long diff = LOOP_PERIOD - loop_time;
    if (diff / 1000 > 0) {
      delay(diff / 1000);
    }
    delayMicroseconds(diff % 1000);
  }
}

void onCountA() {
  int previous_a = count_a;
  count_a = digitalRead(ENCODER0_PINA);
  if ((previous_a == LOW) && (count_a == HIGH)) { 
    if (state_b == LOW) { 
      encoder_count--; 
    } 
    else { 
      encoder_count++; 
    } 
  }
}

void onStateB() {
  state_b = digitalRead(ENCODER0_PINB);
}
