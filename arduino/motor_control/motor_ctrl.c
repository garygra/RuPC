#include <Boards.h>
#include <Firmata.h>
#include <FirmataConstants.h>
#include <FirmataDefines.h>
#include <FirmataMarshaller.h>
#include <FirmataParser.h>

const int a_in1_pin = 6;
const int a_in2_pin = 5;

const int led_debug = 13;

const int nfault_pin = 2;
const int nsleep_pin = 7;

uint16_t a_input_value = 0;
uint8_t ain1 = 0;
uint8_t ain2 = 0;

bool is_connected = false;
void setup() {
  pinMode(nsleep_pin, OUTPUT);
  pinMode(nfault_pin, INPUT);
  digitalWrite(nsleep_pin, LOW);
  Serial.begin(115200);
  digitalWrite(led_debug, HIGH);
  //  while (!is_connected) {
  //
  //    wait_for_bytes(1, 1000);
  //    if(Serial.available() > 0)
  //    {
  //      digitalWrite(led_debug, !digitalRead(led_debug));
  //      is_connected = read_i8() > 0;
  //    }
  //  }
  delay(1000);

  // put your setup code here, to run once:
  pinMode(a_in1_pin, OUTPUT);
  pinMode(a_in2_pin, OUTPUT);
  pinMode(led_debug, OUTPUT);
  digitalWrite(nsleep_pin, HIGH);
  digitalWrite(led_debug, LOW);
  //  digitalWrite(a_in1_pin, HIGH);
  //  digitalWrite(a_in2_pin, HIGH);
  //  Serial.println("Setup done!");
}

void loop() {
  // put your main code here, to run repeatedly:
  //(122-48)*3.44
  if (Serial.available() > 0) {
    a_input_value = read_i16();
    ain1 = (uint8_t)(a_input_value >> 8);
    ain2 = (uint8_t)(a_input_value & 0x00ff);

    //    analogWrite(a_in1_pin, ain1);
    //    analogWrite(a_in2_pin, ain2);

    //    digitalWrite(led_debug, HIGH);
  }
  digitalWrite(a_in1_pin, ain1 > 0);
  digitalWrite(a_in2_pin, ain2 > 0);
  digitalWrite(led_debug, !digitalRead(nfault_pin));

  wait_for_millis(unsigned long 100);
  ain1 = 0;
  ain2 = 0;
}

void wait_for_millis(unsigned long timeout) {
  const unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
  }
}

void wait_for_bytes(int num_bytes, unsigned long timeout) {
  unsigned long startTime = millis();
  // Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)) {
  }
}

int8_t read_i8() {
  wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t)Serial.read();
}

int16_t read_i16() {
  int8_t buffer[2] = {0, 0};
  wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
  read_signed_bytes(buffer, 2);
  return (((int16_t)buffer[0]) & 0xff) | (((int16_t)buffer[1]) << 8 & 0xff00);
}

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t *buffer, size_t n) {
  size_t i = 0;
  int c;
  while (i < n) {
    c = Serial.read();
    if (c < 0)
      break;
    *buffer++ = (int8_t)c; // buffer[i] = (int8_t)c;
    i++;
  }
}
