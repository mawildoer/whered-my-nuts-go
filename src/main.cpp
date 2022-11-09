#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VescUart.h>

VescUart VESC;

uint16_t UPDATE_PERIOD_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

const float current_limit = 10.0;
const float current_gain = 1.0;
const float damping = 3;
float last_current = 0;
float current = 0;

const int safety_bias_pin = 7;
const int safety_feedback_pin = 8;

bool tag_in_place() {
  return digitalRead(safety_feedback_pin) == HIGH;
}

void setup(void)
{
  // pin configuration

  pinMode(safety_feedback_pin, INPUT_PULLDOWN);
  pinMode(safety_bias_pin, OUTPUT);
  digitalWrite(safety_bias_pin, HIGH);

  // StemmaQT / Quuic Connector Pin Selection
  Wire1.setSDA(2);
  Wire1.setSCL(3);

  // Serial Port Configuration for VESC
  // TODO: confirm I don't need to do this
  // Serial1.setRX(3);
  // Serial1.setTX(2);

  // Start peripherals
  Serial.begin(115200); // USB Serial
  Serial1.begin(115200); // VESC Serial
  
  while (!Serial || !Serial1) {;} // Wait for the serial ports to come up

  VESC.setSerialPort(&Serial1);
 
  // Trap Failed Configuration
  while (!bno.begin())
  {
    Serial.println("No BNO055 detected");
    delay(1000);
  }

  Serial.println("Setup Successful!");
  delay(1000); // see if waiting for the VESC to come up helps
  Serial.println("hopefully the vesc is working now?");

  while (tag_in_place()) {
    Serial.println("unplug safety tag");
    delay(1000);
  }

  while (!tag_in_place()) {
    Serial.println("replace safety tag");
    delay(1000);
  }
  
}

void printOrientation(sensors_event_t *event)
{
  Serial.print("X: ");
  Serial.print(event->orientation.x);
  Serial.print(", Y: ");
  Serial.print(event->orientation.y);
  Serial.print(", Z: ");
  Serial.print(event->orientation.z);
}

void printCurrent(const float current)
{
  Serial.print(", Current: ");
  Serial.print(current);
}

void printVescInfo()
{
  if ( VESC.getVescValues() ) {
    Serial.print(", RPM: ");
    Serial.print(VESC.data.rpm);
    Serial.print(", Batt V: ");
    Serial.print(VESC.data.inpVoltage);
  }
}

void loop(void)
{
  unsigned long tStart = micros();

  // if the tags not in place, stop.
  while (!tag_in_place()) {
    VESC.setCurrent(0.0f);
    Serial.println("replace safety tag");
    delay(10);
  }

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // 

  printOrientation(&orientationData); // debug print out

  
  // Control loop
  if(orientationData.orientation.z > 0){
    current = -(180 - orientationData.orientation.z) * current_gain + (last_current - current) * damping;
  } else{
    current = (180 + orientationData.orientation.z) * current_gain - (last_current - current) * damping;
  }
  last_current = current;
  

  // Current Limit
  if (current > current_limit) {
    current = current_limit;
  } else if (current < -current_limit) {
    current = -current_limit;
  }

  // Fall-over detection
  if (abs(orientationData.orientation.y) > 30 || abs(orientationData.orientation.z) < 150)
  {
    current = 0;
  }

  printCurrent(current); // debug print out
  printVescInfo();
  VESC.setCurrent(current);

  Serial.println();

  // wait for the next cycle
  while ((micros() - tStart) < (UPDATE_PERIOD_MS * 1000)) {}
}
