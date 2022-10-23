#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VescUart.h>

VescUart VESC;

uint16_t UPDATE_PERIOD_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

const float current_limit = 10.0;
const float current_gain = 0.5;

void setup(void)
{
  // PIO configuration

  // StemmaQT / Quuic Connector Pin Selection
  Wire1.setSDA(2);
  Wire1.setSCL(3);

  // Serial Port Configuration for VESC
  // Serial1.setRX(3);
  // Serial1.setTX(2);

  // Start peripherals
  Serial.begin(115200); // USB Serial
  Serial1.begin(115200); // VESC Serial
  
  while (!Serial || !Serial1) {;} // Wait for the serial ports to come up

  VESC.setSerialPort(&Serial1);
 
  // Trap Failed Configuration
  if (!bno.begin())
  {
    while (1) {
      Serial.println("No BNO055 detected");
      delay(1000);
    }
  }

  Serial.println("Setup Successful!");
}

void printOrientation(sensors_event_t *event)
{
  Serial.print("Orientation: ");
  Serial.print("X: ");
  Serial.print(event->orientation.x);
  Serial.print(", Y: ");
  Serial.print(event->orientation.y);
  Serial.print(", Z: ");
  Serial.print(event->orientation.z);
  Serial.println();
}

void printCurrent(const float current)
{
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println("A");
}

void loop(void)
{
  //
  unsigned long tStart = micros();
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // 

  printOrientation(&orientationData); // debug print out

  // Control loop
  float current = orientationData.orientation.z * current_gain;

  // Current Limit
  if (current > current_limit) {
    current = current_limit;
  } else if (current < -current_limit) {
    current = -current_limit;
  }

  // Fall-over detection
  if (max(abs(orientationData.orientation.y), abs(orientationData.orientation.z)) > 30)
  {
    current = 0;
    Serial.println("Ahhh!");
  }

  printCurrent(current); // debug print out
  VESC.setCurrent(current);

  // wait for the next cycle
  while ((micros() - tStart) < (UPDATE_PERIOD_MS * 1000)) {}
}
