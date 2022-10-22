#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

void setup(void)
{
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting BNO055 test");
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  if (!bno.begin())
  {
    while (1) {
      Serial.println("No BNO055 detected");
      delay(1000);
    }
  }

  Serial.println("Found BNO055!");
  delay(1000);
}

void loop(void)
{
  //
  unsigned long tStart = micros();
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  Serial.print("Heading: ");
  Serial.println(orientationData.orientation.x);
  Serial.println(orientationData.orientation.y);
  Serial.println(orientationData.orientation.z);

  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
}
