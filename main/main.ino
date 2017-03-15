/*
   Secondary Flight Computer
   UWO Rocket Club
   Contributors:
*/

#include <ResponsiveAnalogRead.h> // Import noise reduction library

// Declare variables
int currentAGL; //current above ground level altitude
int lastAGL = 0; //last altitude reading (above ground level)
int maxAGL = -30000; // maximum altitude reading (above ground level)
int calculatedMaxAGL;
float launchAlt = 0; //altitude at launch
float temperature;
boolean isLaunched = false;
boolean apogeeDetected = false;
boolean parachuteLaunched = false;
unsigned long currentTime; //current time since Arduino started
unsigned long timeAtLaunch = 0; //time when launch was detected
unsigned long timeAtApogee = 0; //time when apogee was detected

float totalVel = 0;
float verticalVel = 0;

float pitch, roll, heading, ax, ay, az;
float totalAccel; // total current acceleration


ResponsiveAnalogRead smoothAGL(0, false); // This will remove noise from the altitude
ResponsiveAnalogRead smoothAccel(0, false); // Removes noise from acceleration


// Constants
const PROGMEM uint8_t LAUNCH_MARGIN = 2; // 2m needs to change in the positive direction before a launch to be detected
const PROGMEM uint8_t APOGEE_MARGIN = 3; // 3m needs to change from the max alt for apogee detection.
const PROGMEM uint8_t ERROR_MARGIN = 1; // 1m needs to change before it's detected
const PROGMEM uint8_t APOGEE_TIME_LIMIT = 30; // in seconds
const PROGMEM uint8_t SOUND_BARRIER = 343; // in m/s


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // initialise SD card and sensors
  initCard();
  initSensors();

  // Initialise Parachute Signal Pin and set to low.
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  // Set launch altitude to ground altitude upon start
  launchAlt = getGroundAlt();

  // Get current acceleration
  updateAccel();

  // Create log header
  writeLog(F("time,totalAccel,totalVel,verticalVel,currentAGL,lastAGL,temperature,launchAlt,isLaunched,calculatedApogee,apogeeDetected,parachuteLaunched"));

}

void loop() {

  // Only do the following if the time has increased
  if (millis() > currentTime) {
    // Get time since Arduino launch
    currentTime = millis();

    // Set current Above Ground Level to current sensor reading
    lastAGL = currentAGL;
    smoothAGL.update((int)(getAGLAlt()));
    currentAGL = (smoothAGL.getValue());

    // Get the current acceleration
    updateAccel();
    smoothAccel.update((int)(getTotalAccel() * 100));
    totalAccel = (smoothAccel.getValue()) * 0.01;

    // Get velocities
    totalVel = getVelocity(0, 0);
    verticalVel = getVelocity(0, 3);

    // Check for a launch: If the currentAGL is 2m greater than the launch margin, then the rocket has launched.
    if (currentAGL >= LAUNCH_MARGIN && !isLaunched) {
      isLaunched = true;
      timeAtLaunch = currentTime;
    }

    // If the rocket has launched, do the following:
    if (isLaunched) {

      // Set max altitude to current altitude if current altitude is higher than the maximum alt reading
      if (currentAGL >= (maxAGL + ERROR_MARGIN)) {
        maxAGL = currentAGL;
      }

      // Check for apogee
      if (currentAGL <= (maxAGL - APOGEE_MARGIN) && !apogeeDetected) {

        if (getVelocity(0, 0) > SOUND_BARRIER) {
          //TODO: logic incase we go faster than speed of sound
        }
        else {
          apogeeDetected = true; // Parachute detection can use this
          timeAtApogee = currentTime;
        }
      }
      // If apogee is not detected beyond the time limit after launch, mark apogee event
      else if (!apogeeDetected && ((currentTime - timeAtLaunch) >= (APOGEE_TIME_LIMIT * 1000))) {
        apogeeDetected = true;
        timeAtApogee = currentTime;
      }

      // Send signal to pin 2 for backup release of parachute IF 5 seconds has elasped since apogee
      if (apogeeDetected && (currentTime >= (timeAtApogee + 50000))) {
        digitalWrite(2, HIGH);
        parachuteLaunched = true;
      }

      // Calculate suspected apogee
      if (!apogeeDetected) {
        // calculation goes here
        // alt = v^2 - v0^2 / 2a - if the rocket is coasting
        calculatedMaxAGL = ((pow(getVelocity(0, 3), 2)) / ((2 * az))) + currentAGL;
      }
    }
    else {
      timeAtLaunch = currentTime;
    }
  }
  // write data to Log
  writeLog(String(currentTime) + F(",") + String(totalAccel) + F(",") + String(totalVel) + F(",") + String(verticalVel) +
           F(",") + String(currentAGL) + F(",") + String(lastAGL) + F(",") + String(temperature) + F(",") + String(launchAlt) +
           F(",") + String(isLaunched) + F(",") + String(calculatedMaxAGL) + F(",") + String(apogeeDetected) + F(",") + String(parachuteLaunched));
}

// takes 100 samples and selects the lowest as ground altitude
float getGroundAlt() {
  float lowestAlt = 30000;
  float sampleAlt = 0;
  for (int i = 0; i < 100; i++) {
    if (altitude_temperature_sensor(sampleAlt, temperature));
    else sampleAlt = 0;
    if (sampleAlt < lowestAlt)
      lowestAlt = sampleAlt;
  }
  return lowestAlt;
}

// Get the altitude above ground level
float getAGLAlt() {
  float altitude;
  if (altitude_temperature_sensor(altitude, temperature)) {
    return altitude - launchAlt;
  }
  return 0;
}

// get the velocity.  0 = total; 1 = x; 2 = y; 3 = z;
float getVelocity(float v0, byte direction) {
  float accel;
  switch (direction) {
    case 0: accel = totalAccel;
    case 1: accel = ax;
    case 2: accel = ay;
    case 3: accel = az;
  }
  return (v0 + ((0.5) * (accel) * (pow(((currentTime - timeAtLaunch) * 0.001), 2))));
}

// Update acceleration from sensors
void updateAccel() {
  if (accelerometer_sensor(pitch, roll, heading, ax, ay, az));
  else Serial.println("Accelerometer update failed!");
}

// get the current total acceleration
float getTotalAccel() {
  return sqrt((pow(ax, 2) + pow(ay, 2) + pow(az, 2)));
}

//void get_accel() {
//  float pitch, roll, heading, ax, ay, az, atotal;
//  if(accelerometer_sensor(pitch,roll,heading,ax,ay,az)) {
//  Serial.print(F("pitch: "));
//  Serial.print(pitch);
//  Serial.print(F(";  roll: "));
//  Serial.print(roll);
//  Serial.print(F(";  heading: "));
//  Serial.print(heading);
//  Serial.print(F(";  ax: "));
//  Serial.print(ax);
//  Serial.print(F(";  ay: "));
//  Serial.print(ay);
//  Serial.print(F(";  az: "));
//  Serial.print(az);
//  atotal = sqrt((pow(ax,2)+pow(ay,2)+pow(az,2)));
//  Serial.print(F(";  total: "));
//  Serial.println(atotal);
//  }
//}

