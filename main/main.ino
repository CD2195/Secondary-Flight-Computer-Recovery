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
float temperature;
float launchAlt = 0; //altitude at launch
boolean isLaunched = false;
boolean apogeeDetected = false;
unsigned int currentTime; //current time since Arduino started
unsigned int timeAtLaunch = 0; //time when launch was detected
unsigned int timeAtApogee = 0; //time when apogee was detected
float velocity = 0; // current vertical velocity with respect to the ground


ResponsiveAnalogRead smoothAGL(0, false); // This will remove noise from the altitude


// Constants
const int LAUNCH_MARGIN = 2; // 2m needs to change in the positive direction before a launch to be detected
const int APOGEE_MARGIN = 3; // 3m needs to change from the max alt for apogee detection.
const int ERROR_MARGIN = 1; // 1m needs to change before it's detected

const int MASS = 1; // mass of the rocket in kg
const int AREA = 1; // cross-sectional area of the rocket in m^2


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // initialise SD card and sensors
  initCard();
  initSensors();

  // Set launch altitude to ground altitude upon start
  launchAlt = getGroundAlt();

  // Check Memory. If memory is less than 700, the log may fail.
  //The SD class requires a buffer of 500 bytes, plus another 200 for the stack
  //freeMem();

  // Create log header
  writeLog(F("time,currentAGL,lastAGL,temperature,launchAlt,isLaunched,timeAtLaunch"));

}

void loop() {

  // Only do the following if the time has increased
  if (millis() > currentTime) {
    // Get time since Arduino launch
    currentTime = millis();

    // Set current Above Ground Level to current sensor reading
    lastAGL = currentAGL;
    smoothAGL.update((int)getAGLAlt());
    currentAGL = smoothAGL.getValue();

    // Check for a launch: If the currentAGL is 2m greater than the launch margin, then the rocket has launched.
    if (currentAGL >= LAUNCH_MARGIN && !isLaunched) {
      isLaunched = true;
      timeAtLaunch = currentTime;
    }

    // If the rocket has launched, do the following:
    if (isLaunched) {

      // Get the current velocity (will become negative if apogee is detected)
      if (apogeeDetected) {
        velocity = -1 * getVelocity();
      }
      else {
        velocity = getVelocity();
      }

      // Set max altitude to current altitude if current altitude is higher than the maximum alt reading
      if (currentAGL >= (maxAGL + ERROR_MARGIN)) {
        maxAGL = currentAGL;
      }

      // Check for apogee
      if (currentAGL <= (maxAGL - APOGEE_MARGIN) && !apogeeDetected) {
        apogeeDetected = true; // Parachute detection can use this
        timeAtApogee = currentTime;
        //TODO: logic incase we go faster than speed of sound
      }

      // Calculate suspected apogee
      if (!apogeeDetected) {
        // calculation goes here
        calculatedMaxAGL = 10000;
      }

      
    }
  }
  // write data to Log
  writeLog(String(currentTime) + F(",") + String(currentAGL) + F(",") + String(lastAGL) + F(",") + String(temperature) + F(",") +
           String(launchAlt) + F(",") + String(isLaunched) + F(",") + String(timeAtLaunch) + F(",") + String(maxAGL) + F(",") + String(apogeeDetected));

}

// takes 100 samples and selects the lowest as ground altitude
float getGroundAlt() {
  float lowestAlt = 30000;
  float sampleAlt = 0;
  for (int i = 0; i < 100; i++) {
    sampleAlt = get_alt();
    if (sampleAlt < lowestAlt)
      lowestAlt = sampleAlt;
  }
  return lowestAlt;
}

// Get the altitude above ground level
float getAGLAlt() {
  return  get_alt() - launchAlt;
}

// get the current velocity
float getVelocity() {
  return (currentAGL / (currentTime - timeAtLaunch));
}


//uint16_t freeMem() {
//  char top;
//  extern char *__brkval;
//  extern char __bss_end;
//  Serial.println( __brkval ? &top - __brkval : &top - &__bss_end);
//}
