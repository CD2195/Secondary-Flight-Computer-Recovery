/*
   Secondary Flight Computer
   UWO Rocket Club
   Contributors:
*/

// Declare variables
float currentAGL; //current above ground level altitude
float lastAGL = 0; //last altitude reading (above ground level)
float temperature; 
float launchAlt = 0; //altitude at launch 
boolean isLaunched = false;
unsigned long currentTime = millis(); //current time since Arduino started
unsigned long timeAtLaunch = 0; //time since launch was detected


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
  
  // Set current Above Ground Level to current sensor reading
  currentAGL = getAGLAlt();
  
  // get current time since Arduino launch
  currentTime = millis();
  
    // If current AGL is 6m or above, then launch has been detected and record the time at launch	
    if(currentAGL >= 6 && !isLaunched) { //signifies that the rocket has launched (6 meters)
      isLaunched = true;
      timeAtLaunch = millis();
    }

    // If the rocket has launched
    if(isLaunched) {

    	// If the rocket has reached apogee
        if(currentAGL < lastAGL) {
          
          //TODO: logic incase we go faster than speed of sound
          //print apogee for testing
          //TODO: after testing fire signal to recovery
          
        }
	
	// Otherwise, continue and record current AGL to compare for the next iteration
        else {
          lastAGL = currentAGL;
        }
    }
  
  
  // write data to Log
  writeLog(String(currentTime)+","+String(currentAGL)+","+String(lastAGL)+","+String(temperature)+","+
    String(launchAlt)+","+String(isLaunched)+","+String(timeAtLaunch));

}

// takes 100 samples and selects the lowest as ground altitude
float getGroundAlt() {
  float lowestAlt = 30000;
  float sampleAlt = 0;
  for(int i = 0; i < 100; i++) {
    sampleAlt = get_alt();
    if(sampleAlt < lowestAlt) 
      lowestAlt = sampleAlt;
  }
  return lowestAlt;
}

// Get the altitude above ground level
float getAGLAlt() {
  return  get_alt() - launchAlt;
}

//uint16_t freeMem() {
//  char top;
//  extern char *__brkval;
//  extern char __bss_end;
//  Serial.println( __brkval ? &top - __brkval : &top - &__bss_end);
//}
