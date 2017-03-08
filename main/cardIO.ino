/*
   Card Input/Output & Logging
   UWO Rocket Club
   Contributors: Conor Dunne


   Everytime the flight computer restarts, there will be a new log file on the SD card.
   The name of the log files will be f#.log, where # is the number of files on the
   SD card.

   You can write to the log simply by calling the writeLog() function, where the
   parameter is a string.


*/
#include <SPI.h> // import Arduino Serial Peripheral Interface (SPI) library
#include <SD.h> // import Arduino SD library


// Sparkfun MicroSD breakout Hookup:
// VCC = 5V (ZERO uses 3.3V)
// GND = GND
/*int int sck = 13; //SCLK, CLK
  int d0 = 12; //MISO
  int d1 = 11; //MOSI
  int cd = 9; */
const PROGMEM uint8_t cs = 8; //*/
String logFileName;


// Function to initialise card. Must be called in setup()
void initCard() {
  Serial.print (F("Initialising SD Card... "));
  if (!SD.begin(cs)) {
    Serial.println(F("Initialisation failed!"));
    return;
  }
  Serial.println(F("Initialisation done."));

  /* // test file write
  File test = SD.open("test.log", FILE_WRITE);
  if (test) { // if the file opens
    Serial.println("Writing to test.log...");
    test.println("Test 1, 2, 3. INDEED.");
    test.close(); // close file
    Serial.println("Done!");
  } else { // if the file fails to open
    Serial.println("Error opening test.log");
  }

  // test file read
  test = SD.open("test.log");
  if (test) {
    Serial.println("test.log:");
    //read from file to eof
    while (test.available()) {
      Serial.write(test.read());
    }
    test.close(); //close file
  } else {
    Serial.println("Error opening test.log");
  }//*/

  // Check memory buffer space
  //freeMem();

  // Count number of files
  int count = 0;
  File root = SD.open(F("/"));
  while (true) {
    File index = root.openNextFile();
    count++;
    if (! index) {
      break; // no more files
    }
    index.close();
  }
  logFileName = "f" + String(count) + ".csv";
  Serial.print(F("\nLatest logfile: "));
  Serial.println(logFileName);
}
  
// Writes strings to latest log file (f#.log). Parameter is a string.
// Format of log (in CSV): milliseconds,(Data here seperated by commas)
void writeLog(String text) {
  File LOG = SD.open(logFileName, FILE_WRITE);
  if (LOG) {
    Serial.println(text);
    LOG.println(text);
    LOG.close();
  } else {
    Serial.println(F("ERROR: Writing to log failed!"));
  }
}

// Function determines how much free memory there is after compiling. If below 500 bytes, logging may fail. 
//uint16_t freeMem() {
//  char top;
//  extern char *__brkval;
//  extern char __bss_end;
//  Serial.print(F("Free memory: "));
//  Serial.print( __brkval ? &top - __brkval : &top - &__bss_end);
//  Serial.println(F(" bytes"));
//}
