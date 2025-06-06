// Include the toolbox header file
#include "Toolbox_Core.h" 

void setup()  {
  /* // Configure the system parameters (optional)
  Config.Delimiter = '.';   // default value is ' '
  Config.Baudrate = 57600;  // default value is 9600 
                            // default values can be changed in data.h
  */
  
  Serial.begin(Config.Baudrate); // Initialize serial communication.
  Logger.Welcome_Msg();          // Displays welcome message at startup.
}

void loop() { 
  SlaveModeLoop(); // Reads and executes commands sent by PC. 
}
