// Define toolbox parameters (Optinal) 
#define Delimiter ' '      // Delimiter for message arguments, default is ' '. 
#define Baudrate 9600      // Baudrate for serial communication, default is 9600.

// Include the toolbox header file
#include "Toolbox_Core.h" 

void setup()  {
  Serial.begin(Baudrate); // Initialize serial communication.
  Welcome_Msg();          // Displays welcome message at startup.
}

void loop() { 
  SlaveModeLoop(); // Reads and executes commands sent by PC. 
}
