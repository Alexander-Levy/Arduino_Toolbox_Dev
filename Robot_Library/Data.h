// ================================================================================================================== //
//                                                      Data Structures                                               //
// ================================================================================================================== //

// ====================================================== Parameters ================================================ //
struct Parameters {
  long Baudrate = 9600;         // Default serial communication rate
  char Delimiter = ' ';         // Default character used to delimit arguments
  //int Num_of_Analog_Pins = 6;   // Number of analog pins the microcontroller has
  //int Num_of_Digital_Pins = 14; // Number of digital pins the microcontroller has
  //String Microntroller = "Arduino Uno"; // Not in use 
};

struct Parameters Config; // Used to store system parameters.

// ======================================================= Message ================================================= //
struct Message {
  char CharIn;                   // Character being read 
  char Command;                  // Command sent to controller
  char Value1[16];               // Values for more complex commands 
  char Value2[16];               // Values for more complex commands
  uint8_t Index = 0;             // Position of the character in the message
  uint8_t value_pos = 0;         // Position of the char in the value string.
  uint32_t NumValue1, NumValue2; // Values converted to numbers
  //unsigned long TimeStamp = 0;   // Time the msg was received (not in used, planned for future)
};
// Note: plan to change this to a class and include some on the global funtions as methods.

struct Message Msg;       // Used to parse and store the received message.
