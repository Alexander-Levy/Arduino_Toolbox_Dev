// ============================================================ Toolbox ============================================== //
/*
  Author: Alexander Levy
  Version: 0.0.3
  Change: Added the toggle command. 

  Need to add some comments here explaining the purpose of the code, basically it just receives commands 
  over serial and sends back a reply msg.I pretty much do this in my free time as a tool to develop cool 
  proyects. I add features as i need them. Coding style is all over the place, sorry about that.
  Dependencies: arduino-core.
*/

/*
  Just a list of things i wanna work on at the moment, some of these will take a while
  DEV:
    [ ] Add preprocesor for different microcontrollers
    [ ] Add log system
    
  TODO:
    [ ] Add the planned commands.
    [ ] Add option to work with Time stamps 
    [ ] Make Message a struct instead of a class.
    [ ] Add more communication protocols for sending and receving data
        [x] Serial
        [ ] I2C
        [ ] WiFi
        [ ] SPI (maybe?)
    [ ] Port to ESP-32 family.
*/

// ================================================================= Libraries ======================================= //
/*
  #include "Arduino.h"         // Must be included if you're not using the Arduino IDE (im basic like that)
*/

// ================================================================= Constants ======================================== //
#define ENTER 10                                  // The char value for the Enter Key.
#define Firmware_Version "0.0.3"                  // Version control metric for myself.
#define New_Msg_Arrives (Serial.available() > 0)  // Determine weather there is a new message on the serial bus.


// =================================================================== Macros ======================================== //
// Define the default baudrate for serialcommunication.
#ifndef Baudrate
  #define Baudrate 9600
#endif

// Define the default delimitir to use when parsing the message.
#ifndef Delimiter
  #define Delimiter ' '
#endif

/* // Define the default board as AtMega328p.
#ifndef Board
  #define Board ATMega328p 
#endif

// Define the number of pins based on the board type.
  // Arduino Uno/Nano.
  #if Board == "Nano" 
    #define Num_of_Digital_Pins 14 // Number of digital I/O pins
    #define Num_of_Analog_Pins 6   // Number of analog pins
    #define LED_BUILTIN 13         // Pin that the built-in led is connected to.

  // Arduino Mega.
  #elif Board == "ATMega2560" 
    #define Num_of_Digital_Pins 54 // Number of digital I/O pins
    #define Num_of_Analog_Pins 16  // Number of analog pins
    #define LED_BUILTIN 13         // Pin that the built-in led is connected to.
  
  #endif
// Note: Need to implement and test esp32 board. */

// ================================================================== Commands ======================================= //
// Used to define the state of the system, it will only execute commands if initialized.
#define Initialize_System  'i'   // Initializes the system
#define Shutdown_System    'c'   // Shutdown the system

// Provide system information 
#define Help               'h'   // Shows command list and program information
#define Get_State          'q'   // Shows debug info

// Test command
#define Blink              'b'   // Blinks Led for 3 seconds

// Used to configure, read and write to GIO pins
#define Toggle             't'   // Invert pin state
#define PinMode            'p'   // Set pin as Input[0] or Output[1]
#define GetPinMode         'g'   // Read pin configuration
#define Digital_Read       'd'   // Read pin logic level
#define Digital_Write      'w'   // Set pin as LOW[0] or HIGH[1]

// Used internally for testing and debuging
#define debug              '.'   // Used for checking messages, dev use only, will be removed in the future.

/*
  Planned commands 
    Analog_Write
    Analog_Read
    Set_PWM_Value
    Set_PWM_Frecuency
*/

// ================================================================== Data Structures ================================== //
struct Message {
  char CharIn;                   // Character being read 
  char Command;                  // Command sent to controller
  char Value1[16];               // Values for more complex commands 
  char Value2[16];               // Values for more complex commands
  uint8_t Index = 0;             // Position of the character in the message
  uint8_t value_pos = 0;         // Position of the char in the value string.
  uint32_t NumValue1, NumValue2; // Values converted to numbers
};
// Note: plan to change this to a class and include some on the global funtions as methods.

// ================================================================== Global Variables ================================== //
// These keep track of program flags and time passed.
bool System = false;           // State of the system, only performs commands when initialized
unsigned long Time;            // Time since the program started running 

// Used to parse and store the received message.
struct Message Msg;

// ================================================================ Funtion Declaration ================================== //
// Control flow funtions
void RunCommand();                // Runs the command received from the PC
void ResetCommand();              // Internal Funtion used to clean the command msg (used internally)
void Parse_Command();             // Listens for a command and executes it (serial over usb)
void SlaveModeLoop();             // Controller enters the slave loop
void Init();                      // Initialize controller (need a better way to do this since this implemntation forces me to change the header file which in not ideal)
void Shutdown();                  // Shutdown system and any running process(also need a better method for this)

// Printing funtions
void Welcome_Msg();               // Prints the welcome message sent when the controller boots up 
void Error_Msg();                 // Prints Error message
void Help_Msg();                  // Prints helpfull information and list of commands
void debug_Msg();                 // Prints the inputed parsed messaged
void Print_Input_Msg();           // Prints the msg recived
void PrintPinStatus(uint8_t pin); // Prints pin pinmode configuration
void PrintPinValue(uint8_t pin);  // Prints pin value 
void PrintSystemStatus();         // Prints controller I/O information

// Hardware Funtions
void TogglePin(uint8_t pin);      // Toggle pin state
uint8_t getPinMode(uint8_t pin);  // Get current pin configuration {INPUT/OUTPUT/INPUT_PULLUP}

// Exe Funtions
void Blink_Func();

// ============================================================== Funtion Implementations ============================== //
void SlaveModeLoop() {
  while(New_Msg_Arrives) { // Serial Communication is available
    Parse_Command();       // Parse msg data and execute command

  }
}

void Welcome_Msg() {
  Serial.print("Welcome to prototype software version ");
  Serial.println(Firmware_Version);
  Serial.println("Press h for help.");
  Serial.println(" ");
}

void RunCommand() {
  // Convert string values to integers
  Msg.NumValue1 = atoi(Msg.Value1);
  Msg.NumValue2 = atoi(Msg.Value2);

  // Run command with args
  switch(Msg.Command) {
    case Initialize_System:
      Init();
      break;
    
    case Shutdown_System:
      Shutdown();
      break;
    
    case Help:
      Help_Msg();
      break;
    
    case Get_State:
      if(System == false) Error_Msg(); 
      else PrintSystemStatus();  
      break;
    
    case Blink:
      if(System == false) Error_Msg(); 
      else Blink_Func();      
      break;
    
    case PinMode:
      if(System == false) Error_Msg();
      else {
        if((Msg.NumValue2 < 0) && (Msg.NumValue2 > 2)) {
          Serial.println("Pinmode value not supported, set pinmode as 0 for input, 1 for output or 2 for input_pullup."); 
          break;
        }
        else if(Msg.NumValue2 == 0) pinMode(Msg.NumValue1, INPUT);
        else if(Msg.NumValue2 == 1) pinMode(Msg.NumValue1, OUTPUT);
        else if(Msg.NumValue2 == 2) pinMode(Msg.NumValue1, INPUT_PULLUP);
        Serial.println("OK: Pin mode has been changed successfully.");
      }
      break;
    
    case Digital_Read:
      if(System == false) Error_Msg();
      else {
        Serial.print("OK: ");
        PrintPinValue(Msg.NumValue1);
        Serial.println("");
      }
      break;
    
    case Digital_Write:
      // Verify system is initialized
      if(System == false) {
        Error_Msg();
        break; 
      }
      // Verify pin is configured as output.
      else if(getPinMode(Msg.NumValue1) != 1) {
        Serial.println("Error: pin is not configured to output, unable to write value.");
        break;
      }
      // Verify assigned value is valid {LOW[0] or HIGH[1]}
      else if((Msg.NumValue2 != 0) && (Msg.NumValue2 != 1)) {
        Serial.println("Pin value not supported, set value as 0 for LOW or 1 for HIGH.");
        break;
      }

      else { 
        digitalWrite(Msg.NumValue1, Msg.NumValue2);
        Serial.print("OK: ");
        digitalWrite(Msg.NumValue1, Msg.NumValue2);
        Serial.print("pin ");
        Serial.print(Msg.NumValue1);
        Serial.print(" was set to  ");
        if(Msg.NumValue2 == 0) Serial.println("LOW.");
        else if(Msg.NumValue2 == 1) Serial.println("HIGH.");
      }
      break;
    
    case GetPinMode:
      if(System == false) Error_Msg();

      else {
        Serial.print("OK: ");
        PrintPinStatus(Msg.NumValue1);
        Serial.println("");
      }
      break;
    
    case Toggle: 
      if(System == false) Error_Msg();

      else {
        TogglePin(Msg.NumValue1);
        Serial.print("OK: pin ");
        Serial.print(Msg.NumValue1);
        Serial.println(" was toggled.");
        Serial.println("");
      }
      return;

    case debug:
      debug_Msg(); 
      break;

    default:
      Serial.println("Command not found, press h for help.");
      Serial.println(""); 
      return;
  }
}

void Parse_Command() {
  Msg.CharIn = Serial.read(); // Read the serial bus

  // Check for message end and perform the command
  if(Msg.CharIn == ENTER) {
    if(Msg.Index == 1) Msg.Value1[Msg.value_pos] = NULL;      // Terminate the value 1 string
    else if(Msg.Index == 2) Msg.Value2[Msg.value_pos] = NULL; // Terminate the value 2 string
    // Used for development, testing and debugging.
    //Print_Input_Msg();  
    // Run command and clear the memory.
    RunCommand();       
    ResetCommand();       
  }

  // Parse message info based on spaces
  else if(Msg.CharIn == Delimiter) {
    // Start reading the first argument.
    if(Msg.Index == 0) Msg.Index = 1;
    // Start reading the second argument.
    else if(Msg.Index == 1) {
      Msg.Value1[Msg.value_pos] = NULL; // Terminate value 1 string
      Msg.Index = 2;              
      Msg.value_pos = 0;                // Reset value position so it can be used by value 2.
    }
  }

  // Assign the parsed values 
  else {
    // Parse the command 
    if(Msg.Index == 0) Msg.Command = Msg.CharIn;
    // Parse the first argument  
    else if(Msg.Index == 1) {
      Msg.Value1[Msg.value_pos] = Msg.CharIn;
      Msg.value_pos++;
    }
    // Parse the second argument 
    else if(Msg.Index == 2) {
      Msg.Value2[Msg.value_pos] = Msg.CharIn;
      Msg.value_pos++;
    }  
  }

}

void ResetCommand() {
  // Clean the information of the last command. 
  Msg.Index = 0;
  Msg.Command = 0;
  Msg.value_pos = 0;
  Msg.NumValue1 = 0;
  Msg.NumValue2 = 0;

  // Clean the memory of the arrays
  memset(Msg.Value1, 0, sizeof(Msg.Value1)); 
  memset(Msg.Value2, 0, sizeof(Msg.Value2)); 
}

uint8_t getPinMode(uint8_t pin) {
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return -1;

	reg = portModeRegister(port);
	out = portOutputRegister(port);
  
  // Check for tris (DDxn) is output
  if( (*reg) & bit ) return 1; //OUTPUT
  
  else{ //Is input, Check state (PORTxn)
    if( (*out) & bit ) return 2; // INPUT_PULLUP, bit is set
    else return 0;               // INPUT, bit is clear
  }
}

void PrintPinStatus(uint8_t pin) {
  Serial.print("pin ");
  Serial.print(pin);

  if(getPinMode(pin) == 0xFF) {
    Serial.println("is NOT A PIN");
    return;
  }

  else {
    Serial.print(" is configured as: ");
    if(getPinMode(pin) == 1) Serial.print("OUTPUT ");
    else if(getPinMode(pin) == 0) Serial.print("INPUT  ");
    else if(getPinMode(pin) == 2) Serial.print("INPUT_PULLUP ");
    return;
  }
} 

void PrintPinValue(uint8_t pin) {
  Serial.print("pin ");
  Serial.print(pin);
  Serial.print(" is ");
  if(digitalRead(pin) == 0) Serial.println("LOW");
  else Serial.println("HIGH");
}

void Error_Msg() {
  Serial.println("Error: The controller is not initiliazed, unable to perform command.");
  Serial.println(" ");
}

void Help_Msg() {
  Serial.println("System Information");
  Serial.println("\tFirmware status: early development.");
  Serial.print("\tFirmware version: ");
  Serial.println(Firmware_Version);
  Serial.println("\tSupported Platforms: Arduino(Uno/Mega/Nano).");
  Serial.print("\tBaudrate: ");
  Serial.println(Baudrate);
  Serial.println(" ");
  Serial.println("List of commands: ");
  Serial.println("  -h: press for help and software info.");
  Serial.println("  -i: press to initialize the system.");
  Serial.println("  -c: press to shutdown the system.");
  Serial.println("  -q: press to get system status info.");
  Serial.println("  -b: press to blink the builtin led.");
  Serial.println("  -d: press to read pin status(Digital read)");
  Serial.println("       -example: 'd 5' reads pihn 5.");
  Serial.println("  -w: press to set pin as LOW[0] or HIGH[1](Digital Write)");
  Serial.println("       -example: 'x 5 0' sets pin 5 as LOW.");
  Serial.println("  -p: press to set pin as either input[0], output[1] or input_pullup[2].");
  Serial.println("       -example: 'p 5 0' sets pin 5 as an input.");
  Serial.println("  -g: press to get the pinmode of the pin.");
  Serial.println("       -example: 'g 5' gets the pinmode of pin 5.");
  Serial.println(" ");
}

void PrintSystemStatus() {
  // General system information
  Serial.println("System information. ");
  Serial.print("\tThe state of the system is: ");
  if(digitalRead(LED_BUILTIN) == HIGH) Serial.println("Initialized.");
  else Serial.println("Shutdown.");
  //Serial.println("");

  /*   // Microntroller info
  Serial.print("\tMicrocontroller: ");
  // Verify the board type
  #if Board == ATMega328p
    Serial.println("ATMega328p Arduino Uno/Nano");
  #endif

  #if Board == ATMega2560
    Serial.println("ATMega2560 Arduino Mega 2560");
  #endif

  Serial.println("") */;

  // Time since program started.
  Serial.print("\tTime since boot: ");
  Serial.print((float)Time/1000);
  Serial.println(" seconds.");
  Serial.println("");
  
  // Digital pins status
  Serial.println("Digital pins status: ");
  for(uint8_t i = 0; i < 54; i++) {
    Serial.print("\t");
    PrintPinStatus(i);
    PrintPinValue(i);
  }
  Serial.println("");

  // Analoge Pins
  //write stuff here 
}

void Init() {
  System = true;
  Time = millis();
  Serial.println("System initializing...");
  // Init Funtion
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Initialization successful!");
  Serial.println(" ");
}

void Shutdown() {
  System = false;
  Serial.println("Closing processes...");
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Process stopped, system is in sleep mode.");
  Serial.println(" ");
}

void Blink_Func() {
  Serial.println("OK: Blinking builtin led.");
  for(uint8_t i = 0; i <= 3; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    Serial.print(".");  
  }
  Serial.println("Led blinking done."); 
}

void debug_Msg() {
  Serial.print("The received message is: ");
  Serial.print(Msg.Command);
  Serial.print(Delimiter);
  Serial.print(Msg.NumValue1);
  Serial.print(Delimiter);
  Serial.println(Msg.NumValue2); 
  Serial.println("");
}

void Print_Input_Msg() {
  Serial.print(Msg.Command);
  Serial.print(Delimiter);
  Serial.print(Msg.NumValue1);
  Serial.print(Delimiter);
  Serial.println(Msg.NumValue2); 
}

void TogglePin(uint8_t pin) {
  digitalWrite(pin, !digitalRead(pin));
}

