// ============================================================ Toolbox ============================================== //
/*
  Author: Alexander Levy
  Version: 0.0.6
  Change: Re-organized the whole code. 

  Need to add some comments here explaining the purpose of the code, basically it just receives commands 
  over serial and sends back a reply msg.I pretty much do this in my free time as a tool to develop cool 
  proyects. I add features as i need them. Coding style is all over the place, sorry about that.
  Dependencies: arduino-core.
*/

/*
  Just a list of things i wanna work on at the moment, some of these will take a while
  DEV:
    [ ] Fix log system code = trash
    
  TODO:
    [ ] Add the planned commands.
    [ ] Add more communication protocols for sending and receving data
        [x] Serial
        [ ] WiFi
*/

// ================================================================= Constants ======================================== //
#define ENTER 10                                  // The ASCII value for the Enter Key.
#define Firmware_Version "0.0.6"                  // Version control metric for myself.
#define New_Msg_Arrives (Serial.available() > 0)  // Determine weather there is a new message on the serial bus.

// ================================================================== Global Variables ================================== //
bool System = false;  // State of the system, only performs commands when initialized
unsigned long Time;   // Time since the program started running 

#include "Data.h"     // Load data structures used globally
#include "Logger.h"   // Load the system logging module.
#include "Commands.h" // Load commmands used for defining the funtion call

// ================================================================ Funtion Declaration ================================== //
// Control flow funtions
void Init();          // Initialize controller (need a better way to do this since this implemntation forces me to change the header file which in not ideal)
void Shutdown();      // Shutdown system and any running process(also need a better method for this)
void RunCommand();    // Runs the command received from the PC
void ResetCommand();  // Internal Funtion used to clean the command msg (used internally)
void Parse_Command(); // Listens for a command and executes it (serial over usb)
void SlaveModeLoop(); // Controller enters the slave loop

// Hardware Funtions
void TogglePin(uint8_t pin);     // Toggle pin state
uint8_t getPinMode(uint8_t pin); // Get current pin configuration {INPUT/OUTPUT/INPUT_PULLUP}

// Exe Funtions
void Blink_Func();

// ============================================================== Funtion Implementations ============================== //
void SlaveModeLoop() {
  while(New_Msg_Arrives) { // Serial Communication is available
    Parse_Command();       // Parse msg data and execute command
  }
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
      Logger.Help_Msg();
      break;
    /*
    case Get_State:
      if(System == false) Logger.Error_Msg(); 
      else Logger.PrintSystemStatus();  
      break;
    */
    case Blink:
      if(System == false) Logger.Error_Msg(); 
      else Blink_Func();      
      break;
    
    case PinMode:
      if(System == false) Logger.Error_Msg();
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
      if(System == false) Logger.Error_Msg();
      else {
        Serial.print("OK: ");
        Logger.PrintPinValue(Msg.NumValue1);
        Serial.println("");
      }
      break;
    
    case Digital_Write:
      // Verify system is initialized
      if(System == false) {
        Logger.Error_Msg();
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
      if(System == false) Logger.Error_Msg();

      else {
        Serial.print("pin ");
        Serial.print(Msg.NumValue1);
        
        if(getPinMode(Msg.NumValue1) == 0xFF) {
          Serial.println("is NOT A PIN");
          return;
        }
        else {
          Serial.print(" is configured as: ");
          if(getPinMode(Msg.NumValue1) == 1) Serial.print("OUTPUT ");
          else if(getPinMode(Msg.NumValue1) == 0) Serial.print("INPUT  ");
          else if(getPinMode(Msg.NumValue1) == 2) Serial.print("INPUT_PULLUP ");
          return;
        }
      }
      break;

    case Toggle: 
      if(System == false) Logger.Error_Msg();

      else {
        TogglePin(Msg.NumValue1);
        Serial.print("OK: pin ");
        Serial.print(Msg.NumValue1);
        Serial.println(" was toggled.");
        Serial.println("");
      }
      return;

    case debug:
      Logger.debug_Msg(); 
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
  else if(Msg.CharIn == Config.Delimiter) {
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

void TogglePin(uint8_t pin) {
  digitalWrite(pin, !digitalRead(pin));
}
