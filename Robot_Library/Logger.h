// ================================================================================================================== //
//                                                           Logger                                                   //
// ================================================================================================================== //



// ======================================================= Declaration ============================================== //
class Logger {
  public:
    void Welcome_Msg();               // Prints the welcome message sent when the controller boots up
    void Error_Msg();                 // Prints Error message
    void Help_Msg();                  // Prints helpfull information and list of commands
    void debug_Msg();                 // Prints the inputed parsed messaged
    void Print_Input_Msg();           // Prints the msg recived
    void PrintPinStatus(uint8_t pin); // Prints pin pinmode configuration
    void PrintPinValue(uint8_t pin);  // Prints pin value 
    //void PrintSystemStatus();         // Prints controller I/O information
};


// =================================================== Implementation ============================================== //
void Logger::Welcome_Msg() {
  Serial.print("Welcome to prototype software version ");
  Serial.println(Firmware_Version);
  Serial.println("Press h for help.");
  Serial.println(" ");
}

void Logger::Error_Msg() {
  Serial.println("Error: The controller is not initiliazed, unable to perform command.");
  Serial.println(" ");
}

void Logger::Help_Msg() {
  Serial.println("System Information");
  Serial.println("\tFirmware status: early development.");
  Serial.print("\tFirmware version: ");
  Serial.println(Firmware_Version);
  Serial.println("\tSupported Platforms: Arduino(Uno/Mega/Nano).");
  Serial.print("\tBaudrate: ");
  Serial.println(Config.Baudrate);
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

void Logger::debug_Msg() {
  Serial.print("The received message is: ");
  Serial.print(Msg.Command);
  Serial.print(Config.Delimiter);
  Serial.print(Msg.NumValue1);
  Serial.print(Config.Delimiter);
  Serial.println(Msg.NumValue2); 
  Serial.println("");
}

void Logger::Print_Input_Msg() {
  Serial.print(Msg.Command);
  Serial.print(Config.Delimiter);
  Serial.print(Msg.NumValue1);
  Serial.print(Config.Delimiter);
  Serial.println(Msg.NumValue2); 
}

void Logger::PrintPinValue(uint8_t pin) {
  Serial.print("pin ");
  Serial.print(pin);
  Serial.print(" is ");
  if(digitalRead(pin) == 0) Serial.println("LOW");
  else Serial.println("HIGH");
}

/*
void Logger::PrintSystemStatus() {
  // General system information
  Serial.println("System information. ");
  Serial.print("\tThe state of the system is: ");
  if(digitalRead(LED_BUILTIN) == HIGH) Serial.println("Initialized.");
  else Serial.println("Shutdown.");
  
  // Time since program started.
  Serial.print("\tTime since boot: ");
  Serial.print((float)Time/1000);
  Serial.println(" seconds.");
  Serial.println("");
  
  // Digital pins status
  Serial.println("Digital pins status: ");
  for(uint8_t i = 0; i < Config.Num_of_Digital_Pins; i++) {
    Serial.print("\t");
    PrintPinStatus(i);
    PrintPinValue(i);
  }

  Serial.println("");

  // Analoge Pins
  //write stuff here 
}
*/

// Object Declaration :)
class Logger Logger;