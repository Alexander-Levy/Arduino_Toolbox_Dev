// ================================================================================================================== //
//                                                          Commands                                                  //
// ================================================================================================================== //



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