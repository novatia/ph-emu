/**
 *
 * @author BSIT, Andrea Novati - andrea.novati@n-3.it
 * @date August 21, 2024
 * @company N3 S.r.l. - Via Varese, 2 - Saronno (21047) VA
 * @version 2.0
 *
 * First need to connect board to arduino and Read with R and serial monitor. Rotate Reference Capacitor in order to place NaCl reading around 512 value
 */
#include <Wire.h>
#include "PHCalibrationSample.h"
#include <EEPROM.h>

#define SERIAL_DEBUG true

#define EEPROM_LOW_CALIBRATION_ADDRESS 0
#define EEPROM_MID_CALIBRATION_ADDRESS sizeof(PHCalibrationSample)
#define EEPROM_HIGH_CALIBRATION_ADDRESS EEPROM_MID_CALIBRATION_ADDRESS + sizeof(PHCalibrationSample)
#define EEPROM_PH_CALIBRATED_ADDRESS EEPROM_HIGH_CALIBRATION_ADDRESS + sizeof(PHCalibrationSample)
#define EEPROM_PH_CALIBRATION_POINTS_ADDRESS EEPROM_PH_CALIBRATED_ADDRESS + sizeof(bool)
#define EEPROM_PH_LOW_CALIBRATED_ADDRESS EEPROM_PH_CALIBRATION_POINTS_ADDRESS + sizeof(unsigned short)
#define EEPROM_PH_MID_CALIBRATED_ADDRESS EEPROM_PH_LOW_CALIBRATED_ADDRESS + sizeof(bool)
#define EEPROM_PH_HIGH_CALIBRATED_ADDRESS EEPROM_PH_MID_CALIBRATED_ADDRESS + sizeof(bool)


#define I2C_ADDRESS 0x63 // Default I2C address for pH EZO devices
#define PH_SENSOR_PIN A0 // Analog pin connected to the pH sensor
#define T_SENSOR_PIN A1 // Analog pin connected to the pH sensor

#define FACTORY_RESET_PIN 5




#define HARDWARE_INFO "pH,EZO,2.0"
#define LED_IS_ON "LED is now ON"
#define LED_IS_OFF "LED is now OFF"
#define ERROR "Error"

//readings at 31°C
PHCalibrationSample g_PHLowCalibration (663.88f,4.01f);
PHCalibrationSample g_PHMidCalibration (577.00f,6.85f);
PHCalibrationSample g_PHHighCalibration (488.21f,9.98f);

bool g_PHCalibrated = false;
bool g_PHLowCalibrated = false;
bool g_PHMidCalibrated = false;
bool g_PHHighCalibrated = false;

unsigned short g_PHCalibrationPoints = 0;

#define COMMAND_LENGHT 32
int tValue;
bool requestPending = false;
char command[COMMAND_LENGHT];
int commandIndex = 0;
bool g_LowEnergyMode = false;


void writeHash()
{
   int eepromSize = EEPROM.length(); // Get the total number of bytes in the EEPROM
   uint8_t hash = simpleHash();
   EEPROM.put(eepromSize-1,hash);
}

uint8_t simpleHash()
{
    int eepromSize = EEPROM.length(); // Get the total number of bytes in the EEPROM

    uint8_t hash = 0;
    for (size_t i = 0; i < eepromSize-1; i++) {
        uint8_t data = EEPROM.read(i);
        hash += data;
    }
   
    return hash;
}

uint8_t getHash()
{
   return EEPROM.read(EEPROM.length()-1);
}



  void LoadSettings()
  {
    #ifdef SERIAL_DEBUG
      Serial.println("Loading EEPROM settings");
    #endif
    EEPROM.get(EEPROM_LOW_CALIBRATION_ADDRESS, g_PHLowCalibration);
    EEPROM.get(EEPROM_MID_CALIBRATION_ADDRESS, g_PHMidCalibration);
    EEPROM.get(EEPROM_HIGH_CALIBRATION_ADDRESS, g_PHHighCalibration);
    EEPROM.get(EEPROM_PH_CALIBRATED_ADDRESS, g_PHCalibrated);
    EEPROM.get(EEPROM_PH_CALIBRATION_POINTS_ADDRESS, g_PHCalibrationPoints);
    EEPROM.get(EEPROM_PH_LOW_CALIBRATED_ADDRESS, g_PHLowCalibrated);
    EEPROM.get(EEPROM_PH_MID_CALIBRATED_ADDRESS, g_PHMidCalibrated);
    EEPROM.get(EEPROM_PH_HIGH_CALIBRATED_ADDRESS, g_PHHighCalibrated);
  }
  
  void SaveSettings()
  {
    EEPROM.put(EEPROM_LOW_CALIBRATION_ADDRESS, g_PHLowCalibration);
    EEPROM.put(EEPROM_MID_CALIBRATION_ADDRESS, g_PHMidCalibration);
    EEPROM.put(EEPROM_HIGH_CALIBRATION_ADDRESS, g_PHHighCalibration);
    EEPROM.put(EEPROM_PH_CALIBRATED_ADDRESS, g_PHCalibrated);
    EEPROM.put(EEPROM_PH_CALIBRATION_POINTS_ADDRESS, g_PHCalibrationPoints);
    EEPROM.put(EEPROM_PH_LOW_CALIBRATED_ADDRESS, g_PHLowCalibrated);
    EEPROM.put(EEPROM_PH_MID_CALIBRATED_ADDRESS, g_PHMidCalibrated);
    EEPROM.put(EEPROM_PH_HIGH_CALIBRATED_ADDRESS, g_PHHighCalibrated);
    
    writeHash();
  }



void setup_EEPROM()
{
   uint8_t hash = simpleHash();
   uint8_t loaded_hash = getHash();

    bool factory_reset = digitalRead(FACTORY_RESET_PIN);

    if (factory_reset)
    {
      
        #ifdef SERIAL_DEBUG
        Serial.println("Factory Reset PIN is HIGH.");
        #endif

        SaveSettings();
    }

    if (hash == loaded_hash ) {
      LoadSettings();
    }
}






void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the onboard LED pin

  Wire.begin(I2C_ADDRESS); // Initialize I2C with the device address
  Wire.onRequest(requestEvent); // Register the request event handler
  Wire.onReceive(receiveEvent); // Register the receive event handler

  Serial.begin(38400); // Initialize serial communication for debugging

  setup_EEPROM();
  #if SERIAL_DEBUG
  Serial.println("PH-EMU");
  
  print_CalibrationValues();
  #endif
}

void print_CalibrationValues(){
  #if SERIAL_DEBUG
  Serial.print("Calibrated: ");
  Serial.println(g_PHCalibrated);
  Serial.print("Low Calibrated: ");
  Serial.println(g_PHLowCalibrated);
  Serial.print("Mid Calibrated: ");
  Serial.println(g_PHMidCalibrated);
  Serial.print("High Calibrated: ");
  Serial.println(g_PHHighCalibrated);

  Serial.print("Calibration points: ");
  Serial.println(g_PHCalibrationPoints);

  if (g_PHLowCalibrated)
  {
    Serial.print("Low Calibration: ");
    Serial.print(g_PHLowCalibration.rawValue);
    Serial.print(",");
    Serial.println(g_PHLowCalibration.Value);
  }

  if (g_PHMidCalibrated)
  {
    Serial.print("Mid Calibration: ");
    Serial.print(g_PHMidCalibration.rawValue);
    Serial.print(",");
    Serial.println(g_PHMidCalibration.Value);
  }
  
  if (g_PHHighCalibrated)
  {
    Serial.print("High Calibration: ");
    Serial.print(g_PHHighCalibration.rawValue);
    Serial.print(",");
    Serial.println(g_PHHighCalibration.Value);
  }
  #endif

}

const int g_PHAverageNumSamples = 10;
float g_PHAverageValue = 0.0f;
float g_PHCalibratedValue = 0.0f;

float g_PHAlpha = 1.0f / g_PHAverageNumSamples; // Smoothing factor
bool g_PHAverageInit = false;

void loop()
{
    int sensorValue = analogRead(PH_SENSOR_PIN); // Read the analog value from the sensor
    
    if (!g_PHAverageInit)
    {
      g_PHAverageValue = sensorValue;
      g_PHAverageInit = true;
    }

    g_PHAverageValue = (g_PHAlpha * sensorValue) + ((1.0f - g_PHAlpha) * g_PHAverageValue);

    g_PHCalibratedValue = calibrate(g_PHAverageValue);
    
    tValue = analogRead(T_SENSOR_PIN); // Read the analog value from the sensor
    
    if (Serial.available())
    {
      String cmd = Serial.readString();
      cmd.trim(); // Remove any leading/trailing whitespace/newlines
      if (cmd.length() < sizeof(command)) {
        // Copy the content of the String into the command buffer
        strncpy(command, cmd.c_str(), sizeof(command) - 1);
        command[sizeof(command) - 1] = '\0'; // Ensure null termination
        requestPending = true;

        requestEvent(); // Call the function to process the command
      } else {
          #ifdef SERIAL_DEBUG
        Serial.println("Error: Command too long");
        #endif
      }
    }

  // Only read the sensor and calculate pH if there's a pending request
 
  if (g_LowEnergyMode)
    delay(1000); // Wait for a second before the next reading
  else 
    delay(100);
}


// Function to interpolate between three points
float calibrate(float rawValue) {
  if (g_PHLowCalibration.rawValue < g_PHHighCalibration.rawValue) { 
    if (rawValue <= g_PHLowCalibration.rawValue) {
      // Extrapolate below the low-point calibration (pH 4.0)
      return g_PHLowCalibration.Value;
    } else if (rawValue <= g_PHMidCalibration.rawValue) {
      // Interpolate between the low-point and mid-point calibration (pH 4.0 to pH 7.0)
      return mapValue(rawValue, g_PHLowCalibration.rawValue, g_PHMidCalibration.rawValue, g_PHLowCalibration.Value, g_PHMidCalibration.Value);
    } else if (rawValue <= g_PHHighCalibration.rawValue) {
      // Interpolate between the mid-point and high-point calibration (pH 7.0 to pH 10.0)
      return mapValue(rawValue, g_PHMidCalibration.rawValue, g_PHHighCalibration.rawValue,  g_PHMidCalibration.Value,  g_PHHighCalibration.Value);
    } else {
      // Extrapolate above the high-point calibration (pH 10.0)
      return g_PHHighCalibration.Value;
    }
  } else {
    if (rawValue <= g_PHHighCalibration.rawValue) {
      // Extrapolate below the low-point calibration (pH 4.0)
      return g_PHHighCalibration.Value;
    } else if (rawValue <= g_PHMidCalibration.rawValue) {
      // Interpolate between the low-point and mid-point calibration (pH 4.0 to pH 7.0)
      return mapValue(rawValue, g_PHHighCalibration.rawValue, g_PHMidCalibration.rawValue, g_PHHighCalibration.Value, g_PHMidCalibration.Value);
    } else if (rawValue <= g_PHLowCalibration.rawValue) {
      // Interpolate between the mid-point and high-point calibration (pH 7.0 to pH 10.0)
      return mapValue(rawValue, g_PHMidCalibration.rawValue, g_PHLowCalibration.rawValue,  g_PHMidCalibration.Value,  g_PHLowCalibration.Value);
    } else {
      // Extrapolate above the high-point calibration (pH 10.0)
      return g_PHLowCalibration.Value;
    }
  }

}
// Function to perform linear interpolation
float mapValue(float x, float x1, float x2, float y1, float y2) {
  return y1 + ((x - x1) * (y2 - y1)) / (x2 - x1);
}


// Function to handle I2C data requests from the master (Raspberry Pi)
void requestEvent() 
{
  if (strcmp(command, "R") == 0) {
    exec_R();
  } else if (strcmp(command, "Info") == 0 || strcmp(command, "i") == 0) {
    exec_INFO();
  } else if (strcmp(command, "Cal,?") == 0) {
    exec_CAL_QUERY();
  } else if (strcmp(command, "Cal,clear") == 0) {
    exec_CAL_CLEAR();
  } else if (strncmp(command, "Cal,low,", 8) == 0) {
    exec_CAL_LOW();
  } else if (strcmp(command, "Cal,default") == 0) {
    exec_CAL_DEFAULT();
  } else if (strncmp(command, "Cal,mid,", 8) == 0) {
    exec_CAL_MID();
  } else if (strncmp(command, "Cal,high,", 9) == 0) {
    exec_CAL_HIGH();
  } else if (strcmp(command, "SLEEP") == 0) {
    exec_SLEEP();
  } else if (strcmp(command, "I2C,?") == 0) {
    exec_I2C_QUERY();
  } else if (strncmp(command, "I2C,", 4) == 0) {
    exec_I2C_SET();
  } else if (strcmp(command, "L,?") == 0) {
    exec_LED_QUERY();
  } else if (strncmp(command, "L,", 2) == 0) {
    exec_LED_SET();
  } else if (strncmp(command, "T,", 2) == 0) {
    exec_TEMP_COMP();
  } else if (strcmp(command, "T,?") == 0) {
    exec_TEMP_QUERY();
  } else if (strcmp(command, "Status") == 0) {
    exec_STATUS();
  } else if (strcmp(command, "Find") == 0) {
    exec_FIND();
  } else if (strcmp(command, "Factory") == 0) {
    exec_FACTORY();
  } else {
    Wire.write("Unknown");
  }

  memset(command, 0, sizeof(command));
  commandIndex = 0;
}

// Function to handle I2C data reception from the master (Raspberry Pi)
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
    if (c == '\n' || commandIndex >= sizeof(command) - 1) {
      command[commandIndex] = '\0'; // Null-terminate the string
      commandIndex = 0; // Reset command index
    } else {
      command[commandIndex++] = c; // Store the character
    }
  }
}

// R: Takes a single pH reading.
void exec_R()
{
    // Send pH value
    char response[10];
    response[0] = 0;
   
    dtostrf(g_PHCalibratedValue, 5, 2, response); // Convert float to string
      #ifdef SERIAL_DEBUG
    Serial.println(response);
    #endif
    Wire.write(response);

    dtostrf(g_PHAverageValue, 5, 2, response); // Convert float to string
      #ifdef SERIAL_DEBUG
    Serial.println(response);
    #endif
    
}

// Configuration Commands
// SLEEP: Puts the device into low-power sleep mode.
void exec_SLEEP()
{
    g_LowEnergyMode = true;
}

// Temperature Compensation
// T, <temperature>: Sets the temperature for automatic temperature compensation.
void exec_TEMP_COMP()
{
    //DO NOTHING
}

void exec_TEMP_QUERY()
{
 // Send pH value
    char response[10];
    response[0] = 0;

    itoa(tValue, response, 10); // Convert float to string
      #ifdef SERIAL_DEBUG
    Serial.println(response);
    #endif
    Wire.write(response);
}
// Information Commands
//     Status: Retrieves the status of the device.
//First Field (0): This is the device status code.

//     0: Indicates that the device is operating normally without any errors.
//     Other codes might indicate specific errors or states, such as calibration errors or out-of-range readings.

// Second Field (0.0): This field represents the voltage at the output pin, which can give insight into the operation of the sensor, although for pH readings, this might not always be the primary data point of interest.

// Third Field (3): This field indicates the number of consecutive successful readings or a counter of some other status metric depending on the firmware version.
void exec_STATUS()
{
  Serial.write("0,0.0,3");
  Wire.write("0,0.0,3");
}

//     Find: Causes the LED to blink for identifying the device.
void exec_FIND()
{
    #ifdef SERIAL_DEBUG
    Serial.println("Find blinking board...");
    #endif
    for (int i=0;i<5;i++){
      digitalWrite(LED_BUILTIN, HIGH); // Turn the onboard LED on
      delay(300);
      digitalWrite(LED_BUILTIN, LOW); // Turn the onboard LED on
      delay(300);
    }
}

//     Info: Provides information about the device, including firmware version.
void exec_INFO()
{
  Serial.write(HARDWARE_INFO);
  Wire.write(HARDWARE_INFO);
}

// Factory Reset
//     Factory: Restores the device to its factory default settings.
void exec_FACTORY()
{
  Serial.write("Reset to factory");
  //DO NOTHING
}

//    Cal,?: Queries the current calibration status.
// Single-Point Calibration: 1
// Two-Point Calibration: 2
// Three-Point Calibration: 3
// Not Calibrated: When the sensor is not calibrated, it will typically return: 0
void exec_CAL_QUERY()
{
  #if SERIAL_DEBUG
  print_CalibrationValues();
  #endif

  if (g_PHCalibrated) {
    switch (g_PHCalibrationPoints) {
      case 1:
        #ifdef SERIAL_DEBUG
        Serial.println("*CAL,1");
        #endif
        Wire.write("*CAL,1");
        break;
      case 2:
        #ifdef SERIAL_DEBUG
        Serial.println("*CAL,2");
        #endif
        Wire.write("*CAL,2");
        break;
      case 3:
        #ifdef SERIAL_DEBUG
        Serial.println("*CAL,3");
        #endif
        Wire.write("*CAL,3");
        break;
    };
  }else {
  #ifdef SERIAL_DEBUG
   Serial.println("*CAL");
   #endif
   Wire.write("*CAL");
  }
  
}


/*
 readings at 31°C with buffer solution and sensor model: PH Electrode ph range ph 0-14 working temperature 0-60 Apex ce specialists GMBH MILA 31200 E312
 (577.00f,6.85f);
 (663.88f,4.01f);
 (488.21f,9.98f);
*/
void exec_CAL_DEFAULT()
{
  g_PHCalibrated = true;
  g_PHLowCalibrated = true;
  g_PHMidCalibrated = true;
  g_PHHighCalibrated = true;

  g_PHMidCalibration.rawValue = 577.00f;
  g_PHMidCalibration.Value = 6.85f;
  
  g_PHLowCalibration.rawValue = 663.88f;
  g_PHLowCalibration.Value = 4.01f;

  g_PHHighCalibration.rawValue = 488.21f;
  g_PHHighCalibration.Value = 9.98f;
  
  g_PHCalibrationPoints = 3;
  SaveSettings();

  // Respond with the current calibration status (not calibrated)
  exec_CAL_QUERY();
}

//    Cal,clear: Clears all calibration data.
void exec_CAL_CLEAR()
{
  g_PHCalibrated = false;
  g_PHLowCalibrated = false;
  g_PHMidCalibrated = false;
  g_PHHighCalibrated = false;

  g_PHMidCalibration.rawValue = 512;
  g_PHMidCalibration.Value = 7.0f;
  
  g_PHLowCalibration.rawValue = 0;
  g_PHLowCalibration.Value = 0.0f; // Set to an unrealistic pH value to indicate uncalibrated

  
  g_PHHighCalibration.rawValue = 1024;
  g_PHHighCalibration.Value = 14.0f;  // Set to an unrealistic pH value to indicate uncalibrated
  
  g_PHCalibrationPoints = 0;
  SaveSettings();
  
  // Respond with the current calibration status (not calibrated)
  exec_CAL_QUERY();
}

//    Cal,low, <value>: Sets a low-point calibration.
void exec_CAL_LOW()
{
  if (g_PHCalibrationPoints>0)
  {
    if (g_PHCalibrationPoints == 2)
      g_PHCalibrationPoints = 3;
      
    if (g_PHCalibrationPoints == 1)
      g_PHCalibrationPoints = 2;
  }

    char* pHValueStr = command + 8; // Skip "Cal,mid,"
    float low_ph = atof(pHValueStr); // Convert the extracted string to a float

    // Update the mid calibration value
    g_PHLowCalibration.rawValue = g_PHAverageValue; // Use the averaged sensor value
    g_PHLowCalibration.Value = low_ph;

  // Mark the mid calibration as completed
    g_PHLowCalibrated = true;
    g_PHCalibrated = true;

    // Save the calibration settings to EEPROM
    SaveSettings();

    // Respond with the current calibration status
    exec_CAL_QUERY();
}

//    Cal,mid, <value>: Sets a mid-point calibration (usually 7.00).
void exec_CAL_MID()
{
  if (g_PHCalibrationPoints==0)
  {
    g_PHCalibrationPoints = 1;
  }
    char* pHValueStr = command + 8; // Skip "Cal,mid,"
    float mid_ph = atof(pHValueStr); // Convert the extracted string to a float

    // Update the mid calibration value
    g_PHMidCalibration.rawValue = g_PHAverageValue; // Use the averaged sensor value
    g_PHMidCalibration.Value = mid_ph;

  // Mark the mid calibration as completed
    g_PHMidCalibrated = true;
    g_PHCalibrated = true;

    // Save the calibration settings to EEPROM
    SaveSettings();

    // Respond with the current calibration status
    exec_CAL_QUERY();
}

//    Cal,high, <value>: Sets a high-point calibration.
void exec_CAL_HIGH()
{
  if (g_PHCalibrationPoints>0)
  {
    if (g_PHCalibrationPoints == 2)
      g_PHCalibrationPoints = 3;

    if (g_PHCalibrationPoints == 1)
      g_PHCalibrationPoints = 2;
  }
    char* pHValueStr = command + 9; // Skip "Cal,high,"
    float high_ph = atof(pHValueStr); // Convert the extracted string to a float

    // Update the mid calibration value
    g_PHHighCalibration.rawValue = g_PHAverageValue; // Use the averaged sensor value
    g_PHHighCalibration.Value = high_ph;

  // Mark the mid calibration as completed
    g_PHHighCalibrated = true;
    g_PHCalibrated = true;

    // Save the calibration settings to EEPROM
    SaveSettings();

    // Respond with the current calibration status
    exec_CAL_QUERY();
}

//    I2C,?: Queries the current I2C address.
void exec_I2C_QUERY()
{

}

//    I2C, <address>: Sets a new I2C address for the device.
void exec_I2C_SET()
{

}

int g_LedStatus = 0;

// L,?: Queries the current status of the LED.
void exec_LED_QUERY()
{
  if (g_LedStatus)
  {
      #ifdef SERIAL_DEBUG
    Serial.println("1");
    #endif
    Wire.write("1");
  }
  else
  {
      #ifdef SERIAL_DEBUG
    Serial.println("0");
    #endif
    Wire.write("0");
  }
}


// L, <state>: Turns the LED on or off, where <state> can be 1 (on) or 0 (off).
void exec_LED_SET()
{
  if (command[2] == '1') {
    g_LedStatus = 1;
    digitalWrite(LED_BUILTIN, HIGH); // Turn the onboard LED on
    Wire.write(LED_IS_ON);
      #ifdef SERIAL_DEBUG
    Serial.println(LED_IS_ON);
    #endif
  } else if (command[2] == '0') {
    digitalWrite(LED_BUILTIN, LOW);  // Turn the onboard LED off
    Wire.write(LED_IS_OFF);
      #ifdef SERIAL_DEBUG
    Serial.println(LED_IS_OFF);
    #endif
    g_LedStatus = 0;
  } else {
      #ifdef SERIAL_DEBUG
    Serial.println(ERROR);
    #endif
    Wire.write(ERROR);
  }
}
