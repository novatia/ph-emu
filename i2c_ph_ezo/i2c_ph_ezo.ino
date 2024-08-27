/**
 *
 * @author BSIT, Andrea Novati - andrea.novati@n-3.it
 * @date August 21, 2024
 * @company N3 S.r.l. - Via Varese, 2 - Saronno (21047) VA
 * @version 2.0
 *
 * With this project you can easily interface your Arduino and PH-4502C Sensor as a PH-EZO sensor.
 * 
 * Why ? 
 *
 * Because I need it working on my reef-pi setup and reef-pi contributors does not provide driver code for writing a custom ph module reader.
 *
 * First of all need to calibrate PH-4502C board, put the sensor on a PH 7  buffer solution and rotate the potentiometer near the BNC connector until you read 2.5V (if supply voltage is 5V)
 * 
 * Features: 
 *    - 1,2,3 Calibration points
 *    - Temperature compensation
 *    - Factory reset by wire or by PIN
 *    - EEPROM management for calibration, I2C address and Compensation temperature storage
 *    - PH and Temperature readings with floating average
 *    - LED for board identification
 */
#include <Wire.h>
#include <EEPROM.h>

#include "PHCalibrationSample.h"

#define VERSION "2.1"

#define ATLAS_COMMAND_TERMINATOR 0x0
#define ATLAS_FAILED 0x02
#define ATLAS_SUCCEDED 0x01
#define ATLAS_NO_DATA 0xFF
#define ATLAS_PENDINS 0xFE

#define ATLAS_COMMAND_CHAR_TERMINATOR '\x00'


#define HARDWARE_INFO "?I,PH," VERSION
#define LED_IS_ON "LED is now ON"
#define LED_IS_OFF "LED is now OFF"
#define ERROR "Error"
#define SUCCESS "OK"
#define DEFAULT_NAME "PH-EMU"
#define COMMAND_LENGHT 32
#define TEMPERATURE_FACTOR 23.4f



#define SERIAL_DEBUG true // Set it to false if you do not want Serial port to be used for log purpose


#define EEPROM_LOW_CALIBRATION_ADDRESS 0
#define EEPROM_MID_CALIBRATION_ADDRESS sizeof(PHCalibrationSample)
#define EEPROM_HIGH_CALIBRATION_ADDRESS EEPROM_MID_CALIBRATION_ADDRESS + sizeof(PHCalibrationSample)
#define EEPROM_PH_CALIBRATED_ADDRESS EEPROM_HIGH_CALIBRATION_ADDRESS + sizeof(PHCalibrationSample)
#define EEPROM_PH_CALIBRATION_POINTS_ADDRESS EEPROM_PH_CALIBRATED_ADDRESS + sizeof(bool)
#define EEPROM_PH_LOW_CALIBRATED_ADDRESS EEPROM_PH_CALIBRATION_POINTS_ADDRESS + sizeof(unsigned short)
#define EEPROM_PH_MID_CALIBRATED_ADDRESS EEPROM_PH_LOW_CALIBRATED_ADDRESS + sizeof(bool)
#define EEPROM_PH_HIGH_CALIBRATED_ADDRESS EEPROM_PH_MID_CALIBRATED_ADDRESS + sizeof(bool)
#define EEPROM_T_COMPENSATION_TEMPERATURE_ADDRESS EEPROM_PH_HIGH_CALIBRATED_ADDRESS + sizeof(bool)
#define EEPROM_T_COMPENSATION_SET_ADDRESS EEPROM_T_COMPENSATION_TEMPERATURE_ADDRESS + sizeof(float)
#define EEPROM_I2C_ADDRESS_SETADDRESS EEPROM_T_COMPENSATION_SET_ADDRESS + sizeof(bool)


#define DEFAULT_I2C_ADDRESS 0x63 // Default I2C address for pH EZO devices
#define DEFAULT_COMPENSATION_TEMPERATURE 25.0f

#define PH_SENSOR_PIN A0 // Analog pin connected to the pH sensor
#define T_SENSOR_PIN A1 // Analog pin connected to the pH sensor
#define FACTORY_RESET_PIN 5


//readings at 31°C
PHCalibrationSample g_PHLowCalibration (628.41f,4.01f);
PHCalibrationSample g_PHMidCalibration (523.00f,6.85f);
PHCalibrationSample g_PHHighCalibration (450.21f,9.98f);

bool g_PHCalibrated = false;
bool g_PHLowCalibrated = false;
bool g_PHMidCalibrated = false;
bool g_PHHighCalibrated = false;

unsigned short g_PHCalibrationPoints = 0;

bool requestPending = false;
char command[COMMAND_LENGHT];
int commandIndex = 0;
bool g_LowEnergyMode = false;

float g_CompensationTemperature = DEFAULT_COMPENSATION_TEMPERATURE;
bool g_CompensationTemperatureSet = false;

int g_I2CAddress = DEFAULT_I2C_ADDRESS;

const int g_AverageNumSamples = 10;
float g_PHAverageValue = 0.0f;
float g_TAverageValue = 0.0f;

float g_PHCalibratedValue = 0.0f;

float g_TCalibratedValue = 0.0f;

float g_Alpha = 1.0f / g_AverageNumSamples; // Smoothing factor
bool g_AverageInit = false;


char g_I2CBuffer[COMMAND_LENGHT];

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
    EEPROM.get(EEPROM_T_COMPENSATION_TEMPERATURE_ADDRESS, g_CompensationTemperature);
    EEPROM.get(EEPROM_T_COMPENSATION_SET_ADDRESS, g_CompensationTemperatureSet);
    EEPROM.get(EEPROM_I2C_ADDRESS_SETADDRESS, g_I2CAddress);
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
    EEPROM.put(EEPROM_T_COMPENSATION_TEMPERATURE_ADDRESS, g_CompensationTemperature);
    EEPROM.put(EEPROM_T_COMPENSATION_SET_ADDRESS, g_CompensationTemperatureSet);
    EEPROM.put(EEPROM_I2C_ADDRESS_SETADDRESS, g_I2CAddress);
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
        exec_FACTORY();
    }

    if (hash == loaded_hash )
    {
      LoadSettings();
    }
}

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the onboard LED pin
  pinMode(PH_SENSOR_PIN, INPUT); // Initialize the onboard LED pin
  pinMode(T_SENSOR_PIN, INPUT); // Initialize the onboard LED pin
  pinMode(FACTORY_RESET_PIN,INPUT);

  Serial.begin(38400); // Initialize serial communication for debugging

  setup_EEPROM();


  Wire.begin(g_I2CAddress); // Initialize I2C with the device address
  Wire.onRequest(requestEvent); // Register the request event handler
  Wire.onReceive(receiveEvent); // Register the receive event handler


  #if SERIAL_DEBUG
  Serial.println("PH-EMU");
  Serial.print("At address: ");
  Serial.println(g_I2CAddress);
  print_CalibrationValues();
  print_TemperatureCompensationValues();
  #endif

}

void print_TemperatureCompensationValues()
{
 #if SERIAL_DEBUG
  Serial.print("Temperature compensation set: ");
  Serial.println(g_CompensationTemperatureSet);
  Serial.print("Compensation T: ");
  Serial.println(g_CompensationTemperature);
  #endif
}

void print_CalibrationValues() {
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


void loop()
{
    int ph_sensorValue = analogRead(PH_SENSOR_PIN); // Read the analog value from the sensor
    int t_sensorValue = analogRead(T_SENSOR_PIN); // Read the analog value from the sensor
    
    if (!g_AverageInit)
    {
      g_TAverageValue = t_sensorValue;
      g_PHAverageValue = ph_sensorValue;
      g_AverageInit = true;
    }

    g_PHAverageValue = (g_Alpha * ph_sensorValue) + ((1.0f - g_Alpha) * g_PHAverageValue);
    g_TAverageValue = (g_Alpha * t_sensorValue) + ((1.0f - g_Alpha) * g_TAverageValue);

    g_PHCalibratedValue = calibrate(g_PHAverageValue);

    g_TCalibratedValue = g_TAverageValue;

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
  } else if (strcmp(command, "INFO") == 0 || strcmp(command, "I") == 0) {
    exec_INFO();
  } else if (strcmp(command, "CAL,?") == 0) {
    exec_CAL_QUERY();
  } else if (strcmp(command, "CAL,CLEAR") == 0) {
    exec_CAL_CLEAR();
  } else if (strncmp(command, "CAL,LOW,", 8) == 0) {
    exec_CAL_LOW();
  } else if (strcmp(command, "CAL,DEFAULT") == 0) {
    exec_CAL_DEFAULT();
  } else if (strncmp(command, "CAL,MID,", 8) == 0) {
    exec_CAL_MID();
  } else if (strncmp(command, "CAL,HIGH,", 9) == 0) {
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
  } else if (strcmp(command, "T,?") == 0) {
    exec_TEMP_QUERY();
  }else if (strncmp(command, "T,", 2) == 0) {
    exec_TEMP_COMP();
  } else if ( strcmp(command, "STATUS") == 0 ) {
    exec_STATUS();
  } else if (strcmp(command, "FIND") == 0) {
    exec_FIND();
  } else if (strcmp(command, "FACTORY") == 0 || strcmp(command, "X") == 0) {
    exec_FACTORY();
  } else if (strcmp(command, "NAME,?") == 0) {
    exec_NAME_QUERY();
  }else {
    I2CResponse( ATLAS_FAILED ,"Unknown");
  }

  memset(command, 0, sizeof(command));
  commandIndex = 0;
}
 
// Function to handle I2C data reception from the master (Raspberry Pi)
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
    if (c == ATLAS_COMMAND_CHAR_TERMINATOR || commandIndex >= sizeof(command) - 1) {
      command[commandIndex] = '\0'; // Null-terminate the string
    
      // Convert command to uppercase
      for (int i = 0; i < commandIndex; i++) {
        command[i] = toupper(command[i]);
      }

      commandIndex = 0; // Reset command index

    } else {
      command[commandIndex++] = c; // Store the character
    }
  }
}

// R: Takes a single pH reading.
void exec_R()
{
    char c_PHValue[6];
    float PHValue = temperatureCompensate(g_PHCalibratedValue,g_CompensationTemperature);

    dtostrf(PHValue, 5, 2, c_PHValue); // Convert float to string
    #ifdef SERIAL_DEBUG
    Serial.println(c_PHValue);
    #endif
    I2CResponse(ATLAS_SUCCEDED ,c_PHValue );
    
    dtostrf(g_PHAverageValue, 5, 2, c_PHValue); // Convert float to string
    #ifdef SERIAL_DEBUG
    Serial.println(c_PHValue);
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
    g_CompensationTemperatureSet = true;
    // Extract the temperature value from the command
    char* tempValueStr = command + 2; // Skip the "T," part
    g_CompensationTemperature = atof(tempValueStr); // Convert the extracted string to a float

    SaveSettings();

    I2CResponse(ATLAS_SUCCEDED,"");
}

void I2CResponse(int statusCode, const char* message)
{
   String s_message(message);
   I2CResponse(statusCode, s_message);
}

void I2CResponse(char result, String value) 
{
    memset(g_I2CBuffer, 0, sizeof(g_I2CBuffer));
    
    g_I2CBuffer[0] = result;

    size_t valueLength = value.length();
    size_t maxLength = sizeof(g_I2CBuffer) - 2; // 1 byte for result, 1 byte for terminator
    size_t lengthToCopy = min(valueLength, maxLength);
    // Copy the string into the buffer starting from position 1
    memcpy(&g_I2CBuffer[1], value.c_str(), lengthToCopy);

    // Add terminator
    g_I2CBuffer[lengthToCopy + 1] = ATLAS_COMMAND_TERMINATOR;
    
    Wire.write((const uint8_t*)g_I2CBuffer, sizeof(g_I2CBuffer));
}

void exec_TEMP_QUERY()
{
    #ifdef SERIAL_DEBUG
    print_TemperatureCompensationValues();
    #endif

 // Send pH value
    char c_temp[10];
    c_temp[0] = 0;
    dtostrf(g_CompensationTemperature, 5, 2, c_temp); // Convert float to string

    String s_Response("?T,");
    s_Response += c_temp; // Append the C-style string to the String object
    
    I2CResponse(ATLAS_SUCCEDED,s_Response );
}

// Information Commands
//     Status: Retrieves the status of the device.
//First Field (0): This is the device status code.

//     0: Indicates that the device is operating normally without any errors.
//     Other codes might indicate specific errors or states, such as calibration errors or out-of-range readings.

// Second Field (0.0): This field represents the voltage at the output pin, which can give insight into the operation of the sensor, although for pH readings, this might not always be the primary data point of interest.

// Third Field (3): This field indicates the number of consecutive successful readings or a counter of some other status metric depending on the firmware version.
// Restart codes
// P power on reset
// S software reset
// B brown out reset
// W watchdog reset
// U unknown
void exec_STATUS()
{
  #ifdef SERIAL_DEBUG
  Serial.write("?STATUS,0");
  #endif
  
  I2CResponse(ATLAS_SUCCEDED ,"?STATUS,0");
}

void exec_NAME_QUERY()
{
  #ifdef SERIAL_DEBUG
  Serial.write(DEFAULT_NAME);
  #endif
  I2CResponse(ATLAS_SUCCEDED ,"?name," DEFAULT_NAME);
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
  #ifdef SERIAL_DEBUG
  Serial.write(HARDWARE_INFO);
  #endif
  I2CResponse(ATLAS_SUCCEDED , HARDWARE_INFO );
}

// Factory Reset
//     Factory: Restores the device to its factory default settings.
// need to bring status to ?STATUS,S,5.038
void exec_FACTORY()
{
  #ifdef SERIAL_DEBUG
  Serial.write("Reset to factory");
  #endif
  //DO NOTHING
  g_CompensationTemperature = DEFAULT_COMPENSATION_TEMPERATURE;
  g_CompensationTemperatureSet = false;

  g_I2CAddress = DEFAULT_I2C_ADDRESS;

  exec_CAL_CLEAR();
  SaveSettings();
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
        Serial.println("?CAL,1");
        #endif
        I2CResponse(ATLAS_SUCCEDED,"*CAL,1");
        break;
      case 2:
        #ifdef SERIAL_DEBUG
        Serial.println("?CAL,2");
        #endif
        I2CResponse(ATLAS_SUCCEDED,"*CAL,2");
        break;
      case 3:
        #ifdef SERIAL_DEBUG
        Serial.println("?CAL,3");
        #endif
        I2CResponse(ATLAS_SUCCEDED,"*CAL,3");
        break;
    };
  }else {
  #ifdef SERIAL_DEBUG
  Serial.println("?CAL,0");
  #endif
  I2CResponse(ATLAS_SUCCEDED,"*CAL,0");
  }
  
}

/*
The general formula for pH temperature compensation is:
pHcompensated=pHmeasured+(Tcurrent−25)×Temperature Coefficient

Where:

    pHcompensatedpHcompensated​ is the temperature-compensated pH value.
    pHmeasuredpHmeasured​ is the pH value measured at the current temperature.
    TcurrentTcurrent​ is the current temperature in °C.
    Temperature Coefficient is typically around 0.03 pH units per degree Celsius.
*/
float temperatureCompensate(float pH_measured, float temperature) {
    const float temperatureCoefficient = 0.03; // pH units per degree Celsius
    float pH_compensated = pH_measured + ((temperature - 25.0f) * temperatureCoefficient);
    return pH_compensated;
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

  I2CResponse(ATLAS_SUCCEDED, "");
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
    
  I2CResponse(ATLAS_SUCCEDED, "");
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
    I2CResponse(ATLAS_SUCCEDED, "");
  
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
    I2CResponse(ATLAS_SUCCEDED, "");
}

//    I2C,?: Queries the current I2C address.
void exec_I2C_QUERY()
{
    char c_address[5];
    
    // Convert the I2C address to a string in hexadecimal format
    snprintf(c_address, sizeof(c_address), "0x%02X", g_I2CAddress);
    
    // Send the response over I2C
    I2CResponse(ATLAS_SUCCEDED, c_address);
    
    #ifdef SERIAL_DEBUG
    Serial.print("Current I2C Address: ");
    Serial.println(g_I2CBuffer);
    #endif
}

//    I2C, <address>: Sets a new I2C address for the device.
void exec_I2C_SET()
{
    // Extract the new I2C address from the command string
    char* newAddressStr = command + 4; // Skip "I2C,"
    int newAddress = strtol(newAddressStr, NULL, 16); // Convert hex string to an integer

    // Validate the new address
    if (newAddress < 0x08 || newAddress > 0x77) {
        #ifdef SERIAL_DEBUG
        Serial.println("Invalid I2C Address. Must be between 0x08 and 0x77.");
        #endif
        I2CResponse(ATLAS_FAILED , ERROR );
        return;
    }
    else 
    {
      g_I2CAddress = newAddress;
      SaveSettings();
      I2CResponse(ATLAS_SUCCEDED , SUCCESS );
    }
}

int g_LedStatus = 0;

// L,?: Queries the current status of the LED.
void exec_LED_QUERY()
{
  if (g_LedStatus)
  {
    #ifdef SERIAL_DEBUG
    Serial.println("?L,1");
    #endif
    I2CResponse(ATLAS_SUCCEDED , "?L,1" );
  }
  else
  {
    #ifdef SERIAL_DEBUG
    Serial.println("?L,0");
    #endif
    I2CResponse(ATLAS_SUCCEDED , "?L,0" );
  }
}

// L, <state>: Turns the LED on or off, where <state> can be 1 (on) or 0 (off).
void exec_LED_SET()
{
  if (command[2] == '1') {
    g_LedStatus = 1;
    digitalWrite(LED_BUILTIN, HIGH); // Turn the onboard LED on
    
    I2CResponse(ATLAS_SUCCEDED , LED_IS_ON );
      #ifdef SERIAL_DEBUG
    Serial.println(LED_IS_ON);
    #endif
  } else if (command[2] == '0') {
    digitalWrite(LED_BUILTIN, LOW);  // Turn the onboard LED off
    I2CResponse(ATLAS_SUCCEDED , LED_IS_OFF );
    #ifdef SERIAL_DEBUG
    Serial.println(LED_IS_OFF);
    #endif
    g_LedStatus = 0;
  } else {
      #ifdef SERIAL_DEBUG
    Serial.println(ERROR);
    #endif
    I2CResponse(ATLAS_FAILED , ERROR );
  }
}
