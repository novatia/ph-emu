 With this project you can easily interface your Arduino and PH-4502C Sensor as a PH-EZO sensor.
 
 Why ? 
 
 Because I need it working on my reef-pi setup and reef-pi contributors does not provide driver code for writing a custom ph module reader.
 
 First of all need to calibrate PH-4502C board, put the sensor on a PH 7  buffer solution and rotate the potentiometer near the BNC connector until you read 2.5V (if supply voltage is 5V)
 
 Features: 
    - 1,2,3 Calibration points
    - Temperature compensation
    - Factory reset by wire or by PIN
    - EEPROM management for calibration, I2C address and Compensation temperature storage
    - PH and Temperature readings with floating average
    - LED for board identification