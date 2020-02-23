# Low Frequency Radio (LoRa) Communication System
A secondary communication system between the ground and flight system to provide sensor data from the rocket to the ground. The systems are capable of a full-duplex communication system, and will be implementing commands in the future.

## Flight Module
Parses data from thermocouples and i2c sensor modules and sends them over LoRa to the ground. Creates the packet format, which is still to be determined, but as of currently, exists as a string. Packets can not exceed 255 bytes in length, so more than 1 type of frame may be designed to send other module information. Currently, the available sensor data is as follows:

### Raw Frame Format:
```
  1    1       1      1        4         1         6          1
Dest  Send  DataID  MsgID    Millis    MsgLen  Gyro/Accel  ChkSum
0x45  0x57   00-FF   00-FF  0-FFFFFFFF  00-F6    
```
**Dest:** Destination Address

**Send:** Sender Address      (Sender and Destionation is WE in ASCII, cause I dont know what else to use)

**DataID:** Frame data, either will be in:

  **[0x00] Preperation** - Gives output on the integrity and operation of all sensors and pre-flight checks. Will last 10 seconds before launch
  
  **[0x01] Launch** - Gives rapid sensor data. Will trigger when the accelerometer reaches a high threshold. Sends the data quickly without any normaization or averaging. Will last 10 seconds after threshold is reached
  
  **[0x02] Normal** - Default setting. Will give averaged samples of all sensors, as the data obtained from sensors can be done so much quicker than sending them all over LoRa. Will give more accurate results
  
  **[0xFE] ACK** - Acknowledgement of any data sent to the rocket
  
  **[0xFF] Alert** - Failure of any component or irregularities in the trajectory
  
**MessageID:** A count from 0-255, then overflow. Used to check if any frames were dropped or missing

**Millis:** Time in which the rocket has sent the data as a 32-bit long.

**MsgLen:** Message length
  
### Gyroscope/Accelerometer
Raw packet format will be a string of 14 bytes. Each data value will have the length of 2 bytes, taking up 12 bytes for the total gyro/accelerometer data, and 2 bytes for the checksum. Each 2 bytes will represent a signed 16-bit integer value, in the order of GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ. As of currently, the Gyroscope and Accelerometer is getting the average of 127 samples, which may not pick up sudden moments of acceleration such as take-off, and only serve to normalize the data. This value is subject to change in the future after further testing. If the module failes to initialize, it will send an error message as a value over LoRa containing "0x00", which will be sent to the serial monitor as "Gyroscope/Accelerometer initializtion failed". If initialization succeeds, "0xFF" will be sent instead

The following data will be sent to the Serial Monitor as such:
```
GyX ###
GyY ###
GyZ ###
AcX ###
AcY ###
AcZ ###
...
```

### Checksum
The LoRa system will implement a simple 8-bit checksum after each module data. If the checksum fails, the data will be outputted to the serial monitor as "CS Failed"



## Ground Module
The ground module will be responsible for recieving data sent by the rocket, as well as calculating signal strength and quality. The ground module will also be calculating the checksum to ensure the data from the rocket is correct. Any failed checksum will output the following as an example:
```
GyX CS FAILED
GyY CS FAILED
GyZ CS FAILED
AcX ###
AcY ###
AcZ ###
```
The serial monitor will be the main communication system between the LoRa system and the Ground team. Below are some examples of what data will be outputted to the Serial Monitor:
```
Mode Prep
Time ###
Acc/Gryo OK
Alt/Pressure OK
GPS OK
TEMP1 OK
TEMP2 OK
TEMP3 OK
TEMP4 ERROR
RSSI ###
SNR ###
```
Note, Launch and Normal will have the same output
```
Mode Launch
Time ###
GyX ###
GyY ###
GyZ ###
AcX ###
AcY ###
AcZ ###
ALT ###
PRESS ###
TEMP ###
GPSAlt ###
GPSLat ###
GPSLong ###
GPSSpeed ###
GPSAngle ###
GPSDoP ###
GPSFails ###
TEMP1 ###
TEMP2 ###
TEMP3 ###
TEMP4 ###
RSSI ###
SNR ###
```
```
Mode Alert
Time ###
Temp1 exceeded ### degrees 
RSSI ###
SNR ###
```
```
Mode Alert
Time ###
### packets have been lost
RSSI ###
SNR ###
```
```
Mode Alert
Time ###
Checksum failed on ### data values
RSSI ###
SNR ###
```
```
Mode Alert
Time ###
Gyro pulled irregular data, check orientation
RSSI ###
SNR ###
```
