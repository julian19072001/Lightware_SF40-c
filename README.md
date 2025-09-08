# Lightware_SF40-c
Control library for Lighware SF40/c lidar to be used with RPI-serial repo

##  
### `void getName(char* name)`

**Description:**  
Retrieve the product model name of the LIDAR.

**Parameters:**  
- `name` — Location where the 16-byte model name will be saved (null-terminated).  

**Details:**  
The returned name is always `"SF40"` followed by a null terminator. Can be used to verify that the LIDAR is connected and operational.

---

### `void getSerialNumber(char* serialNumber)`

**Description:**  
Retrieve the serial number assigned to the LIDAR during production.

**Parameters:**  
- `serialNumber` — Location where the 16-byte serial number will be saved (null-terminated).  

---

### `void sendUserData(uint8_t* data)`

**Description:**  
Store 16 bytes of user-defined data on the LIDAR.

**Parameters:**  
- `data` — Data to be stored.

---

### `void getUserData(uint8_t* data)`

**Description:**  
Read 16 bytes of user-defined data stored on the LIDAR.

**Parameters:**  
- `data` — Buffer to receive the data.

---

### `void setBaudrate(lidarBaudrate_t baudrate)`

**Description:**  
Set the LIDAR serial interface baud rate.

**Parameters:**  
- `baudrate` — Desired baud rate.  

**Details:**  
This setting takes effect only after a power-up or restart.

---

### `uint16_t getToken(void)`

**Description:**  
Retrieve the current safety token required for certain operations.

**Returns:**  
- 16-bit security token.  

**Details:**  
Tokens expire after use; a new token is generated automatically.

---

### `void saveParameters(uint16_t token)`

**Description:**  
Save current LIDAR settings permanently.

**Parameters:**  
- `token` — Security token required to authorize saving.  

**Details:**  
Prevents unintentional writes; token expires after successful save.

---

### `void restartLidar(uint16_t token)`

**Description:**  
Restart the LIDAR using a security token.

**Parameters:**  
- `token` — Security token.

---

### `float getVoltage(void)`

**Description:**  
Read the incoming logic voltage of the LIDAR.

**Returns:**  
- Voltage in volts.

---

### `float getMotorVoltage(void)`

**Description:**  
Read the voltage supplied to the motor.

**Returns:**  
- Motor voltage in volts.

---

### `float getTemperature(void)`

**Description:**  
Read the current LIDAR temperature.

**Returns:**  
- Temperature in degrees Celsius.

---

### `uint32_t getRevolutions(void)`

**Description:**  
Read the number of full motor revolutions since startup.

**Returns:**  
- 32-bit revolution count.  

**Details:**  
Resets to zero after 4,294,967,295 revolutions.

---

### `alarms_t getAlarmState(void)`

**Description:**  
Read the current state of all alarms.

**Returns:**  
- Byte where each bit represents one of seven alarms.  
- Most significant bit set indicates any alarm is triggered.

---

### `motorState_t getMotorState(void)`

**Description:**  
Read the current state of the motor.

**Returns:**  
- Motor state (enumeration).  

**Details:**  
Useful for debugging or checking startup conditions.

---

### `void enableStream(bool enabled)`

**Description:**  
Enable or disable continuous distance output streaming.

**Parameters:**  
- `enabled` — `true` to start streaming, `false` to stop.

---

### `int getStream(streamOutput_t* outputData)`

**Description:**  
Retrieve a complete data stream packet from the LIDAR.

**Parameters:**  
- `outputData` — Location where stream data will be saved.

**Returns:**  
- `0` — Packet successfully retrieved.  
- `-1` — Failed to get packet.  
- `-2` — Received data is not a streamed packet.

---

### `void enableLaser(bool enabled)`

**Description:**  
Enable or disable firing of the LIDAR laser.

**Parameters:**  
- `enabled` — `true` to turn on, `false` to turn off.

---

### `bool checkLaser(void)`

**Description:**  
Check whether the laser is currently firing.

**Returns:**  
- `true` if firing, `false` otherwise.

---

### `void setOutputRate(lidarOutputRate_t outputRate)`

**Description:**  
Set the amount of points per second in streamed distance output.

**Parameters:**  
- `outputRate` — Desired output rate.

---

### `lidarOutputRate_t getOutputRate(void)`

**Description:**  
Read the currently set output rate.

**Returns:**  
- Points per second in distance output stream.

---

### `void getDistance(writeDistance_t distanceSettings, readDistance_t* receivedDistances)`

**Description:**  
Measure distances within a specified angular view.

**Parameters:**  
- `distanceSettings` — Settings for the distance measurement.  
- `receivedDistances` — Structure where average, closest, and furthest distances will be saved.

---

### `void setOffset(int16_t offset)`

**Description:**  
Set the forward offset to adjust the 0-degree direction.

**Parameters:**  
- `offset` — Offset in degrees.

---

### `int16_t getOffset(void)`

**Description:**  
Read the currently applied forward offset.

**Returns:**  
- Offset in degrees.

---

### `void setAlarm(alarm_t alarmSettings, lidar_alarm_t alarmNumber)`

**Description:**  
Configure parameters for a specific alarm.

**Parameters:**  
- `alarmSettings` — Alarm configuration structure.  
- `alarmNumber` — Alarm number (1–7).

---

### `alarms_t checkAlarm(lidar_alarm_t alarmNumber)`

**Description:**  
Check the configuration of a specific alarm.

**Parameters:**  
- `alarmNumber` — Alarm number (1–7).  

**Returns:**  
- Struct with alarm settings.

---

### `void setupLidar(const char* port, lidarBaudrate_t baudrate)`

**Description:**  
Establish serial connection with the LIDAR.

**Parameters:**  
- `port` — Serial port device name.  
- `baudrate` — Baud rate to use.

---

### `void closeLidar(void)`

**Description:**  
Close the serial connection with the LIDAR.  