/*!
 *  \file    lightwareSF40.c
 *  \author  Julian Della Guardia
 *  \date    07-09-2025
 *  \version 1.0
 *
 *  \brief   Library to control and read the Lightware SF40/c lidar
 *           based on: https://lightwarelidar.com/wp-content/uploads/2025/07/SF40-Laser-Scanner-Manual-Rev-7.pdf
 */

#include "lightwareSF40.h"
#include <unistd.h>


device_t lidarCOM;

typedef struct flags{
	union{
		uint16_t sr;
		struct{
			unsigned int rw :1,
			:5,
			pay_len :10;
		};
	};
}flag_t;

/*! \brief Calculate checksum for lidar data
 *  
 *  \param Data Data that has been received / is going to be send to the lidar
 * 
 *  \param Size Number of bytes in package
 * 
 *  \return returns 2 byte checksum
 */
uint16_t createCRC(uint8_t* data, uint16_t size){
    uint16_t crc = 0;
        for(uint32_t i = 0; i < size; ++i){
            uint16_t code = crc >> 8;
            code ^= data[i];
            code ^= code >> 4;
            crc = crc << 8;
            crc ^= code;
            code = code << 5;
            crc ^= code;
            code = code << 7;
            crc ^= code;
        }
    return crc;
} /*createCRC*/


/*! \brief Get a packet form the lidar
 *  
 *  \param payload location where payload needs to be saved
 *  
 *  \retval Amount of bytes in data packet.
 *  \retval -1 : first byte doesnt equal the start byte.
 *  \retval -2 : the received datapacket is either to small or to large.
 *  \retval -3 : checksums didn't match.
 * 
 */
int16_t getPacket(uint8_t *payload){
	uint16_t crc;
	flag_t header;

    for(int i = 0; i < 3; i++){
        readByte(&lidarCOM, &payload[i]);
    }

    // format the header into the seprate parts
    header.sr = payload[1] | (uint16_t)(payload[2] << 8);

    //check if first bit is start bit
	if(payload[0] != STARTBIT) return -1;
    if(header.pay_len < 1 || header.pay_len > MAX_RESPONSE_SIZE - 5) return -2;
	
	for (int i = 0; i < header.pay_len + 2; i++){
		readByte(&lidarCOM, &payload[i+3]);
	}
	
	crc = payload[header.pay_len + 3] | (payload[header.pay_len + 4] << 8);
	if (crc == createCRC(payload, 3 + header.pay_len)){
		return header.pay_len;
	}
	return -3;
} /*getPacket*/


/*! \brief Read data from specific lidar command
 *  
 *  \param command command that needs to be read from
 *  
 *  \param payload location where data needs to be saved
 *  
 *  \retval Amount of bytes in data packet.
 *  \retval -1 : if reading from the lidar has failed
 * 
 */
int16_t readCommand(uint8_t command, uint8_t* payload){
	flag_t header;
	header.pay_len = 1;
	header.rw = 0;
	
	flushBuffer(&lidarCOM);

	uint8_t packet[6];
	packet[0] = STARTBIT;
	packet[1] = header.sr;
	packet[2] = header.sr >> 8;
	packet[3] = command;
	packet[4] = createCRC(packet, 4);
	packet[5] = createCRC(packet, 4) >> 8;
	
	#ifdef DEBUG
	printf("Sending: ");
	#endif
	for(int i = 0; i < 6; i++){
		sendByte(&lidarCOM, packet[i]);
		#ifdef DEBUG
		printf("%02x ", packet[i]);
		#endif
	}
	#ifdef DEBUG
	printf("\n");
	#endif

    uint16_t cycles_Waited = 0;
    while(true){
        usleep(10);
        cycles_Waited++;

        //if the wait time is longer then 100ms report it as a not succesfull
        if(cycles_Waited > 10000){
			fprintf(stderr, "didnt receive response from lidar\n\r");
			return -1;
		}

        uint8_t receivedPayload[220] = {0};
		uint16_t receivedLenght = 0;
        if(canReadByte(&lidarCOM)) receivedLenght = getPacket(receivedPayload); 
		if(receivedPayload[3] == packet[3]){
			#ifdef DEBUG
			printf("Receiving: ");
			#endif
			for(int i = 0; i < receivedLenght+5; i++){
				payload[i] = receivedPayload[i];
				#ifdef DEBUG
				printf("%02x ", receivedPayload[i]);
				#endif
			}
			#ifdef DEBUG
			printf("\n");
			#endif
			return receivedLenght;
		}
    }
    return -1;
} /*readCommand*/


/*! \brief Read data to specific lidar command
 *  
 *  \param command command that needs to be written to
 *   
 *  \param payload payload that needs to be send
 * 
 *  \param data_len number of bytes in payload
 *  
 *  \retval  0 : data has been correctly send and proper response has been received
 *  \retval -1 : if sending from the lidar has failed
 * 
 */
int writeCommand(uint8_t command, void* payload, uint16_t data_len){
	flag_t header;
	header.pay_len = 1 + data_len;
	header.rw = 1;
	
	uint8_t packet[6 + data_len];
	packet[0] = STARTBIT;
	packet[1] = header.sr;
	packet[2] = header.sr >> 8;
	packet[3] = command;
	for(int i = 0; i < data_len; i++){
		packet[4 + i] = ((uint8_t *)payload)[i];
	}
	packet[4 + data_len] = createCRC(packet, 4 + data_len);
	packet[5 + data_len] = createCRC(packet, 4 + data_len) >> 8;
	
	#ifdef DEBUG
	printf("Sending: ");
	#endif
	for(int i = 0; i < 6 + data_len; i++){
		sendByte(&lidarCOM, packet[i]);
		#ifdef DEBUG
		printf("%02x ", packet[i]);
		#endif
	}
	#ifdef DEBUG
	printf("\n");
	#endif

    uint16_t cycles_Waited = 0;
    while(true){
        usleep(10);
        cycles_Waited++;
        
        //if the wait time is longer then 100ms report it as a not succesfull
        if(cycles_Waited > 10000){
			fprintf(stderr, "didnt receive response from lidar\n\r");
			return -1;
		}

        uint8_t receivedPayload[220] = {0};
        if(canReadByte(&lidarCOM)) getPacket(receivedPayload);
        if(receivedPayload[3] == command) return 0;
    }
    return -1;
}/*writeCommand*/

/*! \brief A 16 byte string indicating the product model name.
 *  
 *  \param name location where the name should be saved
 *   
 * 	\details This will always be SF40 followed by a null terminator.
 *		 	 You can use this to verify the SF40/C is connected and operational over the selected interface.
 */
void getName(char* name){
	uint8_t payload[22];
	readCommand(LIDAR_PRODUCT_NAME, payload);

	int i;
	for(i = 0; i < 16; i++){
		name[i] = (char)payload[i+4];
		if(payload[i+4] == '\0') break;
	}
	name[i+4] = '\0';
}/*getName*/


/*! \brief A 16 byte string (null terminated) of the serial identifier assigned during production
 *  
 *  \param serialNumber location where the serial number should be saved
 *   
 */
void getSerialNumber(char* serialNumber){
	uint8_t payload[22];

	uint8_t dataLenght = readCommand(LIDAR_SERIAL_NUMBER, payload);

	int i;
	for(i = 0; i < dataLenght; i++){
		serialNumber[i] = (char)payload[i+4];
		if(payload[i+4] == '\0') break;
	}
	payload[i+4] = '\0';
}/*getSerialNumber*/


/*! \brief Userdata allows 16 bytes to be stored for any purpose.
 *
 *  \param data data to be stored on the lidar;
 *   
 */
void sendUserData(uint8_t* data){
	writeCommand(LIDAR_USER_DATA, data, 16);
}/*sendUserData*/


/*! \brief Userdata allows 16 bytes to be read for any purpose.
 *
 *  \param data data to be read from the lidar;
 *   
 */
void getUserData(uint8_t* data){
	uint8_t payload[22];
	uint8_t dataLenght = readCommand(LIDAR_SERIAL_NUMBER, payload);
	
	for(int i = 0; i < dataLenght; i++){
		data[i] = (char)payload[i+4];
	}
}/*getUserData*/


/*! \brief The baud rate as used by the serial interface.
 *
 *  \param baudrate baudrate to be set
 *   
 * 	\details  This parameter only takes effect when the serial interface is first
 *			  enabled after power-up or restart.
 */
void setBaudrate(lidarBaudrate_t baudrate){
	writeCommand(LIDAR_BAUD_RATE, &baudrate, 1);
}/*setBaudrate*/


/*! \brief Current safety token required for performing certain operations. 
 *
 *  \return 16 bit security token
 *   
 * 	\details Once a token has been used it will expire and a new token is created.
 */
uint16_t getToken(void){
	uint8_t payload[8];
	
	readCommand(LIDAR_TOKEN, payload);

	return payload[4] + ((uint16_t)payload[5] << 8);
}/*getToken*/


/*! \brief Save current lidar settings.
 *
 *  \param token security token.
 *   
 * 	\details  Several commands write to parameters that can persist across power cycles. 
 *            These parameters will only persist once the Save parameters command has been written with the appropriate token . 
 *            The safety token is used to prevent unintentional writes and once a successful save has completed the token will expire.
 */
void saveParameters(uint16_t token){
	writeCommand(LIDAR_SAVE_PARAMETERS, &token, 2);
}/*saveParameters*/


/*! \brief Writing the safety token to this function will restart the SF40/C.
 *
 *  \param token security token.
 */
void restartLidar(uint16_t token){
	writeCommand(LIDAR_RESET, &token, 2);
}/*restartLidar*/


/*! \brief The incoming voltage is directly measured from the incoming 5 V line.
 *
 *  \return A float with the logic voltage in volts.
 */
float getVoltage(void){
	uint8_t payload[10];

	readCommand(LIDAR_INCOMING_VOLTAGE, payload);

	return LIDAR_VOLTAGE((uint32_t)payload[7]<<24 || (uint32_t)payload[6]<<16 || 
						 (uint32_t)payload[5]<<8 || (uint32_t)payload[4]);
}/*getVoltage*/
    

/*! \brief Reading this function will return the voltage drawn by the motor.
 *
 *  \return A float with the motor voltage in volts.
 */
float getMotorVoltage(void){
	uint8_t payload[8];
	
	readCommand(LIDAR_MOTOR_VOLTAGE, payload);

	return (float)(payload[5]<<8 | payload[4])/1000.0f;
}/*getMotorVoltage*/
    

/*! \brief Reading this function will return the temperature.
 *
 *  \return A float with the temperature in degree's Celcius
 */
float getTemperature(void){
	uint8_t payload[10];
	
	readCommand(LIDAR_TEMPRATURE, payload);

	return ((uint32_t)payload[7]<<24 || (uint32_t)payload[6]<<16 || 
			(uint32_t)payload[5]<<8 || (uint32_t)payload[4]) / 100.0f;
}/*getTemperature*/
    

/*! \brief Reading this function will return the number of full revolutions since start-up
 *
 *  \return 32 bit number with the amount of revolutions
 * 
 *  \details Note that this value will reset to zero after 4294967295 revolutions.
 */
uint32_t getRevolutions(void){
	uint8_t payload[10];
	
	readCommand(LIDAR_TEMPRATURE, payload);

	return 	((uint32_t)payload[7]<<24 || (uint32_t)payload[6]<<16 || 
			 (uint32_t)payload[5]<<8 || (uint32_t)payload[4]);
}/*getRevolutions*/
    

/*! \brief Reading this function will return a byte with the current state of all alarms.
 *
 *  \param alarms array if 1 bit states for all alarms
 * 
 *  \details Each bit represents 1 of the 7 alarms, if the bit is set then the alarm is currently triggered. 
 * 			 The most significant bit is set when any alarm is currently triggered.
 */
void getAlarmState(alarms_t* alarms){
	uint8_t payload[7];
	
	readCommand(LIDAR_ALARM_STATE, payload);

	alarms->byte = payload[4];
}/*getAlarmState*/
    

/*! \brief Reading this function will return the current state of the motor. 
 *
 *  \return returns motor state
 * 
 *  \details This can be useful to debug or check start-up conditions.
 */
motorState_t getMotorState(void){
	uint8_t payload[7];
	
	readCommand(LIDAR_MOTOR_STATE, payload);

	return (motorState_t)payload[4];
}/*getMotorState*/


/*! \brief Turn on or off continuosly outputting data without requests.
 *
 *  \param enabled turn on or off the stream function
 */
void enableStream(bool enabled){
	uint8_t payload[4] = {0};
    payload[0] = enabled ? 3 : 0;

    writeCommand(LIDAR_STREAM, payload, 4);
}

uint8_t getStreamState(void){
	uint8_t payload[7];
	
	readCommand(LIDAR_STREAM, payload);

	return payload[4];
}

/*! \brief Retrieve complete stream packed from incomming buffer
 *  
 *  \param outputData Location where streamdata packet needs to be saved
 *  
 *  \retval  0 : the outputeData has correctly be update with a new list of data points.
 *  \retval -1 : failed getting packet
 *  \retval -2 : received data is not streamed data. 
 * 
 */
int getStream(streamOutput_t* outputData){
	uint8_t payload[420];
	if(getPacket(payload) <= 0) return -1;
	if(payload[3] != LIDAR_DISTANCE_OUTPUT) return -2;

	outputData->alarmState.byte = payload[4];
	outputData->pps 			= (uint16_t)(payload[6]<<8 | payload[5]);
	outputData->forwardOffset 	= (int16_t)(payload[8]<<8 | payload[7]);
	outputData->motorVoltage	= (int16_t)(payload[10]<<8 | payload[9]);
	outputData->revolutionIndex = payload[11];
	outputData->pointTotal		= (uint16_t)(payload[13]<<8 | payload[12]);
	outputData->pointCount		= (uint16_t)(payload[15]<<8 | payload[14]);
	outputData->pointStartIndex = (uint16_t)(payload[17]<<8 | payload[16]);

	for(uint16_t i = 0; i < outputData->pointCount; i++){
		outputData->pointDistances[i] = (int16_t)(payload[(i*2)+19]<<8 | payload[(i*2)+18]);
	}

	return 0;
}/*getStream*/


/*! \brief Writing to this function will enable or disable the firing of the laser.
 *
 *  \param enabled turn on or off the laser
 */
void enableLaser(bool enabled){
	writeCommand(LIDAR_LASER_FIRING, &enabled, 1);
}/*enableLaser*/


/*! \brief Reading this function will indicate the current laser firing state.
 *
 *  \return boolean if laser is firing or not
 */
bool checkLaser(void){
	uint8_t payload[7];

	readCommand(LIDAR_LASER_FIRING, payload);

	return (bool)payload[4];
}/*checkLaser*/


/*! \brief The output rate controls the amount of data sent to the host when distance output streaming is enabled.
 *
 *  \param outputRate Amount of points per second
 */
void setOutputRate(lidarOutputRate_t outputRate){
	writeCommand(LIDAR_OUTPUT_RATE, &outputRate, 1);
}/*setOutputRate*/


/*! \brief The output rate controls the amount of data sent to the host when distance output streaming is enabled.
 *
 *  \return Amount of points per second
 */
lidarOutputRate_t getOutputRate(void){
	uint8_t payload[7];

	readCommand(LIDAR_OUTPUT_RATE, payload);

	return (lidarOutputRate_t)payload[4];
}/*getOutputRate*/


/*! \brief Reading this command will return the average , closest and furthest distance within an angular view
 *		   pointing in a specified direction.
 *
 *  \param distanceSettings struct with different lidar distance settings
 * 
 *  \param receivedDistances struct where the received distances should be saved.
 */
void getDistance(writeDistance_t distanceSettings, readDistance_t* receivedDistances){
	writeCommand(LIDAR_DISTANCE, &distanceSettings, 6);

	uint8_t payload[18];

	readCommand(LIDAR_DISTANCE, payload);

	receivedDistances->averageDistance 	= (payload[5]<<8 | payload[4]);
	receivedDistances->closestDistance 	= (payload[7]<<8 | payload[6]);
	receivedDistances->furthestDistance = (payload[9]<<8 | payload[8]);
	receivedDistances->angle 			= (payload[11]<<8 | payload[10]);
	receivedDistances->calculationTime	= ((uint32_t)payload[15]<<24 | (uint32_t)payload[14]<<16 || 
			 							   (uint32_t)payload[13]<<8 | (uint32_t)payload[12]);
}/*getDistance*/


/*! \brief The forward offset affects the position of the 0 degree direction.
 *
 *  \param offset offset to be applied
 * 
 *  \details The orientation label on the front of the SF40/C marks the default 0 degree direction.
 */
void setOffset(int16_t offset){
	writeCommand(LIDAR_FORWARD_OFFSET, &offset, 2);
}/*setOffset*/


/*! \brief The forward offset affects the position of the 0 degree direction.
 *
 *  \return offset that is applied
 * 
 *  \details The orientation label on the front of the SF40/C marks the default 0 degree direction.
 */
int16_t getOffset(void){
	uint8_t payload[8];
	readCommand(LIDAR_DISTANCE, payload);

	return (int16_t)((payload[5] << 8) | payload[4]);
}


/*! \brief Function can be used to set parameters for a specific alarm
 *
 *  \param alarmSettings Struct with alarm settings
 * 
 *  \param alarmNumber Select alarm number (1 to 7)
 */
void setAlarm(alarm_t alarmSettings, lidar_alarm_t alarmNumber){
	writeCommand(alarmNumber, &alarmSettings, 7);
}/*setAlarm*/


/*! \brief Function can be used to check parameters for a specific alarm
 * 
 *  \param alarmNumber Select alarm number (1 to 7)
 * 
 *  \return Struct with alarm settings
 */
alarms_t checkAlarm(lidar_alarm_t alarmNumber){
	uint8_t payload[7];  
    readCommand(alarmNumber, payload);

	alarms_t alarms;
    alarms.byte = payload[4]; 

    return alarms;
}


/*! \brief Establish serial connection with lidar
 * 
 *  \param port tty portname where the lidar is connected to
 * 
 *  \param baudrate select baudrate that the lidar is expecting
 */
void setupLidar(const char* port, lidarBaudrate_t baudrate){
	
	switch(baudrate){
		case LIDAR_115K2:
			lidarCOM.baudrate = B115200;
			break;
		case LIDAR_230K4:
			lidarCOM.baudrate = B230400;
			break;
		case LIDAR_460K8:
			lidarCOM.baudrate = B460800;
			break;
		case LIDAR_921K6:
			lidarCOM.baudrate = B921600;
			break;
		default:
			lidarCOM.baudrate = B115200;
			break;
	}
	
	lidarCOM.devicePort = port;
	setupDevice(&lidarCOM);

	flushBuffer(&lidarCOM);
}/*setupLidar*/


/*! \brief Close serial connection with lidar
 */
void closeLidar(void){
	closeDevice(&lidarCOM, true);
}