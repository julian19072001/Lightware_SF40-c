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


/*! \brief Get a packed form the lidar
 *  
 *  \param payload location where payload needs to be saved
 *  
 *  \retval Amount of bytes in data packed.
 *  \retval -1 : first byte doesnt equal the start byte.
 *  \retval -2 : the received datapacked is either to small or to large.
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
 *  \retval Amount of bytes in data packed.
 *  \retval -1 : if reading from the lidar has failed
 * 
 */
int16_t readCommand(uint8_t command, uint8_t* payload){
	flag_t header;
	header.pay_len = 1;
	header.rw = 0;
	
	uint8_t packet[6];
	packet[0] = STARTBIT;
	packet[1] = header.sr;
	packet[2] = header.sr >> 8;
	packet[3] = command;
	packet[4] = createCRC(packet, 4);
	packet[5] = createCRC(packet, 4) >> 8;
	
	for(int i = 0; i < 6; i++){
		sendByte(&lidarCOM, packet[i]);
	}

    uint16_t cycles_Waited = 0;
    while(true){
        _delay_us(10);
        cycles_Waited++;

        //if the wait time is longer then 100ms report it as a not succesfull
        if(cycles_Waited > 10000){
			fprintf(stderr, "didnt receive response from lidar\n\r");
			return -1;
		}

        uint8_t receivedPayload[200] = '0';
		uint16_t receivedLenght = 0;
        if(canReadByte(&lidarCOM)) receivedLenght = getPacket(&receivedPayload);
        if(receivedPayload[3] == command) return receivedLenght;
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
		packet[4 + i] = *payload >> (i * 8);
	}
	packet[4 + data_len] = createCRC(packet, 4 + data_len);
	packet[5 + data_len] = createCRC(packet, 4 + data_len) >> 8;
	
	for(int i = 0; i < 6 + data_len; i++){
		sendByte(lidarCOM, packet[i]);
	}

    uint16_t cycles_Waited = 0;
    while(true){
        _delay_us(10);
        cycles_Waited++;
        
        //if the wait time is longer then 100ms report it as a not succesfull
        if(cycles_Waited > 10000){
			fprintf(stderr, "didnt receive response from lidar\n\r");
			return -1;
		}

        uint8_t receivedPayload[200] = '0';
        if(canReadByte(&lidarCOM)) getPacket(&receivedPayload);
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
	uint8_t payload[24];

	uint8_t dataLenght = readCommand(LIDAR_PRODUCT_NAME, &payload);

	int i;
	for(i = 0; i < dataLenght; i++){
		name[i] = (char)payload[i+4];
		if(payload[i+4] == '\0') break;
	}
	payload[i+4] = '\0';
}/*getName*/


/*! \brief A 16 byte string (null terminated) of the serial identifier assigned during production
 *  
 *  \param serialNumber location where the serial number should be saved
 *   
 */
void getSerialNumber(char* serialNumber){
	uint8_t payload[24];

	uint8_t dataLenght = readCommand(LIDAR_SERIAL_NUMBER, &payload);

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
	uint8_t payload[24];
	uint8_t dataLenght = readCommand(LIDAR_SERIAL_NUMBER, &payload);
	
	for(int i = 0; i < dataLenght; i++){
		data[i] = (char)payload[i+4];
		if(payload[i+4] == '\0') break;
	}
}/*getUserData*/