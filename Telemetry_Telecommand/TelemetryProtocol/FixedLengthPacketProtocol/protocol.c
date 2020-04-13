#include "Telemetry_Packet_Protocol.h"


#define DEBUG (1)

/***** Functions for the ground station side *****/

/* This function will decode the time of the CubeSat by looking at the packet received */
double Decode_CubeSat_Time (uint8_t *packet)
{
	// Packet header structure: First 8 bytes are time data
	double julianDate = bytes_to_double(&packet[0]);
	
	if (DEBUG) printf("Value: %lf\n",  julianDate);
	
	return julianDate;	
}

/* This function will decode packets that have come into the ground station */
uint8_t Decode_Ground_Packet(uint8_t *packet, uint8_t hashValue)
{
	double julianDate = 0.0;
	
	// Decode the packet given to the ground station from the CubeSat
	uint8_t command = (packet[8] & 0x07);
	
	// First decode the opcode
	switch (command)
	{
		case (SAT_STATUS):
			printf("Received a packet containing the status of the CubeSat\n");
			// Handle the request here
			julianDate = Decode_CubeSat_Time(packet);
			Handle_Sat_Status(packet);
			break;
		case (SCI_DATA):
			printf("Received a packet of science data\n");
			// Handle the request here
			julianDate = Decode_CubeSat_Time(packet);
			Handle_Sat_Sci(packet);
			break;
		case (SAT_LOCATION):
			printf("Received a packet with the CubeSat's location\n");
			// Handle the request here
			julianDate = Decode_CubeSat_Time(packet);
			Handle_Sat_Location(packet);
			break;
		case (SAT_ACK):
			printf("Received an Acknowledgement\n");
			// Handle the request here
			julianDate = Decode_CubeSat_Time(packet);
			if (Handle_Sat_Ack(packet, hashValue) == 1)
			{
				return command | 0xF0;
			}
			break;
		default:
			fprintf(stderr, "Invalid packet command received\n");
			return FAIL; // Packet cold not be decoded	
	}
	
	return command;
}

/* Create the packet used for sending a Kill signal */
uint8_t Create_Kill_Packet(uint8_t *retPacket)
{
	// Opcode: 1111
	
	uint8_t packet = KILL;
	
	retPacket[0] = packet;
	
	return SUCCESS;
}

/* Create a log science event packet */
uint8_t Create_Command_LogSciEvent(uint8_t *retPacket, uint8_t logType, time_of_day *StartTime)
{	
	uint8_t packet[3] = {0};
	
	// 1st byte is 00010X00 where X is the value of logtype, 1 or 0
	
	if (logType == 0) // log now
	{
		packet[0] = REQ_LOG;
	}
	else if (logType == 1) // log at a specific later date
	{
		packet[0] = REQ_LOG_TIME;
		
		// Packet               [0]       [1]       [2]
		// Packet structure: 0001 010H HHHH MMMM MMSS SSSS	
		// H = hour, M = minute, S = seconds
		
		packet[0] = packet[0] | ((StartTime->hour & 0x10) >> 5);
		packet[1] = (StartTime->hour & 0x0F) << 4; 
		
		uint8_t temp = (StartTime->min & 0x3C) >> 2;
		packet[1] = packet[1] | temp;
		temp = (StartTime->min & 0x03) << 6;
		packet[2] = temp | (StartTime->sec & 0x3F);
	}
	else
	{
		retPacket = NULL;
		return FAIL; // Packet could not be created
	}

	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 3);
	
	return SUCCESS;
}

/* Create a request for the status of the CubeSat */ 
uint8_t Create_Request_Status(uint8_t *retPacket)
{
	// 1st byte is 0010 0000, where the bottom 4 bits could be used for optional messages in the future
	uint8_t packet = REQ_STATUS;
	
	// Return the packet to be used outside this function
	retPacket[0] = packet;
	
	return SUCCESS;
}

/* Create a request for science data */
uint8_t Create_Request_ScienceData(uint8_t *retPacket, time_of_day *StartTime, time_of_day *FinishTime, uint32_t chunkSize)
{
	uint8_t packet[7] = {0};

	// 1st byte is 0100 0X00, where X is whether there is a chunk or end time
	if ((chunkSize != 0) && (chunkSize < 0xFFFFFF))
	{
		packet[0] = REQ_SCI_DATA2;
		packet[1] = StartTime->hour;
		packet[2] = StartTime->min;
		packet[3] = StartTime->sec;
		packet[4] = (chunkSize & 0x00FF0000);
		packet[5] = (chunkSize & 0x0000FF00);
		packet[6] = (chunkSize & 0x000000FF);
	} 
	else if (chunkSize == 0) // chunk is not specified
	{
		packet[0] = REQ_SCI_DATA;
		packet[1] = StartTime->hour;
		packet[2] = StartTime->min;
		packet[3] = StartTime->sec;
		packet[4] = FinishTime->hour;
		packet[5] = FinishTime->min;
		packet[6] = FinishTime->sec;
	}
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 7);
	
	return SUCCESS;
}

/* Create a request for the CubeSat's location (Keplerian Elements) */
uint8_t Create_Request_Location(uint8_t *retPacket)
{
	// 1st byte is 0101 0000, where the bottom 4 bits could be used for optional messages in the future
	uint8_t packet = REQ_LOCATION;
	
	// Return the packet to be used outside this function
	retPacket[0] = packet;
	return SUCCESS;
}

/* Create a packet to update the Keplerian elements on the CubeSat */
uint8_t Create_Command_UpdateKep(uint8_t *retPacket, uint8_t KepElem1, uint8_t KepElem2, uint8_t KepElem3)
{
	// 1st byte is 0011 0000
	// The other 3 bytes are each Keplerian elements
	uint8_t packet[4] = {UPDATE_KEP,
						 KepElem1,
						 KepElem2,
						 KepElem3};
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 4);	
	
	return SUCCESS;
}


/***** Functions for the CubeSat side *****/

/* This function will decode packets that have come into the CubeSat */
uint8_t Decode_Sat_Packet(uint8_t *packet)
{
	// Decode the packet given to the CubeSat from the ground station
	uint16_t command = (packet[0] & 0xFF);
	
	// First decode the opcode
	switch (command)
	{
		case (REQ_LOG):
			printf("Received a command to log a science event now\n");
			// Handle the request here
			Handle_LogSci_Packet(packet);
			break;
		case (REQ_LOG_TIME):
			printf("Received a command to log a science event\n");
			// Handle the request here
			break;
		case (REQ_STATUS):
			printf("Received a request for the status of the CubeSat\n");
			// Handle the request here
			Handle_ReqStatus_Packet(packet);
			break;
		case (UPDATE_KEP):
			printf("Received a command to update the CubeSat's location with Keplerian elements\n");
			// Handle the request here
			Handle_UpdateKep_Packet(packet);
			break;
		case (REQ_SCI_DATA):
			printf("Received a request for science data\n");
			// Handle the request here
			Handle_ReqSciData_Packet(packet);
			break;
		case (REQ_SCI_DATA2):
			printf("Received a request for a chunk of science data\n");
			// Handle the request here
			Handle_ReqSciData_Packet(packet);
			break;
		case (REQ_LOCATION):
			printf("Received a request for the CubeSat's location\n");
			// Handle the request here
			Handle_ReqLoc_Packet(packet);
			break;
		case (KILL):
			printf("Received a kill command\n");
			// Handle the request here
			Handle_Kill_Packet(packet);
			break;
		default:
			fprintf(stderr, "Invalid packet command received\n");
			return FAIL; // Packet cold not be decoded	
	}
	
	return command;
}

/* Create a packet responding to the status request from ground station */
uint8_t Create_Response_Status(uint8_t *retPacket, uint8_t status, double julianDate)
{
	// Packet header: TIME(8 bytes) 0000 0XXX
	
	//Initialize the packet
	uint8_t packet[10] = {0};

	// Convert the date into bytes
	double_to_bytes(julianDate, &packet[0]);
	
	// The command code filling in the XXX is 010
	packet[8] = 0x02;

	// Store the status 
	packet[9] = status;
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 10);
	
	return SUCCESS;
}

/* Create a packet containing science payload data */
uint8_t Create_ScienceData(uint8_t *retPacket, ScienceDataPoint *data, uint16_t dataLength, double julianDate)
{
	// Packet header: TIME(8 bytes) 0000 0XXX
	
	// Initialize the packet
	uint8_t packet[24] = {0};
	
	// Convert the date into bytes
	double_to_bytes(julianDate, &packet[0]);
	
	// The command code filling in the XXX is 100
	packet[8] = 0x04;
	
	// Store the science payload data
	for (uint8_t i = 0; i < dataLength; i++)
	{
		memcpy(&packet[9 + 5*i], &data[i], 5);
	}
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 9 + 5*dataLength);
	
	return SUCCESS;
}

/* Acknowledgement of certain messages or responses */
uint8_t Create_Acknowledgement(uint8_t *retPacket, uint8_t hashValue, double julianDate)
{
	// Packet header: TIME(8 bytes) 0000 0XXX
	
	// Initialize the packet
	uint8_t packet[10] = {0};
	
	// Convert the date into bytes
	double_to_bytes(julianDate, &packet[0]);
	
	// The command code filling in the XXX is 111
	packet[8] = 0x07;
	
	// Store the hashValue
	packet[9] = hashValue;	
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 10);
	
	return SUCCESS;
}

/* Create a packet with the current CubeSat's location (Keplerian Elements) */
uint8_t Create_LocationData(uint8_t *retPacket, float latitude, float longitude, float altitude, double julianDate)
{
	// Packet header: TIME(8 bytes) 0000 0XXX

	// Initialize the packet
	uint8_t packet[21] = {0};
	
	// Convert the date into bytes
	double_to_bytes(julianDate, &packet[0]);
	
	// The command code filling in the XXX is 101
	packet[8] = 0x05;
	
	// Store the location data
	float_to_bytes(latitude, &packet[9]);
	float_to_bytes(longitude, &packet[13]);
	float_to_bytes(altitude, &packet[17]);
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 21);
	
	return SUCCESS;
}

/*** Functions for handling packets sent by the ground station to the CubeSat***/

void Handle_Kill_Packet(uint8_t *packet)
{
	// Send an Ack
	// Send interrupt to power modes to kill the system
	
}

void Handle_LogSci_Packet(uint8_t *packet)
{
	// Write to SPI FRAM in the location for a logged science event
	// Write the time that the data should be logged at.
	// Send an Ack
}

void Handle_ReqStatus_Packet(uint8_t *packet)
{
	// Decode packet to see what type of status is requested
	// Go to transmit mode > Send a packet with the requested status
}

void Handle_ReqSciData_Packet(uint8_t *packet)
{
	// Decode packet to see which data points to get. 
	// Access I2C FRAM and read the proper data points
	// Go to transmit mode, send the data down to earth
}

void Handle_ReqLoc_Packet(uint8_t *packet)
{
	// Access SPI FRAM to get the latitude, longitude, and altitude
	// Create a packet that will go back to the ground station
}


void Handle_UpdateKep_Packet(uint8_t *packet)
{
	// Read new keplerian elements. 
	// Store them into the shared SPI FRAM.
	// Send an interrupt to the mechanical board
}



/* These functions will print out the response from the CubeSat */
void Handle_Sat_Status(uint8_t *packet)
{
	// Read the status
	uint8_t status = packet[9];
	
	// Print out the status of the requested packet
	//Hal_UART_Transmit(&huart, 
	
}

void Handle_Sat_Sci(uint8_t *packet)
{
	char buffer[100] = {0};
	float time;
	uint8_t energy;
	
	snprintf(&buffer[0],  33, "Time: %f\tData: %u\n", bytes_to_float(&packet[9]),  packet[13]);
	snprintf(&buffer[33], 33, "Time: %f\tData: %u\n", bytes_to_float(&packet[14]),  packet[18]);
	snprintf(&buffer[66], 33, "Time: %f\tData: %u\n", bytes_to_float(&packet[19]),  packet[23]);
	
	// Hal_UART_Transmit(&huart, buffer,

}

uint8_t Handle_Sat_Ack(uint8_t *packet, uint8_t hashValue)
{
	// Check if the proper hash value was given
	if (packet[9] == hashValue)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void Handle_Sat_Location(uint8_t *packet)
{
	
}