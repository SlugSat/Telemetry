#include "protocol.h"


#define DEBUG (0)

/***** Functions for the ground station side *****/

/* This function will decode the time of the CubeSat by looking at the packet received */
uint8_t Decode_CubeSat_Time (uint8_t *packet, time_of_day *satTime)
{
	// Packet header structure: HHHH H0MM MMMM 0SSS SSS0 0XXX
	//time_of_day satTime;
	satTime->hour = (packet[0] >> 3) & 0x1F;
	satTime->min = ((packet[0] & 0x03) << 4) | ((packet[1] >> 4) & 0x0F);
	satTime->sec = ((packet[1] & 0x07) << 3) | ((packet[2] >> 5) & 0x07);
	
	if (DEBUG) printf("\nHours: 0x%02x\t, Min: 0x%02x\t, Sec: 0x%02x\n",  satTime->hour, satTime->min, satTime->sec);
	if (DEBUG) printf("\nHours: %d\t, Min: %d\t, Sec: %d\n",  satTime->hour, satTime->min, satTime->sec);
	
	return 1;	
}

/* This function will decode packets that have come into the ground station */
uint8_t Decode_Ground_Packet(uint8_t *packet)
{
	// Decode the packet given to the ground station from the CubeSat
	uint8_t command = (packet[2] & 0x07);
	time_of_day satTime;
	// First decode the opcode
	switch (command)
	{
		case (SAT_STATUS):
			printf("Received a packet containing the status of the CubeSat\n");
			// Handle the request here
			Decode_CubeSat_Time(packet, &satTime);
			break;
		case (SCI_DATA):
			printf("Received a packet of science data\n");
			// Handle the request here
			break;
		case (SAT_LOCATION):
			printf("Received a packet with the CubeSat's location\n");
			// Handle the request here
			break;
		case (SAT_ACK):
			printf("Received an Acknowledgement\n");
			// Handle the request here
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
			break;
		case (REQ_LOG_TIME):
			printf("Received a command to log a science event\n");
			// Handle the request here
			break;
		case (REQ_STATUS):
			printf("Received a request for the status of the CubeSat\n");
			// Handle the request here
			break;
		case (UPDATE_KEP):
			printf("Received a command to update the CubeSat's location with Keplerian elements\n");
			// Handle the request here
			break;
		case (REQ_SCI_DATA):
			printf("Received a request for science data\n");
			// Handle the request here
			break;
		case (REQ_SCI_DATA2):
			printf("Received a request for a chunk of science data\n");
			// Handle the request here
			break;
		case (REQ_LOCATION):
			printf("Received a request for the CubeSat's location\n");
			// Handle the request here
			break;
		case (KILL):
			printf("Received a kill command\n");
			// Handle the request here
			break;
		default:
			fprintf(stderr, "Invalid packet command received\n");
			return FAIL; // Packet cold not be decoded	
	}
	
	return command;
}

/* Create a packet responding to the status request from ground station */
uint8_t Create_Response_Status(uint8_t *retPacket, uint8_t status, time_of_day SatTime)
{
	// Packets structure: HHHH H0MM MMMM 0SSS SSS0 0XXX
	// H = hour, M = minute, S = seconds, XXX represents the command
	
	// The command code filling in the XXX is 010
	uint8_t packet[4] = {0};
	
	// HHHH H0MM
	packet[0] = SatTime.hour << 3;
	packet[0] = packet[0] | ((SatTime.min & 0x30) >> 4);
	
	// MMMM 0SSS
	packet[1] = (SatTime.min & 0x0F) << 4;
	packet[1] = packet[1] | ((SatTime.sec & 0x38) >> 3);
	
	// SSS0 0XXX
	packet[2] = ((SatTime.sec & 0x07) << 5);
	packet[2] = packet[2] | SAT_STATUS;
	
	packet[3] = status;
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 4);
	
	return SUCCESS;
}

/* Create a packet containing science payload data */
uint8_t Create_ScienceData(uint8_t *retPacket, uint32_t *data, uint16_t dataLength, time_of_day SatTime)
{
	// Packets structure: HHHH H0MM MMMM 0SSS SSS0 0XXX
	// H = hour, M = minute, S = seconds, XXX represents the command
	
	// The command code filling in the XXX is 100
	uint8_t packet[MAX_PACK_SIZE] = {0};
	
	// HHHH H0MM
	packet[0] = SatTime.hour << 3;
	packet[0] = packet[0] | ((SatTime.min & 0x30) >> 4);
	
	// MMMM 0SSS
	packet[1] = (SatTime.min & 0x0F) << 4;
	packet[1] = packet[1] | ((SatTime.sec & 0x38) >> 3);
	
	// SSS0 0XXX
	packet[2] = ((SatTime.sec & 0x07) << 5);
	packet[2] = packet[2] | SCI_DATA;
	
	// Store the science payload data
	// The data input to this function is 3 bye value in the format of science payload data
	// i.e. HHHH HMMM MMMS SSSS SDDD DDDD where H, m, and s are the time and D is the data value, 0-100
	for (int i = 0, j = 3; i < dataLength; i++)
	{
		packet[j + 0] = (data[i] & 0xFF0000) >> 16;
		packet[j + 1] = (data[i] & 0x00FF00) >> 8;
		packet[j + 2] = (data[i] & 0x0000FF) >> 0;
		j += 3;
	}
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 3 + dataLength);
	
	return SUCCESS;
}

/* Acknowledgement of certain messages or responses */
uint8_t Create_Acknowledgement(uint8_t *retPacket, uint8_t hashValue, time_of_day SatTime)
{
	// Packets structure: HHHH H0MM MMMM 0SSS SSS0 0XXX
	// H = hour, M = minute, S = seconds, XXX represents the command
	
	// The command code filling in the XXX is 111
	uint8_t packet[4] = {0};
	
	// HHHH H0MM
	packet[0] = SatTime.hour << 3;
	packet[0] = packet[0] | ((SatTime.min & 0x30) >> 4);
	
	// MMMM 0SSS
	packet[1] = (SatTime.min & 0x0F) << 4;
	packet[1] = packet[1] | ((SatTime.sec & 0x38) >> 3);
	
	// SSS0 0XXX
	packet[2] = ((SatTime.sec & 0x07) << 5);
	packet[2] = packet[2] | SAT_ACK;
	
	// Store the hash of the command being acknowledged
	packet[3] = hashValue;
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 4);
	
	return SUCCESS;
}

/* Create a packet with the current CubeSat's location (Keplerian Elements) */
uint8_t Create_LocationData(uint8_t *retPacket, uint8_t KepElem1, uint8_t KepElem2, uint8_t KepElem3, time_of_day SatTime)
{
	// Packets structure: HHHH H0MM MMMM 0SSS SSS0 0XXX
	// H = hour, M = minute, S = seconds, XXX represents the command
	
	// The command code filling in the XXX is 101
	uint8_t packet[6] = {0};
	
	// HHHH H0MM
	packet[0] = SatTime.hour << 3;
	packet[0] = packet[0] | ((SatTime.min & 0x30) >> 4);
	
	// MMMM 0SSS
	packet[1] = (SatTime.min & 0x0F) << 4;
	packet[1] = packet[1] | ((SatTime.sec & 0x38) >> 3);
	
	// SSS0 0XXX
	packet[2] = ((SatTime.sec & 0x07) << 5);
	packet[2] = packet[2] | SAT_LOCATION;
	
	// Store the Keplerian elements
	packet[3] = KepElem1;
	packet[4] = KepElem2;
	packet[5] = KepElem3;
	
	// Return the packet to be used outside this function
	memcpy(retPacket, packet, 6);
	
	return SUCCESS;
}
