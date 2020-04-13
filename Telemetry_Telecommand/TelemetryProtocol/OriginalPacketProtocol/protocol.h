#ifndef PROTO_H
#define PROTO_H

// Standard include files
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
//#include <readline/readline.h>
//#include <readline/history.h>
#include <sys/stat.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
// Files needed for sockets
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <netdb.h>
//#include <arpa/inet.h>
// Threading libraries
//#include <pthread.h>

/********** Type defined structs and enums **********/

// This struct will hold each of the times needed 
typedef struct time_of_day
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t month;
	uint8_t day;
	uint8_t year; // year since 1900
} time_of_day;


/********** Defined macros **********/
#define MAX_PACK_SIZE (1024)
#define SUCCESS (1)
#define FAIL (0)

#define SAT_STATUS 		(0x2)
#define SCI_DATA   		(0x4)
#define SAT_LOCATION 	(0x5)
#define SAT_ACK 		(0x7)

#define REQ_LOG 		(0x10)
#define REQ_LOG_TIME	(0x14)
#define REQ_STATUS 		(0x20)
#define UPDATE_KEP 		(0x30)
#define REQ_SCI_DATA	(0x40)
#define REQ_SCI_DATA2	(0x44)
#define REQ_LOCATION 	(0x50)
#define KILL			(0xF0)


/********** Function Declarations **********/

/*** Functions for the ground station side ***/
/* This function will decode the time of the CubeSat by looking at the packet received */
uint8_t Decode_CubeSat_Time (uint8_t *packet, time_of_day *satTime);

/* This function will decode packets that have come into the ground station */
uint8_t Decode_Ground_Packet(uint8_t *packet);

/* Create the packet used for sending a Kill signal */
uint8_t Create_Kill_Packet(uint8_t *retPacket);

/* Create a log science event packet */
uint8_t Create_Command_LogSciEvent(uint8_t *retPacket, uint8_t logType, time_of_day *StartTime);

/* Create a request for the status of the CubeSat */ 
uint8_t Create_Request_Status(uint8_t *retPacket);

/* Create a request for science data */
uint8_t Create_Request_ScienceData(uint8_t *retPacket, time_of_day *StartTime, time_of_day *FinishTime, uint32_t chunkSize);

/* Create a request for the CubeSat's location (Keplerian Elements) */
uint8_t Create_Request_Location(uint8_t *retPacket);

/* Create a packet to update the Keplerian elements on the CubeSat */
uint8_t Create_Command_UpdateKep(uint8_t *retPacket, uint8_t KepElem1, uint8_t KepElem2, uint8_t KepElem3);


/*** Functions for the CubeSat side ***/

/* This function will decode packets that have come into the ground station */
uint8_t Decode_Sat_Packet(uint8_t *packet);

/* Create a packet responding to the stats request from ground station */
uint8_t Create_Response_Status(uint8_t *retPacket, uint8_t status, time_of_day SatTime);

/* Create a packet containing science payload data */
uint8_t Create_ScienceData(uint8_t *retPacket, uint32_t *data, uint16_t dataLength, time_of_day SatTime);

/* Acknowledgement of certain messages or responses */
uint8_t Create_Acknowledgement(uint8_t *retPacket, uint8_t hashValue, time_of_day SatTime);

/* Create a packet with the current CubeSat's location (Keplerian Elements) */
uint8_t Create_LocationData(uint8_t *retPacket, uint8_t KepElem1, uint8_t KepElem2, uint8_t KepElem3, time_of_day SatTime);


#endif // PROTO_H