#include "Telemetry_Packet_Protocol.h"

#include <limits.h>

double JD_2_decdate(double JD);


int main(int argc, char **argv)
{
	
	printf("*First create tests that will encapsulate a packet and then test the packet contents*\n");
	
//This is for the ground station side
	////////////////////// Testing the log science command///////////////////////////
	uint8_t pack[3];
	uint8_t logType = 1;
	time_of_day testTime;
	testTime.hour = 20; // 0001 0100
	testTime.min = 35; // 0010 0011
	testTime.sec = 59; // 0011 1011
	uint8_t t = Create_Command_LogSciEvent(pack, logType, &testTime);
	if (t)
	{	
		printf("Request: Log Science Event at a specified time\n");
		printf("\tThe packet should be:\t14 48 FB\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 3; i++)	printf("%02X ", pack[i]);
		printf("\n");
	}
	
	/////////////////////////////////////////////////////////////////////////////
	
	uint8_t packet[3] = {0};
	uint8_t logType1 = 0;
	time_of_day testTime1;
	testTime1.hour = 20; // 0001 0100
	testTime1.min = 35; // 0010 0011
	testTime1.sec = 59; // 0011 1011
	uint8_t u = Create_Command_LogSciEvent(packet, logType1, &testTime1);
	if (u)
	{
		printf("Request: Log Science Event Now\n");
		printf("\tThe packet should be:\t10 00 00\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 3; i++)	printf("%02X ", packet[i]);
		printf("\n");
	}	
	
	////////////////////////////////////////////////////////////////////////////////	
	uint8_t pack2[3];
	uint8_t logType2 = 1;
	time_of_day testTime2;
	testTime2.hour = 13; // 0000 1101
	testTime2.min = 29; // 0001 1101
	testTime2.sec = 02; // 0000 0010
	uint8_t v = Create_Command_LogSciEvent(pack2, logType2, &testTime2);
	if (v)
	{
		printf("Request: Log Science Event at a specified time\n");
		printf("\tThe packet should be:\t14 D7 42\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 3; i++)	printf("%02X ", pack2[i]);
		printf("\n");
	}
	
	//////////////////////////Testing  kill command //////////////////////
	uint8_t packKill;
	uint8_t w = Create_Kill_Packet(&packKill);
	if (w)
	{
		printf("Command: Kill\n");
		printf("\tThe packet should be:\tF0\n");
		printf("\tPacket created:\t\t%02X\n", packKill);
	}
	
	//////////////////////////Testing request status command  //////////////////////
	uint8_t packReqstat;
	uint8_t x = Create_Request_Status(&packReqstat);
	if (x)
	{
		printf("Request: Status of CubeSat\n");
		printf("\tThe packet should be:\t20\n");
		printf("\tPacket created:\t\t%02X\n", packReqstat);
	}
	
	//////////////////////////Testing request science data command  //////////////////////
	/* Chunk Size is 0 so the packet should only use the start and finish time */
	uint8_t packReqSci[7];
	uint32_t chunkSize = 0;
	time_of_day testTimeStart;
	testTimeStart.hour = 13; // 0000 1101//0d
	testTimeStart.min = 29; // 0001 1101//1D
	testTimeStart.sec = 02; // 0000 0010//02
	time_of_day testTimeFinish;
	testTimeFinish.hour = 20; // 0001 0100//14
	testTimeFinish.min = 35; // 0010 0011//23
	testTimeFinish.sec = 59; // 0011 1011//3B
	uint8_t y = Create_Request_ScienceData(packReqSci, &testTimeStart, &testTimeFinish,  chunkSize);
	if (y)
	{	
		printf("Request: Science Data with an end time\n");
		printf("\tThe packet should be:\t40 0D 1D 02 14 23 3B\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 7; i++)	printf("%02X ", packReqSci[i]);
		printf("\n");
	}
	
	/* Chunk Size is non zero so the packet should only use the start time and the chunk size */
	uint8_t packReqSci2[7];
	uint32_t chunkSize2 = 16;
	time_of_day testTimeStart2;
	testTimeStart2.hour = 13; // 0000 1101//0d
	testTimeStart2.min = 29; // 0001 1101//1D
	testTimeStart2.sec = 02; // 0000 0010//02
	time_of_day testTimeFinish2;//wont be: needed since chunk size is > 0
	
	uint8_t z = Create_Request_ScienceData(packReqSci2, &testTimeStart2, &testTimeFinish2,  chunkSize2);
	if (z)
	{	
		printf("Request: Science Data with a chunk\n");
		printf("\tThe packet should be:\t44 0D 1D 02 00 00 10\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 7; i++)	printf("%02X ", packReqSci2[i]);
		printf("\n");
	}
	
	//////////////////////////Testing request location command  //////////////////////
	uint8_t packReqLoc;
	uint8_t aa = Create_Request_Location(&packReqLoc);
	if (aa)
	{
		printf("Request: CubeSat location\n");
		printf("\tThe packet should be:\t50\n");
		printf("\tPacket created:\t\t%02X\n", packReqLoc);
	}
	
	//////////////////////////Testing update location command  //////////////////////
	uint8_t packUpdateLoc[4];
	uint8_t KepElem1 = 13; // 0000 1101//0d
	uint8_t KepElem2 = 29; // 0001 1101//1D
	uint8_t KepElem3 = 02; // 0000 0010//02
	uint8_t a = Create_Command_UpdateKep( packUpdateLoc,  KepElem1,  KepElem2,  KepElem3);
	if (a)
	{
		printf("Command: Update Keplerian Elements\n");
		printf("\tThe packet should be:\t30 0D 1D 02\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 4; i++)	printf("%02X ", packUpdateLoc[i]);
		printf("\n");
	}
	
	/////////////////////////////////////////////////////////////////////////////
	/*This is for the cubesat side*/
	/////////////////////////////////////////////////////////////////////////////
	double julianDate = 2458610.512340;
	ScienceDataPoint sdp[3];
	for (int q = 0; q < 3; q++)
	{
		sdp[q].Time = 1234.567;
		sdp[q].Energy = q+1;
	}
	printf("\n1:  %d  2: %d  3: %d\n\n", sdp[0].Energy, sdp[1].Energy, sdp[2].Energy);
	//////////////////////////Testing cube sat status   //////////////////////	
	uint8_t packResponseStat[24] = {0};
	uint8_t status  = 7; 
	uint8_t b = Create_Response_Status(packResponseStat, status, julianDate);
	if (b)
	{
		printf("Respond with Satellite Status\n");
		printf("\tThe packet should be:\t69 D0 42 02\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 24; i++)	printf("%02X ", packResponseStat[i]);
		printf("\n");
	}
	////////////////////////// testing ScienceData//////////////////////
	uint8_t packSciData[24] = {0};//change this
	uint16_t dataLength = 3;
	uint8_t c = Create_ScienceData(packSciData, sdp, dataLength,  julianDate);
	if (c)
	{
		printf("Respond with Science data\n");
		printf("\tThe packet should be:\t69 D0 44 64 3E 4C BA D1 9C 99 26 9F 0E 54 65 1D 1A E3 \n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 24; i++)      printf("%02X ",packSciData[i]); 
		printf("\n");
	}
	/////////////////// testing Create Acknowledgement for cubesat///////////////
	uint8_t packAckCube[24] = {0};
	uint8_t hashValue  = 2; 
	uint8_t d = Create_Acknowledgement( packAckCube,  hashValue,  julianDate);
	if (d)
	{
		printf("Satellite Acknowledgement\n");
		printf("\tThe packet should be:\t69 D0 47 02\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 24; i++)	printf("%02X ", packAckCube[i]);
		printf("\n");
	}
	
	/////////////////// testing Create location data for cubesat///////////////
	uint8_t packLocCube[24] = {0};
	float KepElem11 = 13.0; 
	float KepElem22 = 29.5; 
	float KepElem33 = 200.12;  
	uint8_t e = Create_LocationData(packLocCube,  KepElem11,  KepElem22,  KepElem33,  julianDate);
	if (e)
	{
		printf("Respond with Location Data\n");
		printf("\tThe packet should be:\tA2 37 65 0D 1D 02\n");
		
		printf("\tPacket created:\t\t");
		for (int i = 0; i < 24; i++)	printf("%02X ", packLocCube[i]);
		printf("\n");
	}
	/////////////////////////////////////////////////////////////////////////////

	// Test decoding certain types of packets
	printf("\n*Now Testing Decoding Packets from the CubeSat:*\n");
	Decode_Ground_Packet(packResponseStat, 0);
	Decode_Ground_Packet(packSciData, 0);
	Decode_Ground_Packet(packAckCube, 0);
	Decode_Ground_Packet(packLocCube, 0);

	printf("\n*Now Testing Decoding Packets from the Ground Station:*\n");
	
	Decode_Sat_Packet(pack);
	Decode_Sat_Packet(packet);
	Decode_Sat_Packet(pack2);
	Decode_Sat_Packet(&packKill);
	Decode_Sat_Packet(&packReqstat);
	Decode_Sat_Packet(packReqSci2);
	Decode_Sat_Packet(packReqSci);
	Decode_Sat_Packet(&packReqLoc);
	Decode_Sat_Packet(packUpdateLoc);


	printf("\n\nTesting going from a double to a byte and then recreating it\n");
	
	double testDouble =  2458610.512340;// 2458610.71462;//2458610.72067;//
	double outDouble = 0.0;

	double yearTest = 2457132.71462;//2457247.71462;
	
	outDouble = JD_2_year(yearTest);
	
	printf("Output: %lf\n", outDouble);

	double inter = outDouble - 2015;
	printf("Days: %lf\n", inter);

	unsigned days = inter * 365 + 1;
	printf("Days: %u\n", days);

	double remaind = testDouble - (unsigned) testDouble;
	unsigned daysss = (remaind - 0.5) * 365 + 1;
	printf("days: %u\n", daysss);

	const unsigned long long base = 1000000; 
	const unsigned long long halfbase = 500000; 
	const unsigned secsPerDay = 86400; 

	double remainder = testDouble - (unsigned) testDouble;

	//"rounded" remainder after adding half a day 
   	unsigned long long rndRemainder = (unsigned long long)(remainder * base + halfbase) % base; 
 
	rndRemainder *= secsPerDay; 
 
	// "rounded" number of seconds 
	unsigned long long nsecs = (rndRemainder + halfbase) / base; 
 
	//hours: secs/3600 % 24, min: secs/60 % 60, secs secs % 60 
	unsigned rtn = (nsecs/3600 % 24) * 10000 + (nsecs/60 % 60) * 100 + (nsecs % 60); 
        
	printf("Hours: %u\n", rtn); 

	uint8_t testP[8] = {0};
	double_to_bytes(testDouble, testP);
	
	for (int q = 7; q >= 0; q--)
	{
		printf("byte: %d\tvalue: 0x%02x\n", q, testP[q]);
	}

	float hi = 1234.567;
	uint8_t testF[4] = {0};
	float_to_bytes(hi, testF);
	printf("\n\n");
	for (int q = 3; q >= 0; q--)
	{
		printf("byte: %d\tvalue: 0x%02x\n", q, testF[q]);
	}
	printf("Float: %f\n", bytes_to_float(testF));
	

}
