#include "DateConversion.h"



uint64_t double_to_unsigned(double input)
{
	union Date {
		double date;
		uint64_t udate;
	};
	union Date D;
	D.date = input;
	return D.udate;
}

void double_to_bytes(double input, uint8_t *retArr)
{	
	union Date {
		double date;
		uint64_t udate;
		uint8_t bdate[8];
	};
	union Date D;
	D.date = input;
	memcpy(retArr, D.bdate, 8);
}

double bytes_to_double(uint8_t input[8])
{
	union Date {
		double date;
		uint8_t bdate[8];
	};
	union Date D;
	memcpy(D.bdate, input, 8);
	return D.date;
}

double JD_2_decdate(double JD)
{
	JD = JD - 2451545 + 0.5;
	int year = 2000;
	int days_per_year = 365;
	while(1) {
		if(((year % 4) == 0) && (((year % 100) != 0) || ((year % 400) == 0))) {
			days_per_year = 366; // Leap year
		}
		else {
			days_per_year = 365;
		}
		if (JD < days_per_year)
		{
			break;
		}
		
		year++;
		JD -= days_per_year;
	}
	
	return (double)year + JD/days_per_year;
}

