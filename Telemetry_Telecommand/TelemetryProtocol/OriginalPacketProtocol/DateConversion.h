#ifndef DATE_CONV
#define DATE_CONV

// Convert from the bytes to double 
double bytes_to_double(uint8_t input[8]);

// Convert from a double to an unsigned 64 bit value
uint64_t double_to_unsigned(double input);

// Convert from a double to a packet of size 8
void double_to_bytes(double input, uint8_t *retArr);

// Convert Julian date to a human readable date
double JD_2_decdate(double JD);



#endif // DATE_CONV
