/**
* @file CircularBuffer.h
* @author Cameron Moreno
* @date 11 Feb 2021
* @brief Header file for CircularBuffer.c
*
*/
#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

#define	BUFFER_SIZE 128

/**
 *  Main structure for the circular buffer
 */
typedef struct circ_buf{
	uint8_t head;
	uint8_t tail;
	uint8_t data[BUFFER_SIZE];
	uint8_t isFull:1;
	uint8_t isEmpty:1;
	uint8_t buf_size;
	uint8_t num_elements;
}circ_buf_t;

/**
 *  Initialize the circular buffer
 */
void circ_buf_init(circ_buf_t *cbuf);

/**
 *  Get the number of elements currently in the circular buffer
 */
uint8_t circ_buf_len(circ_buf_t *cbuf);

/**
 * Add a new element to the data buffer
 */
void circ_buf_add(circ_buf_t *cbuf, uint8_t data);

/**
 *  Remove an element from the data buffer
 */
uint8_t circ_buf_remove(circ_buf_t *cbuf);

/**
 *  Return whether or not the buffer is empty
 */
uint8_t circ_buf_empty(circ_buf_t *cbuf);

/**
 *  Return whether or not the buffer is full
 */
uint8_t circ_buf_full(circ_buf_t *cbuf);

#endif
