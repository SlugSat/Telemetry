/**
* @file CircularBuffer.c
* @author Cameron Moreno
* @date 11 Feb 2021
* @brief Implementation of a circular buffer
*
*/
#include <stdio.h>
#include "CircularBuffer.h"

/**
 *  \brief Initializes the circular buffer
 *  
 *  \param [in] cbuf The circular buffer to access
 *  \return Return void
 *  
 *  \details More details
 */
void circ_buf_init(circ_buf_t *cbuf){
	cbuf->head = 0;
	cbuf->tail = 0;
	cbuf->isFull = 0;
	cbuf->isEmpty = 1;
	cbuf->buf_size = BUFFER_SIZE;
	cbuf->num_elements = 0;
}

/**
 *  \brief Getter function for the number of elements in the buffer
 *  
 *  \param [in] cbuf The circular buffer to access
 *  \return Returns the number of elements in the buffer
 *  
 *  \details More details
 */
uint8_t circ_buf_len(circ_buf_t *cbuf){
	return cbuf->num_elements;
}

/**
 *  \brief Add an element to the buffer
 *  
 *  \param [in] cbuf The circular buffer to access
 *  \param [in] data The element to load into the buffer
 *  \return Returns void
 *  
 *  \details More details
 */
void circ_buf_add(circ_buf_t *cbuf, uint8_t data){
	if(!cbuf->isFull){
		cbuf->data[cbuf->tail] = data;
		cbuf->num_elements++;
		
		// The buffer currently isn't full as we have entered the loop
		// This means that tail does not equal head, therefore
		// I can safely set tail to zero if the data was added
		// to the last element space in the buffer
		if(cbuf->tail == cbuf->buf_size-1)
			cbuf->tail = 0;
		else{
           cbuf->tail++; 
        }
		
		if(cbuf->isEmpty)
			cbuf->isEmpty = 0;
		else if(cbuf->tail == cbuf->head)
			cbuf->isFull = 1;
	}
}

/**
 *  \brief Remove an element from the buffer
 *  
 *  \param [in] cbuf The circular buffer to access
 *  \return Return the element that was removed
 *  
 *  \details More details
 */
uint8_t circ_buf_remove(circ_buf_t *cbuf){
	if(!cbuf->isEmpty){
		uint8_t data = cbuf->data[cbuf->head];
		if(cbuf->head == cbuf->buf_size-1)
			cbuf->head = 0;
		else
			cbuf->head++;
		
		if(cbuf->isFull)
			cbuf->isFull = 0;
		else if(cbuf->tail == cbuf->head)
			cbuf->isEmpty = 1;
		
		cbuf->num_elements--;
		return data;
	}

	return -1;
}

/**
 *  \brief Getter for determining if the buffer is empty
 *  
 *  \param [in] cbuf The circular buffer to access
 *  \return Return 1 if empty else return 0
 *  
 *  \details More details
 */
uint8_t circ_buf_empty(circ_buf_t *cbuf){
	return cbuf->isEmpty;
}

/**
 *  \brief Getter for determining if the buffer is full
 *  
 *  \param [in] cbuf The circular buffer to access
 *  \return Return 1 if full else return 0
 *  
 *  \details More details
 */
uint8_t circ_buf_full(circ_buf_t *cbuf){
	return cbuf->isFull;
}
