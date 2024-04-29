/*
 * circbuf.h
 *
 *  Created on: Apr 21, 2024
 *      Author: Christopher Tinker
 */

#ifndef INC_CIRCBUF_H_
#define INC_CIRCBUF_H_

typedef struct {
	uint16_t * volatile rdbuf;
	uint16_t * volatile wrbuf;
} circbuf_uint16;

//void swap_circbuf_uint16(volatile circbuf_uint16 *);


#endif /* INC_CIRCBUF_H_ */
