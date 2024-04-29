/*
 * circbuf.h
 *
 *  Created on: Apr 21, 2024
 *      Author: Christopher Tinker
 */

#ifndef INC_PPBUF_H_
#define INC_PPBUF_H_

typedef struct {
	uint16_t * volatile rdbuf;
	uint16_t * volatile wrbuf;
} ppbuf_uint16;

//void swap_circbuf_uint16(volatile circbuf_uint16 *);


#endif /* INC_PPBUF_H_ */
