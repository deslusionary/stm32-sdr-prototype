/*
 * circbuf.h
 *
 *  Created on: Apr 21, 2024
 *      Author: Christopher Tinker
 */

#ifndef INC_CIRCBUF_H_
#define INC_CIRCBUF_H_

typedef struct {
	uint16_t *rdbuf;
	uint16_t *wrbuf;
} circbuf_uint16;

inline void swap_circbuf_uint16(cirbuf_uint16 *circbuf) {
	uint16_t *tmp = circbuf->rdbuf;
	circbuf->rdbuf = circbuf->wrbuf;
	circbuf->wrbuf = tmp;
}

#endif /* INC_CIRCBUF_H_ */
