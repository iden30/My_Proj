


#ifndef __RC5_H
#define __RC5_H 

#include <stdint.h>

extern void rc5_send(uint32_t *data);
extern uint16_t rc5_get(void);
extern void rc5_init(void);

#endif
