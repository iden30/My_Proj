


#ifndef __RC5_H
#define __RC5_H 

#include <stdint.h>

extern void rc5_send(uint32_t *data);
extern void rc5_get(uint32_t *data);
extern void rc5_init(void);

#endif
