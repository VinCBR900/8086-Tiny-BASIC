/* Stub for XTulator i8259.h - we don't use the PIC, NMI is wired direct */
#ifndef _I8259_H_
#define _I8259_H_
#include <stdint.h>
typedef struct {
    uint8_t irr;
    uint8_t imr;
} I8259_t;
static inline uint8_t i8259_nextintr(I8259_t *p) { (void)p; return 0; }
#endif
