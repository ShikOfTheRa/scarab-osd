// TODO: understand this file and put it somewhere reasonable
#ifndef __HAX_H
#define __HAX_H

#define DEVELOPMENT // to enable development checking and set debug[x] value
//#define MEMCHECK 3  // to enable memory checking and set debug[x] value. Requires DEVELOPMENT to be enabled

#ifdef MEMCHECK
extern uint8_t _end;  //end of program variables 
extern uint8_t __stack; //start of stack (highest RAM address)

void PaintStack(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));    //Make sure this is executed at the first time

void PaintStack(void) 
{ 
  //using asm since compiller could not be trusted here
    __asm volatile ("    ldi r30,lo8(_end)\n" 
                    "    ldi r31,hi8(_end)\n" 
                    "    ldi r24,lo8(0xa5)\n" /* Paint color = 0xa5 */ 
                    "    ldi r25,hi8(__stack)\n" 
                    "    rjmp .cmp\n" 
                    ".loop:\n" 
                    "    st Z+,r24\n" 
                    ".cmp:\n" 
                    "    cpi r30,lo8(__stack)\n" 
                    "    cpc r31,r25\n" 
                    "    brlo .loop\n" 
                    "    breq .loop"::); 
} 

uint16_t UntouchedStack(void) 
{ 
    const uint8_t *ptr = &_end; 
    uint16_t       count = 0; 

    while(*ptr == 0xa5 && ptr <= &__stack) 
    { 
        ptr++; count++; 
    } 

    return count; 
} 
#endif

#endif
