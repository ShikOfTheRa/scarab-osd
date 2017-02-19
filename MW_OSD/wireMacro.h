#include <stdio.h>
//#define ATOMIC_BLOCK(how) // Enable this for compiler output testing

#define ATOMIC_RESTORE_STATE ATOMIC_RESTORESTATE

#define _xxxconcat(a,b) a ## b

#define _dp2rbit0 0
#define _dp2rbit1 1
#define _dp2rbit2 2
#define _dp2rbit3 3
#define _dp2rbit4 4
#define _dp2rbit5 5
#define _dp2rbit6 6
#define _dp2rbit7 7
#define _dp2rbit8 0
#define _dp2rbit9 1
#define _dp2rbit10 2
#define _dp2rbit11 3
#define _dp2rbit12 4
#define _dp2rbit13 5
#define _dp2rbit14 0
#define _dp2rbit15 1
#define _dp2rbit16 2
#define _dp2rbit17 3
#define _dp2rbit18 4
#define _dp2rbit19 5

#define _digital_pin_to_rbit(n) _xxxconcat(_dp2rbit, n)
#define _DTOB(n) _digital_pin_to_rbit(n)

#define _BV(n) (1 << (n))

#define _dp2bm0  _BV(0) /* 0, port D */
#define _dp2bm1  _BV(1)
#define _dp2bm2  _BV(2)
#define _dp2bm3  _BV(3)
#define _dp2bm4  _BV(4)
#define _dp2bm5  _BV(5)
#define _dp2bm6  _BV(6)
#define _dp2bm7  _BV(7)
#define _dp2bm8  _BV(0) /* 8, port B */
#define _dp2bm9  _BV(1)
#define _dp2bm10 _BV(2)
#define _dp2bm11 _BV(3)
#define _dp2bm12 _BV(4)
#define _dp2bm13 _BV(5)
#define _dp2bm14 _BV(0) /* 14, port C */
#define _dp2bm15 _BV(1)
#define _dp2bm16 _BV(2)
#define _dp2bm17 _BV(3)
#define _dp2bm18 _BV(4)
#define _dp2bm19 _BV(5)

#define _digital_pin_to_bit_mask(n) _xxxconcat(_dp2bm, n)

#define _dp2p0   D /* 0 */
#define _dp2p1   D
#define _dp2p2   D
#define _dp2p3   D
#define _dp2p4   D
#define _dp2p5   D
#define _dp2p6   D
#define _dp2p7   D
#define _dp2p8   B /* 8 */
#define _dp2p9   B
#define _dp2p10  B
#define _dp2p11  B
#define _dp2p12  B
#define _dp2p13  B
#define _dp2p14  C /* 14 */
#define _dp2p15  C
#define _dp2p16  C
#define _dp2p17  C
#define _dp2p18  C
#define _dp2p19  C

#define _dp2p(n) _dp2p ## n

#define _digital_pin_to_xxx(reg, group) _xxxconcat(reg, group)
#define _digital_pin_to_ddr(n) _digital_pin_to_xxx(DDR, _dp2p(n))
#define _digital_pin_to_port(n) _digital_pin_to_xxx(PORT, _dp2p(n))

#define _pinModeOut(pin) ATOMIC_BLOCK(ATOMIC_RESTORE_STATE) do {\
  _digital_pin_to_ddr(pin) |= _digital_pin_to_bit_mask(pin);} while (false)

#define _pinModeIn(pin) ATOMIC_BLOCK(ATOMIC_RESTORE_STATE) do {\
  _digital_pin_to_ddr(pin) &= ~_digital_pin_to_bit_mask(pin);} while (false)

#define _digitalHi(pin) ATOMIC_BLOCK(ATOMIC_RESTORE_STATE) do {\
  _digital_pin_to_port(pin) |= _digital_pin_to_bit_mask(pin);} while (false)

#define _digitalLo(pin) ATOMIC_BLOCK(ATOMIC_RESTORE_STATE) do {\
  _digital_pin_to_port(pin) &= ~_digital_pin_to_bit_mask(pin);} while (false)

#define _digitalToggle(pin) ATOMIC_BLOCK(ATOMIC_RESTORE_STATE) do {\
  _digital_pin_to_port(pin) ^= _digital_pin_to_bit_mask(pin);} while (false)

#define _dp2xbm0  (_BV(0) << 16) /* 0, port D */
#define _dp2xbm1  (_BV(1) << 16)
#define _dp2xbm2  (_BV(2) << 16)
#define _dp2xbm3  (_BV(3) << 16)
#define _dp2xbm4  (_BV(4) << 16)
#define _dp2xbm5  (_BV(5) << 16)
#define _dp2xbm6  (_BV(6) << 16)
#define _dp2xbm7  (_BV(7) << 16)
#define _dp2xbm8  _BV(0) /* 8, port B */
#define _dp2xbm9  _BV(1)
#define _dp2xbm10 _BV(2)
#define _dp2xbm11 _BV(3)
#define _dp2xbm12 _BV(4)
#define _dp2xbm13 _BV(5)
#define _dp2xbm14 (_BV(0) << 8) /* 14, port C */
#define _dp2xbm15 (_BV(1) << 8)
#define _dp2xbm16 (_BV(2) << 8)
#define _dp2xbm17 (_BV(3) << 8)
#define _dp2xbm18 (_BV(4) << 8)
#define _dp2xbm19 (_BV(5) << 8)

#define _digital_pin_to_extended_bit_mask(n) _dp2xbm ## n
#define _xm_(n) _digital_pin_to_extended_bit_mask(n) // short hand

#define _pinModeOutGroup(bm) ATOMIC_BLOCK(ATOMIC_RESTORE_STATE) do { \
  if ((bm) & 0xff) { DDRB |= ((bm) & 0xff); } \
  if (((bm) >> 8) & 0xff) { DDRC |= (((bm) >> 8) & 0xff); } \
  if (((bm) >> 16) & 0xff) { DDRD |= (((bm) >> 16) & 0xff); } } while (false)

#define _pinModeInGroup(bm) ATOMIC_BLOCK(ATOMIC_RESTORE_STATE) do { \
  if ((bm) & 0xff) { DDRB &= ~((bm) & 0xff); } \
  if (((bm) >> 8) & 0xff) { DDRC &= ~(((bm) >> 8) & 0xff); } \
  if (((bm) >> 16) & 0xff) { DDRD &= ~(((bm) >> 16) & 0xff); } } while (false)

#define _pinModeIO(ibm, obm) ATOMIC_BLOCK(ATOMIC_RESTORE_STATE) do { \
  DDRB = (DDRB & ~((ibm) & 0xff)) | ((obm) & 0xff);\
  DDRC = (DDRC & ~(((ibm) >> 8) & 0xff)) | (((obm) >> 8) & 0xff);\
  DDRD = (DDRD & ~(((ibm) >> 16) & 0xff)) | (((obm) >> 16) & 0xff); } while (false)
