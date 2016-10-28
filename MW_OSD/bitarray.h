#define bitISSET(bits, off) (bits[(off)/8] & (1 << ((off) % 8)))
#define bitISCLR(bits, off) (!bitISSET(bits, off))
#define bitCLR(bits, off) { bits[(off)/8] &= ~(1 << ((off) % 8)); }
#define bitSET(bits, off) { bits[(off)/8] |= ~(1 << ((off) % 8)); }
