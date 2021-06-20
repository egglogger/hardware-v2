/* Macros for enabling and disabling interrupts (for critical code sections) */
#define _DI()		__asm__ volatile("disi #0x3FFF")
#define _EI()		__asm__ volatile("disi #0")
