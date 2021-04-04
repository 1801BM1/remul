//
//  Project:         RE-mulator (1801PE2/PP1 emulator)
//  File:            debug.h
//  Author:          VSO
//
//  Compiled Using:  CodeSourcery GCC 4.7.2
//                   GNU Tools ARM Embedded. ver. 4.7 2012q4
//                   IAR ICC 5.41.2
//
//  Processor:       STM32F205
//  Board:           RE-mulator 1801PE2/PP1
//_____________________________________________________________________________
//
#ifndef _DEBUG_H_
#define _DEBUG_H_

#if DBG_ENABLE
int     dbg_vprintf (const char *fmt, va_list ap);
int     dbg_printf(const char *fmt, ...);
void    dbg_assert(unsigned long line, const char* file, ...);

unsigned char dbg_inkey(void);
unsigned char dbg_getch(void);
void dbg_putch(char sym);

void dbg_background(void);
unsigned char dbg_read(void);
void dbg_write(unsigned char data);
void dbg_init(void);
void dbg_flush(void);

#if DBG_DOASSERT
#define DBG_ASSERT( Value, ...)                          \
    if (!(Value))                                        \
    {                                                    \
        dbg_assert( __LINE__, __FILE__, __VA_ARGS__);    \
    }
#else
#define DBG_ASSERT( Value, ...)
#endif

#else

#if defined (__ICCARM__)
#define	dbg_init()
#define	dbg_printf 	_Pragma("diag_suppress=Pe174")
#define DBG_ASSERT( Value, ...)
#endif

#if defined ( __GNUC__ )
#define	dbg_init()
#define	dbg_printf( Value, ...)
#define DBG_ASSERT( Value, ...)
#endif

#endif  // DBG_ENABLE
#endif  // _DEBUG_H_
