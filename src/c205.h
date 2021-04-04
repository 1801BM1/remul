//
//  Project:         RE-mulator (1801PE2/PP1 emulator)
//  File:            c205.h
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
#ifndef __IO_C205_H_
#define __IO_C205_H_

#include <stdarg.h>
#ifndef __GNUC__
#include <intrinsics.h>
#endif
#include <stddef.h>
#include "stm32f2.h"

//_____________________________________________________________________________
//
#ifndef BOOL
#define BOOL      int
#endif

#ifndef TRUE
#define TRUE      1
#endif

#ifndef FALSE
#define FALSE     0
#endif

#ifndef NULL
#define NULL      0
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

//_____________________________________________________________________________
//
// Debug Communication Channel configuration parameters
//
#ifndef DBG_ENABLE
//
// Этот флаг включает отладочный вывод, обычно определяется
// снаружи в makefile и передается через опции компилятора
//
#define DBG_ENABLE          1
#endif
#define DBG_DOASSERT        1
#define DBG_UART_BASE       UART1_BASE
#define DBG_UART_BAUD       115200

//_____________________________________________________________________________
//
// Board configuration parameters
//
#define BSP_HSI_FREQ        16000000            // RC oscillator frequency (Herz)
#define BSP_CORE_CLOCK      120000000           // desired CPU clock frequency (Herz)
#define BSP_APB_CLOCK       (BSP_CORE_CLOCK/4)
#define BSP_FLASH_WS        ((BSP_CORE_CLOCK - 1000)/30000000)

//_____________________________________________________________________________
//
//  Board specific pin configuration
//
#define _DBG_UART_TXD       _TXD
#define _DBG_UART_RXD       _RXD

//_____________________________________________________________________________
//
#define ROM_TABLE_BASE      (STM_IFLASH + 0x8000)
#define ROM_TABLE_INDEX     64
#define ROM_IMAGE_SIZE      0x2000
#define ROM_IMAGE_SIGN      0x03

//_____________________________________________________________________________
//
// Прототип функции основного цикла
//
typedef void (*APP_LOOP_PTR)(unsigned char** ,PSTM_PIO, PSTM_PIO);
void app_rom_loop(unsigned char** mapper, PSTM_PIO data, PSTM_PIO ctrl);
void app_end_loop(void);

#pragma pack(push, 4)
typedef struct _ROM_DESC
{
    //
    // Собственно данные для вывода
    // Должны быть представлены в "формате программатора",
    // то есть без инверсии адресов и данных
    //
    const unsigned char data[ROM_IMAGE_SIZE];
    //
    // Код микросхемы соответствует инвертированному состоянию на шине адреса:
    //
    // 111  - 000000..017777 (диапазон в адресном пространстве)
    // 110  - 020000..037777
    // 101  - 040000..057777
    // 100  - 060000..077777
    // 011  - 100000..117777
    // 010  - 120000..137777
    // 001  - 140000..157777
    // 000  - 160000..177777
    //
    const unsigned char code;
    //
    // Сигнатура прошивки - байт со значением ROM_IMAGE_SIGN
    //
    const unsigned char sign;
    //
    // Номера chip select-ов для конфигураций 0..13
    //  0x00 - активируется на сигнал выбора CS
    //  0x11 - активируется на сигнал выбора XCS1
    //  0x22 - активируется на сигнал выбора XCS2
    //  0x33 - активируется на сигнал выбора XCS3
    //
    // Одна прошивка не может соответствовать нескольким CSx
    //
    const unsigned char csel[14];

} ROM_DESC, *PROM_DESC;
#pragma pack(pop)

//_____________________________________________________________________________
//
// Определяем значение контактов портов
//
#define _MCO        (IO_FUNC_AF0 | IO_MODE_OUTPUT       | IO_ATTR_50MHZ   | IO_PORT_A | 8)
#define _TXD        (IO_FUNC_AF7 | IO_MODE_OUTPUT       | IO_ATTR_25MHZ   | IO_PORT_A | 9)
#define _RXD        (IO_FUNC_AF7 | IO_MODE_INPUT        | IO_ATTR_25MHZ   | IO_PORT_A | 10)

#define _CFG0       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 7)
#define _CFG1       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 6)
#define _CFG2       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 5)
#define _CFG3       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 4)
#define _CFG4       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 2)
#define _CFG5       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 1)

#define _TMS        (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 13)
#define _TCK        (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 14)
#define _TDI        (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_A | 15)

#define IO_AD_ATTR      IO_ATTR_2MHZ
#define IO_RPLY_ATTR    IO_ATTR_25MHZ
#define _AD         (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 0)
#define _AD0        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 0)
#define _AD1        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 1)
#define _AD2        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 2)
#define _AD3        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 3)
#define _AD4        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 4)
#define _AD5        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 5)
#define _AD6        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 6)
#define _AD7        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 7)
#define _AD8        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 8)
#define _AD9        (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 9)
#define _AD10       (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 10)
#define _AD11       (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 11)
#define _AD12       (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 12)
#define _AD13       (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 13)
#define _AD14       (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 14)
#define _AD15       (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_AD_ATTR      | IO_PORT_B | 15)

#define _READY      (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_ATTR_2MHZ    | IO_PORT_A | 3)
#define _RPLY       (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_RPLY_ATTR    | IO_PORT_C | 0)
#define _SYNC       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 3)
#define _DIN        (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 13)
#define _SEL0       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 9)
#define _SEL1       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 4)
#define _SEL2       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 5)
#define _SEL3       (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 7)

#define _QRPLY      (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 0)
#define _QSYNC      (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_RPLY_ATTR    | IO_PORT_C | 3)
#define _QDIN       (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_RPLY_ATTR    | IO_PORT_C | 5)  // SEL2
#define _QDOUT      (IO_FUNC_IO  | IO_MODE_OPEN_DRAIN   | IO_RPLY_ATTR    | IO_PORT_C | 7)  // SEL3

#define _USEL0      (IO_FUNC_IO  | IO_MODE_OUTPUT       | IO_ATTR_2MHZ    | IO_PORT_C | 6)
#define _USEL1      (IO_FUNC_IO  | IO_MODE_OUTPUT       | IO_ATTR_2MHZ    | IO_PORT_C | 8)
#define _Q32_IN     (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 14)
#define _Q32_OUT    (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_C | 15)
#define _OSC_IN     (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_H | 0)
#define _OSC_OUT    (IO_FUNC_IO  | IO_MODE_INPUT        | IO_MODE_PULL_UP | IO_PORT_H | 1)

//_____________________________________________________________________________
//
// Exceptions and Interrupts priority schema
//
//  0   (bNVIC_HIGHEST_PRIORITY) - the highest configurable priority
//  248 (bNVIC_LOWEST_PRIORITY)  - the lowest configurable priority
//
// The default priority schema (from higher to lower)
//
//  - the highest priority
//  - RESET         - hardware reset (fixed)            (-3)
//  - NMI           - non-maskable interrupt (fixed)    (-2)
//  - HARD FAULT    - hard fault (fixed)                (-1)
//
//  - the highest configurable (not fixed) priority
//  - MMU FAULT     - no rights to access memory        (BSP_MMUFAULT_PRI)
//  - BUS FAULT     - bus fault                         (BSP_BUSFAULT_PRI)
//  - USAGE FAULT   - execution fault                   (BSP_USAGEFAULT_PRI)
//  - SV CALL       - system service call (not used)    (BSP_SVCALL_PRI)
//  - SYSTICK       - system tick timer                 (BSP_SYSTICK_PRI)
//  - HW INT        - hardware interrupts               (BSP_HWINT_PRI)
//  - PEND SV       - context switch pending            (BSP_PENDSV_PRI)
//
#define BSP_MMUFAULT_PRI            (1u<<3)
#define BSP_BUSFAULT_PRI            (2u<<3)
#define BSP_USAGEFAULT_PRI          (3u<<3)
#define BSP_SVCALL_PRI              (4u<<3)
#define BSP_SYSTICK_PRI             BSP_HWINT_LOWEST_PRI
#define BSP_HWINT_HIGHEST_PRI       (8u<<3)
#define BSP_HWINT_PRI               (16u<<3)
#define BSP_HWINT_LOWEST_PRI        (24u<<3)
#define BSP_PENDSV_PRI              (31u<<3)

//_____________________________________________________________________________
//
#pragma pack(push, 4)
typedef struct _EXCEPTION_CONTEXT
{
    unsigned long   user_R4;
    unsigned long   user_R5;
    unsigned long   user_R6;
    unsigned long   user_R7;
    unsigned long   user_R8;
    unsigned long   user_R9;
    unsigned long   user_R10;
    unsigned long   user_R11;
    unsigned long   user_PSP;
    unsigned long   user_MSP;
    unsigned long   user_ELR;
    unsigned long   reserved;
    unsigned long   user_R0;
    unsigned long   user_R1;
    unsigned long   user_R2;
    unsigned long   user_R3;
    unsigned long   user_R12;
    unsigned long   user_LR;
    unsigned long   user_PC;
    unsigned long   user_PSR;

} EXCEPTION_CONTEXT;
#pragma pack(pop)

//_____________________________________________________________________________
//
#if defined (__ICCARM__)

#ifndef INLINE_FORCED
#define INLINE_FORCED   _Pragma("inline=forced")
#endif

#define DO_PRAGMA(x)    _Pragma (#x)
#define DATA_ALIGN(a)   DO_PRAGMA(data_alignment=##a)

#if (__BUILD_NUMBER__ < 0x7CB)
unsigned int __get_PSR(void);
#endif

//_____________________________________________________________________________
//
#elif defined ( __GNUC__ )

#ifndef INLINE_FORCED
#define INLINE_FORCED   static inline __attribute__ ((always_inline))
#endif

#define DATA_ALIGN(a)   __attribute__ ((aligned (a)))

INLINE_FORCED
unsigned long
__get_PSR(void)
{
    unsigned long ret;

    __asm volatile
    (
        "mrs    %0, PSR;" : "=r"(ret)
    );
    return ret;
}

INLINE_FORCED
unsigned long
__get_CONTROL(void)
{
    unsigned long ret;

    __asm volatile
    (
        "mrs    %0, CONTROL;" : "=r"(ret)
    );
    return ret;
}

INLINE_FORCED
unsigned long
__get_FAULTMASK(void)
{
    unsigned long ret;

    __asm volatile
    (
        "mrs    %0, FAULTMASK;" : "=r"(ret)
    );
    return ret;
}

INLINE_FORCED
unsigned long
__get_BASEPRI(void)
{
    unsigned long ret;

    __asm volatile
    (
        "mrs    %0, BASEPRI;" : "=r"(ret)
    );
    return ret;
}

INLINE_FORCED
void
__set_CONTROL(unsigned long mask)
{
    __asm volatile
    (
        "msr    CONTROL, %0;" : : "r"(mask)
    );
}

INLINE_FORCED
void
__set_FAULTMASK(unsigned long mask)
{
    __asm volatile
    (
        "msr    FAULTMASK, %0;" : : "r"(mask)
    );
}

INLINE_FORCED
void
__set_BASEPRI(unsigned long mask)
{
    __asm volatile
    (
        "msr    BASEPRI, %0;" : : "r"(mask)
    );
}

INLINE_FORCED
void
__disable_interrupt(void)
{
    __asm volatile("cpsid I;");
}

INLINE_FORCED
void
__enable_interrupt(void)
{
    __asm volatile("cpsie I;");
}

INLINE_FORCED
unsigned long
__get_PRIMASK(void)
{
    unsigned long ret;

    __asm volatile
    (
        "mrs    %0, PRIMASK;" : "=r"(ret)
    );
    return ret;
}

INLINE_FORCED
void
__set_PRIMASK(unsigned long mask)
{
    __asm volatile
    (
        "msr    PRIMASK, %0;" : : "r"(mask)
    );
}

INLINE_FORCED
void
__ISB(void)
{
    __asm volatile  ("isb;");
}

INLINE_FORCED
void
__DSB(void)
{
    __asm volatile  ("dsb;");
}

#else
  #error "Unknown compiler"
#endif

//_____________________________________________________________________________
//
void bsp_init_interrupt(void);
void bsp_init_clk(void);
void bsp_reset_handler(void);
void bsp_segment_init(void);
void bsp_enable_system_timer(void);
void bsp_thunk_handler(void);

void* bsp_memchr(const void* s, int c, unsigned int n);
void* bsp_memset(void* d, int c, unsigned int n);
void* bsp_memcpy(void* d, const void* s, unsigned int n);
unsigned int bsp_strlen(const char *s);

//_____________________________________________________________________________
//
#endif  // _IO_C207_H_
