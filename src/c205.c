//
//  Project:         RE-mulator (1801PE2/PP1 emulator)
//  File:            c205.c
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
#include "c205.h"
#include "debug.h"
#include "ioport.h"

extern void app_start(void);
//_____________________________________________________________________________
//
// Функция настройки основного генератора тактовой частоты на BSP_CORE_CLOCK
//
static void io_init_clock(void)
{
    unsigned int enable, value, setup, nmin, mmin, n, m;
    //
    // Настраивает согласно параметров:
    //      BSP_FLASH_WS    - число циклов ожидания FLASH
    //      BSP_QUARTZ_FREQ - частота основного кварца
    //      BSP_CORE_CLOCK  - выходная частота PLL/CPU
    //
    // Для перестраховки запретим прерывания
    //
    __disable_interrupt();
    //
    // Выбираем самый медленный режим работы памяти
    // Также сбрасываем и отключаем все кеши флеша
    //
    *FPEC_ACR = bFPEC_7WS;
    *FPEC_ACR = bFPEC_7WS | bFPEC_DCRST | bFPEC_ICRST;
    *FPEC_ACR = bFPEC_7WS;
    //
    // Гарантировано запускаем HSI
    //
    *RCC_CR |= bRCC_HSION;
    while((*RCC_CR & bRCC_HSIRDY) == 0);
    //
    // Переключаем контроллер на использование HSI
    //
    *RCC_CFG = (*RCC_CFG & ~bRCC_SW_MASK) | bRCC_SW_HSI;
    __ISB();
    __DSB();
    while((*RCC_CFG & bRCC_SWS_MASK) != bRCC_SWS_HSI);
    //
    // Выключаем PLL
    //
    *RCC_CR &= ~bRCC_PLLON;
    __ISB();
    __DSB();
    //
    // VCO может быть настроен в пределах 192..432 MHz
    // Опорная частота PLL может составлять 1..2 MHz
    // Выходная частота PLL 24..120 MHz
    //
    // Поскольку модуль OTG_FS требует 48МHz, то реальными
    // будут следующие варианты частот VCO и системных частот:
    //
    //  Q=4, 192 MHz    -> 24, 32, 48, 96 MHz
    //  Q=5, 240 MHz    -> 30, 40, 60, 120 MHz
    //  Q=6, 288 MHz    -> 36, 48, 72, 144 MHz (overclock)
    //  Q=7, 336 MHz    -> 42, 56, 84,
    //  Q=8, 384 MHz    -> 48, 64, 96,
    //  Q=9, 432 MHz    -> 54, 72, 108,
    //
    // Соответственно ряд допустимых частот системы:
    //  24, 30, 32, 36, 40, 42, 48, 54, 56,
    //  60, 64, 72, 84, 96, 108, 120, 144
    // Выбирается минимально возможная частота VCO
    // и максимально возможная входная частота PLL
    // (при этом минимизируется джиттер)
    //
    switch(BSP_CORE_CLOCK)
    {
        //
        //  Q=4, 192 MHz    -> 24, 32, 48, 96 MHz
        //  Q=5, 240 MHz    -> 30, 40, 60, 120 MHz
        //  Q=6, 288 MHz    -> 36, 48, 72, 144 MHz (overclock)
        //  Q=7, 336 MHz    -> 42, 56, 84,
        //  Q=8, 384 MHz    -> 48, 64, 96,
        //  Q=9, 432 MHz    -> 54, 72, 108,
        //
        case 24000000:  value = 192000; setup = bRCC_PLL_Q4; break;
        case 30000000:  value = 240000; setup = bRCC_PLL_Q5; break;
        case 32000000:  value = 192000; setup = bRCC_PLL_Q4; break;
        case 36000000:  value = 288000; setup = bRCC_PLL_Q6; break;
        case 40000000:  value = 240000; setup = bRCC_PLL_Q5; break;
        case 42000000:  value = 336000; setup = bRCC_PLL_Q7; break;
        case 48000000:  value = 192000; setup = bRCC_PLL_Q4; break;
        case 54000000:  value = 432000; setup = bRCC_PLL_Q9; break;
        case 56000000:  value = 336000; setup = bRCC_PLL_Q7; break;
        case 60000000:  value = 240000; setup = bRCC_PLL_Q5; break;
        case 64000000:  value = 384000; setup = bRCC_PLL_Q8; break;
        case 72000000:  value = 288000; setup = bRCC_PLL_Q6; break;
        case 84000000:  value = 336000; setup = bRCC_PLL_Q7; break;
        case 96000000:  value = 192000; setup = bRCC_PLL_Q4; break;
        case 108000000: value = 432000; setup = bRCC_PLL_Q9; break;
        case 120000000: value = 240000; setup = bRCC_PLL_Q5; break;
        case 144000000: value = 288000; setup = bRCC_PLL_Q6; break;
        default:
        {
            DBG_ASSERT(FALSE, "Unsupported SYSCLK configuration");
            return;
        }
    }
    DBG_ASSERT((value % (BSP_CORE_CLOCK/1000)) == 0, "Invalid SYSCLK configuration");
    switch(value / (BSP_CORE_CLOCK/1000))
    {
        case 2: setup |= bRCC_PLL_P2; break;
        case 4: setup |= bRCC_PLL_P4; break;
        case 6: setup |= bRCC_PLL_P6; break;
        case 8: setup |= bRCC_PLL_P8; break;
        default:
        {
            DBG_ASSERT(FALSE, "Unsupported SYSCLK configuration");
            return;
        }
    }
    //
    // Выбираем такую конфигурацию чтобы N был минимальный,
    // при фиксированном соотношении N:M = vco:quartz
    // это также означает минимальный M, при этом входная
    // частота PLL будет максимальной
    //
    nmin = 0xFFFF;
    mmin = 0xFFFF;
    //
    // Использование внутреннего RC-генератора
    //
    for(m=2; m<64; m++)
    {
        unsigned int temp;
        //
        // Вычисляем максимальную частоту для данного m
        //
        temp = ((BSP_HSI_FREQ/1000)*432) / m;
        if (temp < value)
        {
            //
            // Дальше смысла продолжать цикл нет
            // Нужная частота VCО достигнута не будет
            //
            break;
        }
        //
        // Вычисляем минимальную частоту для данного m
        //
        temp = ((BSP_HSI_FREQ/1000)*192) / m;
        if (temp > value)
        {
            //
            // Нет смысла перебирать частоты
            //
            continue;
        }
        n = (value * m) / (BSP_HSI_FREQ/1000);
        temp = ((BSP_HSI_FREQ/1000)*n)/m;
        if (temp == value)
        {
            if (n < nmin)
            {
                nmin = n;
                mmin = m;
            }
            continue;
        }
        n--;
        temp = ((BSP_HSI_FREQ/1000)*n)/m;
        if (temp == value)
        {
            if (n < nmin)
            {
                nmin = n;
                mmin = m;
            }
            continue;
        }
    }
//
//  dbg_init();
//  dbg_printf("\r\nbsp_init_clock()\f");
//  dbg_printf("\r\nVCO: %d, m=%d, n=%d\f", value, mmin, nmin);
//
    DBG_ASSERT((nmin >= 192) && (nmin <= 432), "Invalid SYSCLK configuration");
    DBG_ASSERT((mmin >= 2) && (mmin <= 63), "Invalid SYSCLK configuration");
    DBG_ASSERT(((BSP_HSI_FREQ/1000)/mmin) >= 1000, "Invalid SYSCLK configuration");
    DBG_ASSERT(((BSP_HSI_FREQ/1000)/mmin) <= 2000, "Invalid SYSCLK configuration");

    setup |= nmin << bRCC_PLL_NSHIFT;
    setup |= mmin << bRCC_PLL_MSHIFT;
    //
    // Настраиваем основную PLL1 на нужную частоту
    //
    *RCC_PLLCFG = setup;
    __ISB();
    __DSB();
    //
    // Запускаем PLL1 и ждем готовности
    //
    *RCC_CR |= bRCC_PLLON;
    *RCC_CR &= ~bRCC_CSSON;
    __ISB();
    __DSB();
    while((*RCC_CR & bRCC_PLLRDY) == 0);

    //
    // Настраиваем делители шин APB1, APB2 и AHBx
    //
    value = bRCC_SW_HSI | bRCC_HPRE_1;

#if (BSP_CORE_CLOCK % BSP_APB_CLOCK)
#error  "Unable to configure APB clock"
#endif
    switch(BSP_CORE_CLOCK / BSP_APB_CLOCK)
    {
        case 1:     value |= bRCC_PPRE2_1  | bRCC_PPRE1_1; break;
        case 2:     value |= bRCC_PPRE2_2  | bRCC_PPRE1_2; break;
        case 4:     value |= bRCC_PPRE2_4  | bRCC_PPRE1_4; break;
        case 8:     value |= bRCC_PPRE2_8  | bRCC_PPRE1_8; break;
        case 16:    value |= bRCC_PPRE2_16 | bRCC_PPRE1_16; break;
        default:
        {
            DBG_ASSERT(FALSE, "Unable to configure APB clock");
            value |= bRCC_PPRE2_16 | bRCC_PPRE1_16;
            break;
        }
    }
    *RCC_CFG = value;
    //
    // Настроим контроллер флеша
    //
    *FPEC_ACR = BSP_FLASH_WS;
    __ISB();
    __DSB();
    *FPEC_ACR = BSP_FLASH_WS | bFPEC_DCRST | bFPEC_ICRST;
    __ISB();
    __DSB();
    *FPEC_ACR = BSP_FLASH_WS;
    __ISB();
    __DSB();
    *FPEC_ACR = BSP_FLASH_WS | bFPEC_DCEN | bFPEC_ICREN | bFPEC_PRFTEN;
    //
    // Переключаемся на освновную PLL
    //
    *RCC_CFG = (*RCC_CFG & ~bRCC_SW_MASK) | bRCC_SW_PLL;
    __ISB();
    __DSB();
    while((*RCC_CFG & bRCC_SWS_MASK) != bRCC_SWS_PLL);
    //
    // Включаем нужную периферию
    //
    enable  = bRCC_DMA2EN
            | bRCC_DMA1EN
            | bRCC_BKPSRAMEN
            | bRCC_SRAM2EN
            | bRCC_SRAM1EN
            | bRCC_FLITFEN
            | bRCC_IOPIEN
            | bRCC_IOPHEN
            | bRCC_IOPGEN
            | bRCC_IOPFEN
            | bRCC_IOPEEN
            | bRCC_IOPDEN
            | bRCC_IOPCEN
            | bRCC_IOPBEN
            | bRCC_IOPAEN;
    *RCC_AHB1EN = enable;
    *RCC_AHB1LP = enable;

    enable  = 0;
    *RCC_AHB2EN = enable;
    *RCC_AHB2LP = enable;

    enable  = 0;
    *RCC_AHB3EN = enable;
    *RCC_AHB3LP = enable;

    enable      = bRCC_PWREN;
    *RCC_APB1EN = enable;
    *RCC_APB1LP = enable;

    enable      = bRCC_SYSCFGEN;
    *RCC_APB2EN = enable;
    *RCC_APB2LP = enable;
}

//_____________________________________________________________________________
//
// Функция начальной инициализации ячеек памяти
// после аппаратного сброса
//
static void io_init_memory(void)
{
    //
    // Общая инициализация и обнуление переменных
    //
    bsp_segment_init();
//
//  Все переменные, требующие инициализации должны быть
//  вынесены из сегмента NO_INIT (убран модификатор RAM),
//  тогда будет иметь место автоматическая инициализация
//  функцией bsp_segment_init()
//
}

//_____________________________________________________________________________
//
extern void* bsp_vector_table;
void bsp_init_interrupt(void)
{
    __disable_interrupt();
    //
    // Disable system timer interrupts
    //
    *SC_STCTRL = bSTCTRL_CLKSOURCE;
    //
    // Disable all hardware interrupts
    //
    *NVIC_IDCR0 = 0xFFFFFFFF;
    *NVIC_IDCR1 = 0xFFFFFFFF;
    *NVIC_IDCR2 = 0x000FFFFF;
    *NVIC_ICPR0 = 0xFFFFFFFF;
    *NVIC_ICPR1 = 0xFFFFFFFF;
    *NVIC_ICPR2 = 0x000FFFFF;
    //
    // Disable external interrupts controller
    //
    *EXTI_IMR   = 0;
    *EXTI_EMR   = 0;
    *EXTI_SWIER = 0;
    *EXTI_PR    = 0xFFFFFFFF;
    //
    // Setup the priority schema
    //
    *NVIC_IPR0  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR1  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR2  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR3  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR4  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR5  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR6  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR7  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;

    *NVIC_IPR8  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR9  = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR10 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR11 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR12 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR13 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR14 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR15 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;

    *NVIC_IPR16 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR17 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR18 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR19 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    *NVIC_IPR20 = (BSP_HWINT_PRI<<24) | (BSP_HWINT_PRI<<16) | (BSP_HWINT_PRI<<8) | BSP_HWINT_PRI;
    //
    // Setup the priority for exceptions
    //
    *SC_SHPR1   = (BSP_USAGEFAULT_PRI<<16) | (BSP_BUSFAULT_PRI<<8) | BSP_MMUFAULT_PRI;
    *SC_SHPR2   = (BSP_SVCALL_PRI<<24);
    *SC_SHPR3   = (BSP_SYSTICK_PRI<<24) | (BSP_PENDSV_PRI<<16);
    *SC_ICSR    = bICSR_PENDSVCLR | bICSR_PENDSTCLR;
    *SC_AIRCR   = bAIRCR_VECTKEY | bAIRCR_PRIGROUP_32;
    *SC_SCR     = 0;
    //
    // Set the pointer to the vector table
    //
    DBG_ASSERT(((unsigned long)&bsp_vector_table & 0x3FF) == 0, "Unaligned vector table");
    *SC_VTOR    = (unsigned long)&bsp_vector_table;
    __DSB();
    //
    // Enable all useful exceptions
    //
    *SC_SHCSR   = bSHCSR_USGFAULTENA
                | bSHCSR_BUSFAULTENA
                | bSHCSR_MEMFAULTENA;
    __set_FAULTMASK(0);
    __set_BASEPRI(0);

    *SC_CCR     = bCCR_STKALIGN
                | bCCR_DIV_0_TRP
                | bCCR_UNALIGN_TRP;
}

//_____________________________________________________________________________
//
void bsp_show_context(EXCEPTION_CONTEXT *context, unsigned long primask)
{
    unsigned long* pBuf;
    unsigned long i, j;

    dbg_printf("\r\n");
    dbg_printf("\r\n  CONTROL=%02X    FLTMASK=%02X    PRIMASK=%02X    BASEPRI=%02X (%d)",
                __get_CONTROL(),
                __get_FAULTMASK(),
                primask,
                __get_BASEPRI(),
                __get_BASEPRI()>>3);
    dbg_printf("\r\n  R0=%08X   R1=%08X   R2=%08X   R3=%08X",
                        context->user_R0,
                        context->user_R1,
                        context->user_R2,
                        context->user_R3);

    dbg_printf("\r\n  R4=%08X   R5=%08X   R6=%08X   R7=%08X",
                        context->user_R4,
                        context->user_R5,
                        context->user_R6,
                        context->user_R7);

    dbg_printf("\r\n  R8=%08X   R9=%08X  R10=%08X  R11=%08X",
                        context->user_R8,
                        context->user_R9,
                        context->user_R10,
                        context->user_R11);

    dbg_printf("\r\n R12=%08X  MSP=%08X  PSP=%08X   LR=%08X",
                        context->user_R12,
                        context->user_MSP,
                        context->user_PSP,
                        context->user_LR);

    dbg_printf("\r\n  PC=%08X  ELR=%08X",
                        context->user_PC,
                        context->user_ELR);
    switch(context->user_ELR)
    {
        case 0xFFFFFFE1:
        case 0xFFFFFFF1: dbg_printf(" (handler mode)"); break;
        case 0xFFFFFFE9:
        case 0xFFFFFFF9: dbg_printf(" (thread mode, main stack)"); break;
        case 0xFFFFFFED:
        case 0xFFFFFFFD: dbg_printf(" (thread mode, process stack)"); break;
        default: dbg_printf(" (invalid exit value)"); break;
    }
    dbg_printf("\r\n\f");

    pBuf = (unsigned long*)(((context->user_PC)-8) & ~0x03);
    if ((unsigned long)pBuf >= (unsigned long)STM_XN_LIMIT)
    {
        dbg_printf("\r\nNon-executable PC, failed to fetch instruction opcodes");
    }
    else
    {
        for(i=0; i<4; i++)
        {
            dbg_printf("\r\n%08X: ", pBuf);
            for(j=0; j<4; j++)
            {
                dbg_printf(" %08X", pBuf[j]);
            }
            pBuf += 4;
        }
    }
    dbg_printf("\r\n\f");
}

//_____________________________________________________________________________
//
//  The default exception and interrupt handler for debugging purposes
//
void bsp_default_handler(EXCEPTION_CONTEXT *context)
{
#if DBG_ENABLE
    const char* name;
    unsigned long tmp, lock;

    lock = __get_PRIMASK();
    __disable_interrupt();
    dbg_init();
    dbg_printf("\r\nDEBUG assertion failure at line %d in \"%s\"\f", __LINE__, __FILE__);
    tmp = __get_PSR() & 0x3FF;
    switch(tmp)
    {
        case 0x02: name = "NMI"; break;
        case 0x03: name = "Hard Fault"; break;
        case 0x04: name = "MMU Fault"; break;
        case 0x05: name = "Bus Fault"; break;
        case 0x06: name = "Usage Fault"; break;
        case 0x0B: name = "SV Call"; break;
        case 0x0C: name = "Debug"; break;
        case 0x0E: name = "SV Pend"; break;
        case 0x0F: name = "Sys Tick"; break;
        default:
        {
            if (tmp >= 16)
            {
                name = "Interrupt";
            }
            else
            {
                name = "Reserved";
            }
            break;
        }
    }
    dbg_printf("\r\n\"Not initialized exception occured (%d, %s)\"\f", tmp, name);
    switch(tmp)
    {
        case 0x04:
        {
            tmp = *SC_CFSR;
            dbg_printf(", %08X", tmp);
            if (tmp & bCFSR_MMARVALID)
            {
                dbg_printf("\r\nMMFAR=%08X", *SC_MMFAR);
            }
            break;
        }
        case 0x05:
        {
            tmp = *SC_CFSR;
            dbg_printf(", %08X", tmp);
            if (tmp & bCFSR_BFARVALID)
            {
                dbg_printf("\r\nBFAR=%08X", *SC_BFAR);
            }
            break;
        }
        case 0x06:
        {
            tmp = *SC_CFSR;
            if (tmp & (   bCFSR_DIVBYZERO
                        | bCFSR_UNALIGNED
                        | bCFSR_NOCP
                        | bCFSR_INVPC
                        | bCFSR_INVSTATE
                        | bCFSR_UNDEFINSTR))
            {
                dbg_printf(", %08X\r\n", tmp);
                if (tmp & bCFSR_DIVBYZERO)  dbg_printf(" ZERO_DIVISION");
                if (tmp & bCFSR_UNALIGNED)  dbg_printf(" UNALIGNED_ACCESS");
                if (tmp & bCFSR_NOCP)       dbg_printf(" NO_COPROCESSOR");
                if (tmp & bCFSR_INVPC)      dbg_printf(" INVALID_PC_LOAD");
                if (tmp & bCFSR_INVSTATE)   dbg_printf(" INVALID_STATE");
                if (tmp & bCFSR_UNDEFINSTR) dbg_printf(" UNDEFINED_OPCODE");
            }
            break;
        }
    }
    bsp_show_context(context, lock);
#endif
    for(;;);
}

void abort(void)
{
    DBG_ASSERT(FALSE, "Abort exception occured");
    for(;;);
}

//_____________________________________________________________________________
//
#ifdef __GNUC__
extern unsigned char __bss_start__;
extern unsigned char __bss_end__;
extern unsigned char __data_init__;
extern unsigned char __data_start__;
extern unsigned char __data_end__;

void bsp_segment_init(void)
{
//
//  dbg_init();
//  dbg_printf("\r\nSegment init zero...(%08X, %08X)\f", &__bss_start__, &__bss_end__- &__bss_start__);
//  dbg_printf("\r\nSegment init copy...(%08X, %08X)\f", &__data_init__, &__data_end__ - &__data_start__);
//
    bsp_memset(&__bss_start__, 0, &__bss_end__ - &__bss_start__);
    bsp_memcpy(&__data_start__, &__data_init__, &__data_end__ - &__data_start__);

}
#endif

#ifdef __ICCARM__
//_____________________________________________________________________________
//
// The initialization for IAR 5.x && IAR 6.x
// The dedicated function for processing buffers from the tables
//
typedef unsigned long const* INIT_FUNC(unsigned long const*);

#ifdef __cplusplus
#error "This file should be compiled in C mode"
#endif

typedef void (*__func_ptr) (void);
typedef __func_ptr * __difunct_ptr;
#pragma section = "Region$$Table"

void bsp_segment_init(void)
{
    unsigned long const* p = __section_begin("Region$$Table");

    while (p != __section_end("Region$$Table"))
    {
#if (__BUILD_NUMBER__ >= 0x7CB)

        // With versions greater then 5.50, compiler placed not the ptr to function,
        // but offset to function

        INIT_FUNC* fun = (INIT_FUNC*) ((char*)p + *p);
        p++;
#else
        // With versions less than 5.50, compiler placed  the ptr to function,

        INIT_FUNC* fun = (INIT_FUNC*)*p++;
#endif
        p = fun(p);
    }
}
#endif

//_____________________________________________________________________________
//
// Инициализация контроллера прерываний
//
static void io_init_interrupt(void)
{
    bsp_init_interrupt();
    //
    // Разрешим системный таймер для работы bsp_delay_us()
    //
    *SC_STCTRL = bSTCTRL_CLKSOURCE;
    *SC_STLOAD = (BSP_CORE_CLOCK/1000) - 1;
    *SC_STVAL  = (BSP_CORE_CLOCK/1000) - 1;
    *SC_STCTRL = bSTCTRL_CLKSOURCE
               | bSTCTRL_ENABLE;
}

//_____________________________________________________________________________
//
// Основная точка входа  после ассемблерной инициализации
//
void io_startup(void)
{
    io_init_clock();                // инициализируем тактовый генератор
    io_init_memory();               // инициализируем переменные памяти
    io_init_interrupt();            // инициализируем контроллер прерываний
    //
    // Переходим к выполнению основного приложения
    //
    app_start();
}

