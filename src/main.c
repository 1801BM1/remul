//
//  Project:         RE-mulator (1801PE2/PP1 emulator)
//  File:            main.c
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

//_____________________________________________________________________________
//
unsigned int    app_config;
unsigned char*  app_mapper[8*ROM_TABLE_INDEX];
unsigned int    app_copy[256];

#define IO_OUTPUT               bPIO_MODE_OUTPUT
#define IO_INPUT                bPIO_MODE_INPUT
#define IO_INPUT_PULL           (bPIO_MODE_INPUT | 0x10)
#define IO_INPUT_ANALOG         bPIO_MODE_ANALOG

#define IO_PORT_SETUP_ENTRY(port)                           \
{                                                           \
    Type        = 0;                                        \
    Data        = port << 16;                               \
    Mode        = (bPIO_MODE_INPUT << 0)                    \
                | (bPIO_MODE_INPUT << 2)                    \
                | (bPIO_MODE_INPUT << 4)                    \
                | (bPIO_MODE_INPUT << 6)                    \
                | (bPIO_MODE_INPUT << 8)                    \
                | (bPIO_MODE_INPUT << 10)                   \
                | (bPIO_MODE_INPUT << 12)                   \
                | (bPIO_MODE_INPUT << 14)                   \
                | (bPIO_MODE_INPUT << 16)                   \
                | (bPIO_MODE_INPUT << 18)                   \
                | (bPIO_MODE_INPUT << 20)                   \
                | (bPIO_MODE_INPUT << 22)                   \
                | (bPIO_MODE_INPUT << 24)                   \
                | (bPIO_MODE_INPUT << 26)                   \
                | (bPIO_MODE_INPUT << 28)                   \
                | (bPIO_MODE_INPUT << 30);                  \
                                                            \
    Attr        = (bPIO_OUTPUT_2MHZ << 0)                   \
                | (bPIO_OUTPUT_2MHZ << 2)                   \
                | (bPIO_OUTPUT_2MHZ << 4)                   \
                | (bPIO_OUTPUT_2MHZ << 6)                   \
                | (bPIO_OUTPUT_2MHZ << 8)                   \
                | (bPIO_OUTPUT_2MHZ << 10)                  \
                | (bPIO_OUTPUT_2MHZ << 12)                  \
                | (bPIO_OUTPUT_2MHZ << 14)                  \
                | (bPIO_OUTPUT_2MHZ << 16)                  \
                | (bPIO_OUTPUT_2MHZ << 18)                  \
                | (bPIO_OUTPUT_2MHZ << 20)                  \
                | (bPIO_OUTPUT_2MHZ << 22)                  \
                | (bPIO_OUTPUT_2MHZ << 24)                  \
                | (bPIO_OUTPUT_2MHZ << 26)                  \
                | (bPIO_OUTPUT_2MHZ << 28)                  \
                | (bPIO_OUTPUT_2MHZ << 30);                 \
                                                            \
    Pull        = (bPIO_PULL_DOWN << 0)                     \
                | (bPIO_PULL_DOWN << 2)                     \
                | (bPIO_PULL_DOWN << 4)                     \
                | (bPIO_PULL_DOWN << 6)                     \
                | (bPIO_PULL_DOWN << 8)                     \
                | (bPIO_PULL_DOWN << 10)                    \
                | (bPIO_PULL_DOWN << 12)                    \
                | (bPIO_PULL_DOWN << 14)                    \
                | (bPIO_PULL_DOWN << 16)                    \
                | (bPIO_PULL_DOWN << 18)                    \
                | (bPIO_PULL_DOWN << 20)                    \
                | (bPIO_PULL_DOWN << 22)                    \
                | (bPIO_PULL_DOWN << 24)                    \
                | (bPIO_PULL_DOWN << 26)                    \
                | (bPIO_PULL_DOWN << 28)                    \
                | (bPIO_PULL_DOWN << 30);                   \
}

#define IO_PORT_SETUP_LEAVE(port)                           \
{                                                           \
    DBG_ASSERT(port == (Data >> 16), "Invalid port setup"); \
    IO_PORT(port)->sPIO_TYPE  = (Type & 0xFFFF);            \
    IO_PORT(port)->sPIO_ODR   = (Data & 0xFFFF);            \
    IO_PORT(port)->sPIO_MODE  = Mode;                       \
    IO_PORT(port)->sPIO_PUPD  = Pull;                       \
    IO_PORT(port)->sPIO_SPEED = Attr;                       \
}

#define IO_PORT_SETUP(name, mode, data, type)               \
{                                                           \
    unsigned int shift;                                     \
                                                            \
    Type |= type << IO_MASK_N(name);                        \
    shift = IO_MASK_N(name)*2;                              \
    Pull &= ~(bPIO_PULL_MASK<<shift);                       \
    Pull |= (bPIO_PULL_NONE<<shift);                        \
                                                            \
    switch(mode)                                            \
    {                                                       \
        case IO_OUTPUT:                                     \
        {                                                   \
            Mode |= bPIO_MODE_OUTPUT<<shift;                \
            switch(name & IO_ATTR_MSK)                      \
            {                                               \
                case IO_ATTR_2MHZ:                          \
                {                                           \
                    Attr |= bPIO_OUTPUT_2MHZ<<shift;        \
                    break;                                  \
                }                                           \
                case IO_ATTR_25MHZ:                         \
                {                                           \
                    Attr |= bPIO_OUTPUT_25MHZ<<shift;       \
                    break;                                  \
                }                                           \
                case IO_ATTR_50MHZ:                         \
                {                                           \
                    Attr |= bPIO_OUTPUT_50MHZ<<shift;       \
                    break;                                  \
                }                                           \
                case IO_ATTR_100MHZ:                        \
                {                                           \
                    Attr |= bPIO_OUTPUT_100MHZ<<shift;      \
                    break;                                  \
                }                                           \
            }                                               \
            Data |= (data & 0x01) << (shift/2);             \
            break;                                          \
        }                                                   \
        case IO_INPUT:                                      \
        {                                                   \
            Mode |= bPIO_MODE_INPUT<<shift;                 \
            break;                                          \
        }                                                   \
        case IO_INPUT_PULL:                                 \
        {                                                   \
            Pull &= ~(bPIO_PULL_MASK<<shift);               \
            Mode |= bPIO_MODE_INPUT<<shift;                 \
            if (data & 0x01)                                \
            {                                               \
                Pull |= bPIO_PULL_UP<<shift;                \
            }                                               \
            else                                            \
            {                                               \
                Pull |= bPIO_PULL_DOWN<<shift;              \
            }                                               \
            break;                                          \
        }                                                   \
        case IO_INPUT_ANALOG:                               \
        {                                                   \
            Mode |= bPIO_MODE_ANALOG<<shift;                \
            break;                                          \
        }                                                   \
        default:                                            \
        {                                                   \
            DBG_ASSERT(FALSE, "Invalid mode setting");      \
            break;                                          \
        }                                                   \
    }                                                       \
    DBG_ASSERT((Data >> 16) == (IO_PORT_N(name)<<8),        \
                                    "Invalid port setup");  \
    DBG_ASSERT(IO_MASK_N(name) < 16, "Invalid pin mask");   \
}

//_____________________________________________________________________________
//
// Функции для инициализации периферийных регистров
//
static void app_init_port(void)
{
    unsigned int Mode, Data, Pull, Attr, Type;

    //
    // Разрешаем работу Compemsation Cell чтобы
    // корректно отрабатывало ограничение
    // скорости работы входов портов
    //
    *RCC_APB2EN |= bRCC_SYSCFGEN;
    *RCC_APB2LP |= bRCC_SYSCFGEN;
    __DSB();
    __ISB();
    *SYSCFG_CMPCR = bSYSCFG_CMP_PD;
    while((*SYSCFG_CMPCR & bSYSCFG_READY) == 0);
    //
    // Настройка порта A
    //
    IO_PORT_SETUP_ENTRY(IO_PORT_A);
    {
        IO_PORT_SETUP(_CFG0,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_CFG1,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_CFG2,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_CFG3,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_CFG4,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_CFG5,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_TXD,     IO_OUTPUT,      1, 0);
        IO_PORT_SETUP(_RXD,     IO_INPUT,       0, 0);
        IO_PORT_SETUP(_READY,   IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_TMS,     IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_TCK,     IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_TDI,     IO_INPUT_PULL,  1, 0);
    }
    IO_PORT_SETUP_LEAVE(IO_PORT_A);
    //
    // Настройка порта B
    //
    IO_PORT_SETUP_ENTRY(IO_PORT_B);
    {
        IO_PORT_SETUP(_AD0,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD1,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD2,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD3,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD4,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD5,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD6,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD7,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD8,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD9,     IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD10,    IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD11,    IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD12,    IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD13,    IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD14,    IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_AD15,    IO_OUTPUT,      1, 1);
    }
    IO_PORT_SETUP_LEAVE(IO_PORT_B);
    //
    // Настройка порта C
    //
    IO_PORT_SETUP_ENTRY(IO_PORT_C);
    {
        IO_PORT_SETUP(_RPLY,    IO_OUTPUT,      1, 1);
        IO_PORT_SETUP(_SYNC,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_DIN,     IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_SEL0,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_SEL1,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_SEL2,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_SEL3,    IO_INPUT_PULL,  1, 0);
        IO_PORT_SETUP(_Q32_IN,  IO_INPUT_PULL,  0, 0);
        IO_PORT_SETUP(_Q32_OUT, IO_INPUT_PULL,  0, 0);
        IO_PORT_SETUP(_USEL0,   IO_OUTPUT,      0, 1);
        IO_PORT_SETUP(_USEL1,   IO_OUTPUT,      0, 1);
    }
    IO_PORT_SETUP_LEAVE(IO_PORT_C);
    //
    // Перестраховочная настройка неиспользуемых портов
    //
    IO_PORT_SETUP_ENTRY(IO_PORT_D);
    IO_PORT_SETUP_LEAVE(IO_PORT_D);
    IO_PORT_SETUP_ENTRY(IO_PORT_E);
    IO_PORT_SETUP_LEAVE(IO_PORT_E);
    IO_PORT_SETUP_ENTRY(IO_PORT_F);
    IO_PORT_SETUP_LEAVE(IO_PORT_F);
    IO_PORT_SETUP_ENTRY(IO_PORT_G);
    IO_PORT_SETUP_LEAVE(IO_PORT_G);

    IO_PORT_SETUP_ENTRY(IO_PORT_H);
    {
        IO_PORT_SETUP(_OSC_IN,  IO_INPUT_PULL,  0, 0);
        IO_PORT_SETUP(_OSC_OUT, IO_INPUT_PULL,  0, 0);
    }
    IO_PORT_SETUP_LEAVE(IO_PORT_H);

    *PWR_CR = bPWR_PVD_27
            | bPWR_PVDE
            | bPWR_CSBF
            | bPWR_CWUF;
}

//_____________________________________________________________________________
//
// Приблизительная задержка на указанное число микросекунд
// Требует корректной настройки системного таймера
//
static void io_delay_us(unsigned int us)
{
    unsigned int stop, cur, prev, div;
    //
    // Системный таймер настроен на частоту 1 кГц, значит 1 мкс
    // соответствует примерно 1/1000 от ограничителя счетчика
    // Также учитываем что счетчик декрементируется - инвертируем
    //
    div  = *SC_STLOAD + 1;
    prev = div - *SC_STVAL;
    stop = prev + (us * div)/1000;

    while ((cur = div - *SC_STVAL) < stop)
    {
        if (cur < prev)         //
        {                       // проверим не произошел
            if (stop < div)     // ли перенос таймера
            {                   //
                break;          //
            }
            stop -= div;
        }
        prev = cur;
    }
}

static void io_delay_ms(unsigned int ms)
{
    if (ms != 0)
    {
        do
        {
            io_delay_us(1000);
            ms--;
        }
        while(ms);
    }
}

//_____________________________________________________________________________
//
static void app_read_config(void)
{
    unsigned int temp;

    temp = 0;
    if (!IO_PORT_IN(_CFG0)) temp |= (1<<0);
    if (!IO_PORT_IN(_CFG1)) temp |= (1<<1);
    if (!IO_PORT_IN(_CFG2)) temp |= (1<<2);
    if (!IO_PORT_IN(_CFG3)) temp |= (1<<3);
    if (!IO_PORT_IN(_CFG4)) temp |= (1<<4);
//
// _CFG5 зарезервирован для управления загрузчиком
//
//  if (!IO_PORT_IN(_CFG5)) temp |= (1<<5);
//
    app_config = temp;
#if DBG_ENABLE
    dbg_printf("\r\nConfiguration %d selected\f", app_config);
#endif
}

//_____________________________________________________________________________
//
static void app_map_image(void)
{
    PROM_DESC prom;
    unsigned char** line;
    unsigned int i, mask, conf;

    bsp_memset(app_mapper, 0, sizeof(app_mapper));
    prom = (PROM_DESC)ROM_TABLE_BASE;
    //
    // Вывод _CFG0 задействован для активации загрузчика
    //
    conf = app_config & 0x0F;
    if (conf > sizeof(prom->csel))
    {
        //
        // Старшие конфигурации специальные (тестирование),
        // они не предполагают загрузки прошивок
        //
#if DBG_ENABLE
        dbg_printf("\r\nImage mapper skipped for configuration %d", conf);
#endif
        return;
    }
    do
    {
        if (prom->sign != ROM_IMAGE_SIGN)
        {
            //
            // Не обнаружена сигнатура, конец массива
            //
            break;
        }
        if (prom->code > 0x07)
        {
            //
            // Некорректный чип код, конец массива
            //
            break;
        }
#if DBG_ENABLE
        dbg_printf("\r\nImage found: code=%d, csel=%02X",
                            prom->code,
                            prom->csel[conf]);
#endif
        line = app_mapper + (prom->code * ROM_TABLE_INDEX);
        mask = 0;
        switch(prom->csel[conf])
        {
            case 0x00:
            {
                mask = (IO_MASK(_SEL0) >> (MIN( IO_MASK_N(_SEL0),
                                           MIN( IO_MASK_N(_SEL1),
                                           MIN( IO_MASK_N(_SEL2),
                                                IO_MASK_N(_SEL3))))));

                break;
            }
            case 0x11:
            {
                mask = (IO_MASK(_SEL1) >> (MIN( IO_MASK_N(_SEL0),
                                           MIN( IO_MASK_N(_SEL1),
                                           MIN( IO_MASK_N(_SEL2),
                                                IO_MASK_N(_SEL3))))));

                break;
            }
            case 0x22:
            {
                mask = (IO_MASK(_SEL2) >> (MIN( IO_MASK_N(_SEL0),
                                           MIN( IO_MASK_N(_SEL1),
                                           MIN( IO_MASK_N(_SEL2),
                                                IO_MASK_N(_SEL3))))));

                break;
            }
            case 0x33:
            {
                mask = (IO_MASK(_SEL3) >> (MIN( IO_MASK_N(_SEL0),
                                           MIN( IO_MASK_N(_SEL1),
                                           MIN( IO_MASK_N(_SEL2),
                                                IO_MASK_N(_SEL3))))));

                break;
            }
            default:
            {
                break;
            }
        }
        dbg_printf(", mask=%04X", mask);

        if (mask != 0)
        {
            //
            // В данной конфигурации эта прошивка используется
            //
            for(i=0; i < ROM_TABLE_INDEX; i++)
            {
                if ((i & mask) == 0)
                {
                    //
                    // Выбранный селект активен для данного индекса
                    // При нормальной работе конфликтов с одновременно
                    // активированными чип-селектами быть не должно
                    // Для устранения конфликта приоритет будет иметь
                    // прошивка идущая в массиве раньше
                    //
                    if (*line == NULL)
                    {
                        //
                        // Данный указатель не используется другим селектом
                        // (не установлен ранее и ячейка таблицы пуста)
                        //
                        *line = (unsigned char*)prom->data;
                    }
                }
                line++;
            }
        }
        prom++;
    }
    while(TRUE);

#if DBG_ENABLE
    //
    // Выведем массив в отладку
    //
    for(i=0; i<8; i++)
    {
        BOOL out;
        unsigned int j;

        out = FALSE;
        for(j=0; j<ROM_TABLE_INDEX; j++)
        {
            if (app_mapper[i*ROM_TABLE_INDEX + j] != NULL)
            {
                if (!out)
                {
                    out = TRUE;
                    dbg_printf("\r\ncode %d: ", i);
                }
                dbg_printf(" %02X", j);
            }
        }
    }
#endif
}

//_____________________________________________________________________________
//
static void app_start_proc(void)
{
    unsigned char *start, *end;
    APP_LOOP_PTR addr;

    //
    // Выполняем копирование процедуры в ОЗУ для
    // получения макисмального быстродействия
    //
    start = (unsigned char*)(((unsigned int)&app_rom_loop) & ~1);
    end   = (unsigned char*)(((unsigned int)&app_end_loop) & ~1);
    bsp_memcpy(app_copy, start, end - start + 2*sizeof(unsigned int));
    //
    // Снимаем сигнал внешнего сброса
    //
    IO_PORT_ODC(_READY);
    //
    // Выполняем переход на циклическую процедуру
    //
    // Отладочный вариант исполнения из флеш-памяти
    // addr = &app_rom_loop;
    //
    addr = (APP_LOOP_PTR)(((unsigned int)app_copy) | 1);
    addr(app_mapper, IO_PORT(_AD), IO_PORT(_SYNC));
}

//_____________________________________________________________________________
//
#if DBG_ENABLE
static void app_qbus_attach(void)
{
    IO_PORT(_AD)->sPIO_ODR = 0xFFFF;
    IO_PORT_ODS(_QSYNC);
    IO_SET_MODE(_QSYNC);

    IO_PORT_ODS(_QDIN);
    IO_SET_MODE(_QDIN);

    IO_PORT_ODS(_QDOUT);
    IO_SET_MODE(_QDOUT);

    IO_SET_MODE(_QRPLY);
    io_delay_us(50);
}

static void app_qbus_detach(void)
{
    IO_PORT(_AD)->sPIO_ODR = 0xFFFF;
    IO_PORT_ODS(_QSYNC);
    IO_SET_MODE(_SYNC);

    IO_PORT_ODS(_QDIN);
    IO_SET_MODE(_DIN);

    IO_PORT_ODS(_QDOUT);
    IO_PORT_ODS(_RPLY);
    IO_SET_MODE(_RPLY);
    IO_SET_MODE(_SEL1);
    IO_SET_MODE(_SEL2);
    IO_SET_MODE(_SEL3);

    io_delay_us(50);
}

static int app_qbus_read(unsigned int addr)
{
    unsigned int wait;

    IO_PORT(_AD)->sPIO_ODR = addr;
    io_delay_us(1);
    IO_PORT_ODC(_QSYNC);
    io_delay_us(1);
    IO_PORT(_AD)->sPIO_ODR = 0xFFFF;
    io_delay_us(1);
    IO_PORT_ODC(_QDIN);

    wait = 64;
    do
    {
        if (!IO_PORT_IN(_QRPLY))
        {
            wait = IO_PORT(_AD)->sPIO_IDR;
            IO_PORT_ODS(_QDIN);
            io_delay_us(1);
            IO_PORT_ODS(_QSYNC);
            io_delay_us(1);
            return wait;
        }
        io_delay_us(1);
        wait--;
    }
    while(wait);
    //
    // Возник тайм-аут ожидания сигнала ~RPLY
    //
    IO_PORT_ODS(_QDIN);
    io_delay_us(1);
    IO_PORT_ODS(_QSYNC);
    io_delay_us(1);
    return -1;
}

static unsigned short ihex_addr;
static unsigned char  ihex_summ;
static unsigned char  ihex_count;

static void app_ihex_open(void)
{
    ihex_addr  = 0;
    ihex_count = 0;
}

static void app_ihex_write16(unsigned int data)
{
    if (ihex_count == 0)
    {
        dbg_printf("\r\n:10%02X%02X00", ihex_addr >> 8, ihex_addr & 0xFF);
        ihex_summ = 0x10 + ((ihex_addr >> 8) & 0xFF) + (ihex_addr & 0xFF);
        ihex_addr += 0x10;
    }
    dbg_printf("%02X%02X", data & 0xFF, (data >> 8) & 0xFF);
    ihex_summ += (data & 0xFF) + ((data >> 8) & 0xFF);
    ihex_count += 2;
    if (ihex_count >= 0x10)
    {
        dbg_printf("%02X", (~ihex_summ + 1) & 0xFF);
        ihex_count = 0;
    }
}

static void app_ihex_close(unsigned int code)
{
    ihex_summ = 0x02 + 0x20 + code + 0x03;
    dbg_printf("\r\n:02200000%02X03%02X"
               "\r\n:00000001FF\r\n", code, (~ihex_summ + 1) & 0xFF);
}

static void app_read_rom(unsigned int code)
{
    unsigned int addr;
    int data;

    app_ihex_open();
    app_qbus_attach();
    for(addr=0; addr<0x2000; addr+=2)
    {
        data = app_qbus_read(addr + 0x2000*code);
        if (data < 0)
        {
            dbg_printf("\r\nAcknowledge timeout at %04X", addr + 0x2000*code);
            app_qbus_detach();
            return;
        }
        app_ihex_write16(data);
    }
    app_ihex_close(code);
    app_qbus_detach();
}

static void app_debug_monitor(void)
{
    unsigned char Key;

    dbg_printf("\r\nDebug monitor started\f");
    //
    // Гарантировано активируем сигнал сброса внешней системы
    //
    IO_SET_MODE(_READY);
    IO_PORT_ODS(_READY);

    for(;;)
    {
        dbg_printf("\r\n>");
        Key = dbg_getch();
        if (Key)
        {
            dbg_putch(Key);
        }
        switch(Key)
        {
            case 27:
            {
                dbg_printf("\r\nMonitor completed");
                //
                // Восстановим конфигурации использованных портов
                //
                app_qbus_detach();
                return;
            }
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            {
                app_read_rom(Key-'0');
                break;
            }
            case '?':
            {
                dbg_printf(
                    "\r\n 0 - read ROM with chip code 0 (160000..177777)"
                    "\r\n 1 - read ROM with chip code 1 (140000..157777)"
                    "\r\n 2 - read ROM with chip code 2 (120000..137777)"
                    "\r\n 3 - read ROM with chip code 3 (100000..117777)"
                    "\r\n 4 - read ROM with chip code 4 (060000..077777)"
                    "\r\n 5 - read ROM with chip code 5 (040000..057777)"
                    "\r\n 6 - read ROM with chip code 6 (020000..037777)"
                    "\r\n 7 - read ROM with chip code 7 (000000..017777)"
                    "\r\n ? - Help message");
                break;
            }
        }
    }

}
#endif

//_____________________________________________________________________________
//
// Модуль выполнения основной задачи - фактически функция main()
//
void app_start(void)
{
    dbg_init();
    dbg_printf("\r\nApplication started\f");
    //
    // Сначала активируем сигнал сброса внешней системы
    //
    IO_SET_MODE(_READY);
    IO_PORT_ODS(_READY);

    app_init_port();
    dbg_init();
    dbg_printf("\r\nI/O ports initialized\f");
    //
    // Подождем пока выполниться надежный сброс и
    // установяться стабильные уровни на входах
    // выбора конфигурации
    //
    io_delay_ms(2);
//
//  {
//      unsigned int tmp;
//
//      tmp = *RCC_CFG;
//      tmp &= ~bRCC_MCO_MASK;
//      tmp &= ~bRCC_MCO_PRE_MASK;
//      tmp |= bRCC_MCO_PLL;
//      tmp |= bRCC_MCO_PRE4;
//      *RCC_CFG = tmp;
//      IO_SET_MODE(_MCO);
//  }
//
    //
    // Считаем значение конфигурации поля перемычек
    //
    app_read_config();
#if DBG_ENABLE
    if (app_config >= 14)
    {
        //
        // В отладочной версии поддерживаем специальный
        // режим монитора для выполнения различных тестов
        //
        app_debug_monitor();
    }
#endif
    //
    // Сформируем таблицу отображения прошивок
    // для выбранной конфигурации
    //
    app_map_image();
    //
    // Переходим на процедуру основного цикла
    //
    dbg_printf("\r\nStarting the processing loop\f");
    app_start_proc();
    //
    // Нет возврата
    //
    DBG_ASSERT(FALSE, "Invalid return from main loop");
    for(;;);
}
