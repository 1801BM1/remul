//
//  Project:         RE-mulator (1801PE2/PP1 emulator)
//  File:            ioport.h
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
#ifndef _IOPORT_H_
#define _IOPORT_H_

#define IO_MASK_N(a)        ((a) & 0x0F)
#define IO_PORT_N(a)        (((a) >> 8)&0x07)
#define IO_FUNC_N(a)        (((a) >> 16)&0x0F)

#define IO_PORT(a)          ((PSTM_PIO) ((IO_PORT_N(a)*0x400) + (unsigned long)(GPIO_A_BASE)))
#define IO_MASK(a)          (1u<<IO_MASK_N(a))
#define IO_FUNC(a)          ((a) & (0x1F<<16))
#define IO_MASK2(a, b)      ((b)<<(IO_MASK_N(a)*2))
#define IO_MASK4L(a, b)     ((b)<<(IO_MASK_N(a)*4))
#define IO_MASK4H(a, b)     ((b)<<(IO_MASK_N(a)*4-32))

#define IO_PORT_A           (0<<8)
#define IO_PORT_B           (1<<8)
#define IO_PORT_C           (2<<8)
#define IO_PORT_D           (3<<8)
#define IO_PORT_E           (4<<8)
#define IO_PORT_F           (5<<8)
#define IO_PORT_G           (6<<8)
#define IO_PORT_H           (7<<8)
#define IO_PORT_I           (8<<8)
#define IO_PORT_MSK         (15<<8)

#define IO_FUNC_AF0         ((bPIO_AF0<<16) | IO_FUNC_AF)
#define IO_FUNC_AF1         ((bPIO_AF1<<16) | IO_FUNC_AF)
#define IO_FUNC_AF2         ((bPIO_AF2<<16) | IO_FUNC_AF)
#define IO_FUNC_AF3         ((bPIO_AF3<<16) | IO_FUNC_AF)
#define IO_FUNC_AF4         ((bPIO_AF4<<16) | IO_FUNC_AF)
#define IO_FUNC_AF5         ((bPIO_AF5<<16) | IO_FUNC_AF)
#define IO_FUNC_AF6         ((bPIO_AF6<<16) | IO_FUNC_AF)
#define IO_FUNC_AF7         ((bPIO_AF7<<16) | IO_FUNC_AF)
#define IO_FUNC_AF8         ((bPIO_AF8<<16) | IO_FUNC_AF)
#define IO_FUNC_AF9         ((bPIO_AF9<<16) | IO_FUNC_AF)
#define IO_FUNC_AF10        ((bPIO_AF10<<16) | IO_FUNC_AF)
#define IO_FUNC_AF11        ((bPIO_AF11<<16) | IO_FUNC_AF)
#define IO_FUNC_AF12        ((bPIO_AF12<<16) | IO_FUNC_AF)
#define IO_FUNC_AF13        ((bPIO_AF13<<16) | IO_FUNC_AF)
#define IO_FUNC_AF14        ((bPIO_AF14<<16) | IO_FUNC_AF)
#define IO_FUNC_AF15        ((bPIO_AF15<<16) | IO_FUNC_AF)

#define IO_FUNC_IO          (0<<16)
#define IO_FUNC_AN          (1<<16)
#define IO_FUNC_AF          (0x10<<16)
#define IO_FUNC_MSK         (0x1F<<16)

#define IO_MODE_INOUT_MSK   (3<<23)
#define IO_MODE_INPUT       (1<<23)
#define IO_MODE_OUTPUT      (2<<23)
#define IO_MODE_OPEN_DRAIN  (3<<23)

#define IO_MODE_PULL_MSK    (3<<25)
#define IO_MODE_PULL_NONE   (1<<25)
#define IO_MODE_PULL_UP     (2<<25)
#define IO_MODE_PULL_DOWN   (3<<25)

#define IO_ATTR_MSK         (3<<27)
#define IO_ATTR_2MHZ        (0<<27)
#define IO_ATTR_25MHZ       (1<<27)
#define IO_ATTR_50MHZ       (2<<27)
#define IO_ATTR_100MHZ      (3<<27)

#define IO_PORT_IN(a)       ((IO_PORT(a)->sPIO_IDR) & IO_MASK(a))
#define IO_PORT_SET(a)      (IO_PORT(a)->sPIO_BSR = IO_MASK(a))
#define IO_PORT_CLR(a)      (IO_PORT(a)->sPIO_BSR = (IO_MASK(a)<<16))
#define IO_PORT_ODS(a)      (IO_PORT(a)->sPIO_BSR = IO_MASK(a))
#define IO_PORT_ODC(a)      (IO_PORT(a)->sPIO_BSR = (IO_MASK(a)<<16))

#define IO_PORT_REG2(a, reg, clr, set)          \
{                                               \
    unsigned long data;                         \
                                                \
    data  =  IO_PORT((a))->reg;                 \
    data &= ~IO_MASK2((a), clr);                \
    data |=  IO_MASK2((a), set);                \
    IO_PORT((a))->reg = data;                   \
}

#define IO_PORT_AFR4(a)                         \
{                                               \
    unsigned long data;                         \
                                                \
    if (IO_MASK_N(a) < 8)                       \
    {                                           \
        data  =  IO_PORT((a))->sPIO_AFL;        \
        data &= ~IO_MASK4L((a), bPIO_FUNC_MASK);\
        data |=  IO_MASK4L((a), IO_FUNC_N((a)));\
        IO_PORT((a))->sPIO_AFL = data;          \
    }                                           \
    else                                        \
    {                                           \
        data  =  IO_PORT((a))->sPIO_AFH;        \
        data &= ~IO_MASK4H((a), bPIO_FUNC_MASK);\
        data |=  IO_MASK4H((a), IO_FUNC_N((a)));\
        IO_PORT((a))->sPIO_AFH = data;          \
    }                                           \
}

INLINE_FORCED
void
IO_PORT_PUPD(unsigned long a)
{
    if ((a & IO_MODE_PULL_MSK) == IO_MODE_PULL_UP)
    {
        IO_PORT_REG2(a, sPIO_PUPD, bPIO_PULL_MASK, bPIO_PULL_UP);
    }
    if ((a & IO_MODE_PULL_MSK) == IO_MODE_PULL_DOWN)
    {
        IO_PORT_REG2(a, sPIO_PUPD, bPIO_PULL_MASK, bPIO_PULL_DOWN);
    }
    if ((a & IO_MODE_PULL_MSK) == IO_MODE_PULL_NONE)
    {
        IO_PORT_REG2(a, sPIO_PUPD, bPIO_PULL_MASK, bPIO_PULL_NONE);
    }
}

INLINE_FORCED
void
IO_PORT_ATTR(unsigned long a)
{
    switch(a & IO_ATTR_MSK)
    {
        case IO_ATTR_2MHZ:
        {
            IO_PORT_REG2(a, sPIO_SPEED, bPIO_OUTPUT_MASK, bPIO_OUTPUT_2MHZ);
            break;
        }
        case IO_ATTR_25MHZ:
        {
            IO_PORT_REG2(a, sPIO_SPEED, bPIO_OUTPUT_MASK, bPIO_OUTPUT_25MHZ);
            break;
        }
        case IO_ATTR_50MHZ:
        {
            IO_PORT_REG2(a, sPIO_SPEED, bPIO_OUTPUT_MASK, bPIO_OUTPUT_50MHZ);
            break;
        }
        case IO_ATTR_100MHZ:
        {
            IO_PORT_REG2(a, sPIO_SPEED, bPIO_OUTPUT_MASK, bPIO_OUTPUT_100MHZ);
            break;
        }
    }
}

INLINE_FORCED
void
IO_SET_MODE(unsigned long a)
{
    switch(IO_FUNC(a))
    {
        case IO_FUNC_IO:
        {
            if ((a & IO_MODE_INOUT_MSK) == IO_MODE_INPUT)
            {
                IO_PORT_REG2(a, sPIO_MODE, bPIO_MODE_MASK, bPIO_MODE_INPUT);
                IO_PORT_PUPD(a);
                break;
            }
            if ((a & IO_MODE_INOUT_MSK) == IO_MODE_OUTPUT)
            {
                IO_PORT(a)->sPIO_TYPE &= ~IO_MASK(a);
                IO_PORT_REG2(a, sPIO_MODE, bPIO_MODE_MASK, bPIO_MODE_OUTPUT);
                IO_PORT_REG2(a, sPIO_PUPD, bPIO_PULL_MASK, bPIO_PULL_NONE);
                IO_PORT_ATTR(a);
                break;
            }
            if ((a & IO_MODE_INOUT_MSK) == IO_MODE_OPEN_DRAIN)
            {
                IO_PORT(a)->sPIO_TYPE |= IO_MASK(a);
                IO_PORT_REG2(a, sPIO_MODE, bPIO_MODE_MASK, bPIO_MODE_OUTPUT);
                IO_PORT_REG2(a, sPIO_PUPD, bPIO_PULL_MASK, bPIO_PULL_NONE);
                IO_PORT_ATTR(a);
                break;
            }
            break;
        }
        case IO_FUNC_AN:
        {
            IO_PORT_REG2(a, sPIO_MODE, bPIO_MODE_MASK, bPIO_MODE_ANALOG);
            IO_PORT_REG2(a, sPIO_PUPD, bPIO_PULL_MASK, bPIO_PULL_NONE);
            break;
        }
        case IO_FUNC_AF0:
        case IO_FUNC_AF1:
        case IO_FUNC_AF2:
        case IO_FUNC_AF3:
        case IO_FUNC_AF4:
        case IO_FUNC_AF5:
        case IO_FUNC_AF6:
        case IO_FUNC_AF7:
        case IO_FUNC_AF8:
        case IO_FUNC_AF9:
        case IO_FUNC_AF10:
        case IO_FUNC_AF11:
        case IO_FUNC_AF12:
        case IO_FUNC_AF13:
        case IO_FUNC_AF14:
        case IO_FUNC_AF15:
        {
            IO_PORT_AFR4(a);
            IO_PORT_REG2(a, sPIO_MODE, bPIO_MODE_MASK, bPIO_MODE_AFUNC);
            IO_PORT_PUPD(a);
            IO_PORT_ATTR(a);

            if ((a & IO_MODE_INOUT_MSK) == IO_MODE_OUTPUT)
            {
                IO_PORT(a)->sPIO_TYPE &= ~IO_MASK(a);
                break;
            }
            if ((a & IO_MODE_INOUT_MSK) == IO_MODE_OPEN_DRAIN)
            {
                IO_PORT(a)->sPIO_TYPE |= IO_MASK(a);
                break;
            }
            break;
        }
        default:
        {
            DBG_ASSERT(FALSE, "Unable to IO_SET_MODE(%08X)", a);
        }
    }
}
#endif // _IO_PORT_H_

