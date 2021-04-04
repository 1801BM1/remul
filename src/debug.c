//
//  Project:         RE-mulator (1801PE2/PP1 emulator)
//  File:            debug.c
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

#if DBG_ENABLE
//_____________________________________________________________________________
//
// Char conversion macros
//
#define to_digit(c) ((c) - '0')
#define is_digit(c) ((unsigned)to_digit(c) <= 9)
#define to_char(n)  ((n) + '0')

//
// The internal converion flags
//
#define ALT         0x001       // alternative form output
#define HEXPREFIX   0x002       // output the 0x or 0X prefix
#define LADJUST     0x004       // left ajustment
#define LONGINT     0x008       // 32-bit wide integer
#define SHORTINT    0x010       // 16-bit wide integer
#define ZEROPAD     0x020       // padd with zeroes (not spaces)
#define BASEHEX     0x040       // radix 16
#define BASEOCT     0x080       // radix 8
#define BUF_SIZE    40          // storage fo %c, %[diouxX]

//
// Macros to extract short/long signed/unsigned arguments
// with correct sign extension
//
#define SARG()                                                              \
    (flags & LONGINT  ? va_arg(ap, long) :                                  \
     flags & SHORTINT ? (long)(short)va_arg(ap, int) :                      \
     (long)va_arg(ap, int))

#define UARG()                                                              \
    (flags & LONGINT ? va_arg(ap, unsigned long) :                          \
     flags & SHORTINT ? (unsigned long)(unsigned short)va_arg(ap, int) :    \
     (unsigned long)va_arg(ap, unsigned int))

//
// The routine outputs the buffer with specified length to the channel
//
void dbg_print_line(char *Ptr, int Len)
{
    int i;
    for(i = 0; i < Len; i++)
    {
        dbg_putch(*Ptr++);
    }
}

//
// The routine outputs the buffer with specified amount of padding symbols
//
void dbg_pad_line(int HowMany, char With)
{
    int i;
    for (i = 0; i < HowMany; i++)
    {
        dbg_putch(With);
    }
}

//_____________________________________________________________________________
//
//  vprintf imlementation, no float/doubble support
//
int dbg_vprintf (const char *fmt0, va_list ap)
{
    char    *fmt;
    int     ch;
    char    *cp;
    int     flags;
    int     ret;
    int     width;
    int     prec;
    char    sign;
    unsigned long _uquad;
    int     dprec;
    int     realsz;
    int     size;
    char    *xdigs;
    char    buf[BUF_SIZE];

    //
    // Prepare the initial pointer and the counter of chars printed
    //
    fmt = (char *)fmt0;
    ret = 0;

    //
    // Scan the line looking for the '%' symbol
    //
    for (;;)
    {
        while (*fmt)
        {
            if(*fmt == '%') break;
            dbg_putch(*fmt);
            ret ++;                     // print all symbols
            fmt ++;                     // till '%' encountered
        }
        if (*fmt == '\0') return ret;

        fmt++;                          // skip the '%' itself
                                        // prepare to convert
        flags = dprec = width = 0;
        prec = -1;
        sign = '\0';
        xdigs = "0123456789ABCDEF";

rflag:  ch = *fmt++;                    // get the format specifier
again:  switch (ch)                     // process the specifier
        {
            case ' ':
            {
                //
                //  Space specified, suppress positive sign '+' print
                //
                if (!sign) sign = ' ';
                goto rflag;
            }

            case '#':
            {
                //
                // Alternative form selected
                //
                flags |= ALT;
                goto rflag;
            }

            case '*':
            {
                //
                // Process the field width, specified by argument
                // If width is negative, then assume left adjustment
                // and invert the sign
                //
                width = va_arg(ap, int);
                if (width >= 0) goto rflag;
                width = -width;
            }

            case '-':
            {                               //
                flags |= LADJUST;           // left adjustment
                goto rflag;                 //
            }

            case '+':
            {                               //
                sign = '+';                 // Explicit sign specified
                goto rflag;                 //
            }

            case '.':
            {                               //
                ch = *fmt++;                // Minimal symbols amount
                if (ch == '*')              //
                {
                    prec = va_arg(ap, int);
                    if (prec < 0) prec = -1;
                    goto rflag;
                }
                                            //
                prec = 0;                   // Get precision
                while (is_digit(ch))        //
                {
                    prec = 10 * prec + to_digit(ch);
                    ch = *fmt++;
                }
                if (prec < 0) prec = -1;
                goto again;
            }

            case '0':
            {                               //
                flags |= ZEROPAD;           // Zeroes padding
                goto rflag;                 //
            }

            case '1': case '2': case '3':
            case '4': case '5': case '6':
            case '7': case '8': case '9':
            {
                width = 0;
                do
                {
                    width = 10 * width + to_digit(ch);
                    ch = *fmt++;
                }
                while (is_digit(ch));
                goto again;
            }

            case 'h':
            {
                flags |= SHORTINT;
                goto rflag;
            }

            case 'l':
            {
                flags |= LONGINT;
                goto rflag;
            }

            case 'c':
            {
                cp = buf;
                *cp = va_arg(ap, int);
                size = 1;
                sign = '\0';
                break;
            }

            case 'D':
            {
                flags |= LONGINT;
            }
            case 'd':
            case 'i':
            {
                _uquad = SARG();
                if ((long) _uquad < 0)
                {
                    //
                    // Invert the sign for unsigned "_uquad = -_uquad"
                    //
                    _uquad = ~_uquad + 1;
                    sign = '-';
                }
                goto number;
            }

            case 'n':
            {
                if (flags & LONGINT)
                {
                    *va_arg(ap, long *) = ret;
                }
                else if (flags & SHORTINT)
                {
                    *va_arg(ap, short *) = ret;
                }
                else
                {
                    *va_arg(ap, int *) = ret;
                }
                continue;
            }

            case 'O':
            {
                flags |= LONGINT;
            }
            case 'o':
            {
                _uquad = UARG();
                flags |= BASEOCT;
                goto nosign;
            }

            case 'p':
            {
                //
                // Argument is the void* pointer
                // Print is in hexadecimal form
                //
                _uquad = (unsigned long)(unsigned long*) va_arg(ap, void *);
                flags |= HEXPREFIX | BASEHEX;
                ch = 'x';
                goto nosign;
            }

            case 's':
            {
                cp = va_arg(ap, char *);
                if (cp == NULL) cp = "(null)";
                if (prec >= 0)
                {
                    //
                    // We cannot use the strlen() - we should find '\0'
                    // whithin the first 'prec' bytes
                    //
                    char *p = (char*)bsp_memchr(cp, 0, prec);
                    if (p != NULL)
                    {
                        size = p - cp;
                        if (size > prec) size = prec;
                    }
                    else
                        size = prec;
                }
                else
                {
                    size = bsp_strlen(cp);
                }
                sign = '\0';
                break;
            }

            case 'U':
            {
                flags |= LONGINT;
            }
            case 'u':
            {
                _uquad = UARG();
                goto nosign;
            }

            case 'x':
            {
                xdigs = "0123456789abcdef";
            }

            case 'X':
            {
                _uquad = UARG();
                flags |= BASEHEX;
                //
                // Printf prefix if needed
                //
                if (flags & ALT && _uquad != 0)
                {
                    flags |= HEXPREFIX;
                }

                //
                // Unsigned conversion
                //
nosign:         sign = '\0';
                //
                // Conversion [diouXx]
                // If precision is specified, the '0' flag is ignored
                //
number:         if ((dprec = prec) >= 0)
                {
                    flags &= ~ZEROPAD;
                }
                //
                // The result of zero value conversion with explicitly specified
                // precision does not generate the output -- ANSI X3J11
                //
                cp = buf + BUF_SIZE;
                if (_uquad != 0 || prec != 0)
                {
                    switch (flags & (BASEHEX | BASEOCT))
                    {
                        case 0:
                        {
                            while (_uquad >= 10)
                            {
                                *--cp = (char)(to_char(_uquad % 10));
                                _uquad /= 10;
                            }
                            *--cp = (char)(to_char(_uquad));
                            break;
                        }
                        case BASEOCT:
                        {
                            do {
                                *--cp = (char)(to_char(_uquad & 7));
                                _uquad >>= 3;
                            } while (_uquad);
                            //
                            // Process the leading octal '0'
                            //
                            if (flags & ALT && *cp != '0') *--cp = '0';
                            break;
                        }
                        case BASEHEX:
                        {
                            do {
                                *--cp = xdigs[_uquad & 15];
                                _uquad >>= 4;
                            } while (_uquad);
                            break;
                        }

                        default:
                        {
                            cp = "bug in vuprintf: unknown base";
                            size = bsp_strlen(cp);
                            goto skipsize;
                        }
                    }
                }
                else if ((flags & BASEOCT) && (flags & ALT)) *--cp = '0';

                size = buf + BUF_SIZE - cp;
skipsize:
                break;
            }

            default:            // "%?" prints '?', excluding NUL
            {
                if (ch == '\0')
                {
                    return ret;
                }
                //
                // Assume that it is the "%c"
                //
                cp = buf;
                *cp = ch;
                size = 1;
                sign = '\0';
                break;
            }
        }

    //
    // All acceptable formats goes here, 'cp' points to the
    // converted string, which possible should be padded,
    // prefixed or/and adjusted, depending on the flags
    //
    realsz = dprec > size ? dprec : size;
    if (sign)
       realsz++;
    else
       if (flags & HEXPREFIX) realsz+= 2;

    //
    // Left adjustment with space padding right
    //
    if ((flags & (LADJUST|ZEROPAD)) == 0)
    {
        dbg_pad_line(width - realsz, ' ');
    }
    //
    // Generate the prefix
    //
    if (sign)
    {
        dbg_putch(sign);
    }
    else
       if (flags & HEXPREFIX)
       {
          dbg_putch('0');
          dbg_putch((char)ch);
       }

    //
    // Right adjusting with leading zeroes
    //
    if ((flags & (LADJUST|ZEROPAD)) == ZEROPAD)
    {
        dbg_pad_line(width - realsz, '0');
    }

        //
        // Leading zeroes for decimal point
        //
        dbg_pad_line(dprec - size, '0');

        //
        // The line or number itself
        //
        dbg_print_line(cp, size);

        //
        // Left adjustment with left space padding
        //
        if (flags & LADJUST)
        {
            dbg_pad_line(width - realsz, ' ');
        }

        //
        // Update the amount of chars printed
        //
        ret += width > realsz ? width : realsz;
    }
}

int dbg_printf(const char *fmt, ...)
{
    int ret;
    va_list ap;

    va_start (ap, fmt);
    ret = dbg_vprintf (fmt, ap);
    va_end (ap);
    return ret;
}

void
dbg_assert(
    unsigned long Line,
    const char* File,
    ...)
{
    va_list ap;
    char* fmt;

    __disable_interrupt();
    dbg_init();
    dbg_printf("\r\nDEBUG assertion failure at line %d in \"%s\"\r\n\f", Line, File);

    va_start (ap, File);
    fmt = va_arg(ap, char*);
    dbg_vprintf(fmt, ap);
    va_end (ap);
    dbg_flush();
    for(;;);
}

//_____________________________________________________________________________
//
unsigned char dbg_inkey(void)
{
    unsigned char key;

    key = dbg_read();
    if (key == 0) return 0;

    if ((key >= 'a') && (key <= 'z'))
    {
        key -= ('a' - 'A');
    }
    return key;
}

unsigned char dbg_getch(void)
{
    char key;

    for(;;)
    {
        key = dbg_inkey();
        if (key != 0)
        {
            return key;
        }
    }
}

void dbg_putch(char sym)
{
    if (sym == '\f')
    {
        dbg_flush();
    }
    else
    {
        dbg_write(sym);
    }
}

//_____________________________________________________________________________
//
// Reading the one byte from Debug Communication Channel (UART)
//
unsigned char dbg_read(void)
{
    PSTM_UART puart = DBG_UART_BASE;

    if (puart->sUART_SR & bUART_RXNE)
    {
        return puart->sUART_DR & 0xFF;
    }
    return 0;
}

//_____________________________________________________________________________
//
// Writing the byte to the Debug Communication Channel (UART)
//
void dbg_write(unsigned char data)
{
    PSTM_UART puart = DBG_UART_BASE;

    for(;;)
    {
        if (puart->sUART_SR & bUART_TXE)
        {
            puart->sUART_DR = data;
            return;
        }
    }
}

//_____________________________________________________________________________
//
void dbg_init(void)
{
    PSTM_UART puart = DBG_UART_BASE;

    if (DBG_UART_BASE == UART1_BASE)
    {
        *RCC_APB2EN |= bRCC_UART1EN;
        *RCC_APB2LP |= bRCC_UART1EN;
    }
    if (DBG_UART_BASE == UART2_BASE)
    {
        *RCC_APB1EN |= bRCC_UART2EN;
        *RCC_APB1LP |= bRCC_UART2EN;
    }
    if (DBG_UART_BASE == UART3_BASE)
    {
        *RCC_APB1EN |= bRCC_UART3EN;
        *RCC_APB1LP |= bRCC_UART3EN;
    }
    if (DBG_UART_BASE == UART4_BASE)
    {
        *RCC_APB1EN |= bRCC_UART4EN;
        *RCC_APB1LP |= bRCC_UART4EN;
    }
    if (DBG_UART_BASE == UART5_BASE)
    {
        *RCC_APB1EN |= bRCC_UART5EN;
        *RCC_APB1LP |= bRCC_UART5EN;
    }
    if (DBG_UART_BASE == UART6_BASE)
    {
        *RCC_APB2EN |= bRCC_UART6EN;
        *RCC_APB2LP |= bRCC_UART6EN;
    }
    //
    // Setup the controllers pins
    // Clocks for UARTs and GPIOs should be enabled
    //
    IO_SET_MODE(_DBG_UART_RXD);
    IO_SET_MODE(_DBG_UART_TXD);

    puart->sUART_CR1    = 0;
    puart->sUART_BRR    = (BSP_APB_CLOCK + DBG_UART_BAUD/2) / DBG_UART_BAUD;
    puart->sUART_CR3    = 0;
    puart->sUART_CR2    = bUART_STOP_1;
    puart->sUART_GTPR   = 0;

    puart->sUART_CR1 = bUART_UE
                     | bUART_TE
                     | bUART_RE;
    puart->sUART_DR = 0x20;
}

//_____________________________________________________________________________
//
void dbg_flush(void)
{
    PSTM_UART puart = DBG_UART_BASE;

    while((puart->sUART_SR & bUART_TXE) == 0);
    while((puart->sUART_SR & bUART_TC) == 0);
}
#endif  // DBG_ENABLE
