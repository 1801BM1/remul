//________________________________________________________________
//
#ifndef __STM32F2xx_H__
#define __STM32F2xx_H__

#define IO_REG32(_a_)       ((volatile unsigned long*)(_a_))
#define IO_REG16(_a_)       ((volatile unsigned short*)(_a_))
#define IO_REG8(_a_)        ((volatile unsigned char*)(_a_))
typedef volatile unsigned long STM_REG;
typedef volatile unsigned short STM_REG16;

//________________________________________________________________
//
// Табличка процесоров STM32F2xx
//
//          Medium-Density  High/XL-Density
//          xB      xC      xE      xF      xG
//  F205    64/128  96/256  128/512 128/768 128/1024
//  F207    -       96/256  128/512 128/768 128/1024
//  F215    -       -       128/512 -       128/1024
//  F217    -       -       128/512 -       128/1024
//
//________________________________________________________________
//
// Условные идентификаторы микроконтроллеров STM32F2xx
//
#define STM32F205xB_ID      0x205B
#define STM32F205xC_ID      0x205C
#define STM32F205xE_ID      0x205E
#define STM32F205xF_ID      0x205F
#define STM32F205xG_ID      0x2051

#define STM32F207xC_ID      0x207C
#define STM32F207xE_ID      0x207E
#define STM32F207xF_ID      0x207F
#define STM32F207xG_ID      0x2071

#define STM32F215xE_ID      0x215E
#define STM32F215xG_ID      0x2151
#define STM32F217xE_ID      0x217E
#define STM32F217xG_ID      0x2171

//________________________________________________________________
//
// Peripheral blocks base addresses (not defined yet)
//
#define FSMC_BASE       ((PSTM_FSMC)    0xA0000000)
#define RNG_BASE        ((PSTM_RNG)     0x50060800)
#define HASH_BASE       ((PSTM_HASH)    0x50060400)
#define CRYP_BASE       ((PSTM_HASH)    0x50060000)
#define DCMI_BASE       ((PSTM_HASH)    0x50050000)

#define CRC_BASE        ((PSTM_CRC)     0x40023000)
#define DAC_BASE        ((PSTM_DAC)     0x40007400)
#define CAN2_BASE       ((PSTM_CAN)     0x40006800)
#define CAN1_BASE       ((PSTM_CAN)     0x40006400)

#define I2C3_BASE       ((PSTM_I2C)     0x40005C00)
#define I2C2_BASE       ((PSTM_I2C)     0x40005800)
#define I2C1_BASE       ((PSTM_I2C)     0x40005400)

//________________________________________________________________
//
//  NVIC - Nested Vectored Interrupt Controller
//
#define NVIC_IECR0      IO_REG32(0xE000E100)    // Interrupt Set Enable
#define NVIC_IECR1      IO_REG32(0xE000E104)    //
#define NVIC_IECR2      IO_REG32(0xE000E108)    //
#define NVIC_IDCR0      IO_REG32(0xE000E180)    // Interrupt Clear Enable
#define NVIC_IDCR1      IO_REG32(0xE000E184)    //
#define NVIC_IDCR2      IO_REG32(0xE000E188)    //
#define NVIC_ISPR0      IO_REG32(0xE000E200)    // Interrupt Set Pending
#define NVIC_ISPR1      IO_REG32(0xE000E204)    //
#define NVIC_ISPR2      IO_REG32(0xE000E208)    //
#define NVIC_ICPR0      IO_REG32(0xE000E280)    // Interrupt Clear Pending
#define NVIC_ICPR1      IO_REG32(0xE000E284)    //
#define NVIC_ICPR2      IO_REG32(0xE000E288)    //
#define NVIC_IABR0      IO_REG32(0xE000E300)    // Interrupt Active Bit
#define NVIC_IABR1      IO_REG32(0xE000E304)    //
#define NVIC_IABR2      IO_REG32(0xE000E308)    //
                                                //
#define NVIC_IPR0       IO_REG32(0xE000E400)    // Interrupt Priority
#define NVIC_IPR1       IO_REG32(0xE000E404)    //
#define NVIC_IPR2       IO_REG32(0xE000E408)    //
#define NVIC_IPR3       IO_REG32(0xE000E40C)    //
#define NVIC_IPR4       IO_REG32(0xE000E410)    //
#define NVIC_IPR5       IO_REG32(0xE000E414)    //
#define NVIC_IPR6       IO_REG32(0xE000E418)    //
#define NVIC_IPR7       IO_REG32(0xE000E41C)    //
#define NVIC_IPR8       IO_REG32(0xE000E420)    //
#define NVIC_IPR9       IO_REG32(0xE000E424)    //
#define NVIC_IPR10      IO_REG32(0xE000E428)    //
#define NVIC_IPR11      IO_REG32(0xE000E42C)    //
#define NVIC_IPR12      IO_REG32(0xE000E430)    //
#define NVIC_IPR13      IO_REG32(0xE000E434)    //
#define NVIC_IPR14      IO_REG32(0xE000E438)    //
#define NVIC_IPR15      IO_REG32(0xE000E43C)    //
#define NVIC_IPR16      IO_REG32(0xE000E440)    //
#define NVIC_IPR17      IO_REG32(0xE000E444)    //
#define NVIC_IPR18      IO_REG32(0xE000E448)    //
#define NVIC_IPR19      IO_REG32(0xE000E44C)    //
#define NVIC_IPR20      IO_REG32(0xE000E450)    //
                                                //
#pragma pack(push, 4)                           // interrupt context
typedef struct _STM_CONTEXT                     //
{                                               //
    unsigned long   user_R0;                    //
    unsigned long   user_R1;                    //
    unsigned long   user_R2;                    //
    unsigned long   user_R3;                    //
    unsigned long   user_R12;                   //
    unsigned long   user_LR;                    //
    unsigned long   user_PC;                    //
    unsigned long   user_PSR;                   //
                                                //
} STM_CONTEXT, *PSTM_CONTEXT;                   //
#pragma pack(pop)                               //
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_NVIC               //
{                                               //
    STM_REG     sNVIC_ISER[3];                  // Interrupt Set Enable
    STM_REG     sReserved1[29];                 //
    STM_REG     sNVIC_ICER[3];                  // Interrupt Clear Enable
    STM_REG     sReserved2[29];                 //
    STM_REG     sNVIC_ISPR[3];                  // Interrupt Set Pending
    STM_REG     sReserved3[29];                 //
    STM_REG     sNVIC_ICPR[3];                  // Interrupt Clear Pending
    STM_REG     sReserved4[29];                 //
    STM_REG     sNVIC_IABR[3];                  // Interrupt Active Bit
    STM_REG     sReserved5[61];                 //
    STM_REG     sNVIC_IPR[21];                  // Interrupt Priority
    STM_REG     sReserved6[683];                //
    STM_REG     sNVIC_STIR;                     // Software Trigger Interrupt

} STM_NVIC, *PSTM_NVIC;
#pragma pack(pop)

#define NVIC_BASE    ((PSTM_NVIC) 0xE000E100)

//________________________________________________________________
//
#define bNVIC_HIGHEST_PRIORITY  (0<<3)          // наивысший приоритет
#define bNVIC_LOWEST_PRIORITY   (31u<<3)        // наинизший приоритет
                                                //
//________________________________________________________________
//
// Номера векторов аппаратных прерываний
//
#define IRQ_WWDT                0               // Windows Watchdog Timer
#define IRQ_PVD                 1               // Programmable Voltage Detector
#define IRQ_TAMPER              2               // Tamper (EXTI)
#define IRQ_RTC_WKUP            3               // RTC Wakeup (EXTI)
#define IRQ_FLASH               4               // Flash Controller
#define IRQ_RCC                 5               // RCC global
#define IRQ_EXTI0               6               // EXTI Line0
#define IRQ_EXTI1               7               // EXTI Line1
#define IRQ_EXTI2               8               // EXTI Line2
#define IRQ_EXTI3               9               // EXTI Line3
#define IRQ_EXTI4               10              // EXTI Line4
#define IRQ_DMA1_CH0            11              // DMA1 Stream0
#define IRQ_DMA1_CH1            12              // DMA1 Stream1
#define IRQ_DMA1_CH2            13              // DMA1 Stream2
#define IRQ_DMA1_CH3            14              // DMA1 Stream3
#define IRQ_DMA1_CH4            15              // DMA1 Stream4
#define IRQ_DMA1_CH5            16              // DMA1 Stream5
#define IRQ_DMA1_CH6            17              // DMA1 Stream6
#define IRQ_ADC123              18              // ADC 1 & 2
#define IRQ_CAN1_TX             19              // CAN1 TX
#define IRQ_CAN1_RX0            20              // CAN1 RX0
#define IRQ_CAN1_RX1            21              // CAN1 RX1
#define IRQ_CAN1_SCE            22              // CAN1 SCE
#define IRQ_EXTI59              23              // EXTI Line 5-9
                                                //
#define IRQ_TIM1_BRK            24              // TIM1 Break
#define IRQ_TIM9                24              // TIM9
#define IRQ_TIM1_UP             25              // TIM1 Update
#define IRQ_TIM10               25              // TIM16
#define IRQ_TIM1_TRG_COM        26              // TIM1 Trigger and Commutation
#define IRQ_TIM17               26              // TIM17
#define IRQ_TIM11               26              // TIM11
                                                //
#define IRQ_TIM1                27              // TIM1 Capture Compare
#define IRQ_TIM2                28              // TIM2
#define IRQ_TIM3                29              // TIM3
#define IRQ_TIM4                30              // TIM4
#define IRQ_I2C1_EV             31              // I2C1 event
#define IRQ_I2C1_ER             32              // I2C1 error
#define IRQ_I2C2_EV             33              // I2C2 event
#define IRQ_I2C2_ER             34              // I2C2 error
#define IRQ_SPI1                35              // SPI1
#define IRQ_SPI2                36              // SPI2
#define IRQ_UART1               37              // USART1
#define IRQ_UART2               38              // USART2
#define IRQ_UART3               39              // USART3
#define IRQ_EXTI1015            40              // EXTI Line 10-15
#define IRQ_RTC_ALARM           41              // Alarm RTC
                                                //
#define IRQ_OTG_FS_WKUP         42              // USB On-The-Go FS Wakeup
                                                //
#define IRQ_TIM8_BRK            43              // TIM8 Break
#define IRQ_TIM12               43              // TIM12
#define IRQ_TIM8_UP             44              // TIM8 Update
#define IRQ_TIM13               44              // TIM13
#define IRQ_TIM8_TRG_COM        45              // TIM8 Trigger and Commutation
#define IRQ_TIM14               45              // TIM14
#define IRQ_TIM8                46              // TIM8 Capture Compare
#define IRQ_DMA1_CH7            47              // DMA1 Stream7
                                                //
#define IRQ_FSMC                48              // FSMC
#define IRQ_SDIO                49              // SDIO
#define IRQ_TIM5                50              // TIM5
#define IRQ_SPI3                51              // SPI3
#define IRQ_UART4               52              // UART4
#define IRQ_UART5               53              // UART5
#define IRQ_TIM6                54              // TIM6
#define IRQ_DAC                 54              // DAC
#define IRQ_TIM7                55              // TIM7
                                                //
#define IRQ_DMA2_CH0            56              // DMA2 Stream0
#define IRQ_DMA2_CH1            57              // DMA2 Stream1
#define IRQ_DMA2_CH2            58              // DMA2 Stream2
#define IRQ_DMA2_CH3            59              // DMA2 Stream3
#define IRQ_DMA2_CH4            60              // DMA2 Stream4
                                                //
#define IRQ_EMAC                61              // Ethernet MAC
#define IRQ_EMAC_WKUP           62              // Ethermat wakeup
#define IRQ_CAN2_TX             63              // CAN2 TX
#define IRQ_CAN2_RX0            64              // CAN2 RX0
#define IRQ_CAN2_RX1            65              // CAN2 RX1
#define IRQ_CAN2_SCE            66              // CAN2 SCE
#define IRQ_OTG_FS              67              // USB On The Go FS
                                                //
#define IRQ_DMA2_CH5            68              // DMA2 Stream5
#define IRQ_DMA2_CH6            69              // DMA2 Stream6
#define IRQ_DMA2_CH7            70              // DMA2 Stream7
#define IRQ_UART6               71              // UART6
#define IRQ_I2C3_EV             72              // I2C3 event
#define IRQ_I2C3_ER             73              // I2C3 error
#define IRQ_OTG_HS_EP1_OUT      74              // USB On The Go HS EP1 Out
#define IRQ_OTG_HS_EP1_IN       75              // USB On The Go HS EP1 In
#define IRQ_OTG_HS_WKUP         76              // USB On The Go HS Wakeup through EXTI
#define IRQ_OTG_HS              77              // USB On The Go HS
#define IRQ_DCMI                78              // DCMI
#define IRQ_CRYP                79              // crypto
#define IRQ_HASH_RNG            80              // Hash and Rng
#define IRQ_MAX                 81              //

//________________________________________________________________
//
// System Control Block, Cortex M3, по определению ARMv7
// данные регистры все относятся к NVIC
//
#define SC_ICTR         IO_REG32(0xE000E004)    // Interrupt Control Type Register (ro)
#define SC_ACTLR        IO_REG32(0xE000E008)    // Auxiliary Control
                                                //
#define SC_STCTRL       IO_REG32(0xE000E010)    // System Tick Control
#define SC_STLOAD       IO_REG32(0xE000E014)    // System Tick Reload Value
#define SC_STVAL        IO_REG32(0xE000E018)    // System Tick Current Value
#define SC_STCALIB      IO_REG32(0xE000E01C)    // System Tick Calibration
                                                //
#define SC_CPUID        IO_REG32(0xE000ED00)    // CPUID Base
#define SC_ICSR         IO_REG32(0xE000ED04)    // Interrupt Control and State
#define SC_VTOR         IO_REG32(0xE000ED08)    // Vector Table Offset
#define SC_AIRCR        IO_REG32(0xE000ED0C)    // Application Interrupt and Reset Control
#define SC_SCR          IO_REG32(0xE000ED10)    // System Control
#define SC_CCR          IO_REG32(0xE000ED14)    // Configuration and Control
#define SC_SHPR1        IO_REG32(0xE000ED18)    // System Handler Priority
#define SC_SHPR2        IO_REG32(0xE000ED1C)    //
#define SC_SHPR3        IO_REG32(0xE000ED20)    //
#define SC_SHCSR        IO_REG32(0xE000ED24)    // System Handler Control and State
#define SC_CFSR         IO_REG32(0xE000ED28)    // Configurable Fault Status
#define SC_MMSR         IO_REG8 (0xE000ED28)    // Memory Management Fault Status
#define SC_BFSR         IO_REG8 (0xE000ED29)    // Bus Fault Status
#define SC_UFSR         IO_REG16(0xE000ED2A)    // Usage Fault Status
#define SC_HFSR         IO_REG32(0xE000ED2C)    // Hard Fault Status
#define SC_MMFAR        IO_REG32(0xE000ED34)    // Memory Management Fault Address
#define SC_BFAR         IO_REG32(0xE000ED38)    // Bus Fault Address
#define SC_STIR         IO_REG32(0xE000EF00)    // Software Trigger Interrupt
                                                //
#define MPU_TYPE        IO_REG32(0xE000ED90)    // MPU Type identificator
#define MPU_CTRL        IO_REG32(0xE000ED94)    // MPU Control
#define MPU_RNR         IO_REG32(0xE000ED98)    // MPU Region Number Register
#define MPU_RBAR        IO_REG32(0xE000ED9C)    // MPU Region Base Address Register
#define MPU_RASR        IO_REG32(0xE000EDA0)    // MPU Region Attribute and Size Register
#define MPU_RBAR_A1     IO_REG32(0xE000EDA4)    // Alias of RBAR
#define MPU_RASR_A1     IO_REG32(0xE000EDA8)    // Alias of RASR
#define MPU_RBAR_A2     IO_REG32(0xE000EDAC)    // Alias of RBAR
#define MPU_RASR_A2     IO_REG32(0xE000EDB0)    // Alias of RASR
#define MPU_RBAR_A3     IO_REG32(0xE000EDB4)    // Alias of RBAR
#define MPU_RASR_A3     IO_REG32(0xE000EDB8)    // Alias of RASR
//
// Memory Protection Unit
//
#define bMPU_IREGION_MASK       0xFFul          // MPU_TYPE
#define bMPU_IREGION_SHIFT      16              //
#define bMPU_DREGION_MASK       0xFFul          //
#define bMPU_DREGION_SHIFT      8               //
#define bMPU_SEPARATE           (1<<0)          //
                                                //
#define bMPU_PRIVDEFENA         (1<<2)          // privileged default enable
#define bMPU_HFMIENA            (1<<1)          // MPU enabled in fault mode
#define bMPU_ENABLE             (1<<0)          // MPU enabled
                                                //
#define bMPU_REGION_MAX         8               //
#define bMPU_REGION_MASK        0x0Ful          //
#define bMPU_REGION_SHIFT       0               //
#define bMPU_REGION_VALID       (1<<4)          //
                                                //
#define bMPU_XN                 (1<<28)         //
#define bMPU_AP_SHIFT           24              //
#define bMPU_AP_MASK            7               //

#define bMPU_AP_SNN_UNN         (0<<bMPU_AP_SHIFT)
#define bMPU_AP_SRW_UNN         (1<<bMPU_AP_SHIFT)
#define bMPU_AP_SRW_URO         (2<<bMPU_AP_SHIFT)
#define bMPU_AP_SRW_URW         (3<<bMPU_AP_SHIFT)
#define bMPU_AP_SRO_UNN         (5<<bMPU_AP_SHIFT)
#define bMPU_AP_SRO_URO         (6<<bMPU_AP_SHIFT)

#define bMPU_TEX_SHIFT          19              //
#define bMPU_TEX_MASK           7               //
#define bMPU_S                  (1<<18)         //
#define bMPU_C                  (1<<17)         //
#define bMPU_B                  (1<<16)         //
                                                //
#define bMPU_SRD_SHIFT          8               //
#define bMPU_SRD_MASK           0xFFul          //

#define bMPU_SRD_0              (1<<(bMPU_SRD_SHIFT+0))
#define bMPU_SRD_1              (1<<(bMPU_SRD_SHIFT+1))
#define bMPU_SRD_2              (1<<(bMPU_SRD_SHIFT+2))
#define bMPU_SRD_3              (1<<(bMPU_SRD_SHIFT+3))
#define bMPU_SRD_4              (1<<(bMPU_SRD_SHIFT+4))
#define bMPU_SRD_5              (1<<(bMPU_SRD_SHIFT+5))
#define bMPU_SRD_6              (1<<(bMPU_SRD_SHIFT+6))
#define bMPU_SRD_7              (1<<(bMPU_SRD_SHIFT+7))

#define bMPU_SIZE_32            (4<<1)          //
#define bMPU_SIZE_64            (5<<1)          //
#define bMPU_SIZE_128           (6<<1)          //
#define bMPU_SIZE_256           (7<<1)          //
#define bMPU_SIZE_512           (8<<1)          //
#define bMPU_SIZE_1K            (9<<1)          //
#define bMPU_SIZE_2K            (10<<1)         //
#define bMPU_SIZE_4K            (11<<1)         //
#define bMPU_SIZE_8K            (12<<1)         //
#define bMPU_SIZE_16K           (13<<1)         //
#define bMPU_SIZE_32K           (14<<1)         //
#define bMPU_SIZE_64K           (15<<1)         //
#define bMPU_SIZE_128K          (16<<1)         //
#define bMPU_SIZE_256K          (17<<1)         //
#define bMPU_SIZE_512K          (18<<1)         //
#define bMPU_SIZE_1M            (19<<1)         //
#define bMPU_SIZE_2M            (20<<1)         //
#define bMPU_SIZE_4M            (21<<1)         //
#define bMPU_SIZE_8M            (22<<1)         //
#define bMPU_SIZE_16M           (23<<1)         //
#define bMPU_SIZE_32M           (24<<1)         //
#define bMPU_SIZE_64M           (25<<1)         //
#define bMPU_SIZE_128M          (26<<1)         //
#define bMPU_SIZE_256M          (27<<1)         //
#define bMPU_SIZE_512M          (28<<1)         //
#define bMPU_SIZE_1G            (29<<1)         //
#define bMPU_SIZE_2G            (30<<1)         //
#define bMPU_SIZE_4G            (31<<1)         //

//
// System Tick registers
//
#define bSTCTRL_COUNTFLAG       (1<<16)         // timer counted to 0 since last read
#define bSTCTRL_CLKSOURCE       (1<<2)          // use core clock as source
#define bSTCTRL_TICKINT         (1<<1)          // interrupt generate
#define bSTCTRL_ENABLE          (1<<0)          //
                                                //
//
// SC_ACTLR, SC_ICSR, SC_VTOR, SC_AIRCR, SC_SCR, SC_CCR
//
#define bACTLR_DISFOLD          (1<<2)          // disable IT folding
#define bACTLR_DISDEFWBUF       (1<<1)          // disable write buffer
#define bACTLR_DISMCYCINT       (1<<0)          // disables interruption of ldm/stm
                                                //
#define bICSR_NMIPENDSET        (1u<<31)        // NMI set-pending bit
#define bICSR_PENDSVSET         (1<<28)         // PendSV set-pending bit
#define bICSR_PENDSVCLR         (1<<27)         // PendSV clear-pending bit
#define bICSR_PENDSTSET         (1<<26)         // SysTick exception set-pending bit
#define bICSR_PENDSTCLR         (1<<25)         // SysTick exception clear-pending bit
#define bICSR_ISRPREEMPT        (1<<23)         //
#define bICSR_ISRPENDING        (1<<22)         // Interrupt pending flag, excluding NMI and Faults
#define bICSR_VECTPENDING_MASK  0x1FF           //
#define bICSR_VECTPENDING_SHIFT 12              //
#define bICSR_RETTOBASE         (1<<11)         // There are preempted active exceptions
#define bICSR_VECTACTIVE_MASK   0x1FF           //
#define bICSR_VECTACTIVE_SHIFT  0               //
                                                //
#define bVTOR_TBLOFF            0x3FFFFF00      //
                                                //
#define bAIRCR_VECTKEY          0x05FA0000      //
#define bAIRCR_ENDIANESS        (1<<15)         // Data endianness bit
#define bAIRCR_PRIGROUP_32      (2<<8)          // 32 groups
#define bAIRCR_PRIGROUP_16      (3<<8)          // 16 groups
#define bAIRCR_PRIGROUP_8       (4<<8)          // 8 groups
#define bAIRCR_PRIGROUP_4       (5<<8)          // 4 groups
#define bAIRCR_PRIGROUP_2       (6<<8)          // 2 groups
#define bAIRCR_PRIGROUP_1       (7<<8)          // 1 groups
                                                //
#define bAIRCR_SYSRESETREQ      (1<<2)          // System reset request
#define bAIRCR_VECTCLRACTIVE    (1<<1)          // Reserved for Debug use
#define bAIRCR_VECTRESET        (1<<0)          // Reserved for Debug use
                                                //
#define bSCR_SEVONPEND          (1<<4)          // Send Event on Pending bit
#define bSCR_SLEEPDEEP          (1<<2)          // CPU uses sleep or deep sleep
#define bSCR_SLEEPONEXIT        (1<<1)          // sleep-on-exit when returning
                                                //
#define bCCR_STKALIGN           (1<<9)          // Indicates stack alignment on exception entry
#define bCCR_BFHFNMIGN          (1<<8)          // handlers with priority -1 or -2 to ignore bus faults
#define bCCR_DIV_0_TRP          (1<<4)          // faulting or halting when SDIV or UDIV on 0
#define bCCR_UNALIGN_TRP        (1<<3)          // trap unaligned halfword and word accesses
#define bCCR_USERSETM           (1<<1)          // unprivileged software access to the STIR
#define bCCR_NONEBASETHRDENA    (1<<0)          // Indicates how the processor enters Thread mode
                                                //
#define bSHCSR_USGFAULTENA      (1<<18)         // Usage fault enable bit
#define bSHCSR_BUSFAULTENA      (1<<17)         // Bus fault enable bit
#define bSHCSR_MEMFAULTENA      (1<<16)         // Memory management fault enable bit
#define bSHCSR_SVCALLPENDED     (1<<15)         // SVC call pending bit
#define bSHCSR_BUSFAULTPENDED   (1<<14)         // Bus fault exception pending bit
#define bSHCSR_MEMFAULTPENDED   (1<<13)         // Memory management fault exception pending bit
#define bSHCSR_USGFAULTPENDED   (1<<12)         // Usage fault exception pending bit
#define bSHCSR_SYSTICKACT       (1<<11)         // SysTick exception active bit
#define bSHCSR_PENDSVACT        (1<<10)         // PendSV exception active bit
#define bSHCSR_MONITORACT       (1<<8)          // Debug monitor active bit
#define bSHCSR_SVCALLACT        (1<<7)          // SVC call active bit
#define bSHCSR_USGFAULTACT      (1<<3)          // Usage fault exception active bit
#define bSHCSR_BUSFAULTACT      (1<<1)          // Bus fault exception active bit
#define bSHCSR_MEMFAULTACT      (1<<0)          // Memory management fault exception active bit
                                                //
#define bCFSR_MMARVALID         (1<<7)          // MM Fault Address Register (MMAR) valid flag
#define bCFSR_MSTKERR           (1<<4)          // MM fault on stacking for exception entry
#define bCFSR_MUNSTKERR         (1<<3)          // MM fault on unstacking for a return from exception
#define bCFSR_DACCVIOL          (1<<1)          // Data access violation flag
#define bCFSR_IACCVIOL          (1<<0)          // Instruction access violation flag
                                                //
#define bCFSR_BFARVALID         (1<<15)         // Bus Fault Address Register (BFAR) valid flag
#define bCFSR_STKERR            (1<<12)         // Bus fault on stacking for exception entry
#define bCFSR_UNSTKERR          (1<<11)         // Bus fault on unstacking for a return from exception
#define bCFSR_IMPRECISERR       (1<<10)         // Imprecise data bus error
#define bCFSR_PRECISERR         (1<<9)          // Precise data bus error
#define bCFSR_IBUSERR           (1<<8)          // Instruction bus error
                                                //
#define bCFSR_DIVBYZERO         (1<<25)         // Divide by zero usage fault
#define bCFSR_UNALIGNED         (1<<24)         // Unaligned access usage fault
#define bCFSR_NOCP              (1<<19)         // No coprocessor usage fault
#define bCFSR_INVPC             (1<<18)         // Invalid PC load usage fault
#define bCFSR_INVSTATE          (1<<17)         // Invalid state usage fault
#define bCFSR_UNDEFINSTR        (1<<16)         // Undefined instruction usage fault
                                                //
#define bHFSR_DEBUGEVT          (1u<<31)        // Reserved for Debug use
#define bHFSR_FORCED            (1<<30)         // Indicates a forced hard fault
#define bHFSR_VECTTBL           (1<<1)          // Indicates a bus fault on a vector table read
                                                //
//________________________________________________________________
//
// Cortex-M3 debug features
//
#define DCB_DHCSR       IO_REG32(0xE000EDF0)    // Debug Halting Control and Status
#define DCB_DCRSR       IO_REG32(0xE000EDF4)    // Debug Core Register Selector
#define DCB_DCRDR       IO_REG32(0xE000EDF8)    // Debug Core Register Data
#define DCB_DEMCR       IO_REG32(0xE000EDFC)    // Debug Exception and Monitor Control
                                                //
#define DWT_CTRL        IO_REG32(0xE0001000)    // Data Watchpoint and Trace
#define DWT_CYCCNT      IO_REG32(0xE0001004)    //
#define DWT_COMP0       IO_REG32(0xE0001020)    //
#define DWT_MASK0       IO_REG32(0xE0001024)    //
#define DWT_FUNCTION0   IO_REG32(0xE0001028)    //
#define DWT_COMP1       IO_REG32(0xE0001030)    //
#define DWT_MASK1       IO_REG32(0xE0001034)    //
#define DWT_FUNCTION1   IO_REG32(0xE0001038)    //
                                                //
#define bDHCSR_DEBUG_KEY        (0xA05F<<16)    //
#define bDHCSR_C_DEBUGEN        (1<<0)          //
#define bDHCSR_C_HALT           (1<<1)          //
#define bDHCSR_C_STEP           (1<<2)          //
#define bDHCSR_C_MASKINTS       (1<<3)          //
#define bDHCSR_C_SNAPSTALL      (1<<5)          //
#define bDHCSR_S_REGRDY         (1<<16)         //
#define bDHCSR_S_HALT           (1<<17)         //
#define bDHCSR_S_SLEEP          (1<<18)         //
#define bDHCSR_S_LOCKUP         (1<<19)         //
#define bDHCSR_S_RETIRE_ST      (1<<24)         //
#define bDHCSR_S_RESET_ST       (1<<25)         //
                                                //
#define bDCRSR_RNW              (1<<16)         //
                                                //
#define bDEMCR_TRCENA           (1<<24)         //
#define bDEMCR_MON_REQ          (1<<19)         //
#define bDEMCR_MON_STEP         (1<<18)         //
#define bDEMCR_PEND             (1<<17)         //
#define bDEMCR_MON_EN           (1<<16)         //
#define bDEMCR_VC_HARDERR       (1<<10)         //
#define bDEMCR_VC_INTERR        (1<<9)          //
#define bDEMCR_VC_BUSERR        (1<<8)          //
#define bDEMCR_VC_STATERR       (1<<7)          //
#define bDEMCR_VC_CHKERR        (1<<6)          //
#define bDEMCR_VC_NOCPERR       (1<<5)          //
#define bDEMCR_VC_MMERR         (1<<4)          //
#define bDEMCR_VC_CORERESET     (1<<0)          //
                                                //
#define bDFSR_EXTERNAL          (1<<4)          //
#define bDFSR_VCATCH            (1<<3)          //
#define bDFSR_DWTTRAP           (1<<2)          //
#define bDFSR_BKPT              (1<<1)          //
#define bDFSR_HALTED            (1<<0)          //
                                                //
#define bDWT_CYCCNTENA          (1<<0)          //

//________________________________________________________________
//
// External Interrupt Controller EXTI
//
#define EXTI_IMR        IO_REG32(0x40013C00)    // Interrupt Mask
#define EXTI_EMR        IO_REG32(0x40013C04)    // Event Mask
#define EXTI_RTSR       IO_REG32(0x40013C08)    // Rising Trigger Selection
#define EXTI_FTSR       IO_REG32(0x40013C0C)    // Falling Trigger Selection
#define EXTI_SWIER      IO_REG32(0x40013C10)    // Software Interrupt
#define EXTI_PR         IO_REG32(0x40013C14)    // Pending Requests
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_EXTI               //
{                                               //
    STM_REG     sEXTI_IMR;                      //
    STM_REG     sEXTI_EMR;                      //
    STM_REG     sEXTI_RTSR;                     //
    STM_REG     sEXTI_FTSR;                     //
    STM_REG     sEXTI_SWIER;                    //
    STM_REG     sEXTI_PR;                       //
                                                //
} STM_EXTI, *PSTM_EXTI;                         //
#pragma pack(pop)                               //

#define EXTI_BASE       ((PSTM_EXTI) 0x40013C00)

#define bEXTI_INT0          (1<<0)              //
#define bEXTI_INT1          (1<<1)              //
#define bEXTI_INT2          (1<<2)              //
#define bEXTI_INT3          (1<<3)              //
#define bEXTI_INT4          (1<<4)              //
#define bEXTI_INT5          (1<<5)              //
#define bEXTI_INT6          (1<<6)              //
#define bEXTI_INT7          (1<<7)              //
#define bEXTI_INT8          (1<<8)              //
#define bEXTI_INT9          (1<<9)              //
#define bEXTI_INT10         (1<<10)             //
#define bEXTI_INT11         (1<<11)             //
#define bEXTI_INT12         (1<<12)             //
#define bEXTI_INT13         (1<<13)             //
#define bEXTI_INT14         (1<<14)             //
#define bEXTI_INT15         (1<<15)             //
#define bEXTI_INT16         (1<<16)             //
#define bEXTI_INT17         (1<<17)             //
#define bEXTI_INT18         (1<<18)             //
#define bEXTI_INT19         (1<<19)             //
#define bEXTI_INT20         (1<<20)             //
#define bEXTI_INT21         (1<<21)             //
#define bEXTI_INT22         (1<<22)             //
                                                //
#define bEXTI_PVD           bEXTI_INT16         //
#define bEXTI_ALARM         bEXTI_INT17         //
#define bEXTI_USBFS         bEXTI_INT18         //
#define bEXTI_EMAC          bEXTI_INT19         //
#define bEXTI_USBHS         bEXTI_INT20         //
#define bEXTI_TAMPER        bEXTI_INT21         //
#define bEXTI_WAKEUP        bEXTI_INT22         //

//________________________________________________________________
//
// FPEC - Flash Program and Erase Controller
//
#define FPEC_ACR        IO_REG32(0x40023C00)    // Access Control Register
#define FPEC_KEYR       IO_REG32(0x40023C04)    // Key Register
#define FPEC_OPTKEYR    IO_REG32(0x40023C08)    // Options Key Register
#define FPEC_SR         IO_REG32(0x40023C0C)    // Status
#define FPEC_CR         IO_REG32(0x40023C10)    // Control
#define FPEC_OPTCR      IO_REG32(0x40023C14)    // Option Control Register
#define FPEC_OPTCRB     IO_REG8 (0x40023C14)    //
#define SYSMEM_FSIZE    IO_REG16(0x1FFF7A22)    // STM32 flash size
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_FPEC               //
{                                               //
    STM_REG     sFPEC_ACR;                      //
    STM_REG     sFPEC_KEYR;                     //
    STM_REG     sFPEC_OPTKEYR;                  //
    STM_REG     sFPEC_SR;                       //
    STM_REG     sFPEC_CR;                       //
    STM_REG     sFPEC_OPTCR;                    //
                                                //
} STM_FPEC, *PSTM_FPEC;                         //
#pragma pack(pop)                               //

#define FPEC_BASE       ((PSTM_FMC) 0x40023C00)

#define bFPEC_RDPRT_KEY0        0xAA            // Read Protection level 0
#define bFPEC_RDPRT_KEY1        0x55            // Read Protection level 1
                                                //
#define bFPEC_KEY1              0x45670123      //
#define bFPEC_KEY2              0xCDEF89AB      //
#define bFPEC_OPTKEY1           0x08192A3B      //
#define bFPEC_OPTKEY2           0x4C5D6E7F      //
                                                // FPEC_ACR
#define bFPEC_DCRST             (1<<12)         // Data Cache Reset
#define bFPEC_ICRST             (1<<11)         // Instruction Cache Reset
#define bFPEC_DCEN              (1<<10)         // Data Cache Enable
#define bFPEC_ICREN             (1<<9)          // Instruction Cache Enable
#define bFPEC_PRFTEN            (1<<8)          // Prefetch enable
                                                //
#define bFPEC_0WS               (0<<0)          //
#define bFPEC_1WS               (1<<0)          //
#define bFPEC_2WS               (2<<0)          //
#define bFPEC_3WS               (3<<0)          //
#define bFPEC_4WS               (4<<0)          //
#define bFPEC_5WS               (5<<0)          //
#define bFPEC_6WS               (6<<0)          //
#define bFPEC_7WS               (7<<0)          //
                                                //
                                                // FPEC_SR
#define bFPEC_BSY               (1<<16)         // Busy
#define bFPEC_PGSERR            (1<<7)          // Programming sequence error
#define bFPEC_PGPERR            (1<<6)          // Programming parallelism error
#define bFPEC_PGAERR            (1<<5)          // Programming alignment error
#define bFPEC_WRPRTERR          (1<<4)          // Write protection error
#define bFPEC_OPERR             (1<<1)          // Operation error
#define bFPEC_EOP               (1<<0)          // End of operation
                                                //
                                                // FPEC_CR
#define bFPEC_LOCK              (1u<<31)        // Locked FPEC
#define bFPEC_ERRIE             (1<<25)         // Error interrupt enable
#define bFPEC_EOPIE             (1<<24)         // End of operation interrupt enable
#define bFPEC_STRT              (1<<16)         // Start
#define bFPEC_PSIZE_MASK        (3<<8)          //
#define bFPEC_PSIZE_X8          (0<<8)          //
#define bFPEC_PSIZE_X16         (1<<8)          //
#define bFPEC_PSIZE_X32         (2<<8)          //
#define bFPEC_PSIZE_X64         (3<<8)          //
#define bFPEC_MER               (1<<2)          // Mass erase
#define bFPEC_SER               (1<<1)          // Sector erase
#define bFPEC_PG                (1<<0)          // Programming
#define bFPEC_SNB_MASK          0x0F            //
#define bFPEC_SNB_SHIFT         3               //
                                                //
                                                // FPEC_OBR
#define bFPEC_nRST_STDBY        (1<<7)          //
#define bFPEC_nRST_STOP         (1<<6)          //
#define bFPEC_WDG_SW            (1<<5)          //
                                                //
#define bFPEC_RDPR_MASK         0xFF            //
#define bFPEC_RDPR_SHIFT        8               //
#define bFPEC_WRPR_MASK         0xFFF           //
#define bFPEC_WRPR_SHIFT        16              //
                                                //
#define bFPEC_BODLEV_27         (0<<2)          // BOD level 2.7..3.6 (~2.9)
#define bFPEC_BODLEV_24         (1<<2)          // BOD level 2.4..2.7 (~2.6)
#define bFPEC_BODLEV_21         (2<<2)          // BOD level 2.1..2.4 (~2.3)
#define bFPEC_BODLEV_OFF        (3<<2)          // BOD off (POR 1.8...2.1)
#define bFPEC_BODLEV_MASK       (3<<2)          //
#define bFPEC_BODLEV_SHIFT      2               //
                                                //
#define bFPEC_OPTSTRT           (1<<1)          // Option start
#define bFPEC_OPTLOCK           (1<<0)          // Option locked
                                                //
                                                //
#pragma pack(push, 1)                           //
typedef volatile struct _STM_OPT_F200           //
{                                               //
        volatile unsigned char  User;           //
        volatile unsigned char  RDP;            //
        volatile unsigned char  Reserved0[6];   //
        volatile unsigned char  WRP0;           //
        volatile unsigned char  WRP1;           //
        volatile unsigned char  Reserved1[6];   //
                                                //
} STM_OPT_F200, *PSTM_OPT_F200;                 //
#pragma pack(pop)                               //

//________________________________________________________________
//
// RCC - Reset and Clock Controller
//
#define RCC_CR          IO_REG32(0x40023800)    // control
#define RCC_PLLCFG      IO_REG32(0x40023804)    // main PLL configuration
#define RCC_CFG         IO_REG32(0x40023808)    // configuration
#define RCC_CIR         IO_REG32(0x4002380C)    // interrupt register
                                                //
#define RCC_AHB1RST     IO_REG32(0x40023810)    // AHB1 reset
#define RCC_AHB2RST     IO_REG32(0x40023814)    // AHB2 reset
#define RCC_AHB3RST     IO_REG32(0x40023818)    // AHB3 reset
#define RCC_APB1RST     IO_REG32(0x40023820)    // APB1 reset
#define RCC_APB2RST     IO_REG32(0x40023824)    // APB2 reset
                                                //
#define RCC_AHB1EN      IO_REG32(0x40023830)    // AHB1 clock enable
#define RCC_AHB2EN      IO_REG32(0x40023834)    // AHB2 clock enable
#define RCC_AHB3EN      IO_REG32(0x40023838)    // AHB3 clock enable
#define RCC_APB1EN      IO_REG32(0x40023840)    // APB2 clock enable
#define RCC_APB2EN      IO_REG32(0x40023844)    // APB1 clock enable
                                                //
#define RCC_AHB1LP      IO_REG32(0x40023850)    // AHB1 clock enable (low power mode)
#define RCC_AHB2LP      IO_REG32(0x40023854)    // AHB2 clock enable (low power mode)
#define RCC_AHB3LP      IO_REG32(0x40023858)    // AHB3 clock enable (low power mode)
#define RCC_APB1LP      IO_REG32(0x40023860)    // APB2 clock enable (low power mode)
#define RCC_APB2LP      IO_REG32(0x40023864)    // APB1 clock enable (low power mode)
                                                //
#define RCC_BDCR        IO_REG32(0x40023870)    // backup domain control
#define RCC_CSR         IO_REG32(0x40023874)    // control and status
#define RCC_SSCGR       IO_REG32(0x40023880)    // spread spectrum clock generation
#define RCC_PLLCFG2     IO_REG32(0x40023884)    // PLL I2S configuration
                                                //
                                                // RCC_CR
                                                //
#define bRCC_PLLRDY             (1<<25)         // PLL clock ready flag
#define bRCC_PLLON              (1<<24)         // PLL enable
                                                //
#define bRCC_CSSON              (1<<19)         // Clock security system enable
#define bRCC_HSEBYP             (1<<18)         // External high-speed clock bypass
#define bRCC_HSERDY             (1<<17)         // External high-speed clock ready flag
#define bRCC_HSEON              (1<<16)         // External high-speed clock enable
                                                //
#define bRCC_HSICAL_MASK        0xFFul          // Internal high-speed clock calibration
#define bRCC_HSICAL_SHIFT       8               //
#define bRCC_HSITRIM_MASK       0x1Ful          // Internal high-speed clock trimming
#define bRCC_HSITRIM_SHIFT      3               //
#define bRCC_HSIRDY             (1<<1)          // Internal high-speed clock ready flag
#define bRCC_HSION              (1<<0)          // Internal high-speed clock enable
                                                //
                                                // RCC_PLLCFG
#define bRCC_PLL_QSHIFT         24              //
#define bRCC_PLL_QMASK          (15<<24)        // 48MHz PLLQ divider 4,,15
#define bRCC_PLL_Q4             (4<<24)         //
#define bRCC_PLL_Q5             (5<<24)         //
#define bRCC_PLL_Q6             (6<<24)         //
#define bRCC_PLL_Q7             (7<<24)         //
#define bRCC_PLL_Q8             (8<<24)         //
#define bRCC_PLL_Q9             (9<<24)         //
#define bRCC_PLL_Q10            (10<<24)        //
#define bRCC_PLL_Q11            (11<<24)        //
#define bRCC_PLL_Q12            (12<<24)        //
#define bRCC_PLL_Q13            (13<<24)        //
#define bRCC_PLL_Q14            (14<<24)        //
#define bRCC_PLL_Q15            (15<<24)        //
                                                //
#define bRCC_PLLSRC             (1<<22)         // PLL entry clock source
                                                // (HSI=0, HSE=1)
#define bRCC_PLL_P2             (0<<16)         //
#define bRCC_PLL_P4             (1<<16)         //
#define bRCC_PLL_P6             (2<<16)         //
#define bRCC_PLL_P8             (3<<16)         //
                                                //
#define bRCC_PLL_NSHIFT         6               //
#define bRCC_PLL_NMASK          (0x1FF<<6)      //
#define bRCC_PLL_MSHIFT         0               //
#define bRCC_PLL_MMASK          (0x3F<<0)       //
                                                //
                                                // RCC_CFG
#define bRCC_MCO2_MASK          (3<<30)         //
#define bRCC_MCO2_SYSCLK        (0<<30)         // MCO2 System clock selected
#define bRCC_MCO2_PLL2          (1<<30)         // PLL2 I2S clock selected
#define bRCC_MCO2_HSE           (2<<30)         // HSE clock selected
#define bRCC_MCO2_PLL           (3<<30)         // main PLL clock selected
                                                //
#define bRCC_MCO2_PRE_MASK      (7<<27)         //
#define bRCC_MCO2_PRE1          (0<<27)         //
#define bRCC_MCO2_PRE2          (4<<27)         //
#define bRCC_MCO2_PRE3          (5<<27)         //
#define bRCC_MCO2_PRE4          (6<<27)         //
#define bRCC_MCO2_PRE5          (7<<27)         //
                                                //
#define bRCC_MCO_PRE_MASK       (7<<24)         //
#define bRCC_MCO_PRE1           (0<<24)         //
#define bRCC_MCO_PRE2           (4<<24)         //
#define bRCC_MCO_PRE3           (5<<24)         //
#define bRCC_MCO_PRE4           (6<<24)         //
#define bRCC_MCO_PRE5           (7<<24)         //
                                                //
#define bRCC_I2SSRC             (1<<23)         // I2S clock source
#define bRCC_MCO_MASK           (3<<21)         //
#define bRCC_MCO_HSI            (0<<21)         // HSI clock selected
#define bRCC_MCO_LSE            (1<<21)         // LSE clock selected
#define bRCC_MCO_HSE            (2<<21)         // HSE clock selected
#define bRCC_MCO_PLL            (3<<21)         // PLL clock selected
                                                //
#define bRCC_RTCPRE_MASK        (0x1F<<16)      // HSE RTC prescaler
#define bRCC_RTCPRE_SHIFT       16              //
                                                //
                                                // APB2 prescaler (PCLK2)
#define bRCC_PPRE2_1            (0<<13)         // HCLK/1
#define bRCC_PPRE2_2            (4<<13)         // HCLK/2
#define bRCC_PPRE2_4            (5<<13)         // HCLK/4
#define bRCC_PPRE2_8            (6<<13)         // HCLK/8
#define bRCC_PPRE2_16           (7<<13)         // HCLK/16
#define bRCC_PPRE2_MASK         (7<<13)         //
                                                //
                                                // APB1 prescaler (PCLK1)
#define bRCC_PPRE1_1            (0<<10)         // HCLK/1
#define bRCC_PPRE1_2            (4<<10)         // HCLK/2
#define bRCC_PPRE1_4            (5<<10)         // HCLK/4
#define bRCC_PPRE1_8            (6<<10)         // HCLK/8
#define bRCC_PPRE1_16           (7<<10)         // HCLK/16
#define bRCC_PPRE1_MASK         (7<<10)         //
                                                //
                                                // AHB prescaler (SYSCLK)
#define bRCC_HPRE_1             (0<<4)          // SYSCLK/1
#define bRCC_HPRE_2             (8<<4)          // SYSCLK/2
#define bRCC_HPRE_4             (9<<4)          // SYSCLK/4
#define bRCC_HPRE_8             (10<<4)         // SYSCLK/8
#define bRCC_HPRE_16            (11<<4)         // SYSCLK/16
#define bRCC_HPRE_64            (12<<4)         // SYSCLK/64
#define bRCC_HPRE_128           (13<<4)         // SYSCLK/128
#define bRCC_HPRE_256           (14<<4)         // SYSCLK/256
#define bRCC_HPRE_512           (15<<4)         // SYSCLK/512
#define bRCC_HPRE_MASK          (15<<10)        //
                                                //
#define bRCC_SWS_HSI            (0<<2)          // HSI is SYSCLK source (status)
#define bRCC_SWS_HSE            (1<<2)          // HSE is SYSCLK source (status)
#define bRCC_SWS_PLL            (2<<2)          // PLL is SYSCLK source (status)
#define bRCC_SWS_MASK           (3<<2)          //
                                                //
#define bRCC_SW_HSI             (0<<0)          // HSI is SYSCLK source
#define bRCC_SW_HSE             (1<<0)          // HSE is SYSCLK source
#define bRCC_SW_PLL             (2<<0)          // PLL is SYSCLK source
#define bRCC_SW_MASK            (3<<0)          //
                                                //
                                                // RCC_CSR
#define bRCC_LPWRRSTF           (1ul<<31)       // Low-power reset flag
#define bRCC_WWDGRSTF           (1<<30)         // Window watchdog reset flag
#define bRCC_IWDGRSTF           (1<<29)         // Independent watchdog reset flag
#define bRCC_SFTRSTF            (1<<28)         // Software reset flag
#define bRCC_PORRSTF            (1<<27)         // POR/PDR reset flag
#define bRCC_PINRSTF            (1<<26)         // PIN reset flag
#define bRCC_BORRSTF            (1<<25)         // BOD reset flag
#define bRCC_RMVF               (1<<24)         // Remove reset flag
#define bRCC_LSIRDY             (1<<1)          // Internal low-speed oscillator ready
#define bRCC_LSION              (1<<0)          // Internal low-speed oscillator enable
                                                //
                                                // RCC_AHB1RST
#define bRCC_OTGHSRST           (1<<29)         // USB OTG HS module reset
#define bRCC_ETHMACRST          (1<<25)         // Ethernet MAC reset
#define bRCC_DMA2RST            (1<<22)         // DMA2 reset
#define bRCC_DMA1RST            (1<<21)         // DMA1 reset
#define bRCC_CRCRST             (1<<12)         // CRC reset
#define bRCC_IOPIRST            (1<<8)          // IO port I reset
#define bRCC_IOPHRST            (1<<7)          // IO port H reset
#define bRCC_IOPGRST            (1<<6)          // IO port G reset
#define bRCC_IOPFRST            (1<<5)          // IO port F reset
#define bRCC_IOPERST            (1<<4)          // IO port E reset
#define bRCC_IOPDRST            (1<<3)          // IO port D reset
#define bRCC_IOPCRST            (1<<2)          // IO port C reset
#define bRCC_IOPBRST            (1<<1)          // IO port B reset
#define bRCC_IOPARST            (1<<0)          // IO port A reset
                                                //
                                                // RCC_AHB2RST
#define bRCC_OTGFSRST           (1<<7)          // USB OTG FS module reset
#define bRCC_RNGRST             (1<<6)          // Random number generator module reset
#define bRCC_HASHRST            (1<<5)          // Hash module reset
#define bRCC_CRYPRST            (1<<4)          // Cryptographic module reset
#define bRCC_DCMIRST            (1<<0)          // Camera interface reset
                                                //
                                                // RCC_AHB3RST
#define bRCC_FSMCRST            (1<<0)          // Flexible Static Memory Controler
                                                //
                                                // RCC_APB1RST
#define bRCC_DACRST             (1<<29)         // DAC interface reset
#define bRCC_PWRRST             (1<<28)         // Power interface reset
#define bRCC_CAN2RST            (1<<26)         // CAN2 reset
#define bRCC_CAN1RST            (1<<25)         // CAN1 reset
#define bRCC_I2C3RST            (1<<23)         // I2C3 reset
#define bRCC_I2C2RST            (1<<22)         // I2C2 reset
#define bRCC_I2C1RST            (1<<21)         // I2C1 reset
#define bRCC_UART5RST           (1<<20)         // UART5 reset
#define bRCC_UART4RST           (1<<19)         // UART4 reset
#define bRCC_UART3RST           (1<<18)         // UART3 reset
#define bRCC_UART2RST           (1<<17)         // UART2 reset
#define bRCC_SPI3RST            (1<<15)         // SPI3 reset
#define bRCC_SPI2RST            (1<<14)         // SPI2 reset
#define bRCC_WWDGRST            (1<<11)         // Window watchdog reset
#define bRCC_TIM14RST           (1<<8)          // TIM14 reset
#define bRCC_TIM13RST           (1<<7)          // TIM13 reset
#define bRCC_TIM12RST           (1<<6)          // TIM12 reset
#define bRCC_TIM7RST            (1<<5)          // TIM7 reset
#define bRCC_TIM6RST            (1<<4)          // TIM6 reset
#define bRCC_TIM5RST            (1<<3)          // TIM5 reset
#define bRCC_TIM4RST            (1<<2)          // TIM4 reset
#define bRCC_TIM3RST            (1<<1)          // TIM3 reset
#define bRCC_TIM2RST            (1<<0)          // TIM2 reset
                                                //
                                                // RCC_APB2RST
#define bRCC_TIM11RST           (1<<18)         // TIM11 timer reset
#define bRCC_TIM10RST           (1<<17)         // TIM10 timer reset
#define bRCC_TIM9RST            (1<<16)         // TIM9 timer reset
#define bRCC_SYSCFGRST          (1<<14)         // SYSCFG reset
#define bRCC_SPI1RST            (1<<12)         // SPI1 reset
#define bRCC_SDIORST            (1<<11)         // SDIO interface reset
#define bRCC_ADCRST             (1<<8)          // ADC interface reset
#define bRCC_UART6RST           (1<<5)          // USART6 reset
#define bRCC_UART1RST           (1<<4)          // USART1 reset
#define bRCC_TIM8RST            (1<<1)          // TIM8 timer reset
#define bRCC_TIM1RST            (1<<0)          // TIM1 timer reset
                                                //
                                                // RCC_AHB1EN/RCC_AHB1LP
#define bRCC_OTGHSULPIEN        (1<<30)         // USB OTG HSULPI clock enable
#define bRCC_OTGHSEN            (1<<29)         // USB OTG HS clock enable
#define bRCC_EMACPTPEN          (1<<28)         // Ethernet PTP clock enable
#define bRCC_EMACRXEN           (1<<27)         // Ethernet Reception clock enable
#define bRCC_EMACTXEN           (1<<26)         // Ethernet Transmission clock enable
#define bRCC_EMACEN             (1<<25)         // Ethernet MAC clock enable
                                                //
#define bRCC_DMA2EN             (1<<22)         // DMA2 clock enable
#define bRCC_DMA1EN             (1<<21)         // DMA1 clock enable
#define bRCC_BKPSRAMEN          (1<<18)         // Backup SRAM interface clock enable
#define bRCC_SRAM2EN            (1<<17)         // SRAM2 clock enable (Sleep Mode)
#define bRCC_SRAM1EN            (1<<16)         // SRAM1 clock enable (Sleep Mode)
#define bRCC_FLITFEN            (1<<15)         // Flash clock enable (Sleep Mode)
#define bRCC_CRCEN              (1<<12)         // CRC clock enable
                                                //
#define bRCC_IOPIEN             (1<<8)          // IO port I clock enable
#define bRCC_IOPHEN             (1<<7)          // IO port H clock enable
#define bRCC_IOPGEN             (1<<6)          // IO port G clock enable
#define bRCC_IOPFEN             (1<<5)          // IO port F clock enable
#define bRCC_IOPEEN             (1<<4)          // IO port E clock enable
#define bRCC_IOPDEN             (1<<3)          // IO port D clock enable
#define bRCC_IOPCEN             (1<<2)          // IO port C clock enable
#define bRCC_IOPBEN             (1<<1)          // IO port B clock enable
#define bRCC_IOPAEN             (1<<0)          // IO port A clock enable
                                                //
                                                // RCC_AHB2EN/RCC_AHB2LP
#define bRCC_OTGFSEN            (1<<7)          // USB OTG FS clock enable
#define bRCC_RNGEN              (1<<6)          // Random number generator clock enable
#define bRCC_HASHEN             (1<<5)          // Hash modules clock enable
#define bRCC_CRYPEN             (1<<4)          // Cryptographic modules clock enable
#define bRCC_DCMIEN             (1<<0)          // Camera clock enable
                                                //
                                                // RCC_AHB3EN/RCC_AHB3LP
#define bRCC_FSMCEN             (1<<0)          // Flexible Static Memory Controler
                                                //
                                                // RCC_APB1EN/RCC_APB1LP
#define bRCC_DACEN              (1<<29)         // DAC interface clock enable
#define bRCC_PWREN              (1<<28)         // Power interface clock enable
#define bRCC_CAN2EN             (1<<26)         // CAN2 clock enable
#define bRCC_CAN1EN             (1<<25)         // CAN1 clock enable
#define bRCC_I2C3EN             (1<<23)         // I2C3 clock enable
#define bRCC_I2C2EN             (1<<22)         // I2C2 clock enable
#define bRCC_I2C1EN             (1<<21)         // I2C1 clock enable
#define bRCC_UART5EN            (1<<20)         // UART5 clock enable
#define bRCC_UART4EN            (1<<19)         // UART4 clock enable
#define bRCC_UART3EN            (1<<18)         // USART3 clock enable
#define bRCC_UART2EN            (1<<17)         // USART2 clock enable
#define bRCC_SPI3EN             (1<<15)         // SPI3 clock enable
#define bRCC_SPI2EN             (1<<14)         // SPI2 clock enable
#define bRCC_WWDGEN             (1<<11)         // Window watchdog clock enable
#define bRCC_TIM14EN            (1<<8)          // TIM14 clock enable
#define bRCC_TIM13EN            (1<<7)          // TIM13 clock enable
#define bRCC_TIM12EN            (1<<6)          // TIM12 clock enable
#define bRCC_TIM7EN             (1<<5)          // TIM7 clock enable
#define bRCC_TIM6EN             (1<<4)          // TIM6 clock enable
#define bRCC_TIM5EN             (1<<3)          // TIM5 clock enable
#define bRCC_TIM4EN             (1<<2)          // TIM4 clock enable
#define bRCC_TIM3EN             (1<<1)          // TIM3 clock enable
#define bRCC_TIM2EN             (1<<0)          // TIM2 clock enable
                                                //
                                                // RCC_APB2EN/RCC_APB2LP
#define bRCC_TIM11EN            (1<<18)         // TIM11 timer clock enable
#define bRCC_TIM10EN            (1<<17)         // TIM10 timer clock enable
#define bRCC_TIM9EN             (1<<16)         // TIM9 timer clock enable
#define bRCC_SYSCFGEN           (1<<14)         // SYSCFG clock enable
#define bRCC_SPI1EN             (1<<12)         // SPI1 clock enable
#define bRCC_SDIOEN             (1<<11)         // SDIO interface clock enable
#define bRCC_ADC3EN             (1<<10)         // ADC interface clock enable
#define bRCC_ADC2EN             (1<<9)          // ADC interface clock enable
#define bRCC_ADC1EN             (1<<8)          // ADC interface clock enable
#define bRCC_UART6EN            (1<<5)          // USART6 clock enable
#define bRCC_UART1EN            (1<<4)          // USART1 clock enable
#define bRCC_TIM8EN             (1<<1)          // TIM8 timer clock enable
#define bRCC_TIM1EN             (1<<0)          // TIM1 timer clock enable
                                                //
                                                // RCC_BDCR
#define bRCC_BDRST              (1<<16)         // Backup domain software reset
#define bRCC_RTCEN              (1<<15)         // RTC clock enable
                                                //
#define bRCC_RTCSEL_NONE        (0<<8)          // RTC No clock
#define bRCC_RTCSEL_LSE         (1<<8)          // LSE used as RTC clock
#define bRCC_RTCSEL_LSI         (2<<8)          // LSI used as RTC clock
#define bRCC_RTCSEL_HSE         (3<<8)          // HSE/128 used as RTC clock
#define bRCC_RTCSEL_MASK        (3<<8)          //
                                                //
#define bRCC_LSEBYP             (1<<2)          // External low-speed oscillator bypass
#define bRCC_LSERDY             (1<<1)          // External low-speed oscillator ready
#define bRCC_LSEON              (1<<0)          // External low-speed oscillator enable
                                                //
#pragma pack(push, 4)
typedef volatile struct _STM_RCC
{
    STM_REG     sRCC_CR;
    STM_REG     sRCC_PLLCFG;
    STM_REG     sRCC_CFG;
    STM_REG     sRCC_CIR;
    STM_REG     sRCC_AHB1RST;
    STM_REG     sRCC_AHB2RST;
    STM_REG     sRCC_AHB3RST;
    STM_REG     sReserved0[1];
    STM_REG     sRCC_APB1RST;
    STM_REG     sRCC_APB2RST;
    STM_REG     sReserved1[2];
    STM_REG     sRCC_AHB1EN;
    STM_REG     sRCC_AHB2EN;
    STM_REG     sRCC_AHB3EN;
    STM_REG     sReserved2[1];
    STM_REG     sRCC_APB1EN;
    STM_REG     sRCC_APB2EN;
    STM_REG     sReserved3[2];
    STM_REG     sRCC_AHB1LP;
    STM_REG     sRCC_AHB2LP;
    STM_REG     sRCC_AHB3LP;
    STM_REG     sReserved4[1];
    STM_REG     sRCC_APB1LP;
    STM_REG     sRCC_APB2LP;
    STM_REG     sReserved5[2];
    STM_REG     sRCC_BDCR;
    STM_REG     sRCC_CSR;
    STM_REG     sReserved6[2];
    STM_REG     sRCC_SSCGR;
    STM_REG     sRCC_PLLCFG2;

} STM_RCC, *PSTM_RCC;
#pragma pack(pop)

#define RCC_BASE        ((PSTM_RCC) 0x40023800)
//________________________________________________________________
//
// GPIO - General Purpose I/O registers
//
#define PIOA_MODE       IO_REG32(0x40020000)    // Port A mode
#define PIOA_TYPE       IO_REG32(0x40020004)    // Port A output type
#define PIOA_SPEED      IO_REG32(0x40020008)    // Port A output speed
#define PIOA_PUPD       IO_REG32(0x4002000C)    // Port A pull up/down control
#define PIOA_IDR        IO_REG32(0x40020010)    // Port A input data
#define PIOA_ODR        IO_REG32(0x40020014)    // Port A output data
#define PIOA_BSR        IO_REG32(0x40020018)    // Port A set and reset
#define PIOA_LOCK       IO_REG32(0x4002001C)    // Port A lock
#define PIOA_AFL        IO_REG32(0x40020020)    // Port A alternate function low
#define PIOA_AFH        IO_REG32(0x40020024)    // Port A alternate function high
                                                //
#define PIOB_MODE       IO_REG32(0x40020400)    // Port B mode
#define PIOB_TYPE       IO_REG32(0x40020404)    // Port B output type
#define PIOB_SPEED      IO_REG32(0x40020408)    // Port B output speed
#define PIOB_PUPD       IO_REG32(0x4002040C)    // Port B pull up/down control
#define PIOB_IDR        IO_REG32(0x40020410)    // Port B input data
#define PIOB_ODR        IO_REG32(0x40020414)    // Port B output data
#define PIOB_BSR        IO_REG32(0x40020418)    // Port B set and reset
#define PIOB_LOCK       IO_REG32(0x4002041C)    // Port B lock
#define PIOB_AFL        IO_REG32(0x40020420)    // Port B alternate function low
#define PIOB_AFH        IO_REG32(0x40020424)    // Port B alternate function high
                                                //
#define PIOC_MODE       IO_REG32(0x40020800)    // Port C mode
#define PIOC_TYPE       IO_REG32(0x40020804)    // Port C output type
#define PIOC_SPEED      IO_REG32(0x40020808)    // Port C output speed
#define PIOC_PUPD       IO_REG32(0x4002080C)    // Port C pull up/down control
#define PIOC_IDR        IO_REG32(0x40020810)    // Port C input data
#define PIOC_ODR        IO_REG32(0x40020814)    // Port C output data
#define PIOC_BSR        IO_REG32(0x40020818)    // Port C set and reset
#define PIOC_LOCK       IO_REG32(0x4002081C)    // Port C lock
#define PIOC_AFL        IO_REG32(0x40020820)    // Port C alternate function low
#define PIOC_AFH        IO_REG32(0x40020824)    // Port C alternate function high
                                                //
#define PIOD_MODE       IO_REG32(0x40020C00)    // Port D mode
#define PIOD_TYPE       IO_REG32(0x40020C04)    // Port D output type
#define PIOD_SPEED      IO_REG32(0x40020C08)    // Port D output speed
#define PIOD_PUPD       IO_REG32(0x40020C0C)    // Port D pull up/down control
#define PIOD_IDR        IO_REG32(0x40020C10)    // Port D input data
#define PIOD_ODR        IO_REG32(0x40020C14)    // Port D output data
#define PIOD_BSR        IO_REG32(0x40020C18)    // Port D set and reset
#define PIOD_LOCK       IO_REG32(0x40020C1C)    // Port D lock
#define PIOD_AFL        IO_REG32(0x40020C20)    // Port D alternate function low
#define PIOD_AFH        IO_REG32(0x40020C24)    // Port D alternate function high
                                                //
#define PIOE_MODE       IO_REG32(0x40021000)    // Port E mode
#define PIOE_TYPE       IO_REG32(0x40021004)    // Port E output type
#define PIOE_SPEED      IO_REG32(0x40021008)    // Port E output speed
#define PIOE_PUPD       IO_REG32(0x4002100C)    // Port E pull up/down control
#define PIOE_IDR        IO_REG32(0x40021010)    // Port E input data
#define PIOE_ODR        IO_REG32(0x40021014)    // Port E output data
#define PIOE_BSR        IO_REG32(0x40021018)    // Port E set and reset
#define PIOE_LOCK       IO_REG32(0x4002101C)    // Port E lock
#define PIOE_AFL        IO_REG32(0x40021020)    // Port E alternate function low
#define PIOE_AFH        IO_REG32(0x40021024)    // Port E alternate function high
                                                //
#define PIOF_MODE       IO_REG32(0x40021400)    // Port F mode
#define PIOF_TYPE       IO_REG32(0x40021404)    // Port F output type
#define PIOF_SPEED      IO_REG32(0x40021408)    // Port F output speed
#define PIOF_PUPD       IO_REG32(0x4002140C)    // Port F pull up/down control
#define PIOF_IDR        IO_REG32(0x40021410)    // Port F input data
#define PIOF_ODR        IO_REG32(0x40021414)    // Port F output data
#define PIOF_BSR        IO_REG32(0x40021418)    // Port F set and reset
#define PIOF_LOCK       IO_REG32(0x4002141C)    // Port F lock
#define PIOF_AFL        IO_REG32(0x40021420)    // Port F alternate function low
#define PIOF_AFH        IO_REG32(0x40021424)    // Port F alternate function high
                                                //
#define PIOG_MODE       IO_REG32(0x40021800)    // Port G mode
#define PIOG_TYPE       IO_REG32(0x40021804)    // Port G output type
#define PIOG_SPEED      IO_REG32(0x40021808)    // Port G output speed
#define PIOG_PUPD       IO_REG32(0x4002180C)    // Port G pull up/down control
#define PIOG_IDR        IO_REG32(0x40021810)    // Port G input data
#define PIOG_ODR        IO_REG32(0x40021814)    // Port G output data
#define PIOG_BSR        IO_REG32(0x40021818)    // Port G set and reset
#define PIOG_LOCK       IO_REG32(0x4002181C)    // Port G lock
#define PIOG_AFL        IO_REG32(0x40021820)    // Port G alternate function low
#define PIOG_AFH        IO_REG32(0x40021824)    // Port G alternate function high
                                                //
#define PIOH_MODE       IO_REG32(0x40021C00)    // Port H mode
#define PIOH_TYPE       IO_REG32(0x40021C04)    // Port H output type
#define PIOH_SPEED      IO_REG32(0x40021C08)    // Port H output speed
#define PIOH_PUPD       IO_REG32(0x40021C0C)    // Port H pull up/down control
#define PIOH_IDR        IO_REG32(0x40021C10)    // Port H input data
#define PIOH_ODR        IO_REG32(0x40021C14)    // Port H output data
#define PIOH_BSR        IO_REG32(0x40021C18)    // Port H set and reset
#define PIOH_LOCK       IO_REG32(0x40021C1C)    // Port H lock
#define PIOH_AFL        IO_REG32(0x40021C20)    // Port H alternate function low
#define PIOH_AFH        IO_REG32(0x40021C24)    // Port H alternate function high
                                                //
#define PIOI_MODE       IO_REG32(0x40022000)    // Port I mode
#define PIOI_TYPE       IO_REG32(0x40022004)    // Port I output type
#define PIOI_SPEED      IO_REG32(0x40022008)    // Port I output speed
#define PIOI_PUPD       IO_REG32(0x4002200C)    // Port I pull up/down control
#define PIOI_IDR        IO_REG32(0x40022010)    // Port I input data
#define PIOI_ODR        IO_REG32(0x40022014)    // Port I output data
#define PIOI_BSR        IO_REG32(0x40022018)    // Port I set and reset
#define PIOI_LOCK       IO_REG32(0x4002201C)    // Port I lock
#define PIOI_AFL        IO_REG32(0x40022020)    // Port I alternate function low
#define PIOI_AFH        IO_REG32(0x40022024)    // Port I alternate function high
                                                //
#pragma pack(push, 4)
typedef volatile struct _STM_PIO
{
    STM_REG     sPIO_MODE;
    STM_REG     sPIO_TYPE;
    STM_REG     sPIO_SPEED;
    STM_REG     sPIO_PUPD;
    STM_REG     sPIO_IDR;
    STM_REG     sPIO_ODR;
    STM_REG     sPIO_BSR;
    STM_REG     sPIO_LOCK;
    STM_REG     sPIO_AFL;
    STM_REG     sPIO_AFH;

} STM_PIO, *PSTM_PIO;
#pragma pack(pop)

#define GPIO_A_BASE     ((PSTM_PIO) 0x40020000)
#define GPIO_B_BASE     ((PSTM_PIO) 0x40020400)
#define GPIO_C_BASE     ((PSTM_PIO) 0x40020800)
#define GPIO_D_BASE     ((PSTM_PIO) 0x40020C00)
#define GPIO_E_BASE     ((PSTM_PIO) 0x40021000)
#define GPIO_F_BASE     ((PSTM_PIO) 0x40021400)
#define GPIO_G_BASE     ((PSTM_PIO) 0x40021800)
#define GPIO_H_BASE     ((PSTM_PIO) 0x40021C00)
#define GPIO_I_BASE     ((PSTM_PIO) 0x40022000)

#define bPIO_LCKK               (1<<16)

#define bPIO_FUNC_MASK          0x0F
#define bPIO_MODE_MASK          0x03
#define bPIO_MODE_INPUT         0x00
#define bPIO_MODE_OUTPUT        0x01
#define bPIO_MODE_AFUNC         0x02
#define bPIO_MODE_ANALOG        0x03

#define bPIO_OUTPUT_MASK        0x03
#define bPIO_OUTPUT_2MHZ        0x00
#define bPIO_OUTPUT_25MHZ       0x01
#define bPIO_OUTPUT_50MHZ       0x02
#define bPIO_OUTPUT_100MHZ      0x03

#define bPIO_PULL_NONE          0x00
#define bPIO_PULL_UP            0x01
#define bPIO_PULL_DOWN          0x02
#define bPIO_PULL_MASK          0x03

#define bPIO_AF0                0
#define bPIO_AF1                1
#define bPIO_AF2                2
#define bPIO_AF3                3
#define bPIO_AF4                4
#define bPIO_AF5                5
#define bPIO_AF6                6
#define bPIO_AF7                7
#define bPIO_AF8                8
#define bPIO_AF9                9
#define bPIO_AF10               10
#define bPIO_AF11               11
#define bPIO_AF12               12
#define bPIO_AF13               13
#define bPIO_AF14               14
#define bPIO_AF15               15

//________________________________________________________________
//
#define CRYP_SR         IO_REG32(0x50060004)    // CRYP status
//________________________________________________________________
//
// PWR - power modes controller
//
#define PWR_CR          IO_REG32(0x40007000)    // Power Control
#define PWR_CSR         IO_REG32(0x40007004)    // Power Control and Status
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_PWR                //
{                                               //
    STM_REG     sPWR_CR;                        //
    STM_REG     sPWR_CSR;                       //
                                                //
} STM_PWR, *PSTM_PWR;                           //
#pragma pack(pop)                               //
                                                //
#define PWR_BASE    ((PSTM_PWR) 0x40007000)     //
                                                // PWR_CR
#define bPWR_FPDS               (1<<9)          // Flash in PowerDown
#define bPWR_DBP                (1<<8)          // Disable backup domain write protection
#define bPWR_PVD_MASK           (7<<5)          //
#define bPWR_PVD_20             (0<<5)          // 2.0V
#define bPWR_PVD_21             (1<<5)          // 2.1V
#define bPWR_PVD_23             (2<<5)          // 2.3V
#define bPWR_PVD_25             (3<<5)          // 2.5V
#define bPWR_PVD_26             (4<<5)          // 2.6V
#define bPWR_PVD_27             (5<<5)          // 2.7V
#define bPWR_PVD_28             (6<<5)          // 2.8V
#define bPWR_PVD_29             (7<<5)          // 2.9V
                                                //
#define bPWR_PVDE               (1<<4)          // Power voltage detector enable
#define bPWR_CSBF               (1<<3)          // Clear standby flag
#define bPWR_CWUF               (1<<2)          // Clear wakeup flag
#define bPWR_PDDS               (1<<1)          // Power down deepsleep
#define bPWR_LPDS               (1<<0)          // Low-power deepsleep.
                                                //
                                                // PWR_CSR
#define bPWR_BRE                (1<<9)          // Enable Backup Regulator
#define bPWR_EWUP               (1<<8)          // Enable WKUP pin
#define bPWR_BRR                (1<<3)          // Backup Regulator ready
#define bPWR_PVDO               (1<<2)          // PVD output
#define bPWR_SBF                (1<<1)          // Standby flag
#define bPWR_WUF                (1<<0)          // Wakeup flag

//________________________________________________________________
//
// Watchdog timers
//
#define IWDG_KR         IO_REG32(0x40003000)    // IWDG Key register
#define IWDG_PR         IO_REG32(0x40003004)    // IWDG prescaler
#define IWDG_RLR        IO_REG32(0x40003008)    // IWDG Reload register
#define IWDG_SR         IO_REG32(0x4000300C)    // IWDG Status register
                                                //
#define WWDG_CR         IO_REG32(0x40002C00)    // WWDG Control register
#define WWDG_CFR        IO_REG32(0x40002C04)    // WWDG Configuration
#define WWDG_SR         IO_REG32(0x40002C08)    // WWDG Status register

#pragma pack(push, 4)
typedef volatile struct _STM_IWDG
{
    STM_REG     sIWDG_KR;
    STM_REG     sIWDG_PR;
    STM_REG     sIWDG_RLR;
    STM_REG     sIWDG_SR;

} STM_IWDG, *PSTM_IWDG;
#pragma pack(pop)

#pragma pack(push, 4)
typedef volatile struct _STM_WWDG
{
    STM_REG     sWWDG_CR;
    STM_REG     sWWDG_CFR;
    STM_REG     sWWDG_SR;

} STM_WWDG, *PSTM_WWDG;
#pragma pack(pop)

#define WWDG_BASE       ((PSTM_WWDG) 0x40002C00)
#define IWDG_BASE       ((PSTM_IWDG) 0x40003000)

#define bWDT_KEY_5555           0x5555          // enable IWDG access
#define bWDT_KEY_AAAA           0xAAAA          // restart IWDG
#define bWDT_KEY_CCCC           0xCCCC          // start IWDG
                                                //
#define bWDT_PR_4               (0<<0)          // IWDG divide by 4
#define bWDT_PR_8               (1<<0)          // IWDG divide by 8
#define bWDT_PR_16              (2<<0)          // IWDG divide by 16
#define bWDT_PR_32              (3<<0)          // IWDG divide by 32
#define bWDT_PR_64              (4<<0)          // IWDG divide by 64
#define bWDT_PR_128             (5<<0)          // IWDG divide by 128
#define bWDT_PR_256             (6<<0)          // IWDG divide by 256
                                                //
#define bWDT_RVU                (1<<1)          // Watchdog counter reload value update
#define bWDT_PVU                (1<<0)          // Watchdog prescaler value update
                                                //
#define bWDT_WDGA               (1<<7)          // WWDG Activation bit
#define bWDT_MASK               0x7F            // WWDG T and W mask
#define bWDT_EWI                (1<<9)          // Early wakeup interrupt
#define bWDT_WDGTB_1            (0<<7)          // WWDG clock is (PCLK1/4096)/1
#define bWDT_WDGTB_2            (1<<7)          // WWDG clock is (PCLK1/4096)/2
#define bWDT_WDGTB_4            (2<<7)          // WWDG clock is (PCLK1/4096)/4
#define bWDT_WDGTB_8            (3<<7)          // WWDG clock is (PCLK1/4096)/8
#define bWDT_WDGTB_MASK         (3<<7)          //
#define bWDT_EWIF               (1<<0)          // Early wakeup interrupt flag
                                                //
//________________________________________________________________
//
//  Timers
//
#define IWDG_KR         IO_REG32(0x40003000)    // IWDG Key register
#define TIM1_CR1        IO_REG32(0x40010000)    // control register 1
#define TIM1_CR2        IO_REG32(0x40010004)    // control register 2
#define TIM1_SMCR       IO_REG32(0x40010008)    // slave mode control
#define TIM1_DIER       IO_REG32(0x4001000C)    // DMA/interrupt enable
#define TIM1_SR         IO_REG32(0x40010010)    // status register
#define TIM1_EGR        IO_REG32(0x40010014)    // event generation
#define TIM1_CCMR1      IO_REG32(0x40010018)    // capture/compare mode 1
#define TIM1_CCMR2      IO_REG32(0x4001001C)    // capture/compare mode 2
#define TIM1_CCER       IO_REG32(0x40010020)    // capture/compare event
#define TIM1_CNT        IO_REG32(0x40010024)    // counter
#define TIM1_PSC        IO_REG32(0x40010028)    // prescaler
#define TIM1_ARR        IO_REG32(0x4001002C)    // autoreload
#define TIM1_RCR        IO_REG32(0x40010030)    // repetition counter
#define TIM1_CCR1       IO_REG32(0x40010034)    // capture/compare 1
#define TIM1_CCR2       IO_REG32(0x40010038)    // capture/compare 2
#define TIM1_CCR3       IO_REG32(0x4001003C)    // capture/compare 3
#define TIM1_CCR4       IO_REG32(0x40010040)    // capture/compare 4
#define TIM1_BDTR       IO_REG32(0x40010044)    // break and dead-time
#define TIM1_DCR        IO_REG32(0x40010048)    // DMA control
#define TIM1_DMAR       IO_REG32(0x4001004C)    // DMA address
                                                //
#define TIM2_CR1        IO_REG32(0x40000000)    // control register 1
#define TIM2_CR2        IO_REG32(0x40000004)    // control register 2
#define TIM2_SMCR       IO_REG32(0x40000008)    // slave mode control
#define TIM2_DIER       IO_REG32(0x4000000C)    // DMA/interrupt enable
#define TIM2_SR         IO_REG32(0x40000010)    // status register
#define TIM2_EGR        IO_REG32(0x40000014)    // event generation
#define TIM2_CCMR1      IO_REG32(0x40000018)    // capture/compare mode 1
#define TIM2_CCMR2      IO_REG32(0x4000001C)    // capture/compare mode 2
#define TIM2_CCER       IO_REG32(0x40000020)    // capture/compare event
#define TIM2_CNT        IO_REG32(0x40000024)    // counter
#define TIM2_PSC        IO_REG32(0x40000028)    // prescaler
#define TIM2_ARR        IO_REG32(0x4000002C)    // autoreload
#define TIM2_CCR1       IO_REG32(0x40000034)    // capture/compare 1
#define TIM2_CCR2       IO_REG32(0x40000038)    // capture/compare 2
#define TIM2_CCR3       IO_REG32(0x4000003C)    // capture/compare 3
#define TIM2_CCR4       IO_REG32(0x40000040)    // capture/compare 4
#define TIM2_DCR        IO_REG32(0x40000048)    // DMA control
#define TIM2_DMAR       IO_REG32(0x4000004C)    // DMA address
#define TIM2_OR         IO_REG32(0x40000050)    // options (calibaration)
                                                //
#define TIM3_CR1        IO_REG32(0x40000400)    // control register 1
#define TIM3_CR2        IO_REG32(0x40000404)    // control register 2
#define TIM3_SMCR       IO_REG32(0x40000408)    // slave mode control
#define TIM3_DIER       IO_REG32(0x4000040C)    // DMA/interrupt enable
#define TIM3_SR         IO_REG32(0x40000410)    // status register
#define TIM3_EGR        IO_REG32(0x40000414)    // event generation
#define TIM3_CCMR1      IO_REG32(0x40000418)    // capture/compare mode 1
#define TIM3_CCMR2      IO_REG32(0x4000041C)    // capture/compare mode 2
#define TIM3_CCER       IO_REG32(0x40000420)    // capture/compare event
#define TIM3_CNT        IO_REG32(0x40000424)    // counter
#define TIM3_PSC        IO_REG32(0x40000428)    // prescaler
#define TIM3_ARR        IO_REG32(0x4000042C)    // autoreload
#define TIM3_CCR1       IO_REG32(0x40000434)    // capture/compare 1
#define TIM3_CCR2       IO_REG32(0x40000438)    // capture/compare 2
#define TIM3_CCR3       IO_REG32(0x4000043C)    // capture/compare 3
#define TIM3_CCR4       IO_REG32(0x40000440)    // capture/compare 4
#define TIM3_DCR        IO_REG32(0x40000448)    // DMA control
#define TIM3_DMAR       IO_REG32(0x4000044C)    // DMA address
                                                //
#define TIM4_CR1        IO_REG32(0x40000800)    // control register 1
#define TIM4_CR2        IO_REG32(0x40000804)    // control register 2
#define TIM4_SMCR       IO_REG32(0x40000808)    // slave mode control
#define TIM4_DIER       IO_REG32(0x4000080C)    // DMA/interrupt enable
#define TIM4_SR         IO_REG32(0x40000810)    // status register
#define TIM4_EGR        IO_REG32(0x40000814)    // event generation
#define TIM4_CCMR1      IO_REG32(0x40000818)    // capture/compare mode 1
#define TIM4_CCMR2      IO_REG32(0x4000081C)    // capture/compare mode 2
#define TIM4_CCER       IO_REG32(0x40000820)    // capture/compare event
#define TIM4_CNT        IO_REG32(0x40000824)    // counter
#define TIM4_PSC        IO_REG32(0x40000828)    // prescaler
#define TIM4_ARR        IO_REG32(0x4000082C)    // autoreload
#define TIM4_CCR1       IO_REG32(0x40000834)    // capture/compare 1
#define TIM4_CCR2       IO_REG32(0x40000838)    // capture/compare 2
#define TIM4_CCR3       IO_REG32(0x4000083C)    // capture/compare 3
#define TIM4_CCR4       IO_REG32(0x40000840)    // capture/compare 4
#define TIM4_DCR        IO_REG32(0x40000848)    // DMA control
#define TIM4_DMAR       IO_REG32(0x4000084C)    // DMA address
                                                //
#define TIM5_CR1        IO_REG32(0x40000C00)    // control register 1
#define TIM5_CR2        IO_REG32(0x40000C04)    // control register 2
#define TIM5_SMCR       IO_REG32(0x40000C08)    // slave mode control
#define TIM5_DIER       IO_REG32(0x40000C0C)    // DMA/interrupt enable
#define TIM5_SR         IO_REG32(0x40000C10)    // status register
#define TIM5_EGR        IO_REG32(0x40000C14)    // event generation
#define TIM5_CCMR1      IO_REG32(0x40000C18)    // capture/compare mode 1
#define TIM5_CCMR2      IO_REG32(0x40000C1C)    // capture/compare mode 2
#define TIM5_CCER       IO_REG32(0x40000C20)    // capture/compare event
#define TIM5_CNT        IO_REG32(0x40000C24)    // counter
#define TIM5_PSC        IO_REG32(0x40000C28)    // prescaler
#define TIM5_ARR        IO_REG32(0x40000C2C)    // autoreload
#define TIM5_CCR1       IO_REG32(0x40000C34)    // capture/compare 1
#define TIM5_CCR2       IO_REG32(0x40000C38)    // capture/compare 2
#define TIM5_CCR3       IO_REG32(0x40000C3C)    // capture/compare 3
#define TIM5_CCR4       IO_REG32(0x40000C40)    // capture/compare 4
#define TIM5_DCR        IO_REG32(0x40000C48)    // DMA control
#define TIM5_DMAR       IO_REG32(0x40000C4C)    // DMA address
#define TIM5_OR         IO_REG32(0x40000C50)    // options (calibaration)
                                                //
#define TIM6_CR1        IO_REG32(0x40001000)    // control register 1
#define TIM6_CR2        IO_REG32(0x40001004)    // control register 2
#define TIM6_DIER       IO_REG32(0x4000100C)    // DMA/interrupt enable
#define TIM6_SR         IO_REG32(0x40001010)    // status register
#define TIM6_EGR        IO_REG32(0x40001014)    // event generation
#define TIM6_CNT        IO_REG32(0x40001024)    // counter
#define TIM6_PSC        IO_REG32(0x40001028)    // prescaler
#define TIM6_ARR        IO_REG32(0x4000102C)    // autoreload
                                                //
#define TIM7_CR1        IO_REG32(0x40001400)    // control register 1
#define TIM7_CR2        IO_REG32(0x40001404)    // control register 2
#define TIM7_DIER       IO_REG32(0x4000140C)    // DMA/interrupt enable
#define TIM7_SR         IO_REG32(0x40001410)    // status register
#define TIM7_EGR        IO_REG32(0x40001414)    // event generation
#define TIM7_CNT        IO_REG32(0x40001424)    // counter
#define TIM7_PSC        IO_REG32(0x40001428)    // prescaler
#define TIM7_ARR        IO_REG32(0x4000142C)    // autoreload
                                                //
#define TIM8_CR1        IO_REG32(0x40010400)    // control register 1
#define TIM8_CR2        IO_REG32(0x40010404)    // control register 2
#define TIM8_SMCR       IO_REG32(0x40010408)    // slave mode control
#define TIM8_DIER       IO_REG32(0x4001040C)    // DMA/interrupt enable
#define TIM8_SR         IO_REG32(0x40010410)    // status register
#define TIM8_EGR        IO_REG32(0x40010414)    // event generation
#define TIM8_CCMR1      IO_REG32(0x40010418)    // capture/compare mode 1
#define TIM8_CCMR2      IO_REG32(0x4001041C)    // capture/compare mode 2
#define TIM8_CCER       IO_REG32(0x40010420)    // capture/compare event
#define TIM8_CNT        IO_REG32(0x40010424)    // counter
#define TIM8_PSC        IO_REG32(0x40010428)    // prescaler
#define TIM8_ARR        IO_REG32(0x4001042C)    // autoreload
#define TIM8_RCR        IO_REG32(0x40010430)    // repetition counter
#define TIM8_CCR1       IO_REG32(0x40010434)    // capture/compare 1
#define TIM8_CCR2       IO_REG32(0x40010438)    // capture/compare 2
#define TIM8_CCR3       IO_REG32(0x4001043C)    // capture/compare 3
#define TIM8_CCR4       IO_REG32(0x40010440)    // capture/compare 4
#define TIM8_BDTR       IO_REG32(0x40010444)    // break and dead-time
#define TIM8_DCR        IO_REG32(0x40010448)    // DMA control
#define TIM8_DMAR       IO_REG32(0x4001044C)    // DMA address
                                                //
#define TIM9_CR1        IO_REG32(0x40014000)    // control register 1
#define TIM9_SMCR       IO_REG32(0x40014008)    // slave mode control
#define TIM9_DIER       IO_REG32(0x4001400C)    // DMA/interrupt enable
#define TIM9_SR         IO_REG32(0x40014010)    // status register
#define TIM9_EGR        IO_REG32(0x40014014)    // event generation
#define TIM9_CCMR1      IO_REG32(0x40014018)    // capture/compare mode 1
#define TIM9_CCER       IO_REG32(0x40014020)    // capture/compare event
#define TIM9_CNT        IO_REG32(0x40014024)    // counter
#define TIM9_PSC        IO_REG32(0x40014028)    // prescaler
#define TIM9_ARR        IO_REG32(0x4001402C)    // autoreload
#define TIM9_CCR1       IO_REG32(0x40014034)    // capture/compare 1
#define TIM9_CCR2       IO_REG32(0x40014038)    // capture/compare 2
                                                //
#define TIM10_CR1       IO_REG32(0x40014400)    // control register 1
#define TIM10_DIER      IO_REG32(0x4001440C)    // DMA/interrupt enable
#define TIM10_SR        IO_REG32(0x40014410)    // status register
#define TIM10_EGR       IO_REG32(0x40014414)    // event generation
#define TIM10_CCMR1     IO_REG32(0x40014418)    // capture/compare mode 1
#define TIM10_CCER      IO_REG32(0x40014420)    // capture/compare event
#define TIM10_CNT       IO_REG32(0x40014424)    // counter
#define TIM10_PSC       IO_REG32(0x40014428)    // prescaler
#define TIM10_ARR       IO_REG32(0x4001442C)    // autoreload
#define TIM10_CCR1      IO_REG32(0x40014434)    // capture/compare 1
                                                //
#define TIM11_CR1       IO_REG32(0x40014800)    // control register 1
#define TIM11_DIER      IO_REG32(0x4001480C)    // DMA/interrupt enable
#define TIM11_SR        IO_REG32(0x40014810)    // status register
#define TIM11_EGR       IO_REG32(0x40014814)    // event generation
#define TIM11_CCMR1     IO_REG32(0x40014818)    // capture/compare mode 1
#define TIM11_CCER      IO_REG32(0x40014820)    // capture/compare event
#define TIM11_CNT       IO_REG32(0x40014824)    // counter
#define TIM11_PSC       IO_REG32(0x40014828)    // prescaler
#define TIM11_ARR       IO_REG32(0x4001482C)    // autoreload
#define TIM11_CCR1      IO_REG32(0x40014834)    // capture/compare 1
                                                //
#define TIM12_CR1       IO_REG32(0x40001800)    // control register 1
#define TIM12_SMCR      IO_REG32(0x40001808)    // slave mode control
#define TIM12_DIER      IO_REG32(0x4000180C)    // DMA/interrupt enable
#define TIM12_SR        IO_REG32(0x40001810)    // status register
#define TIM12_EGR       IO_REG32(0x40001814)    // event generation
#define TIM12_CCMR1     IO_REG32(0x40001818)    // capture/compare mode 1
#define TIM12_CCER      IO_REG32(0x40001820)    // capture/compare event
#define TIM12_CNT       IO_REG32(0x40001824)    // counter
#define TIM12_PSC       IO_REG32(0x40001828)    // prescaler
#define TIM12_ARR       IO_REG32(0x4000182C)    // autoreload
#define TIM12_CCR1      IO_REG32(0x40001834)    // capture/compare 1
#define TIM12_CCR2      IO_REG32(0x40001838)    // capture/compare 2
                                                //
#define TIM13_CR1       IO_REG32(0x40001C00)    // control register 1
#define TIM13_DIER      IO_REG32(0x40001C0C)    // DMA/interrupt enable
#define TIM13_SR        IO_REG32(0x40001C10)    // status register
#define TIM13_EGR       IO_REG32(0x40001C14)    // event generation
#define TIM13_CCMR1     IO_REG32(0x40001C18)    // capture/compare mode 1
#define TIM13_CCER      IO_REG32(0x40001C20)    // capture/compare event
#define TIM13_CNT       IO_REG32(0x40001C24)    // counter
#define TIM13_PSC       IO_REG32(0x40001C28)    // prescaler
#define TIM13_ARR       IO_REG32(0x40001C2C)    // autoreload
#define TIM13_CCR1      IO_REG32(0x40001C34)    // capture/compare 1
                                                //
#define TIM14_CR1       IO_REG32(0x40002000)    // control register 1
#define TIM14_DIER      IO_REG32(0x4000200C)    // DMA/interrupt enable
#define TIM14_SR        IO_REG32(0x40002010)    // status register
#define TIM14_EGR       IO_REG32(0x40002014)    // event generation
#define TIM14_CCMR1     IO_REG32(0x40002018)    // capture/compare mode 1
#define TIM14_CCER      IO_REG32(0x40002020)    // capture/compare event
#define TIM14_CNT       IO_REG32(0x40002024)    // counter
#define TIM14_PSC       IO_REG32(0x40002028)    // prescaler
#define TIM14_ARR       IO_REG32(0x4000202C)    // autoreload
#define TIM14_CCR1      IO_REG32(0x40002034)    // capture/compare 1
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_TIM                //
{                                               //
    STM_REG     sTIM_CR1;                       // control register 1
    STM_REG     sTIM_CR2;                       // control register 2
    STM_REG     sTIM_SMCR;                      //
    STM_REG     sTIM_DIER;                      //
    STM_REG     sTIM_SR;                        //
    STM_REG     sTIM_EGR;                       //
    STM_REG     sTIM_CCMR1;                     //
    STM_REG     sTIM_CCMR2;                     //
    STM_REG     sTIM_CCER;                      //
    STM_REG     sTIM_CNT;                       //
    STM_REG     sTIM_PSC;                       //
    STM_REG     sTIM_ARR;                       //
    STM_REG     sTIM_RCR;                       //
    STM_REG     sTIM_CCR1;                      //
    STM_REG     sTIM_CCR2;                      //
    STM_REG     sTIM_CCR3;                      //
    STM_REG     sTIM_CCR4;                      //
    STM_REG     sTIM_BDTR;                      //
    STM_REG     sTIM_DCR;                       //
    STM_REG     sTIM_DMAR;                      //
                                                //
} STM_TIM, *PSTM_TIM;                           //
#pragma pack(pop)                               //

#define TIM1_BASE       ((PSTM_TIM) 0x40010000)
#define TIM2_BASE       ((PSTM_TIM) 0x40000000)
#define TIM3_BASE       ((PSTM_TIM) 0x40000400)
#define TIM4_BASE       ((PSTM_TIM) 0x40000800)
#define TIM5_BASE       ((PSTM_TIM) 0x40000C00)
#define TIM6_BASE       ((PSTM_TIM) 0x40001000)
#define TIM7_BASE       ((PSTM_TIM) 0x40001400)
#define TIM8_BASE       ((PSTM_TIM) 0x40010400)
#define TIM9_BASE       ((PSTM_TIM) 0x40014000)
#define TIM10_BASE      ((PSTM_TIM) 0x40014400)
#define TIM11_BASE      ((PSTM_TIM) 0x40014800)
#define TIM12_BASE      ((PSTM_TIM) 0x40001800)
#define TIM13_BASE      ((PSTM_TIM) 0x40001C00)
#define TIM14_BASE      ((PSTM_TIM) 0x40002000)

//
//  Timer types:
//
//  T1, T8              - T1
//  T2, T3. T4, T5      - T2
//  T6, T7              - T6
//  T9, T12             - T9
//  T10, T11, T13, T14  - T10
//
                                                // TIM_CR1                          T1  T2  T6  T9  T10
#define bTIM_CKD_1              (0<<8)          // tDTS 1:1                         +   +   -   +   +
#define bTIM_CKD_2              (1<<8)          // tDTS 1:2                         +   +   -   +   +
#define bTIM_CKD_4              (2<<8)          // tDTS 1:4                         +   +   -   +   +
#define bTIM_ARPE               (1<<7)          // Auto-reload preload enable       +   +   +   +   +
#define bTIM_CMS0               (0<<5)          // Edge-aligned mode                +   +   -   -   -
#define bTIM_CMS1               (1<<5)          // Center-aligned mode 1            +   +   -   -   -
#define bTIM_CMS2               (2<<5)          // Center-aligned mode 2            +   +   -   -   -
#define bTIM_CMS3               (3<<5)          // Center-aligned mode 3            +   +   -   -   -
#define bTIM_DIR                (1<<4)          // Counter direction                +   +   -   -   -
#define bTIM_OPM                (1<<3)          // One pulse mode                   +   +   +   +   -
#define bTIM_URS                (1<<2)          // Update request source            +   +   +   +   +
#define bTIM_UDIS               (1<<1)          // Update disable                   +   +   +   +   +
#define bTIM_CEN                (1<<0)          // Counter enable                   +   +   +   +   +
                                                //
                                                //
                                                // TIM_CR2                          T1  T2  T6  T9  T10
#define bTIM_OIS4               (1<<14)         // Output Idle state 4 - OC4        +   -   -   -   -
#define bTIM_OIS3N              (1<<13)         // Output Idle state 3 - ~OC3       +   -   -   -   -
#define bTIM_OIS3               (1<<12)         // Output Idle state 3 - OC3        +   -   -   -   -
#define bTIM_OIS2N              (1<<11)         // Output Idle state 2 - ~OC2       +   -   -   -   -
#define bTIM_OIS2               (1<<10)         // Output Idle state 2 - OC2        +   -   -   -   -
#define bTIM_OIS1N              (1<<9)          // Output Idle state 1 - ~OC1       +   -   -   -   -
#define bTIM_OIS1               (1<<8)          // Output Idle state 1 - OC1        +   -   -   -   -
#define bTIM_TI1S               (1<<7)          // TI1 selection                    +   +   -   -   -
                                                // Master mode selection
#define bTIM_MMS_RESET          (0<<4)          //                                  +   +   +   -   -
#define bTIM_MMS_ENABLE         (1<<4)          //                                  +   +   +   -   -
#define bTIM_MMS_UPDATE         (2<<4)          //                                  +   +   +   -   -
#define bTIM_MMS_CMP_PULSE      (3<<4)          //                                  +   +   -   -   -
#define bTIM_MMS_OC1_REF        (4<<4)          //                                  +   +   -   -   -
#define bTIM_MMS_OC2_REF        (5<<4)          //                                  +   +   -   -   -
#define bTIM_MMS_OC3_REF        (6<<4)          //                                  +   +   -   -   -
#define bTIM_MMS_OC4_REF        (7<<4)          //                                  +   +   -   -   -
                                                //
#define bTIM_CCDS               (1<<3)          // Capture/compare DMA select       +   +   -   -   -
#define bTIM_CCUS               (1<<2)          // Capture/compare update select    +   -   -   -   -
#define bTIM_CCPC               (1<<0)          // Capture/compare preloaded        +   -   -   -   -
                                                //
                                                //
                                                // TIM_SMCR                         T1  T2  T6  T9  T10
#define bTIM_ETP                (1<<15)         // External trigger polarity        +   +   -   -   -
#define bTIM_ECE                (1<<14)         // External clock mode 2 enable     +   +   -   -   -
#define bTIM_ETPS_1             (0<<12)         // External trigger prescaler OFF   +   +   -   -   -
#define bTIM_ETPS_2             (1<<12)         // ETRP frequency divided by 2      +   +   -   -   -
#define bTIM_ETPS_4             (2<<12)         // ETRP frequency divided by 4      +   +   -   -   -
#define bTIM_ETPS_8             (3<<12)         // ETPR frequency divided by 8      +   +   -   -   -
                                                //
#define bTIM_ETF_DTS1_N0        (0<<8)          // sampling fDTS, no filter,        +   +   -   -   -
#define bTIM_ETF_CK1_N2         (1<<8)          // sampling fCK_INT, N=2            +   +   -   -   -
#define bTIM_ETF_CK1_N4         (2<<8)          // sampling fCK_INT, N=4            +   +   -   -   -
#define bTIM_ETF_CK1_N8         (3<<8)          // sampling fCK_INT, N=8            +   +   -   -   -
#define bTIM_ETF_DTS2_N6        (4<<8)          // sampling fDTS/2, N=6             +   +   -   -   -
#define bTIM_ETF_DTS2_N8        (5<<8)          // sampling fDTS/2, N=8             +   +   -   -   -
#define bTIM_ETF_DTS4_N6        (6<<8)          // sampling fDTS/4, N=6             +   +   -   -   -
#define bTIM_ETF_DTS4_N8        (7<<8)          // sampling fDTS/4, N=8             +   +   -   -   -
#define bTIM_ETF_DTS8_N6        (8<<8)          // sampling fDTS/8, N=6             +   -   -   -   -
#define bTIM_ETF_DTS8_N8        (9<<8)          // sampling fDTS/8, N=8             +   -   -   -   -
#define bTIM_ETF_DTS16_N5       (10<<8)         // sampling fDTS/16, N=5            +   -   -   -   -
#define bTIM_ETF_DTS16_N6       (11<<8)         // sampling fDTS/16, N=6            +   -   -   -   -
#define bTIM_ETF_DTS16_N8       (12<<8)         // sampling fDTS/16, N=8            +   -   -   -   -
#define bTIM_ETF_DTS32_N5       (13<<8)         // sampling fDTS/32, N=5            +   -   -   -   -
#define bTIM_ETF_DTS32_N6       (14<<8)         // sampling fDTS/32, N=6            +   -   -   -   -
#define bTIM_ETF_DTS32_N8       (15<<8)         // sampling fDTS/32, N=8            +   -   -   -   -
                                                //
#define bTIM_MSM                (1<<7)          // Master/slave mode                +   +   -   -   -
#define bTIM_TS_ITR0            (0<<4)          // Internal Trigger 0               +   +   -   +   -
#define bTIM_TS_ITR1            (1<<4)          // Internal Trigger 1               +   +   -   +   -
#define bTIM_TS_ITR2            (2<<4)          // Internal Trigger 2               +   +   -   +   -
#define bTIM_TS_ITR3            (3<<4)          // Internal Trigger 3               +   +   -   +   -
#define bTIM_TS_TI1F_ED         (4<<4)          // TI1 Edge Detector                +   +   -   +   -
#define bTIM_TS_TI1FP1          (5<<4)          // Filtered Timer Input 1           +   +   -   +   -
#define bTIM_TS_TI1FP2          (6<<4)          // Filtered Timer Input 2           +   +   -   +   -
#define bTIM_TS_ETRF            (7<<4)          // External Trigger input           +   +   -   -   -
                                                //
#define bTIM_SMS_DISABLED       (0<<0)          // Slave mode selection             +   +   -   +   -
#define bTIM_SMS_ENCMODE1       (1<<0)          // Encoder mode 1                   +   +   -   -   -
#define bTIM_SMS_ENCMODE2       (2<<0)          // Encoder mode 2                   +   +   -   -   -
#define bTIM_SMS_ENCMODE3       (3<<0)          // Encoder mode 3                   +   +   -   -   -
#define bTIM_SMS_RESET          (4<<0)          // Reset Mode                       +   +   -   +   -
#define bTIM_SMS_GATED          (5<<0)          // Gated Mode                       +   +   -   +   -
#define bTIM_SMS_TRIGGER        (6<<0)          // Trigger Mode                     +   +   -   +   -
#define bTIM_SMS_EXTCLK         (7<<0)          // External Clock Mode 1            +   +   -   +   -
                                                //
                                                // TIM_DIER                         T1  T2  T6  T9  T10
#define bTIM_TDE                (1<<14)         // Trigger DMA request enable       +   +   -   -   -
#define bTIM_COMDE              (1<<13)         // COM DMA request enable           +   -   -   -   -
#define bTIM_CC4DE              (1<<12)         // CC 4 DMA request enable          +   +   -   -   -
#define bTIM_CC3DE              (1<<11)         // CC 3 DMA request enable          +   +   -   -   -
#define bTIM_CC2DE              (1<<10)         // CC 2 DMA request enable          +   +   -   -   -
#define bTIM_CC1DE              (1<<9)          // CC 1 DMA request enable          +   +   -   -   -
#define bTIM_UDE                (1<<8)          // Update DMA request enable        +   +   +   -   -
#define bTIM_BIE                (1<<7)          // Break interrupt enable           +   -   -   -   -
#define bTIM_TIE                (1<<6)          // Trigger interrupt enable         +   +   -   +   -
#define bTIM_COMIE              (1<<5)          // COM interrupt enable             +   -   -   -   -
#define bTIM_CC4IE              (1<<4)          // CC 4 interrupt enable            +   +   -   -   -
#define bTIM_CC3IE              (1<<3)          // CC 3 interrupt enable            +   +   -   -   -
#define bTIM_CC2IE              (1<<2)          // CC 2 interrupt enable            +   +   -   +   -
#define bTIM_CC1IE              (1<<1)          // CC 1 interrupt enable            +   +   -   +   +
#define bTIM_UIE                (1<<0)          // Update interrupt enable          +   +   +   +   +
                                                //
                                                // TIM_SR                           T1  T2  T6  T9  T10
#define bTIM_CC4OF              (1<<12)         // CC 4 overcapture flag            +   +   -   -   -
#define bTIM_CC3OF              (1<<11)         // CC 3 overcapture flag            +   +   -   -   -
#define bTIM_CC2OF              (1<<10)         // CC 2 overcapture flag            +   +   -   +   -
#define bTIM_CC1OF              (1<<9)          // CC 1 overcapture flag            +   +   -   +   +
#define bTIM_BIF                (1<<7)          // Break interrupt flag             +   -   -   -   -
#define bTIM_TIF                (1<<6)          // Trigger interrupt flag           +   +   -   +   -
#define bTIM_COMIF              (1<<5)          // COM interrupt flag               +   -   -   -   -
#define bTIM_CC4IF              (1<<4)          // CC 4 interrupt flag              +   +   -   -   -
#define bTIM_CC3IF              (1<<3)          // CC 3 interrupt flag              +   +   -   -   -
#define bTIM_CC2IF              (1<<2)          // CC 2 interrupt flag              +   +   -   +   -
#define bTIM_CC1IF              (1<<1)          // CC 1 interrupt flag              +   +   -   +   +
#define bTIM_UIF                (1<<0)          // Update interrupt flag            +   +   +   +   +
                                                //
                                                // TIM_EGR                          T1  T2  T6  T9  T10
#define bTIM_BG                 (1<<7)          // Break generation                 +   -   -   -   -
#define bTIM_TG                 (1<<6)          // Trigger generation               +   +   -   +   -
#define bTIM_COMG               (1<<5)          //                                  +   -   -   -   -
#define bTIM_CC4G               (1<<4)          // CC 4 generation                  +   +   -   -   -
#define bTIM_CC3G               (1<<3)          // CC 3 generation                  +   +   -   -   -
#define bTIM_CC2G               (1<<2)          // CC 2 generation                  +   +   -   +   -
#define bTIM_CC1G               (1<<1)          // CC 1 generation                  +   +   -   +   +
#define bTIM_UG                 (1<<0)          // Update generation                +   +   +   +   +
                                                //
                                                // TIM_CCMR1                        T1  T2  T6  T9  T10
#define bTIM_OC2CE              (1<<15)         // Output Compare 2 clear enable    +   +   -   +   -
#define bTIM_OC2M_NONE          (0<<12)         // Output Compare 2 - None          +   +   -   +   -
#define bTIM_OC2M_ACTIVE        (1<<12)         // Output Compare 2 - Active        +   +   -   +   -
#define bTIM_OC2M_INACTIVE      (2<<12)         // Output Compare 2 - Inactive      +   +   -   +   -
#define bTIM_OC2M_TOGGLE        (3<<12)         // Output Compare 2 - Toggle        +   +   -   +   -
#define bTIM_OC2M_LOW           (4<<12)         // Output Compare 2 - Force Low     +   +   -   +   -
#define bTIM_OC2M_HIGH          (5<<12)         // Output Compare 2 - Force High    +   +   -   +   -
#define bTIM_OC2M_PWM1          (6<<12)         // Output Compare 2 - PWM mode 1    +   +   -   +   -
#define bTIM_OC2M_PWM2          (7<<12)         // Output Compare 2 - PWM mode 2    +   +   -   +   -
#define bTIM_OC2PE              (1<<11)         // Output Compare 2 preload enable  +   +   -   +   -
#define bTIM_OC2FE              (1<<10)         // Output Compare 2 fast enable     +   +   -   +   -
#define bTIM_CC2S_OUT           (0<<8)          // CC 2 configured as output        +   +   -   +   -
#define bTIM_CC2S_TI2           (1<<8)          // CC 2 configured as input, TI2    +   +   -   +   -
#define bTIM_CC2S_TI1           (2<<8)          // CC 2 configured as input, TI1    +   +   -   +   -
#define bTIM_CC2S_TRC           (3<<8)          // CC 2 configured as input, TRC    +   +   -   +   -
                                                //
#define bTIM_OC1CE              (1<<7)          // Output Compare 1 clear enable    +   +   -   +   -
#define bTIM_OC1M_NONE          (0<<4)          // Output Compare 1 - None          +   +   -   +   +
#define bTIM_OC1M_ACTIVE        (1<<4)          // Output Compare 1 - Active        +   +   -   +   +
#define bTIM_OC1M_INACTIVE      (2<<4)          // Output Compare 1 - Inactive      +   +   -   +   +
#define bTIM_OC1M_TOGGLE        (3<<4)          // Output Compare 1 - Toggle        +   +   -   +   +
#define bTIM_OC1M_LOW           (4<<4)          // Output Compare 1 - Force Low     +   +   -   +   +
#define bTIM_OC1M_HIGH          (5<<4)          // Output Compare 1 - Force High    +   +   -   +   +
#define bTIM_OC1M_PWM1          (6<<4)          // Output Compare 1 - PWM mode 1    +   +   -   +   +
#define bTIM_OC1M_PWM2          (7<<4)          // Output Compare 1 - PWM mode 2    +   +   -   +   +
#define bTIM_OC1PE              (1<<3)          // Output Compare 1 preload enable  +   +   -   +   +
#define bTIM_OC1FE              (1<<2)          // Output Compare 1 fast enable     +   +   -   +   +
#define bTIM_CC1S_OUT           (0<<0)          // CC 1 configured as output        +   +   -   +   +
#define bTIM_CC1S_TI2           (1<<0)          // CC 1 configured as input, TI2    +   +   -   +   +
#define bTIM_CC1S_TI1           (2<<0)          // CC 1 configured as input, TI1    +   +   -   +   +
#define bTIM_CC1S_TRC           (3<<0)          // CC 1 configured as input, TRC    +   +   -   +   +
                                                //
                                                //
                                                // TIM_CCMR2                        T1  T2  T6  T9  T10
#define bTIM_OC4CE              (1<<15)         // Output Compare 4 clear enable    +   +   -   -   -
#define bTIM_OC4M_NONE          (0<<12)         // Output Compare 4 - None          +   +   -   -   -
#define bTIM_OC4M_ACTIVE        (1<<12)         // Output Compare 4 - Active        +   +   -   -   -
#define bTIM_OC4M_INACTIVE      (2<<12)         // Output Compare 4 - Inactive      +   +   -   -   -
#define bTIM_OC4M_TOGGLE        (3<<12)         // Output Compare 4 - Toggle        +   +   -   -   -
#define bTIM_OC4M_LOW           (4<<12)         // Output Compare 4 - Force Low     +   +   -   -   -
#define bTIM_OC4M_HIGH          (5<<12)         // Output Compare 4 - Force High    +   +   -   -   -
#define bTIM_OC4M_PWM1          (6<<12)         // Output Compare 4 - PWM mode 1    +   +   -   -   -
#define bTIM_OC4M_PWM2          (7<<12)         // Output Compare 4 - PWM mode 2    +   +   -   -   -
#define bTIM_OC4PE              (1<<11)         // Output Compare 4 preload enable  +   +   -   -   -
#define bTIM_OC4FE              (1<<10)         // Output Compare 4 fast enable     +   +   -   -   -
#define bTIM_CC4S_OUT           (0<<8)          // CC 4 configured as output        +   +   -   -   -
#define bTIM_CC4S_TI2           (1<<8)          // CC 4 configured as input, TI2    +   +   -   -   -
#define bTIM_CC4S_TI1           (2<<8)          // CC 4 configured as input, TI1    +   +   -   -   -
#define bTIM_CC4S_TRC           (3<<8)          // CC 4 configured as input, TRC    +   +   -   -   -
                                                //
#define bTIM_OC3CE              (1<<7)          // Output Compare 3 clear enable    +   +   -   -   -
#define bTIM_OC3M_NONE          (0<<4)          // Output Compare 3 - None          +   +   -   -   -
#define bTIM_OC3M_ACTIVE        (1<<4)          // Output Compare 3 - Active        +   +   -   -   -
#define bTIM_OC3M_INACTIVE      (2<<4)          // Output Compare 3 - Inactive      +   +   -   -   -
#define bTIM_OC3M_TOGGLE        (3<<4)          // Output Compare 3 - Toggle        +   +   -   -   -
#define bTIM_OC3M_LOW           (4<<4)          // Output Compare 3 - Force Low     +   +   -   -   -
#define bTIM_OC3M_HIGH          (5<<4)          // Output Compare 3 - Force High    +   +   -   -   -
#define bTIM_OC3M_PWM1          (6<<4)          // Output Compare 3 - PWM mode 1    +   +   -   -   -
#define bTIM_OC3M_PWM2          (7<<4)          // Output Compare 3 - PWM mode 2    +   +   -   -   -
#define bTIM_OC3PE              (1<<3)          // Output Compare 3 preload enable  +   +   -   -   -
#define bTIM_OC3FE              (1<<2)          // Output Compare 3 fast enable     +   +   -   -   -
#define bTIM_CC3S_OUT           (0<<0)          // CC 3 configured as output        +   +   -   -   -
#define bTIM_CC3S_TI2           (1<<0)          // CC 3 configured as input, TI2    +   +   -   -   -
#define bTIM_CC3S_TI1           (2<<0)          // CC 3 configured as input, TI1    +   +   -   -   -
#define bTIM_CC3S_TRC           (3<<0)          // CC 3 configured as input, TRC    +   +   -   -   -
                                                //
                                                //
                                                // TIM_CCER                         T1  T2  T6  T9  T10
#define bTIM_CC4P               (1<<13)         // CC 4 output polarity             +   +   -   -   -
#define bTIM_CC4E               (1<<12)         // CC 4 output enable               +   +   -   -   -
#define bTIM_CC3NP              (1<<11)         // CCN 3 output polarity            +   -   -   -   -
#define bTIM_CC3NE              (1<<10)         // CCN 3 output enable              +   -   -   -   -
#define bTIM_CC3P               (1<<9)          // CC 3 output polarity             +   +   -   -   -
#define bTIM_CC3E               (1<<8)          // CC 3 output enable               +   +   -   -   -
#define bTIM_CC2NP              (1<<7)          // CCN 2 output polarity            +   -   -   +   -
#define bTIM_CC2NE              (1<<6)          // CCN 2 output enable              +   -   -   -   -
#define bTIM_CC2P               (1<<5)          // CC 2 output polarity             +   +   -   +   -
#define bTIM_CC2E               (1<<4)          // CC 2 output enable               +   +   -   +   -
#define bTIM_CC1NP              (1<<3)          // CCN 1 output polarity            +   -   -   +   +
#define bTIM_CC1NE              (1<<2)          // CCN 1 output enable              +   -   -   -   -
#define bTIM_CC1P               (1<<1)          // CC 1 output polarity             +   +   -   +   +
#define bTIM_CC1E               (1<<0)          // CC 1 output enable               +   +   -   +   +
                                                //
                                                //
                                                //                                  T1  T2  T6  T9  T10
                                                // TIM_RCR                          +   -   -   -   -
                                                // TIM_CCR1                         +   +   -   +   +
                                                // TIM_CCR2                         +   +   -   +   -
                                                // TIM_CCR3                         +   +   -   -   -
                                                // TIM_CCR4                         +   +   -   -   -
                                                // TIM_DMAR                         +   +   -   -   -
                                                // TIM_DCR                          +   +   -   -   -
                                                //
                                                //
                                                // TIM_BDTR                         T1  T2  T6  T9  T10
#define bTIM_MOE                (1<<15)         // Main output enable               +   -   -   -   -
#define bTIM_AOE                (1<<14)         // Automatic output enable          +   -   -   -   -
#define bTIM_BKP                (1<<13)         // Break polarity                   +   -   -   -   -
#define bTIM_BKE                (1<<12)         // Break enable                     +   -   -   -   -
#define bTIM_OSSR               (1<<11)         // Off-state selection Run mode     +   -   -   -   -
#define bTIM_OSSI               (1<<10)         // Off-state selection Idle mode    +   -   -   -   -
#define bTIM_LOCK OFF           (0<<8)          // No bit is write protected        +   -   -   -   -
#define bTIM_LOCK_1             (1<<8)          // Lock level 1                     +   -   -   -   -
#define bTIM_LOCK_2             (2<<8)          // Lock level 2                     +   -   -   -   -
#define bTIM_LOCK_3             (3<<8)          // Lock level 3                     +   -   -   -   -
                                                //
                                                // TIM2_OR
#define bTIM_ITR1_RMP0          (0<<10)         //
#define bTIM_ITR1_RMP1          (1<<10)         //
#define bTIM_ITR1_RMP2          (2<<10)         //
#define bTIM_ITR1_RMP3          (3<<10)         //
                                                // TIM5_OR
#define bTIM_T54_RMP_GPIO       (0<<6)          //
#define bTIM_T54_RMP_LSI        (1<<6)          //
#define bTIM_T54_RMP_LSE        (2<<6)          //
#define bTIM_T54_RMP_RTC        (3<<6)          //
                                                //
//________________________________________________________________
//
// UARTs and USARTs
//
#define UART1_SR        IO_REG32(0x40011000)    // Status
#define UART1_DR        IO_REG32(0x40011004)    // Data
#define UART1_BRR       IO_REG32(0x40011008)    // Baud Rate
#define UART1_CR1       IO_REG32(0x4001100C)    // Control 1
#define UART1_CR2       IO_REG32(0x40011010)    // Control 2
#define UART1_CR3       IO_REG32(0x40011014)    // Control 3
#define UART1_GTPR      IO_REG32(0x40011018)    // Guard Time and Prescaler
                                                //
#define UART2_SR        IO_REG32(0x40004400)    // Status
#define UART2_DR        IO_REG32(0x40004404)    // Data
#define UART2_BRR       IO_REG32(0x40004408)    // Baud Rate
#define UART2_CR1       IO_REG32(0x4000440C)    // Control 1
#define UART2_CR2       IO_REG32(0x40004410)    // Control 2
#define UART2_CR3       IO_REG32(0x40004414)    // Control 3
#define UART2_GTPR      IO_REG32(0x40004418)    // Guard Time and Prescaler
                                                //
#define UART3_SR        IO_REG32(0x40004800)    // Status
#define UART3_DR        IO_REG32(0x40004804)    // Data
#define UART3_BRR       IO_REG32(0x40004808)    // Baud Rate
#define UART3_CR1       IO_REG32(0x4000480C)    // Control 1
#define UART3_CR2       IO_REG32(0x40004810)    // Control 2
#define UART3_CR3       IO_REG32(0x40004814)    // Control 3
#define UART3_GTPR      IO_REG32(0x40004818)    // Guard Time and Prescaler
                                                //
#define UART4_SR        IO_REG32(0x40004C00)    // Status
#define UART4_DR        IO_REG32(0x40004C04)    // Data
#define UART4_BRR       IO_REG32(0x40004C08)    // Baud Rate
#define UART4_CR1       IO_REG32(0x40004C0C)    // Control 1
#define UART4_CR2       IO_REG32(0x40004C10)    // Control 2
#define UART4_CR3       IO_REG32(0x40004C14)    // Control 3
#define UART4_GTPR      IO_REG32(0x40004C18)    // Guard Time and Prescaler
                                                //
#define UART5_SR        IO_REG32(0x40005000)    // Status
#define UART5_DR        IO_REG32(0x40005004)    // Data
#define UART5_BRR       IO_REG32(0x40005008)    // Baud Rate
#define UART5_CR1       IO_REG32(0x4000500C)    // Control 1
#define UART5_CR2       IO_REG32(0x40005010)    // Control 2
#define UART5_CR3       IO_REG32(0x40005014)    // Control 3
#define UART5_GTPR      IO_REG32(0x40005018)    // Guard Time and Prescaler
                                                //
#define UART6_SR        IO_REG32(0x40011400)    // Status
#define UART6_DR        IO_REG32(0x40011404)    // Data
#define UART6_BRR       IO_REG32(0x40011408)    // Baud Rate
#define UART6_CR1       IO_REG32(0x4001140C)    // Control 1
#define UART6_CR2       IO_REG32(0x40011410)    // Control 2
#define UART6_CR3       IO_REG32(0x40011414)    // Control 3
#define UART6_GTPR      IO_REG32(0x40011418)    // Guard Time and Prescaler
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_UART               //
{                                               //
    STM_REG     sUART_SR;                       //
    STM_REG     sUART_DR;                       //
    STM_REG     sUART_BRR;                      //
    STM_REG     sUART_CR1;                      //
    STM_REG     sUART_CR2;                      //
    STM_REG     sUART_CR3;                      //
    STM_REG     sUART_GTPR;                     //
                                                //
} STM_UART, *PSTM_UART;                         //
#pragma pack(pop)                               //

#define UART1_BASE      ((PSTM_UART)    0x40011000)
#define UART2_BASE      ((PSTM_UART)    0x40004400)
#define UART3_BASE      ((PSTM_UART)    0x40004800)
#define UART4_BASE      ((PSTM_UART)    0x40004C00)
#define UART5_BASE      ((PSTM_UART)    0x40005000)
#define UART6_BASE      ((PSTM_UART)    0x40011400)

                                                // UART_SR
#define bUART_CTS               (1<<9)          // CTS toggled flag
#define bUART_LBD               (1<<8)          // LIN break detection flag
#define bUART_TXE               (1<<7)          // Transmit data register empty
#define bUART_TC                (1<<6)          // Transmission complete
#define bUART_RXNE              (1<<5)          // Read data register not empty
#define bUART_IDLE              (1<<4)          // IDLE line detected
#define bUART_ORE               (1<<3)          // Overrun error
#define bUART_NE                (1<<2)          // Noise error flag
#define bUART_FE                (1<<1)          // Framing error
#define bUART_PE                (1<<0)          // Parity error
                                                //
                                                // UART_CR1
#define bUART_OVER8             (1<<15)         // 8 oversample mode
#define bUART_UE                (1<<13)         // UART enable
#define bUART_M                 (1<<12)         // Word length
#define bUART_WAKE              (1<<11)         // Wakeup method
#define bUART_PCE               (1<<10)         // Parity control enable
#define bUART_PS                (1<<9)          // Parity selection
#define bUART_PEIE              (1<<8)          // PE interrupt enable
#define bUART_TXEIE             (1<<7)          // TXE interrupt enable
#define bUART_TCIE              (1<<6)          // Transmission complete interrupt enable
#define bUART_RXNEIE            (1<<5)          // RXNE interrupt enable
#define bUART_IDLEIE            (1<<4)          // IDLE interrupt enable
#define bUART_TE                (1<<3)          // Transmitter enable
#define bUART_RE                (1<<2)          // Receiver enable
#define bUART_RWU               (1<<1)          // Receiver wakeup
#define bUART_SBK               (1<<0)          // Send break
                                                //
                                                // UART_CR2
#define bUART_LINEN             (1<<14)         // LIN mode enable
#define bUART_STOP_1            (0<<12)         // 1 stop bit
#define bUART_STOP_05           (1<<12)         // 0.5 stop bit
#define bUART_STOP_2            (2<<12)         // 2 stop bits
#define bUART_STOP_15           (3<<12)         // 1.5 stop bits
#define bUART_CLKEN             (1<<11)         // Clock pin enable
#define bUART_CPOL              (1<<10)         // Clock polarity
#define bUART_CPHA              (1<<9)          // Clock phase
#define bUART_LBCL              (1<<8)          // Last bit clock pulse
#define bUART_LBDIE             (1<<6)          // LIN break detection interrupt enable
#define bUART_LBDL_10           (0<<5)          // 10 bit break detection (LIN mode)
#define bUART_LBDL_11           (1<<5)          // 11 bit break detection (LIN mode)
#define bUART_ADDR_MASK         (0x0F<<0)       //
                                                //
                                                // UART_CR3
#define bUART_ONEBIT            (1<<11)         // one bit sample method
#define bUART_CTSIE             (1<<10)         // CTS interrupt enable
#define bUART_CTSE              (1<<9)          // CTS flow control enable
#define bUART_RTSE              (1<<8)          // RTS flow control enable
#define bUART_DMAT              (1<<7)          // DMA enable transmitter
#define bUART_DMAR              (1<<6)          // DMA enable receiver
#define bUART_SCEN              (1<<5)          // Smartcard mode enable
#define bUART_NACK              (1<<4)          // Smartcard NACK enable
#define bUART_HDSEL             (1<<3)          // Half-duplex selection
#define bUART_IRLP              (1<<2)          // IrDA low-power
#define bUART_IREN              (1<<1)          // IrDA mode enable
#define bUART_EIE               (1<<0)          // Error interrupt enable
                                                //
                                                // UART_GTPR
#define bUART_GT_MASK           0xFF            //
#define bUART_GT_SHIFT          8               //
#define bUART_PR_MASK           0xFF            //
#define bUART_PR_SHIFT          0               //
//________________________________________________________________
//
//  RTC - Real Time Clock
//
#define RTC_TR          IO_REG32(0x40002800)    // Time Rgister
#define RTC_DR          IO_REG32(0x40002804)    // Date Register
#define RTC_CR          IO_REG32(0x40002808)    // Control Register
#define RTC_ISR         IO_REG32(0x4000280C)    //
#define RTC_PRER        IO_REG32(0x40002810)    //
#define RTC_WUTR        IO_REG32(0x40002814)    //
#define RTC_CALIBR      IO_REG32(0x40002818)    //
#define RTC_ALRMAR      IO_REG32(0x4000281C)    //
#define RTC_ALRMBR      IO_REG32(0x40002820)    //
#define RTC_WPR         IO_REG32(0x40002824)    //
#define RTC_TSTR        IO_REG32(0x40002830)    //
#define RTC_TSDR        IO_REG32(0x40002834)    //
#define RTC_TAFCR       IO_REG32(0x40002840)    //
                                                //
#define RTC_BKP0        IO_REG32(0x40002850)    //
#define RTC_BKP1        IO_REG32(0x40002854)    //
#define RTC_BKP2        IO_REG32(0x40002858)    //
#define RTC_BKP3        IO_REG32(0x4000285C)    //
                                                //
#define RTC_BKP4        IO_REG32(0x40002860)    //
#define RTC_BKP5        IO_REG32(0x40002864)    //
#define RTC_BKP6        IO_REG32(0x40002868)    //
#define RTC_BKP7        IO_REG32(0x4000286C)    //
                                                //
#define RTC_BKP8        IO_REG32(0x40002870)    //
#define RTC_BKP9        IO_REG32(0x40002874)    //
#define RTC_BKP10       IO_REG32(0x40002878)    //
#define RTC_BKP11       IO_REG32(0x4000287C)    //
                                                //
#define RTC_BKP12       IO_REG32(0x40002880)    //
#define RTC_BKP13       IO_REG32(0x40002884)    //
#define RTC_BKP14       IO_REG32(0x40002888)    //
#define RTC_BKP15       IO_REG32(0x4000288C)    //
                                                //
#define RTC_BKP16       IO_REG32(0x40002890)    //
#define RTC_BKP17       IO_REG32(0x40002894)    //
#define RTC_BKP18       IO_REG32(0x40002898)    //
#define RTC_BKP19       IO_REG32(0x4000289C)    //
                                                //
#define bRTC_KEY1               0xCA            //
#define bRTC_KEY2               0x53            //
#define bRTC_KEY3               0xFF            //
                                                // RTC_TR
#define bRTC_PM                 (1<<22)         // AM/PM (12 hour mode)
                                                //
                                                // RTC_CR
#define bRTC_COE                (1<<23)         // Calibration output enable
#define bRTC_OSEL_NONE          (0<<21)         // Output disabled
#define bRTC_OSEL_ALARMA        (1<<21)         // Alarm A output enabled
#define bRTC_OSEL_ALARMB        (2<<21)         // Alarm B output enabled
#define bRTC_OSEL_WKUP          (3<<21)         // Wakeup output enabled
#define bRTC_OSEL_MASK          (3<<21)         //
#define bRTC_POL                (1<<20)         // Output polarity
                                                //
#define bRTC_BKP                (1<<18)         // Backup bit
#define bRTC_SUB1H              (1<<17)         // Subtract 1 hour (winter time change)
#define bRTC_ADD1H              (1<<16)         // Add 1 hour (summer time change)
                                                //
#define bRTC_TSIE               (1<<15)         // Time-stamp interrupt enable
#define bRTC_WUTIE              (1<<14)         // Wakeup timer interrupt enable
#define bRTC_ALRBIE             (1<<13)         // Alarm B interrupt enable
#define bRTC_ALRAIE             (1<<12)         // Alarm A interrupt enable
#define bRTC_TSE                (1<<11)         // Time stamp enable
#define bRTC_WUTE               (1<<10)         // Wakeup timer enable
#define bRTC_ALRBE              (1<<9)          // Alarm B enable
#define bRTC_ALRAE              (1<<8)          // Alarm A enable
#define bRTC_DCE                (1<<7)          // Coarse digital calibration enable
#define bRTC_FMT                (1<<6)          // Hour format (24/AM/PM)
#define bRTC_REFCKON            (1<<4)          // Reference clock detection enable
#define bRTC_TSEDGE             (1<<3)          // Time-stamp event active edge
                                                // Wakeup clock selection
#define bRTC_WKUP_CLK16         (0<<0)          // RTC/16 clock is selected
#define bRTC_WKUP_CLK8          (1<<0)          // RTC/8 clock is selected
#define bRTC_WKUP_CLK4          (2<<0)          // RTC/4 clock is selected
#define bRTC_WKUP_CLK2          (3<<0)          // RTC/2 clock is selected
#define bRTC_WKUP_CKSPRE        (4<<0)          // ck_spre clock is selected
#define bRTC_WKUP_CKSPRE2       (6<<0)          // ck_spre clock is selected + 2^16
                                                //
                                                // bRTC_ISR
#define bRTC_TAMP1F             (1<<13)         // Tamper detection flag
#define bRTC_TSOVF              (1<<12)         // Time-stamp overflow flag
#define bRTC_TSF                (1<<11)         // Time-stamp flag
#define bRTC_WUTF               (1<<10)         // Wakeup timer flag
#define bRTC_ALRBF              (1<<9)          // Alarm B flag
#define bRTC_INIT               (1<<7)          // Initialization mode
#define bRTC_INITF              (1<<6)          // Initialization flag
#define bRTC_RSF                (1<<5)          // Registers synchronization flag
#define bRTC_INITS              (1<<4)          // Initialization status flag
#define bRTC_WUTWF              (1<<2)          // Wakeup timer write flag
#define bRTC_ALRBWF             (1<<1)          // Alarm B write flag
#define bRTC_ALRAWF             (1<<0)          // Alarm A write flag
                                                //
#define bRTC_PREDIVA_MASK       0xFF            // RTC_PRER
#define bRTC_PREDIVA_SHIFT      16              //
#define bRTC_PREDIVS_MASK       0x1FFF          //
#define bRTC_PREDIVS_SHIFT      0               //
                                                //
#define bRTC_MSK4               (1u<<31)        //
#define bRTC_MSK3               (1<<23)         //
#define bRTC_MSK2               (1<<15)         //
#define bRTC_MSK1               (1<<7)          //
#define bRTC_WDSEL              (1<30)          // Week day selection
                                                //
                                                // RTC_TAFCR
#define bRTC_ALARMOUTTYPE       (1<<18)         // AFO_ALARM output type
#define bRTC_TSINSEL            (1<<17)         // TIMESTAMP mapping
#define bRTC_TAMP1INSEL         (1<<16)         // TAMPER1 mapping
#define bRTC_TAMPIE             (1<<2)          // Tamper interrupt enable
#define bRTC_TAMP1TRG           (1<<1)          // Active level for tamper
#define bRTC_TAMP1E             (1<<0)          // Tamper 1 detection enable

//________________________________________________________________
//
//  ADC
//
#define ADC1_SR         IO_REG32(0x40012000)    // Status
#define ADC1_CR1        IO_REG32(0x40012004)    // Control 1
#define ADC1_CR2        IO_REG32(0x40012008)    // Control 2
#define ADC1_SMPR1      IO_REG32(0x4001200C)    // Sample Time 1
#define ADC1_SMPR2      IO_REG32(0x40012010)    // Sample Time 2
#define ADC1_JOFR1      IO_REG32(0x40012014)    // Injected channel data offset 1
#define ADC1_JOFR2      IO_REG32(0x40012018)    // Injected channel data offset 2
#define ADC1_JOFR3      IO_REG32(0x4001201C)    // Injected channel data offset 3
#define ADC1_JOFR4      IO_REG32(0x40012020)    // Injected channel data offset 4
#define ADC1_HTR        IO_REG32(0x40012024)    // Wathcdog high threshold
#define ADC1_LTR        IO_REG32(0x40012028)    // Wathcdog low threshold
#define ADC1_SQR1       IO_REG32(0x4001202C)    // Sequence 1
#define ADC1_SQR2       IO_REG32(0x40012030)    // Sequence 2
#define ADC1_SQR3       IO_REG32(0x40012034)    // Sequence 3
#define ADC1_JSQR       IO_REG32(0x40012038)    // Injected Sequence
#define ADC1_JDR1       IO_REG32(0x4001203C)    // Injected channel data 1
#define ADC1_JDR2       IO_REG32(0x40012040)    // Injected channel data 2
#define ADC1_JDR3       IO_REG32(0x40012044)    // Injected channel data 3
#define ADC1_JDR4       IO_REG32(0x40012048)    // Injected channel data 4
#define ADC1_DR         IO_REG32(0x4001204C)    // Data
                                                //
#define ADC2_SR         IO_REG32(0x40012100)    // Status
#define ADC2_CR1        IO_REG32(0x40012104)    // Control 1
#define ADC2_CR2        IO_REG32(0x40012108)    // Control 2
#define ADC2_SMPR1      IO_REG32(0x4001210C)    // Sample Time 1
#define ADC2_SMPR2      IO_REG32(0x40012110)    // Sample Time 2
#define ADC2_JOFR1      IO_REG32(0x40012114)    // Injected channel data offset 1
#define ADC2_JOFR2      IO_REG32(0x40012118)    // Injected channel data offset 2
#define ADC2_JOFR3      IO_REG32(0x4001211C)    // Injected channel data offset 3
#define ADC2_JOFR4      IO_REG32(0x40012120)    // Injected channel data offset 4
#define ADC2_HTR        IO_REG32(0x40012124)    // Wathcdog high threshold
#define ADC2_LTR        IO_REG32(0x40012128)    // Wathcdog low threshold
#define ADC2_SQR1       IO_REG32(0x4001212C)    // Sequence 1
#define ADC2_SQR2       IO_REG32(0x40012130)    // Sequence 2
#define ADC2_SQR3       IO_REG32(0x40012134)    // Sequence 3
#define ADC2_JSQR       IO_REG32(0x40012138)    // Injected Sequence
#define ADC2_JDR1       IO_REG32(0x4001213C)    // Injected channel data 1
#define ADC2_JDR2       IO_REG32(0x40012140)    // Injected channel data 2
#define ADC2_JDR3       IO_REG32(0x40012144)    // Injected channel data 3
#define ADC2_JDR4       IO_REG32(0x40012148)    // Injected channel data 4
#define ADC2_DR         IO_REG32(0x4001214C)    // Data
                                                //
#define ADC3_SR         IO_REG32(0x40012200)    // Status
#define ADC3_CR1        IO_REG32(0x40012204)    // Control 1
#define ADC3_CR2        IO_REG32(0x40012208)    // Control 2
#define ADC3_SMPR1      IO_REG32(0x4001220C)    // Sample Time 1
#define ADC3_SMPR2      IO_REG32(0x40012210)    // Sample Time 2
#define ADC3_JOFR1      IO_REG32(0x40012214)    // Injected channel data offset 1
#define ADC3_JOFR2      IO_REG32(0x40012218)    // Injected channel data offset 2
#define ADC3_JOFR3      IO_REG32(0x4001221C)    // Injected channel data offset 3
#define ADC3_JOFR4      IO_REG32(0x40012220)    // Injected channel data offset 4
#define ADC3_HTR        IO_REG32(0x40012224)    // Wathcdog high threshold
#define ADC3_LTR        IO_REG32(0x40012228)    // Wathcdog low threshold
#define ADC3_SQR1       IO_REG32(0x4001222C)    // Sequence 1
#define ADC3_SQR2       IO_REG32(0x40012230)    // Sequence 2
#define ADC3_SQR3       IO_REG32(0x40012234)    // Sequence 3
#define ADC3_JSQR       IO_REG32(0x40012238)    // Injected Sequence
#define ADC3_JDR1       IO_REG32(0x4001223C)    // Injected channel data 1
#define ADC3_JDR2       IO_REG32(0x40012240)    // Injected channel data 2
#define ADC3_JDR3       IO_REG32(0x40012244)    // Injected channel data 3
#define ADC3_JDR4       IO_REG32(0x40012248)    // Injected channel data 4
#define ADC3_DR         IO_REG32(0x4001224C)    // Data
                                                //
                                                // Common ADC registers
#define ADC_CSR         IO_REG32(0x40012300)    // Common status
#define ADC_CCR         IO_REG32(0x40012304)    // Common control
#define ADC_CDR         IO_REG32(0x40012308)    // Common data
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_ADC                //
{                                               //
    STM_REG     sADC_SR;                        //
    STM_REG     sADC_CR1;                       //
    STM_REG     sADC_CR2;                       //
    STM_REG     sADC_SMPR1;                     //
    STM_REG     sADC_SMPR2;                     //
    STM_REG     sADC_JOFR1;                     //
    STM_REG     sADC_JOFR2;                     //
    STM_REG     sADC_JOFR3;                     //
    STM_REG     sADC_JOFR4;                     //
    STM_REG     sADC_HTR;                       //
    STM_REG     sADC_LTR;                       //
    STM_REG     sADC_SQR1;                      //
    STM_REG     sADC_SQR2;                      //
    STM_REG     sADC_SQR3;                      //
    STM_REG     sADC_JSQR;                      //
    STM_REG     sADC_JDR1;                      //
    STM_REG     sADC_JDR2;                      //
    STM_REG     sADC_JDR3;                      //
    STM_REG     sADC_JDR4;                      //
    STM_REG     sADC_DR;                        //
                                                //
} STM_ADC, *PSTM_ADC;                           //
#pragma pack(pop)                               //

#define ADC1_BASE       ((PSTM_ADC)     0x40012000)
#define ADC2_BASE       ((PSTM_ADC)     0x40012100)
#define ADC3_BASE       ((PSTM_ADC)     0x40012200)

                                                // ADC_SR
#define bADC_OVR                (1<<5)          // Data overrun
#define bADC_STRT               (1<<4)          // Regular channel Start flag
#define bADC_JSTRT              (1<<3)          // Injected channel Start flag
#define bADC_JEOC               (1<<2)          // Injected channel end of conversion
#define bADC_EOC                (1<<1)          // End of conversion
#define bADC_AWD                (1<<0)          // Analog watchdog flag
                                                //
                                                // ADC_CR1
#define bADC_OVRIE              (1<<26)         // Overrun interrupt enable
#define bADC_RES_MASK           (3<<24)         //
#define bADC_RES_6BIT           (3<<24)         // 6 bit resolution
#define bADC_RES_8BIT           (2<<24)         // 8 bit resolution
#define bADC_RES_10BIT          (1<<24)         // 10 bit resolution
#define bADC_RES_12BIT          (0<<24)         // 12 bit resolution
                                                //
#define bADC_AWDEN              (1<<23)         // Analog watchdog enable on regular channels
#define bADC_JAWDEN             (1<<22)         // Analog watchdog enable on injected channels
#define bADC_DISCNUM_MASK       0x07ul          //
#define bADC_DISCNUM_SHIFT      13              //
#define bADC_JDISCEN            (1<<12)         // Discontinuous mode on injected channels
#define bADC_DISCEN             (1<<11)         // Discontinuous mode on regular channels
#define bADC_JAUTO              (1<<10)         // Automatic Injected Group conversion
#define bADC_AWDSGL             (1<<9)          // Enable the watchdog on a single channel in scan mode
#define bADC_SCAN               (1<<8)          // Scan mode
#define bADC_JEOCIE             (1<<7)          // Interrupt enable for injected channels
#define bADC_AWDIE              (1<<6)          // Analog watchdog interrupt enable
#define bADC_EOCIE              (1<<5)          // Interrupt enable for EOC
#define bADC_AWDCH_MASK         0x1Ful          // Analog watchdog channel select bits
#define bADC_AWDCH_SHIFT        0               //
                                                //
                                                // ADC_CR2
#define bADC_SWSTART                (1<<30)     // Start conversion of regular channels
#define bADC_EXTEN_MASK             (3<<28)     //
#define bADC_EXTEN_NONE             (0<<28)     // trigger detection disabled
#define bADC_EXTEN_RISE             (1<<28)     // trigger detection on the rising edge
#define bADC_EXTEN_FALL             (2<<28)     // trigger detection on the falling edge
#define bADC_EXTEN_BOTH             (3<<28)     // trigger detection on the any edge
                                                //
#define bADC_EXTSEL_T1CC1           (0<<24)     //
#define bADC_EXTSEL_T1CC2           (1<<24)     //
#define bADC_EXTSEL_T1CC3           (2<<24)     //
#define bADC_EXTSEL_T2CC2           (3<<24)     //
#define bADC_EXTSEL_T2CC3           (4<<24)     //
#define bADC_EXTSEL_T2CC4           (5<<24)     //
#define bADC_EXTSEL_T2TRGO          (6<<24)     //
#define bADC_EXTSEL_T3CC1           (7<<24)     //
#define bADC_EXTSEL_T3TRGO          (8<<24)     //
#define bADC_EXTSEL_T4CC4           (9<<24)     //
#define bADC_EXTSEL_T5CC1           (10<<24)    //
#define bADC_EXTSEL_T5CC2           (11<<24)    //
#define bADC_EXTSEL_T5CC3           (12<<24)    //
#define bADC_EXTSEL_T8CC1           (13<<24)    //
#define bADC_EXTSEL_T8TRGO          (14<<24)    //
#define bADC_EXTSEL_EXTI11          (15<<24)    //
                                                //
#define bADC_JSWSTART               (1<<22)     // Start conversion of injected channels
#define bADC_JEXTEN_MASK            (3<<20)     //
#define bADC_JEXTEN_NONE            (0<<20)     // trigger detection disabled
#define bADC_JEXTEN_RISE            (1<<20)     // trigger detection on the rising edge
#define bADC_JEXTEN_FALL            (2<<20)     // trigger detection on the falling edge
#define bADC_JEXTEN_BOTH            (3<<20)     // trigger detection on the any edge
                                                //
#define bADC_JEXTSEL_T1CC4          (0<<16)     //
#define bADC_JEXTSEL_T1TRGO         (1<<16)     //
#define bADC_JEXTSEL_T2CC1          (2<<16)     //
#define bADC_JEXTSEL_T2TRGO         (3<<16)     //
#define bADC_JEXTSEL_T3CC2          (4<<16)     //
#define bADC_JEXTSEL_T3CC4          (5<<16)     //
#define bADC_JEXTSEL_T4CC1          (6<<16)     //
#define bADC_JEXTSEL_T4CC2          (7<<16)     //
#define bADC_JEXTSEL_T4CC3          (8<<16)     //
#define bADC_JEXTSEL_T4TRGO         (9<<16)     //
#define bADC_JEXTSEL_T5CC4          (10<<16)    //
#define bADC_JEXTSEL_T5TRGO         (11<<16)    //
#define bADC_JEXTSEL_T8CC2          (12<<16)    //
#define bADC_JEXTSEL_T8CC3          (13<<16)    //
#define bADC_JEXTSEL_T8CC4          (14<<16)    //
#define bADC_JEXTSEL_EXTI15         (15<<16)    //
                                                //
#define bADC_ALIGN                  (1<<11)     // Data alignment
#define bADC_EOCS                   (1<<10)     // End of conversion selection
#define bADC_DDS                    (1<<9)      // DMA disable selection
#define bADC_DMA                    (1<<8)      // Direct memory access mode
#define bADC_CONT                   (1<<1)      // Continuous conversion
#define bADC_ADON                   (1<<0)      // A/D converter ON / OFF
                                                //
                                                // ADC_SMPR1
                                                // ADC_SMPR2
#define bADC_SMPT_3                 0           // 3 cycles
#define bADC_SMPT_15                1           // 15 cycles
#define bADC_SMPT_28                2           // 28 cycles
#define bADC_SMPT_56                3           // 56 cycles
#define bADC_SMPT_84                4           // 84 cycles
#define bADC_SMPT_112               5           // 112 cycles
#define bADC_SMPT_144               6           // 144 cycles
#define bADC_SMPT_480               7           // 480 cycles
                                                //
                                                // ADC_SQR1
                                                // ADC_SQR2
                                                // ADC_SQR3
#define bADC_SQLEN_SHIFT            20          //
                                                // ADC_JSQR
                                                // ADC_JDR1
                                                // ADC_JDR2
                                                // ADC_JDR3
                                                // ADC_JDR4
                                                // ADC_DR
                                                //
                                                // Common ADC_CSR (readonly)
#define bADC_OVR3                   (1<<21)     // ADC3 OVR copy
#define bADC_STRT3                  (1<<20)     // ADC3 STRT copy
#define bADC_JSTRT3                 (1<<19)     // ADC3 JSTRT copy
#define bADC_JEOC3                  (1<<18)     // ADC3 JEOC copy
#define bADC_EOC3                   (1<<17)     // ADC3 EOC copy
#define bADC_AWD3                   (1<<16)     // ADC3 AWD copy
                                                //
#define bADC_OVR2                   (1<<13)     // ADC2 OVR copy
#define bADC_STRT2                  (1<<12)     // ADC2 STRT copy
#define bADC_JSTRT2                 (1<<11)     // ADC2 JSTRT copy
#define bADC_JEOC2                  (1<<10)     // ADC2 JEOC copy
#define bADC_EOC2                   (1<<9)      // ADC2 EOC copy
#define bADC_AWD2                   (1<<8)      // ADC2 AWD copy
                                                //
#define bADC_OVR1                   (1<<5)      // ADC1 OVR copy
#define bADC_STRT1                  (1<<4)      // ADC1 STRT copy
#define bADC_JSTRT1                 (1<<3)      // ADC1 JSTRT copy
#define bADC_JEOC1                  (1<<2)      // ADC1 JEOC copy
#define bADC_EOC1                   (1<<1)      // ADC1 EOC copy
#define bADC_AWD1                   (1<<0)      // ADC1 AWD copy
                                                //
#define bADC_TSVREFE                (1<<23)     // Temperature sensor and VREFINT enable
#define bADC_VBATE                  (1<<22)     // VBAT channel enable
#define bADC_PRE_2                  (0<<16)     // APB_CLK/2
#define bADC_PRE_4                  (1<<16)     // APB_CLK/4
#define bADC_PRE_6                  (2<<16)     // APB_CLK/6
#define bADC_PRE_8                  (3<<16)     // APB_CLK/8
                                                //
#define bADC_MDMA_NONE              (0<<14)     // DMA mode for multi ADC
#define bADC_MDMA_1                 (1<<14)     //
#define bADC_MDMA_2                 (2<<14)     //
#define bADC_MDMA_3                 (3<<14)     //
#define bADC_MDMA_DDS               (3<<13)     //
                                                //
#define bADC_DELAY_5                (0<<8)      //
#define bADC_DELAY_6                (1<<8)      //
#define bADC_DELAY_7                (2<<8)      //
#define bADC_DELAY_8                (3<<8)      //
#define bADC_DELAY_9                (4<<8)      //
#define bADC_DELAY_10               (5<<8)      //
#define bADC_DELAY_11               (6<<8)      //
#define bADC_DELAY_12               (7<<8)      //
#define bADC_DELAY_13               (8<<8)      //
#define bADC_DELAY_14               (9<<8)      //
#define bADC_DELAY_15               (10<<8)     //
#define bADC_DELAY_16               (11<<8)     //
#define bADC_DELAY_17               (12<<8)     //
#define bADC_DELAY_18               (13<<8)     //
#define bADC_DELAY_19               (14<<8)     //
#define bADC_DELAY_20               (15<<8)     //
                                                //
#define bADC_MULTI_IND              (0<<0)      // independent mode
//________________________________________________________________
//
// DMA controllers
//
#define DMA1_LISR       IO_REG32(0x40026000)    // Low Interrupt Status
#define DMA1_HISR       IO_REG32(0x40026004)    // High Interrupt Status
#define DMA1_LIFCR      IO_REG32(0x40026008)    // Low Interrupt Clear
#define DMA1_HIFCR      IO_REG32(0x4002600C)    // High Interrupt Clear
                                                //
#define DMA1_S0CR       IO_REG32(0x40026010)    // Stream 0 control
#define DMA1_S0NDTR     IO_REG32(0x40026014)    // Stream 0 number of data
#define DMA1_S0PAR      IO_REG32(0x40026018)    // Peripheral address
#define DMA1_S0M0AR     IO_REG32(0x4002601C)    // Memory address 0
#define DMA1_S0M1AR     IO_REG32(0x40026020)    // Memory address 1
#define DMA1_S0FCR      IO_REG32(0x40026024)    // Stream 0 FIFO control
                                                //
#define DMA1_S1CR       IO_REG32(0x40026028)    // Stream 1 control
#define DMA1_S1NDTR     IO_REG32(0x4002602C)    // Stream 1 number of data
#define DMA1_S1PAR      IO_REG32(0x40026030)    // Peripheral address
#define DMA1_S1M0AR     IO_REG32(0x40026034)    // Memory address 0
#define DMA1_S1M1AR     IO_REG32(0x40026038)    // Memory address 1
#define DMA1_S1FCR      IO_REG32(0x4002603C)    // Stream 1 FIFO control
                                                //
#define DMA1_S2CR       IO_REG32(0x40026040)    // Stream 2 control
#define DMA1_S2NDTR     IO_REG32(0x40026044)    // Stream 2 number of data
#define DMA1_S2PAR      IO_REG32(0x40026048)    // Peripheral address
#define DMA1_S2M0AR     IO_REG32(0x4002604C)    // Memory address 0
#define DMA1_S2M1AR     IO_REG32(0x40026050)    // Memory address 1
#define DMA1_S2FCR      IO_REG32(0x40026054)    // Stream 2 FIFO control
                                                //
#define DMA1_S3CR       IO_REG32(0x40026058)    // Stream 3 control
#define DMA1_S3NDTR     IO_REG32(0x4002605C)    // Stream 3 number of data
#define DMA1_S3PAR      IO_REG32(0x40026060)    // Peripheral address
#define DMA1_S3M0AR     IO_REG32(0x40026064)    // Memory address 0
#define DMA1_S3M1AR     IO_REG32(0x40026068)    // Memory address 1
#define DMA1_S3FCR      IO_REG32(0x4002606C)    // Stream 3 FIFO control
                                                //
#define DMA1_S4CR       IO_REG32(0x40026070)    // Stream 4 control
#define DMA1_S4NDTR     IO_REG32(0x40026074)    // Stream 4 number of data
#define DMA1_S4PAR      IO_REG32(0x40026078)    // Peripheral address
#define DMA1_S4M0AR     IO_REG32(0x4002607C)    // Memory address 0
#define DMA1_S4M1AR     IO_REG32(0x40026080)    // Memory address 1
#define DMA1_S4FCR      IO_REG32(0x40026084)    // Stream 4 FIFO control
                                                //
#define DMA1_S5CR       IO_REG32(0x40026088)    // Stream 5 control
#define DMA1_S5NDTR     IO_REG32(0x4002608C)    // Stream 5 number of data
#define DMA1_S5PAR      IO_REG32(0x40026090)    // Peripheral address
#define DMA1_S5M0AR     IO_REG32(0x40026094)    // Memory address 0
#define DMA1_S5M1AR     IO_REG32(0x40026098)    // Memory address 1
#define DMA1_S5FCR      IO_REG32(0x4002609C)    // Stream 5 FIFO control
                                                //
#define DMA1_S6CR       IO_REG32(0x400260A0)    // Stream 6 control
#define DMA1_S6NDTR     IO_REG32(0x400260A4)    // Stream 6 number of data
#define DMA1_S6PAR      IO_REG32(0x400260A8)    // Peripheral address
#define DMA1_S6M0AR     IO_REG32(0x400260AC)    // Memory address 0
#define DMA1_S6M1AR     IO_REG32(0x400260B0)    // Memory address 1
#define DMA1_S6FCR      IO_REG32(0x400260B4)    // Stream 6 FIFO control
                                                //
#define DMA1_S7CR       IO_REG32(0x400260B8)    // Stream 7 control
#define DMA1_S7NDTR     IO_REG32(0x400260BC)    // Stream 7 number of data
#define DMA1_S7PAR      IO_REG32(0x400260C0)    // Peripheral address
#define DMA1_S7M0AR     IO_REG32(0x400260C4)    // Memory address 0
#define DMA1_S7M1AR     IO_REG32(0x400260C8)    // Memory address 1
#define DMA1_S7FCR      IO_REG32(0x400260CC)    // Stream 7 FIFO control
                                                //
#define DMA2_LISR       IO_REG32(0x40026400)    // Low Interrupt Status
#define DMA2_HISR       IO_REG32(0x40026404)    // High Interrupt Status
#define DMA2_LIFCR      IO_REG32(0x40026408)    // Low Interrupt Clear
#define DMA2_HIFCR      IO_REG32(0x4002640C)    // High Interrupt Clear
                                                //
#define DMA2_S0CR       IO_REG32(0x40026410)    // Stream 0 control
#define DMA2_S0NDTR     IO_REG32(0x40026414)    // Stream 0 number of data
#define DMA2_S0PAR      IO_REG32(0x40026418)    // Peripheral address
#define DMA2_S0M0AR     IO_REG32(0x4002641C)    // Memory address 0
#define DMA2_S0M1AR     IO_REG32(0x40026420)    // Memory address 1
#define DMA2_S0FCR      IO_REG32(0x40026424)    // Stream 0 FIFO control
                                                //
#define DMA2_S1CR       IO_REG32(0x40026428)    // Stream 1 control
#define DMA2_S1NDTR     IO_REG32(0x4002642C)    // Stream 1 number of data
#define DMA2_S1PAR      IO_REG32(0x40026430)    // Peripheral address
#define DMA2_S1M0AR     IO_REG32(0x40026434)    // Memory address 0
#define DMA2_S1M1AR     IO_REG32(0x40026438)    // Memory address 1
#define DMA2_S1FCR      IO_REG32(0x4002643C)    // Stream 1 FIFO control
                                                //
#define DMA2_S2CR       IO_REG32(0x40026440)    // Stream 2 control
#define DMA2_S2NDTR     IO_REG32(0x40026444)    // Stream 2 number of data
#define DMA2_S2PAR      IO_REG32(0x40026448)    // Peripheral address
#define DMA2_S2M0AR     IO_REG32(0x4002644C)    // Memory address 0
#define DMA2_S2M1AR     IO_REG32(0x40026450)    // Memory address 1
#define DMA2_S2FCR      IO_REG32(0x40026454)    // Stream 2 FIFO control
                                                //
#define DMA2_S3CR       IO_REG32(0x40026458)    // Stream 3 control
#define DMA2_S3NDTR     IO_REG32(0x4002645C)    // Stream 3 number of data
#define DMA2_S3PAR      IO_REG32(0x40026460)    // Peripheral address
#define DMA2_S3M0AR     IO_REG32(0x40026464)    // Memory address 0
#define DMA2_S3M1AR     IO_REG32(0x40026468)    // Memory address 1
#define DMA2_S3FCR      IO_REG32(0x4002646C)    // Stream 3 FIFO control
                                                //
#define DMA2_S4CR       IO_REG32(0x40026470)    // Stream 4 control
#define DMA2_S4NDTR     IO_REG32(0x40026474)    // Stream 4 number of data
#define DMA2_S4PAR      IO_REG32(0x40026478)    // Peripheral address
#define DMA2_S4M0AR     IO_REG32(0x4002647C)    // Memory address 0
#define DMA2_S4M1AR     IO_REG32(0x40026480)    // Memory address 1
#define DMA2_S4FCR      IO_REG32(0x40026484)    // Stream 4 FIFO control
                                                //
#define DMA2_S5CR       IO_REG32(0x40026488)    // Stream 5 control
#define DMA2_S5NDTR     IO_REG32(0x4002648C)    // Stream 5 number of data
#define DMA2_S5PAR      IO_REG32(0x40026490)    // Peripheral address
#define DMA2_S5M0AR     IO_REG32(0x40026494)    // Memory address 0
#define DMA2_S5M1AR     IO_REG32(0x40026498)    // Memory address 1
#define DMA2_S5FCR      IO_REG32(0x4002649C)    // Stream 5 FIFO control
                                                //
#define DMA2_S6CR       IO_REG32(0x400264A0)    // Stream 6 control
#define DMA2_S6NDTR     IO_REG32(0x400264A4)    // Stream 6 number of data
#define DMA2_S6PAR      IO_REG32(0x400264A8)    // Peripheral address
#define DMA2_S6M0AR     IO_REG32(0x400264AC)    // Memory address 0
#define DMA2_S6M1AR     IO_REG32(0x400264B0)    // Memory address 1
#define DMA2_S6FCR      IO_REG32(0x400264B4)    // Stream 6 FIFO control
                                                //
#define DMA2_S7CR       IO_REG32(0x400264B8)    // Stream 7 control
#define DMA2_S7NDTR     IO_REG32(0x400264BC)    // Stream 7 number of data
#define DMA2_S7PAR      IO_REG32(0x400264C0)    // Peripheral address
#define DMA2_S7M0AR     IO_REG32(0x400264C4)    // Memory address 0
#define DMA2_S7M1AR     IO_REG32(0x400264C8)    // Memory address 1
#define DMA2_S7FCR      IO_REG32(0x400264CC)    // Stream 7 FIFO control
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_DMA_CH             //
{                                               //
    STM_REG     sDMA_CR;                        //
    STM_REG     sDMA_NDTR;                      //
    STM_REG     sDMA_PAR;                       //
    STM_REG     sDMA_M0AR;                      //
    STM_REG     sDMA_M1AR;                      //
    STM_REG     sDMA_FCR;                       //
                                                //
} STM_DMA_CH, *PSTM_DMA_CH;                     //
#pragma pack(pop)                               //
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_DMA                //
{                                               //
    STM_REG     sDMA_LISR;                      //
    STM_REG     sDMA_HISR;                      //
    STM_REG     sDMA_LIFCR;                     //
    STM_REG     sDMA_HIFCR;                     //
    STM_DMA_CH  sDMA_CHAN[8];                   //
                                                //
} STM_DMA, *PSTM_DMA;                           //
#pragma pack(pop)                               //

#define DMA1_BASE       ((PSTM_DMA)     0x40026000)
#define DMA2_BASE       ((PSTM_DMA) 0x40026400)

#define DMA1_CH0_BASE   ((PSTM_DMA_CH) 0x40026010)
#define DMA1_CH1_BASE   ((PSTM_DMA_CH) 0x40026028)
#define DMA1_CH2_BASE   ((PSTM_DMA_CH) 0x40026040)
#define DMA1_CH3_BASE   ((PSTM_DMA_CH) 0x40026058)
#define DMA1_CH4_BASE   ((PSTM_DMA_CH) 0x40026070)
#define DMA1_CH5_BASE   ((PSTM_DMA_CH) 0x40026088)
#define DMA1_CH6_BASE   ((PSTM_DMA_CH) 0x400260A0)
#define DMA1_CH7_BASE   ((PSTM_DMA_CH) 0x400260B8)

#define DMA2_CH0_BASE   ((PSTM_DMA_CH) 0x40026410)
#define DMA2_CH1_BASE   ((PSTM_DMA_CH) 0x40026428)
#define DMA2_CH2_BASE   ((PSTM_DMA_CH) 0x40026440)
#define DMA2_CH3_BASE   ((PSTM_DMA_CH) 0x40026458)
#define DMA2_CH4_BASE   ((PSTM_DMA_CH) 0x40026470)
#define DMA2_CH5_BASE   ((PSTM_DMA_CH) 0x40026488)
#define DMA2_CH6_BASE   ((PSTM_DMA_CH) 0x400264A0)
#define DMA2_CH7_BASE   ((PSTM_DMA_CH) 0x400264B8)

#define bDMA_FEIF0              (1<<0)          // FIFO error interrupt flag
#define bDMA_DMEIF0             (1<<2)          // Direct mode error interrupt flag
#define bDMA_TEIF0              (1<<3)          // Transfer error interrupt flag
#define bDMA_HTIF0              (1<<4)          // Half transferr interrupt flag
#define bDMA_TCIF0              (1<<5)          // Transferr complete interrupt flag
                                                //
#define bDMA_FEIF1              (1<<6)          //
#define bDMA_DMEIF1             (1<<8)          //
#define bDMA_TEIF1              (1<<9)          //
#define bDMA_HTIF1              (1<<10)         //
#define bDMA_TCIF1              (1<<11)         //
                                                //
#define bDMA_FEIF2              (1<<16)         //
#define bDMA_DMEIF2             (1<<18)         //
#define bDMA_TEIF2              (1<<19)         //
#define bDMA_HTIF2              (1<<20)         //
#define bDMA_TCIF2              (1<<21)         //
                                                //
#define bDMA_FEIF3              (1<<22)         //
#define bDMA_DMEIF3             (1<<24)         //
#define bDMA_TEIF3              (1<<25)         //
#define bDMA_HTIF3              (1<<26)         //
#define bDMA_TCIF3              (1<<27)         //
                                                //
#define bDMA_FEIF4              (1<<0)          //
#define bDMA_DMEIF4             (1<<2)          //
#define bDMA_TEIF4              (1<<3)          //
#define bDMA_HTIF4              (1<<4)          //
#define bDMA_TCIF4              (1<<5)          //
                                                //
#define bDMA_FEIF5              (1<<6)          //
#define bDMA_DMEIF5             (1<<8)          //
#define bDMA_TEIF5              (1<<9)          //
#define bDMA_HTIF5              (1<<10)         //
#define bDMA_TCIF5              (1<<11)         //
                                                //
#define bDMA_FEIF6              (1<<16)         //
#define bDMA_DMEIF6             (1<<18)         //
#define bDMA_TEIF6              (1<<19)         //
#define bDMA_HTIF6              (1<<20)         //
#define bDMA_TCIF6              (1<<21)         //
                                                //
#define bDMA_FEIF7              (1<<22)         //
#define bDMA_DMEIF7             (1<<24)         //
#define bDMA_TEIF7              (1<<25)         //
#define bDMA_HTIF7              (1<<26)         //
#define bDMA_TCIF7              (1<<27)         //
                                                //
                                                // DMA_SxCR
                                                //
#define bDMA_CHSEL_MASK         0x07            //
#define bDMA_CHSEL_SHIFT        25              //
                                                //
#define bDMA_MBURST_SINGLE      (0<<23)         //
#define bDMA_MBURST_INCR4       (1<<23)         //
#define bDMA_MBURST_INCR8       (2<<23)         //
#define bDMA_MBURST_INCR16      (3<<23)         //
                                                //
#define bDMA_PBURST_SINGLE      (0<<21)         //
#define bDMA_PBURST_INCR4       (1<<21)         //
#define bDMA_PBURST_INCR8       (2<<21)         //
#define bDMA_PBURST_INCR16      (3<<21)         //
                                                //
#define bDMA_CT                 (1<<19)         // Current Target
#define bDMA_DBM                (1<<18)         // Double Buffer Mode
#define bDMA_PL_LOW             (0<<16)         // Channel priority Low
#define bDMA_PL_MEDIUM          (1<<16)         // Channel priority Medium
#define bDMA_PL_HIGH            (2<<16)         // Channel priority High
#define bDMA_PL_HIGHEST         (3<<16)         // Channel priority Highest
#define bDMA_PINCOS             (1<<15)         // Peripheral Increment Offset Size
                                                //
#define bDMA_MSIZE_8            (0<<13)         // Memory size 8 bits
#define bDMA_MSIZE_16           (1<<13)         // Memory size 16 bits
#define bDMA_MSIZE_32           (2<<13)         // Memory size 32 bits
                                                //
#define bDMA_PSIZE_8            (0<<11)         // Peripheral size 8 bits
#define bDMA_PSIZE_16           (1<<11)         // Peripheral size 16 bits
#define bDMA_PSIZE_32           (2<<11)         // Peripheral size 32 bits
                                                //
#define bDMA_MINC               (1<<10)         // Memory increment mode
#define bDMA_PINC               (1<<9)          // Peripheral increment mode
#define bDMA_CIRC               (1<<8)          // Circular mode
#define bDMA_MEM2MEM            (1<<7)          // Memory to memory mode
#define bDMA_DIR                (1<<6)          // Data transfer direction
#define bDMA_PFCTRL             (1<<5)          // Peripheral Flow Control
                                                //
#define bDMA_TCIE               (1<<4)          // Transfer completer interrupt enable
#define bDMA_HTIE               (1<<3)          // Half transfer interrupt enable
#define bDMA_TEIE               (1<<2)          // Transfer error interrupt enable
#define bDMA_DMEIE              (1<<1)          // Direct Mode Erroe interrupt enable
#define bDMA_EN                 (1<<0)          // Channel enable
                                                //
                                                // DMA_SxFCR
#define bDMA_FEIE               (1<<7)          // FIFO error interrupt enable
#define bDMA_FS_0               (0<<3)          // 0 < fifo_level < 1/4
#define bDMA_FS_14              (1<<3)          // 1/4 <= fifo_level < 1/2
#define bDMA_FS_12              (2<<3)          // 1/2 <= fifo level < 3/4
#define bDMA_FS_34              (3<<3)          // 3/4 <= fifo level < full
#define bDMA_FS_EMPTY           (4<<3)          // fifo empty
#define bDMA_FS_FULL            (5<<3)          // fifo full
#define bDMA_DMDIS              (1<<2)          // Direct mode disable
                                                //
                                                // Request threshold
#define bDMA_TH_14              (0<<0)          // 1/4 full FIFO
#define bDMA_TH_12              (1<<0)          // 1/2 full FIFO
#define bDMA_TH_34              (2<<0)          // 3/4 full FIFO
#define bDMA_TH_44              (3<<0)          // full FIFO

//________________________________________________________________
//
//  SPI controllers
//
#define SPI1_CR1        IO_REG32(0x40013000)    // Control 1
#define SPI1_CR2        IO_REG32(0x40013004)    // Control 2
#define SPI1_SR         IO_REG32(0x40013008)    // Status
#define SPI1_DR         IO_REG32(0x4001300C)    // DATA
#define SPI1_CRCPR      IO_REG32(0x40013010)    // CRC polynom
#define SPI1_RXCRCR     IO_REG32(0x40013014)    // Rx CRC
#define SPI1_TXCRCR     IO_REG32(0x40013018)    // Tx CRC
#define SPI1_I2SCFGR    IO_REG32(0x4001301C)    // I2S config
#define SPI1_I2SPR      IO_REG32(0x40013020)    // I2S prescaler
                                                //
#define SPI2_CR1        IO_REG32(0x40003800)    // Control 1
#define SPI2_CR2        IO_REG32(0x40003804)    // Control 2
#define SPI2_SR         IO_REG32(0x40003808)    // Status
#define SPI2_DR         IO_REG32(0x4000380C)    // DATA
#define SPI2_CRCPR      IO_REG32(0x40003810)    // CRC polynom
#define SPI2_RXCRCR     IO_REG32(0x40003814)    // Rx CRC
#define SPI2_TXCRCR     IO_REG32(0x40003818)    // Tx CRC
#define SPI2_I2SCFGR    IO_REG32(0x4000381C)    // I2S config
#define SPI2_I2SPR      IO_REG32(0x40003820)    // I2S prescaler
                                                //
#define SPI3_CR1        IO_REG32(0x40003C00)    // Control 1
#define SPI3_CR2        IO_REG32(0x40003C04)    // Control 2
#define SPI3_SR         IO_REG32(0x40003C08)    // Status
#define SPI3_DR         IO_REG32(0x40003C0C)    // DATA
#define SPI3_CRCPR      IO_REG32(0x40003C10)    // CRC polynom
#define SPI3_RXCRCR     IO_REG32(0x40003C14)    // Rx CRC
#define SPI3_TXCRCR     IO_REG32(0x40003C18)    // Tx CRC
#define SPI3_I2SCFGR    IO_REG32(0x40003C1C)    // I2S config
#define SPI3_I2SPR      IO_REG32(0x40003C20)    // I2S prescaler
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_SPI                //
{                                               //
    STM_REG     sSPI_CR1;                       //
    STM_REG     sSPI_CR2;                       //
    STM_REG     sSPI_SR;                        //
    STM_REG     sSPI_DR;                        //
    STM_REG     sSPI_CRCPR;                     //
    STM_REG     sSPI_RXCRCR;                    //
    STM_REG     sSPI_TXCRCR;                    //
    STM_REG     sSPI_I2SCFGR;                   //
    STM_REG     sSPI_I2SPR;                     //
                                                //
} STM_SPI, *PSTM_SPI;                           //
#pragma pack(pop)                               //

#define SPI1_BASE       ((PSTM_SPI) 0x40013000)
#define SPI2_BASE       ((PSTM_SPI) 0x40003800)
#define SPI3_BASE       ((PSTM_SPI) 0x40003C00)
                                                //
                                                // SPI_CR1
#define bSPI_BIDIMODE           (1<<15)         // Bidirectional data mode enable
#define bSPI_BIDIOE             (1<<14)         // Output enable in bidirectional mode
#define bSPI_CRCEN              (1<<13)         // Hardware CRC calculation enable
#define bSPI_CRCNEXT            (1<<12)         // CRC transfer next
#define bSPI_DFF_8              (0<<11)         // Data frame format (8 bit)
#define bSPI_DFF_16             (1<<11)         // Data frame format (16 bit)
#define bSPI_RXONLY             (1<<10)         // Receive only
#define bSPI_SSM                (1<<9)          // Software slave management
#define bSPI_SSI                (1<<8)          // Internal slave select
#define bSPI_LSBFIRST           (1<<7)          // Frame format
#define bSPI_SPE                (1<<6)          // SPI enable
                                                //
#define bSPI_PCLK_2             (0<<3)          // baudrate = PCLK/2
#define bSPI_PCLK_4             (1<<3)          // baudrate = PCLK/4
#define bSPI_PCLK_8             (2<<3)          // baudrate = PCLK/8
#define bSPI_PCLK_16            (3<<3)          // baudrate = PCLK/16
#define bSPI_PCLK_32            (4<<3)          // baudrate = PCLK/32
#define bSPI_PCLK_64            (5<<3)          // baudrate = PCLK/64
#define bSPI_PCLK_128           (6<<3)          // baudrate = PCLK/128
#define bSPI_PCLK_256           (7<<3)          // baudrate = PCLK/256
#define bSPI_PCLK_MASK          (7<<3)          //
                                                //
#define bSPI_MSTR               (1<<2)          // Master selection
#define bSPI_CPOL               (1<<1)          // Clock polarity
#define bSPI_CPHA               (1<<0)          // Clock phase
                                                //
                                                // SPI_CR2
#define bSPI_TXEIE              (1<<7)          // Tx buffer empty interrupt enable
#define bSPI_RXNEIE             (1<<6)          // RX buffer not empty interrupt enable
#define bSPI_ERRIE              (1<<5)          // Error interrupt enable
#define bSPI_FRF                (1<<4)          // Motorola/TI frame format
#define bSPI_SSOE               (1<<2)          // SS output enable
#define bSPI_TXDMAEN            (1<<1)          // Tx buffer DMA enable
#define bSPI_RXDMAEN            (1<<0)          // Rx buffer DMA enable
                                                //
                                                // SPI_SR
#define bSPI_TIFRFE             (1<<8)          // TI frame format error
#define bSPI_BSY                (1<<7)          // Busy flag
#define bSPI_OVR                (1<<6)          // Overrun flag
#define bSPI_MODF               (1<<5)          // Mode fault
#define bSPI_CRCERR             (1<<4)          // CRC error flag
#define bSPI_UDR                (1<<3)          // Underrun flag
#define bSPI_CHSIDE             (1<<2)          // Channel side
#define bSPI_TXE                (1<<1)          // Transmit buffer empty
#define bSPI_RXNE               (1<<0)          // Receive buffer not empty
                                                //
                                                // SPI_I2SCFGR
#define bSPI_I2SMOD             (1<<11)         // I2S mode selection
#define bSPI_I2SE               (1<<10)         // I2S Enable
#define bSPI_I2SCFG_STX         (0<<8)          // Slave - transmit
#define bSPI_I2SCFG_SRX         (1<<8)          // Slave - receive
#define bSPI_I2SCFG_MTX         (2<<8)          // Master - transmit
#define bSPI_I2SCFG_MRX         (3<<8)          // Master - receive
#define bSPI__PCMSYNC           (1<<7)          // PCM frame synchronization
#define bSPI_I2SSTD_I2S         (0<<4)          // I2S Phillips standard.
#define bSPI_I2SSTD_MSB         (1<<4)          // MSB justified standard (left justified)
#define bSPI_I2SSTD_LSB         (2<<4)          // LSB justified standard (right justified)
#define bSPI_I2SSTD_PCM         (3<<4)          // PCM standard
#define bSPI_CKPOL              (1<<3)          // Steady state clock polarity
#define bSPI_DATLEN_16          (0<<1)          // 16-bit data length
#define bSPI_DATLEN_24          (1<<1)          // 24-bit data length
#define bSPI_DATLEN_32          (2<<1)          // 32-bit data length
#define bSPI_CHLEN_16           (0<<0)          // 16-bit wide
#define bSPI_CHLEN_32           (1<<0)          // 32-bit wide
                                                //
                                                // SPI_I2SPR
#define bSPI_MCKOE              (1<<9)          // Master clock output enable
#define bSPI_ODD                (1<<8)          // Odd factor for the prescaler
                                                //
//________________________________________________________________
//
//  Ethernet controller
//
#define ETH_MACCR       IO_REG32(0x40028000)    // MAC configuration
#define ETH_MACFFR      IO_REG32(0x40028004)    // MAC frame filter
#define ETH_MACHT_HR    IO_REG32(0x40028008)    // MAC hash table high
#define ETH_MACHT_LR    IO_REG32(0x4002800C)    // MAC hash table low
#define ETH_MACMII_AR   IO_REG32(0x40028010)    // MII management control
#define ETH_MACMII_DR   IO_REG32(0x40028014)    // MII management data
#define ETH_MACFCR      IO_REG32(0x40028018)    // MAC flow control
#define ETH_MACVLANTR   IO_REG32(0x4002801C)    // MAC VLAN tag
#define ETH_MACRWUFFR   IO_REG32(0x40028028)    // remote wakeup frame filter
#define ETH_MACPMTCSR   IO_REG32(0x4002802C)    // PMT control and status
#define ETH_MACDBGR     IO_REG32(0x40028034)    // MAC debug status
#define ETH_MACSR       IO_REG32(0x40028038)    // MAC interrupt status register
#define ETH_MACIMR      IO_REG32(0x4002803C)    // MAC interrupt mask register
#define ETH_MACA0_HR    IO_REG32(0x40028040)    //
#define ETH_MACA0_LR    IO_REG32(0x40028044)    //
#define ETH_MACA1_HR    IO_REG32(0x40028048)    //
#define ETH_MACA1_LR    IO_REG32(0x4002804C)    //
#define ETH_MACA2_HR    IO_REG32(0x40028050)    //
#define ETH_MACA2_LR    IO_REG32(0x40028054)    //
#define ETH_MACA3_HR    IO_REG32(0x40028058)    //
#define ETH_MACA3_LR    IO_REG32(0x4002805C)    //
#define ETH_MMC_CR      IO_REG32(0x40028100)    // MMC control
#define ETH_MMC_RIR     IO_REG32(0x40028104)    // MMC receive interrupt
#define ETH_MMC_TIR     IO_REG32(0x40028108)    // MMC transmit interrupt register
#define ETH_MMC_RIMR    IO_REG32(0x4002810C)    // MMC receive interrupt mask register
#define ETH_MMC_TIMR    IO_REG32(0x40028110)    // MMC transmit interrupt mask register
#define ETH_MMC_TGFSC   IO_REG32(0x4002814C)    // MMC transmitted good frames after a single collision
#define ETH_MMC_TGFMSC  IO_REG32(0x40028150)    // MMC transmitted good frames after more than a single collision
#define ETH_MMC_TGF     IO_REG32(0x40028168)    // MMC transmitted good frames counter
#define ETH_MMC_RFCE    IO_REG32(0x40028194)    // MMC received frames with CRC error
#define ETH_MMC_RFAE    IO_REG32(0x40028198)    // MMC received frames with alignment error
#define ETH_MMC_RGUF    IO_REG32(0x400281C4)    // MMC received good unicast frames counter
#define ETH_PTP_TSCR    IO_REG32(0x40028700)    // PTP time stamp control
#define ETH_PTP_SSIR    IO_REG32(0x40028704)    // PTP subsecond increment
#define ETH_PTP_TS_HR   IO_REG32(0x40028708)    // PTP time stamp high
#define ETH_PTP_TS_LR   IO_REG32(0x4002870C)    // PTP time stamp low
#define ETH_PTP_TS_HUR  IO_REG32(0x40028710)    // PTP time stamp high update
#define ETH_PTP_TS_LUR  IO_REG32(0x40028714)    // PTP time stamp low update
#define ETH_PTP_TS_AR   IO_REG32(0x40028718)    // PTP time stamp addend register
#define ETH_PTP_TT_HR   IO_REG32(0x4002871C)    // PTP target time high register
#define ETH_PTP_TT_LR   IO_REG32(0x40028720)    // PTP target time low register
#define ETH_PTP_TSSR    IO_REG32(0x40028728)    // PTP time stamp status register
#define ETH_PTP_PPSCR   IO_REG32(0x4002872C)    // PTP PPS control register
#define ETH_DMA_BMR     IO_REG32(0x40029000)    // DMA bus mode register
#define ETH_DMA_TPDR    IO_REG32(0x40029004)    // DMA transmit poll demand register
#define ETH_DMA_RPDR    IO_REG32(0x40029008)    // DMA receive poll demand register
#define ETH_DMA_RDLAR   IO_REG32(0x4002900C)    // DMA receive descriptor list address
#define ETH_DMA_TDLAR   IO_REG32(0x40029010)    // DMA transmit descriptor list address
#define ETH_DMA_SR      IO_REG32(0x40029014)    // DMA status
#define ETH_DMA_OMR     IO_REG32(0x40029018)    // DMA operation mode
#define ETH_DMA_IER     IO_REG32(0x4002901C)    // DMA interrupt enable
#define ETH_DMA_MFBOCR  IO_REG32(0x40029020)    // DMA missed frame and buffer overflow counter
#define ETH_DMA_RSWTR   IO_REG32(0x40029024)    // DMA receive status watchdog timer
#define ETH_DMACH_TDR   IO_REG32(0x40029048)    // DMA current host transmit descriptor
#define ETH_DMACH_RDR   IO_REG32(0x4002904C)    // DMA current host receive descriptor
#define ETH_DMACH_TBAR  IO_REG32(0x40029050)    // DMA current host transmit buffer address
#define ETH_DMACH_RBAR  IO_REG32(0x40029054)    // DMA current host receive buffer address

#define EMAC_BASE   ((PSTM_EMAC)    0x40028000)

                                                // ETH_MACCR
#define bEMAC_CSTF              (1<<25)         // CRC stripping for type frames
#define bEMAC_WD                (1<<23)         // receiver watchdog disable
#define bEMAC_JD                (1<<22)         // jabber disable
#define bEMAC_IFG_96            (0<<17)         // Interframe gap 96 bits
#define bEMAC_IFG_88            (1<<17)         // Interframe gap 88 bits
#define bEMAC_IFG_80            (2<<17)         // Interframe gap 80 bits
#define bEMAC_IFG_72            (3<<17)         // Interframe gap 72 bits
#define bEMAC_IFG_64            (4<<17)         // Interframe gap 64 bits
#define bEMAC_IFG_56            (5<<17)         // Interframe gap 56 bits
#define bEMAC_IFG_48            (6<<17)         // Interframe gap 48 bits
#define bEMAC_IFG_40            (7<<17)         // Interframe gap 40 bits
#define bEMAC_IFG_MASK          (7<<17)         //
                                                //
#define bEMAC_CSD               (1<<16)         // Carrier sense disable
#define bEMAC_FES               (1<<14)         // Fast Ethernet speed
#define bEMAC_ROD               (1<<13)         // Receive own disable
#define bEMAC_LM                (1<<12)         // Loopback mode
#define bEMAC_DM                (1<<11)         // Duplex mode
#define bEMAC_IPCO              (1<<10)         // IPv4 checksum offload
#define bEMAC_RD                (1<<9)          // Retry disable
#define bEMAC_APCS              (1<<7)          // Automatic pad/CRC stripping
#define bEMAC_BL_10             (0<<5)          // Back-off limit min (n, 10)
#define bEMAC_BL_8              (1<<5)          // Back-off limit min (n, 8)
#define bEMAC_BL_4              (2<<5)          // Back-off limit min (n, 4)
#define bEMAC_BL_1              (3<<5)          // Back-off limit min (n, 1)
#define bEMAC_BL_MASK           (3<<5)          // Back-off limit min (n, 1)
#define bEMAC_DC                (1<<4)          // Deferral check
#define bEMAC_TE                (1<<3)          // Transmitter enable
#define bEMAC_RE                (1<<2)          // Receiver enable
                                                //
                                                // ETH_MACFFR
#define bEMAC_RA                (1u<<31)        // Receive all
#define bEMAC_HPF               (1<<10)         // Hash or perfect filter
#define bEMAC_SAF               (1<<9)          // Source address filter
#define bEMAC_SAIF              (1<<8)  `       // Source address inverse filtering
                                                //
#define bEMAC_PCF_NONE          (0<<6)          // no control frames passed to app
#define bEMAC_PCF_EXPAUSE       (1<<6)          // all except pause control frames
#define bEMAC_PCF_ALL           (2<<6)          // all control frames
#define bEMAC_PCF_FADDR         (3<<6)          // all frames passed addr filter
                                                //
#define bEMAC_BFD               (1<<5)          // Broadcast frames disable
#define bEMAC_PAM               (1<<4)          // Pass all multicast
#define bEMAC_DAIF              (1<<3)          // Destination address inverse filtering
#define bEMAC_HM                (1<<2)          // Hash multicast
#define bEMAC_HU                (1<<1)          // Hash unicast
#define bEMAC_PM                (1<<0)          // Promiscuous mode
                                                //
                                                // ETH_MACMII_AR
#define bEMAC_PA_SHIFT          11              // PHY address field shift
#define bEMAC_PA_MASK           0x1F            // PHY address field mask
#define bEMAC_MR_SHIFT          6               // PHY register field shift
#define bEMAC_MR_MASK           0x1F            // PHY register field mask
#define bEMAC_CLK_42            (0<<2)          // management clock HCLK/42
#define bEMAC_CLK_62            (1<<2)          // management clock HCLK/62
#define bEMAC_CLK_16            (2<<2)          // management clock HCLK/16
#define bEMAC_CLK_26            (3<<2)          // management clock HCLK/26
#define bEMAC_MWOP              (1<<1)          // write operation
#define bEMAC_MROP              (0<<1)          // read operation
#define bEMAC_MBSY              (1<<0)          // management busy
                                                //
                                                // ETH_MACFCR
#define bEMAC_PT_SHIFT          16              // pause time value
#define bEMAC_PT_MASK           0xFFFFul        // pause time mask
#define bEMAC_ZQPD              (1<<7)          // Zero-quanta pause disable
#define bEMAC_PLT_4             (0<<4)          // Pause low threshold 4 slots
#define bEMAC_PLT_28            (1<<4)          // Pause low threshold 28 slots
#define bEMAC_PLT_144           (2<<4)          // Pause low threshold 144 slots
#define bEMAC_PLT_256           (3<<4)          // Pause low threshold 256 slots
#define bEMAC_UPFD              (1<<3)          // Unicast pause frame detect
#define bEMAC_RFCE              (1<<2)          // Receive flow control enable
#define bEMAC_TFCE              (1<<1)          // Transmit flow control enable
#define bEMAC_FCB               (1<<0)          // Flow control busy/back pressure activate
                                                //
                                                // ETH_MACVLANTR
#define bEMAC_VLANTC            (1<<16)         // 12-bit VLAN tag comparison
                                                //
                                                // ETH_MACPMTCSR
#define bEMAC_WFFRPR            (1u<<31)        // Wakeup frame filter register pointer reset
#define bEMAC_GU                (1<<9)          // Global unicast wakeup frame
#define bEMAC_WFR               (1<<6)          // Wakeup frame received
#define bEMAC_MPR               (1<<5)          // Magic packet received
#define bEMAC_WFE               (1<<2)          // Wakeup frame enable
#define bEMAC_MPE               (1<<1)          // Magic Packet enable
#define bEMAC_PD                (1<<0)          // Power down
                                                //
                                                // ETH_MACDBGR
#define bEMAC_TFF               (1<<25)         // Tx FIFO full
#define bEMAC_TFNE              (1<<24)         // Tx FIFO not empty
#define bEMAC_TFWA              (1<<22)         // Tx FIFO write active
                                                // Tx FIFO read status
#define bEMAC_TFRS_IDLE         (0<<20)         // Tx FIFO Idle state
#define bEMAC_TFRS_READ         (1<<20)         // Tx FIFO Read state (transferring data to the MAC)
#define bEMAC_TFRS_STATUS       (2<<20)         // Tx FIFO Waiting for TxStatus from MAC
#define bEMAC_TFRS_DONE         (3<<20)         // Tx FIFO Writing the received TxStatus
                                                //
#define bEMAC_MTP               (1<<19)         // MAC transmitter in pause
                                                // MAC transmit frame controller status
#define bEMAC_MTFCS_IDLE        (0<<17)         //
#define bEMAC_MTFCS_STATUS      (1<<17)         // Waiting for Status of previous frame or IFG/backoff period
#define bEMAC_MTFCS_PAUSE       (2<<17)         // Generating and transmitting a Pause control frame
#define bEMAC_MTFCS_TX          (3<<17)         // Transferring input frame for transmission
                                                //
#define bEMAC_MMTEA             (1<<16)         // MAC MII transmit engine active
                                                // Rx FIFO fill level
#define bEMAC_RFFL_EMPTY        (0<<8)          // Rx FIFO empty
#define bEMAC_RFFL_BELOW        (1<<8)          // Rx FIFO below flow-control de-activate threshold
#define bEMAC_RFFL_ABOVE        (2<<8)          // Rx FIFO above flow-control activate threshold
#define bEMAC_RFFL_FULL         (3<<8)          // Rx FIFO full
                                                // Rx FIFO read controller status
#define bEMAC_RFRCS_IDLE        (0<<5)          // Rx FIFO idle
#define bEMAC_RFRCS_READ        (1<<5)          // Rx FIFO Reading frame data
#define bEMAC_RFRCS_STATUS      (2<<5)          // Rx FIFO Reading frame status (or time-stamp)
#define bEMAC_RFRCS_FLUSH       (3<<5)          // Rx FIFO Flushing the frame data and status
                                                //
#define bEMAC_RFWRA             (1<<4)          // Rx FIFO write controller active
#define bEMAC_MSFRCS            (1<<2)          // MAC small FIFO read controller status
#define bEMAC_MSFWCS            (1<<1)          // MAC small FIFO write controller status
#define bEMAC_MMRPEA            (1<<0)          // MAC MII receive protocol engine active
                                                //
                                                // ETH_MACSR
#define bEMAC_TSTS              (1<<9)          // Time stamp trigger status
#define bEMAC_MMCTS             (1<<6)          // MMC transmit status
#define bEMAC_MMCRS             (1<<5)          // MMC receive status
#define bEMAC_MMCS              (1<<4)          // MMC status
#define bEMAC_PMTS              (1<<3)          // PMT status
                                                //
                                                // ETH_MACIMR
#define bEMAC_TSTIM             (1<<9)          // Time stamp trigger interrupt mask
#define bEMAC_PMTIM             (1<<3)          // PMT interrupt mask

                                                // ETH_MACA0_HR
                                                // ETH_MACA1_HR
                                                // ETH_MACA2_HR
                                                // ETH_MACA3_HR
#define bEMAC_AE                (1u<<31)        // Address enable
#define bEMAC_SA                (1<<30)         // Source address
#define bEMAC_MBC_5             (1<<29)         // mask byte comparison
#define bEMAC_MBC_4             (1<<28)         //
#define bEMAC_MBC_3             (1<<27)         //
#define bEMAC_MBC_2             (1<<26)         //
#define bEMAC_MBC_1             (1<<25)         //
#define bEMAC_MBC_0             (1<<24)         //
                                                //
                                                // ETH_MMC_CR
#define bEMAC_MCFHP             (1<<5)          // MMC counter Full-Half preset
#define bEMAC_MCP               (1<<4)          // MMC counter preset
#define bEMAC_MCF               (1<<3)          // MMC counter freeze
#define bEMAC_ROR               (1<<2)          // Reset on read
#define bEMAC_CSR               (1<<1)          // Counter stop rollover
#define bEMAC_CR                (1<<0)          // Counter reset
                                                //
                                                // ETH_MMC_RIR
#define bEMAC_RGUFS             (1<<17)         // Received Good Unicast Frames Status
#define bEMAC_RFAES             (1<<6)          // Received frames alignment error status
#define bEMAC_RFCES             (1<<5)          // Received frames CRC error status
                                                //
                                                // ETH_MMC_TIR
#define bEMAC_TGFS              (1<<21)         // Transmitted good frames status
#define bEMAC_TGFMSCS           (1<<15)         // Transmitted good frames more single collision status
#define bEMAC_TGFSCS            (1<<14)         // Transmitted good frames single collision status
                                                //
                                                // ETH_MMC_RIMR
#define bEMAC_RGUFM             (1<<17)         // Received good unicast frames mask
#define bEMAC_RFAEM             (1<<6)          // Received frames alignment error mask
#define bEMAC_RFCEM             (1<<5)          // Received frame CRC error mask
                                                //
                                                // ETH_MMC_TIMR
#define bEMAC_TGFM              (1<<21)         // Transmitted good frames mask
#define bEMAC_TGFMSCM           (1<<15)         // Transmitted good frames more single collision mask
#define bEMAC_TGFSCM            (1<<14)         // Transmitted good frames single collision mask
                                                //
                                                // ETH_PTP_TSCR
#define bEMAC_TSPFFMAE          (1<<18)         // Time stamp PTP frame filtering MAC address enable
#define bEMAC_ORDINARY_CLK      (0<<16)         // Ordinary clock
#define bEMAC_BOUNDARY_CLK      (1<<16)         // Boundary clock
#define bEMAC_END2END_CLK       (2<<16)         // End-to-end transparent clock
#define bEMAC_PEER2PEER_CLK     (3<<16)         // Peer-to-peer transparent clock
#define bEMAC_TSSMRME           (1<<15)         // Time stamp snapshot for message relevant to master enable
#define bEMAC_TSSEME            (1<<14)         // Time stamp snapshot for event message enable
#define bEMAC_TSSIPV4FE         (1<<13)         // Time stamp snapshot for IPv4 frames enable
#define bEMAC_TSSIPV6FE         (1<<12)         // Time stamp snapshot for IPv6 frames enable
#define bEMAC_TSSPTPOEFE        (1<<11)         // Time stamp snapshot for PTP over ethernet frames enable
#define bEMAC_TSPTPPSV2E        (1<<10)         // Time stamp PTP packet snooping for version2 format enable
#define bEMAC_TSSSR             (1<<9)          // Time stamp subsecond rollover: digital or binary rollover control
#define bEMAC_TSSARFE           (1<<8)          // Time stamp snapshot for all received frames enable
#define bEMAC_TSARU             (1<<5)          // Time stamp addend register update
#define bEMAC_TSITE             (1<<4)          // Time stamp interrupt trigger enable
#define bEMAC_TSSTU             (1<<3)          // Time stamp system time update
#define bEMAC_TSSTI             (1<<2)          // Time stamp system time initialize
#define bEMAC_TSFCU             (1<<1)          // Time stamp fine or coarse update
#define bEMAC_TSE               (1<<0)          // Time stamp enable
                                                //
                                                // ETH_PTP_TSSR
#define bEMAC_TSTTR             (1<<1)          // Time stamp target time reached
#define bEMAC_TSSO              (1<<0)          // Time stamp second overflow
                                                //
                                                // ETH_PTP_PPSCR
#define bEMAC_PPSFREQ_1         0               // 1 Hz
#define bEMAC_PPSFREQ_2         1               // 2 Hz
#define bEMAC_PPSFREQ_4         2               // 4 Hz
#define bEMAC_PPSFREQ_8         3               // 8 Hz
#define bEMAC_PPSFREQ_16        4               // 16 Hz
#define bEMAC_PPSFREQ_32        5               // 32 Hz
#define bEMAC_PPSFREQ_64        6               // 64 Hz
#define bEMAC_PPSFREQ_128       7               // 128 Hz
#define bEMAC_PPSFREQ_256       8               // 256 Hz
#define bEMAC_PPSFREQ_512       9               // 512 Hz
#define bEMAC_PPSFREQ_1024      10              // 1024 Hz
#define bEMAC_PPSFREQ_2048      11              // 2048 Hz
#define bEMAC_PPSFREQ_4096      12              // 4096 Hz
#define bEMAC_PPSFREQ_8192      13              // 8192 Hz
#define bEMAC_PPSFREQ_16384     14              // 16384 Hz
#define bEMAC_PPSFREQ_32768     15              // 32768 Hz
                                                //
                                                // ETH_DMA_BMR
#define bEMAC_MB                (1<<26)         // Mixed burst
#define bEMAC_AAB               (1<<25)         // Address-aligned beats
#define bEMAC_FPM               (1<<24)         // 4xPBL mode
#define bEMAC_USP               (1<<23)         // Use separate PBL
                                                //
#define bEMAC_RDP_1             (1<<17)         // Rx DMA PBL
#define bEMAC_RDP_2             (2<<17)         //
#define bEMAC_RDP_4             (4<<17)         //
#define bEMAC_RDP_8             (8<<17)         //
#define bEMAC_RDP_16            (16<<17)        //
#define bEMAC_RDP_32            (32<<17)        //
#define bEMAC_RDP_SHIFT         17              //
                                                //
#define bEMAC_FB                (1<<16)         // Fixed burst
#define bEMAC_RTPR_11           (0<<14)         // Rx Tx priority ratio
#define bEMAC_RTPR_21           (1<<14)         //
#define bEMAC_RTPR_31           (2<<14)         //
#define bEMAC_RTPR_41           (3<<14)         //
                                                //
#define bEMAC_PBL_1             (1<<8)          // Programmable burst length
#define bEMAC_PBL_2             (2<<8)          //
#define bEMAC_PBL_4             (4<<8)          //
#define bEMAC_PBL_8             (8<<8)          //
#define bEMAC_PBL_16            (16<<8)         //
#define bEMAC_PBL_32            (32<<8)         //
#define bEMAC_PBL_SHIFT         8               //
                                                //
#define bEMAC_EDFE              (1<<7)          // Enhanced descriptor format enable
#define bEMAC_DSL_MASK          0x1Ful          // Descriptor skip length
#define bEMAC_DSL_SHIFT         2               //
#define bEMAC_DA                (1<<1)          // DMA Arbitration
#define bEMAC_SR                (1<<0)          // Software reset
                                                //
                                                // ETH_DMA_SR
#define bEMAC_DTSTS             (1<<29)         // Time stamp trigger status
#define bEMAC_DPMTS             (1<<28)         // PMT status
#define bEMAC_DMMCS             (1<<27)         // MMC status
                                                //
#define bEMAC_EBS_RXTX          (1<<23)         // Error bits status
#define bEMAC_EBS_TX            (1<<23)         // Error during data transfer by TxDMA
#define bEMAC_EBS_RX            (0<<23)         // Error during data transfer by RxDMA
#define bEMAC_EBS_RW            (1<<24)         //
#define bEMAC_EBS_READ          (1<<24)         // Error during read transfer
#define bEMAC_EBS_WRITE         (0<<24)         // Error during write transfer
#define bEMAC_EBS_DB            (1<<25)         //
#define bEMAC_EBS_DESC          (1<<25)         // Error during descriptor access
#define bEMAC_EBS_BUF           (0<<25)         // Error during data buffer access
#define bEMAC_EBS_MASK          (7<<23)         //
                                                //
                                                // Transmit process state
#define bEMAC_TPS_STOP          (0<<20)         // Reset or Stop Transmit Command issued
#define bEMAC_TPS_DESC          (1<<20)         // Fetching transmit transfer descriptor
#define bEMAC_TPS_STATUS        (2<<20)         // Waiting for status
#define bEMAC_TPS_DATA          (3<<20)         // Reading Data from host memory buffer
#define bEMAC_TPS_SUSPEND       (6<<20)         // Transmit descriptor unavailable or transmit buffer underflow
#define bEMAC_TPS_CLOSE         (7<<20)         // Closing transmit descriptor
#define bEMAC_TPS_MASK          (7<<20)         //
                                                // Receive process state
#define bEMAC_RPS_STOP          (0<<17)         // Reset or Stop Receive Command issued
#define bEMAC_RPS_DESC          (1<<17)         // Fetching receive transfer descriptor
#define bEMAC_RPS_WAIT          (3<<17)         // Waiting for receive packet
#define bEMAC_RPS_SUSPEND       (4<<17)         // Receive descriptor unavailable
#define bEMAC_RPS_CLOSE         (5<<17)         // Closing receive descriptor
#define bEMAC_RPS_DATA          (7<<17)         // Transferring the receive packet data to host memory
#define bEMAC_RPS_MASK          (7<<17)         //
                                                //
#define bEMAC_NIS               (1<<16)         // Normal interrupt summary
#define bEMAC_AIS               (1<<15)         // Abnormal interrupt summary
#define bEMAC_ERS               (1<<14)         // Early receive status
#define bEMAC_FBES              (1<<13)         // Fatal bus error status
#define bEMAC_ETS               (1<<10)         // Early transmit status
#define bEMAC_RWTS              (1<<9)          // Receive watchdog timeout status
#define bEMAC_RPSS              (1<<8)          // Receive process stopped status
#define bEMAC_RBUS              (1<<7)          // Receive buffer unavailable status
#define bEMAC_RS                (1<<6)          // Receive status
#define bEMAC_TUS               (1<<5)          // Transmit underflow status
#define bEMAC_ROS               (1<<4)          // Receive overflow status
#define bEMAC_TJTS              (1<<3)          // Transmit jabber timeout status
#define bEMAC_TBUS              (1<<2)          // Transmit buffer unavailable status
#define bEMAC_TPSS              (1<<1)          // Transmit process stopped status
#define bEMAC_TS                (1<<0)          // Transmit status
                                                //
                                                // ETH_DMA_OMR
#define bEMAC_DTCEFD            (1<<26)         // Dropping of TCP/IP checksum error frames disable
#define bEMAC_RSF               (1<<25)         // Receive store and forward
#define bEMAC_DFRF              (1<<24)         // Disable flushing of received frames
#define bEMAC_TSF               (1<<21)         // Transmit store and forward
#define bEMAC_FTF               (1<<20)         // Flush transmit FIFO
                                                //
#define bEMAC_TTC_64            (0<<14)         // Transmit threshold control
#define bEMAC_TTC_128           (1<<14)         //
#define bEMAC_TTC_192           (2<<14)         //
#define bEMAC_TTC_256           (3<<14)         //
#define bEMAC_TTC_40            (4<<14)         //
#define bEMAC_TTC_32            (5<<14)         //
#define bEMAC_TTC_24            (6<<14)         //
#define bEMAC_TTC_16            (7<<14)         //
                                                //
#define bEMAC_ST                (1<<13)         // Start/stop transmission
#define bEMAC_FEF               (1<<7)          // Forward error frames
#define bEMAC_FUGF              (1<<6)          // Forward undersized good frames
                                                // Receive threshold control
#define bEMAC_RTC_64            (0<<3)          //
#define bEMAC_RTC_32            (1<<3)          //
#define bEMAC_RTC_96            (2<<3)          //
#define bEMAC_RTC_128           (3<<3)          //
                                                //
#define bEMAC_OSF               (1<<2)          // Operate on second frame
#define bEMAC_STR               (1<<1)          // Start/stop receive
                                                //
                                                // ETH_DMA_IER
#define bEMAC_NISE              (1<<16)         // Normal interrupt summary enable
#define bEMAC_AISE              (1<<15)         // Abnormal interrupt summary enable
#define bEMAC_ERIE              (1<<14)         // Early receive interrupt enable
#define bEMAC_FBEIE             (1<<13)         // Fatal bus error interrupt enable
#define bEMAC_ETIE              (1<<10)         // Early transmit interrupt enable
#define bEMAC_RWTIE             (1<<9)          // Receive watchdog timeout interrupt enable
#define bEMAC_RPSIE             (1<<8)          // Receive process stopped interrupt enable
#define bEMAC_RBUIE             (1<<7)          // Receive buffer unavailable interrupt enable
#define bEMAC_RIE               (1<<6)          // Receive interrupt enable
#define bEMAC_TUIE              (1<<5)          // Underflow interrupt enable
#define bEMAC_ROIE              (1<<4)          // Overflow interrupt enable
#define bEMAC_TJTIE             (1<<3)          // Transmit jabber timeout interrupt enable
#define bEMAC_TBUIE             (1<<2)          // Transmit buffer unavailable interrupt enable
#define bEMAC_TPSIE             (1<<1)          // Transmit process stopped interrupt enable
#define bEMAC_TIE               (1<<0)          // Transmit interrupt enable
                                                //
                                                // ETH_DMAMF_BOCR
#define bEMAC_OFOC              (1<<28)         // Overflow bit for FIFO overflow counter
#define bEMAC_MFA_MASK          0x3FFul         //
#define bEMAC_MFA_SHIFT         17              //
#define bEMAC_MFC_MASK          0xFFFFul        //
#define bEMAC_MFC_SHIFT         0               //
                                                //
#pragma pack(push, 4)                           // улучшенный (полный) передающий дескриптор
typedef struct _EMAC_TDES                       //
{                                               //
    volatile unsigned long status;              //
    volatile unsigned long count;               //
    volatile unsigned long buf1;                //
    union                                       //
    {                                           //
        volatile unsigned long buf2;            //
        volatile unsigned long next;            //
    };                                          //
    volatile unsigned long reserved[2];         //
    volatile unsigned long timestamp[2];        //
                                                //
} EMAC_TDES, *PEMAC_TDES;                       //
#pragma pack(pop)                               //
                                                //
#pragma pack(push, 4)                           // улучшенный (полный) приемный дескриптор
typedef struct _EMAC_RDES                       //
{                                               //
    volatile unsigned long status;              //
    volatile unsigned long count;               //
    volatile unsigned long buf1;                //
    union                                       //
    {                                           //
        volatile unsigned long buf2;            //
        volatile unsigned long next;            //
    };                                          //
    volatile unsigned long extstat;             //
    volatile unsigned long reserved;            //
    volatile unsigned long timestamp[2];        //
                                                //
} EMAC_RDES, *PEMAC_RDES;                       //
#pragma pack(pop)                               //
                                                //
                                                // EMAC_TDES
#define bEMAC_DES_IC            (1<<30)         // Interrupt on completion
#define bEMAC_DES_LS            (1<<29)         // Last segment
#define bEMAC_DES_FS            (1<<28)         // First segment
#define bEMAC_DES_DC            (1<<27)         // Disable CRC
#define bEMAC_DES_DP            (1<<26)         // Disable pad (if shorter 64 bytes)
#define bEMAC_DES_TTSE          (1<<25)         // Transmit time stamp enable
                                                //
#define bEMAC_DES_CIC_NONE      (0<<22)         // Checksum Insertion disabled
#define bEMAC_DES_CIC_HDR       (1<<22)         // Only IP header checksum calculation
#define bEMAC_DES_CIC_IP        (2<<22)         // IP header and payload checksums calculation (excl pseudo)
#define bEMAC_DES_CIC_FULL      (3<<22)         // IP header and payload checksums calculation (incl pseudo)
                                                //
#define bEMAC_DES_TER           (1<<21)         // Transmit end of ring
#define bEMAC_DES_TCH           (1<<20)         // Second address chained
#define bEMAC_DES_TTSS          (1<<17)         // Transmit time stamp status
#define bEMAC_DES_IHE           (1<<16)         // IP header error
#define bEMAC_DES_JT            (1<<14)         // Jabber timeout
#define bEMAC_DES_FF            (1<<13)         // Frame flushed
#define bEMAC_DES_IPE           (1<<12)         // IP payload error
#define bEMAC_DES_LCA           (1<<11)         // Loss of carrier
#define bEMAC_DES_NC            (1<<10)         // No carrier
#define bEMAC_DES_LCO           (1<<9)          // Late collision
#define bEMAC_DES_EC            (1<<8)          // Excessive collision
#define bEMAC_DES_VF            (1<<7)          // VLAN frame
#define bEMAC_DES_CC_MASK       0x0F            //
#define bEMAC_DES_CC_SHIFT      3               // Collision counter
#define bEMAC_DES_ED            (1<<2)          // Excessive deferral
#define bEMAC_DES_UF            (1<<1)          // Underflow error
#define bEMAC_DES_DB            (1<<0)          // Deferred bit
                                                //
                                                // EMAC_RDES
#define bEMAC_DES_AFM           (1<<30)         // Destination address filter fail
#define bEMAC_DES_FL_MASK       0x3FFFul        // Frame length
#define bEMAC_DES_FL_SHIFT      16              //
#define bEMAC_DES_DE            (1<<14)         // Descriptor error
#define bEMAC_DES_SAF           (1<<13)         // Source address filter fail
#define bEMAC_DES_LE            (1<<12)         // Length error
#define bEMAC_DES_OE            (1<<11)         // Overflow error
#define bEMAC_DES_VLAN          (1<<10)         // VLAN tag
#define bEMAC_DES_RXFS          (1<<9)          // First descriptor
#define bEMAC_DES_RXLS          (1<<8)          // Last descriptor
#define bEMAC_DES_TSV           (1<<7)          // Time stamp valid
#define bEMAC_DES_IPHCE         (1<<7)          // IPv4 header checksum error
#define bEMAC_DES_RXLCO         (1<<6)          // Late collision
#define bEMAC_DES_FT            (1<<5)          // Frame type
#define bEMAC_DES_RWT           (1<<4)          // Receive watchdog timeout
#define bEMAC_DES_RE            (1<<3)          // Receive error
#define bEMAC_DES_DRE           (1<<2)          // Dribble bit error
#define bEMAC_DES_CE            (1<<1)          // CRC error
#define bEMAC_DES_PCE           (1<<0)          // Payload checksum error
#define bEMAC_DES_ESA           (1<<0)          // extended status available
                                                //
#define bEMAC_DES_DIC           (1u<<31)        // Disable interrupt on completion
#define bEMAC_DES_RER           (1<<15)         // Receive end of ring
#define bEMAC_DES_RCH           (1<<14)         // Second address chained
                                                //
#define bEMAC_DES_PV            (1<<13)         // PTP version 2 format
#define bEMAC_DES_PFT           (1<<12)         // PTP frame type
#define bEMAC_DES_PMT_MASK      0x0F            //
#define bEMAC_DES_PMT_SHIFT     8               //
#define bEMAC_DES_IPV6PR        (1<<7)          // IPv6 packet received
#define bEMAC_DES_IPV4PR        (1<<6)          // IPv4 packet received
#define bEMAC_DES_IPCB          (1<<5)          // IP checksum bypassed
#define bEMAC_DES_IPPE          (1<<4)          // IP payload error
#define bEMAC_DES_IPHE          (1<<3)          // IP header error
#define bEMAC_DES_IPPT_MASK     0x07            //
#define bEMAC_DES_IPPT_UNK      (0<<0)          //
#define bEMAC_DES_IPPT_UDP      (1<<0)          //
#define bEMAC_DES_IPPT_TCP      (2<<0)          //
#define bEMAC_DES_IPPT_ICMP     (3<<0)          //
                                                //
                                                // EMAC_xDES
#define bEMAC_DES_OWN           (1u<<31)        // Own bit (DMA owns descriptor)
#define bEMAC_DES_ES            (1<<15)         // Error summary
                                                //
#define bEMAC_DES_BS_MASK       0x1FFFul        //
#define bEMAC_DES_BS1_SHIFT     0               //
#define bEMAC_DES_BS2_SHIFT     16              //

//________________________________________________________________
//
//  SD card controller
//
#define SDIO_POWER      IO_REG32(0x40012C00)    // Power control
#define SDIO_CLKCR      IO_REG32(0x40012C04)    // Clock control
#define SDIO_ARG        IO_REG32(0x40012C08)    // Argument register
#define SDIO_CMD        IO_REG32(0x40012C0C)    // Command register
#define SDIO_RESPCM     IO_REG32(0x40012C10)    // Response register
#define SDIO_RESP0      IO_REG32(0x40012C14)    //
#define SDIO_RESP1      IO_REG32(0x40012C18)    //
#define SDIO_RESP2      IO_REG32(0x40012C1C)    //
#define SDIO_RESP3      IO_REG32(0x40012C20)    //
#define SDIO_DTIMER     IO_REG32(0x40012C24)    // Data timer
#define SDIO_DLEN       IO_REG32(0x40012C28)    // Data length
#define SDIO_DCTRL      IO_REG32(0x40012C2C)    // Data control
#define SDIO_DCOUNT     IO_REG32(0x40012C30)    // Data counter
#define SDIO_STA        IO_REG32(0x40012C34)    // Status
#define SDIO_ICR        IO_REG32(0x40012C38)    // Interrupt clear
#define SDIO_MASK       IO_REG32(0x40012C3C)    // Interrupt mask
#define SDIO_FIFOCNT    IO_REG32(0x40012C48)    // FIFO counter
#define SDIO_FIFO       IO_REG32(0x40012C80)    // FIFO data
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_SDIO               //
{                                               //
    STM_REG     sSDIO_POWER;                    //
    STM_REG     sSDIO_CLKCR;                    //
    STM_REG     sSDIO_ARG;                      //
    STM_REG     sSDIO_CMD;                      //
    STM_REG     sSDIO_RESPCM;                   //
    STM_REG     sSDIO_RESP1;                    //
    STM_REG     sSDIO_RESP2;                    //
    STM_REG     sSDIO_RESP3;                    //
    STM_REG     sSDIO_RESP4;                    //
    STM_REG     sSDIO_DTIMER;                   //
    STM_REG     sSDIO_DLEN;                     //
    STM_REG     sSDIO_DCTRL;                    //
    STM_REG     sSDIO_DCOUNT;                   //
    STM_REG     sSDIO_STA;                      //
    STM_REG     sSDIO_ICR;                      //
    STM_REG     sSDIO_MASK;                     //
    STM_REG     sReserved0[2];                  //
    STM_REG     sSDIO_FIFOCNT;                  //
    STM_REG     sReserved1[13];                 //
    STM_REG     sSDIO_FIFO;                     //
                                                //
} STM_SDIO, *PSTM_SDIO;                         //
#pragma pack(pop)                               //

#define SDIO_BASE       ((PSTM_SDIO)    0x40012C00)

                                                // SDIO_POWER
#define bSDIO_POWER_OFF         (0<<0)          // power off
#define bSDIO_POWER_UP          (2<<0)          // power up
#define bSDIO_POWER_ON          (3<<0)          // power on
                                                //
#define bSDIO_LEN_MASK          0x01FFFFFFul    //
                                                // SDIO_CLKCR
#define bSDIO_HWFC_EN           (1<<14)         // HW Flow Control enable
#define bSDIO_NEGEDGE           (1<<13)         // SDIO_CK dephasing selection bit
                                                //
#define bSDIO_WIDBUS_1          (0<<11)         // 1 bit wide bus
#define bSDIO_WIDBUS_4          (1<<11)         // 4 bit wide bus
#define bSDIO_WIDBUS_8          (2<<11)         // 8 bit wide bus
#define bSDIO_WIDBUS_MSK        (3<<11)         //
                                                //
#define bSDIO_BYPASS            (1<<10)         // Clock divider bypass enable bit
#define bSDIO_PWRSAV            (1<<9)          // Power saving configuration bit
#define bSDIO_CLKEN             (1<<8)          // Clock enable bit
#define bSDIO_CLKDIV_MASK       0xFF            // clock divider
#define bSDIO_CLKDIV_SHIFT      0               //
                                                //
                                                // SDIO_CMD
#define bSDIO_ATACMD            (1<<14)         // CE-ATA command
#define bSDIO_NIEN              (1<<13)         // not Interrupt Enable
#define bSDIO_ENCMDCOMPL        (1<<12)         // Enable CMD completion
#define bSDIO_SUSPEND           (1<<11)         // SD I/O suspend command
#define bSDIO_CPSM_ENA          (1<<10)         // CPSM enable bit
#define bSDIO_WAITPEND          (1<<9)          // CPSM Waits for ends of data transfer
#define bSDIO_WAITINT           (1<<8)          // CPSM waits for interrupt request
#define bSDIO_RESPONCE          (1<<6)          // Short response
#define bSDIO_LONG_RSP          (1<<7)          // Long response
#define bSDIO_CMD_MASK          0x3F            //
#define bSDIO_CMD_SHIFT         0               //
                                                //
                                                // SDIO_DCTRL
#define bSDIO_DPSM_ENA          (1<<0)          // enable transfer
#define bSDIO_DIR_WRITE         (0<<1)          // from controller to card
#define bSDIO_DIR_READ          (1<<1)          // from card to controller
#define bSDIO_BLOCK_MODE        (0<<2)          // block data transfer
#define bSDIO_STREAM_MODE       (1<<2)          // stream data transfer
#define bSDIO_DMA_ENA           (1<<3)          // DMA enable
                                                //
#define bSDIO_BLOCK_1           (0<<4)          // 1 byte block size
#define bSDIO_BLOCK_2           (1<<4)          // 2 byte block size
#define bSDIO_BLOCK_4           (2<<4)          // 4 byte block size
#define bSDIO_BLOCK_8           (3<<4)          // 8 byte block size
#define bSDIO_BLOCK_16          (4<<4)          // 16 byte block size
#define bSDIO_BLOCK_32          (5<<4)          // 32 byte block size
#define bSDIO_BLOCK_64          (6<<4)          // 64 byte block size
#define bSDIO_BLOCK_128         (7<<4)          // 128 byte block size
#define bSDIO_BLOCK_256         (8<<4)          // 256 byte block size
#define bSDIO_BLOCK_512         (9<<4)          // 512 byte block size
#define bSDIO_BLOCK_1024        (10<<4)         // 1024 byte block size
#define bSDIO_BLOCK_2048        (11<<4)         // 2048 byte block size
#define bSDIO_BLOCK_4096        (12<<4)         // 4096 byte block size
#define bSDIO_BLOCK_8192        (13<<4)         // 8192 byte block size
#define bSDIO_BLOCK_16384       (14<<4)         // 16384 byte block size
                                                //
#define bSDIO_RWSTART           (1<<8)          // Read wait start
#define bSDIO_RWSTOP            (1<<9)          // Read wait stop
#define bSDIO_RWMOD             (1<<10)         // Read wait mode
#define bSDIO_SDIOEN            (1<<11)         // SD I/O enable functions
                                                //
                                                // SDIO_STA
                                                // SDIO_ICR
                                                // SDIO_MASK
#define bSDIO_CMD_CRC_FAIL          (1<<0)      // Command response received (CRC check failed)
#define bSDIO_DATA_CRC_FAIL         (1<<1)      // Data block sent/received (CRC check failed)
#define bSDIO_CMD_TIMEOUT           (1<<2)      // Command response timeout
#define bSDIO_DATA_TIMEOUT          (1<<3)      // Data timeout
#define bSDIO_TX_UNDERRUN           (1<<4)      // Transmit FIFO underrun error
#define bSDIO_RX_OVERRUN            (1<<5)      // Overrun Receive FIFO overrun error
#define bSDIO_CMD_RESP_END          (1<<6)      // Command response received (CRC check passed)
#define bSDIO_CMD_SENT              (1<<7)      // Command sent (no response required)
#define bSDIO_DATA_END              (1<<8)      // Data end (data counter is zero)
#define bSDIO_START_BIT_ERR         (1<<9)      // Start bit not detected on all data signals in wide bus mode
#define bSDIO_DATA_BLOCK_END        (1<<10)     // Data block sent/received (CRC check passed)
#define bSDIO_CMD_ACTIVE            (1<<11)     // Command transfer in progress
#define bSDIO_TX_ACTIVE             (1<<12)     // Data transmit in progress
#define bSDIO_RX_ACTIVE             (1<<13)     // Data receive in progress
#define bSDIO_TX_HALF_EMPTY         (1<<14)     // Transmit FIFO half empty
#define bSDIO_RX_HALF_FULL          (1<<15)     // Receive FIFO half full
#define bSDIO_TX_FULL               (1<<16)     // Transmit FIFO full
#define bSDIO_RX_FULL               (1<<17)     // Receive FIFO full
#define bSDIO_TX_EMPTY              (1<<18)     // Transmit FIFO empty
#define bSDIO_RX_EMPTY              (1<<19)     // Receive FIFO empty
#define bSDIO_TX_DATA_AVLBL         (1<<20)     // Data available in transmit FIFO
#define bSDIO_RX_DATA_AVLBL         (1<<21)     // Data available in receive FIFO
#define bSDIO_SDIO_INT              (1<<22)     // SDIO interrupt received
#define bSDIO_CEATA_END             (1<<23)     // CE-ATA command completion signal received for CMD61
                                                //
#define bSDIO_ALLINT                0xFFF       // All clearable interrupt flags
//________________________________________________________________
//
//  USB OTG FS
//
#define OTGFS_GOTGCTL   IO_REG32(0x50000000)    // control and status register
#define OTGFS_GOTGINT   IO_REG32(0x50000004)    // interrupt register
#define OTGFS_GAHBCFG   IO_REG32(0x50000008)    // AHB configuration register
#define OTGFS_GUSBCFG   IO_REG32(0x5000000C)    // USB configuration register
#define OTGFS_GRSTCTL   IO_REG32(0x50000010)    // reset register
#define OTGFS_GINTSTS   IO_REG32(0x50000014)    // core interrupt register
#define OTGFS_GINTMSK   IO_REG32(0x50000018)    // interrupt mask register
#define OTGFS_GRXSTSR   IO_REG32(0x5000001C)    // receive status debug read
#define OTGFS_GRXSTSP   IO_REG32(0x50000020)    // receive status read and pop
#define OTGFS_GRXFSIZ   IO_REG32(0x50000024)    // receive FIFO size register
                                                //
#define OTGFS_HNPTXFSIZ IO_REG32(0x50000028)    // Host non-periodic transmit FIFO size
#define OTGFS_DIEPTXF0  IO_REG32(0x50000028)    // Endpoint 0 Transmit FIFO size
#define OTGFS_HNPTXSTS  IO_REG32(0x5000002C)    // non-periodic transmit FIFO/queue status
#define OTGFS_GCCFG     IO_REG32(0x50000038)    // general core configuration register
#define OTGFS_CID       IO_REG32(0x5000003C)    // core ID register
                                                //
#define OTGFS_HPTXFSIZ  IO_REG32(0x50000100)    // host periodic transmit FIFO size
#define OTGFS_DIEPTXF1  IO_REG32(0x50000104)    // device IN endpoint 1 transmit FIFO size
#define OTGFS_DIEPTXF2  IO_REG32(0x50000108)    // device IN endpoint 2 transmit FIFO size
#define OTGFS_DIEPTXF3  IO_REG32(0x5000010C)    // device IN endpoint 3 transmit FIFO size
                                                //
#define OTGFS_HCFG      IO_REG32(0x50000400)    // host configuration register
#define OTGFS_HFIR      IO_REG32(0x50000404)    // host frame interval register
#define OTGFS_HFNUM     IO_REG32(0x50000408)    // host frame number/frame time remaining
#define OTGFS_HPTXSTS   IO_REG32(0x50000410)    // host periodic transmit FIFO/queue status
#define OTGFS_HAINT     IO_REG32(0x50000414)    // host all channels interrupt
#define OTGFS_HAINTMSK  IO_REG32(0x50000418)    // host all channels interrupt mask
#define OTGFS_HPRT      IO_REG32(0x50000440)    // host port control and status
                                                //
#define OTGFS_HCCHAR0   IO_REG32(0x50000500)    // host channel 0 characteristics
#define OTGFS_HCINT0    IO_REG32(0x50000508)    // host channel 0 interrupt
#define OTGFS_HCINTMSK0 IO_REG32(0x5000050C)    // host channel 0 interrupt mask
#define OTGFS_HCTSIZ0   IO_REG32(0x50000510)    // host channel 0 transfer size
                                                //
#define OTGFS_HCCHAR1   IO_REG32(0x50000520)    // host channel 1 characteristics
#define OTGFS_HCINT1    IO_REG32(0x50000528)    // host channel 1 interrupt
#define OTGFS_HCINTMSK1 IO_REG32(0x5000052C)    // host channel 1 interrupt mask
#define OTGFS_HCTSIZ1   IO_REG32(0x50000530)    // host channel 1 transfer size
                                                //
#define OTGFS_HCCHAR2   IO_REG32(0x50000540)    // host channel 2 characteristics
#define OTGFS_HCINT2    IO_REG32(0x50000548)    // host channel 2 interrupt
#define OTGFS_HCINTMSK2 IO_REG32(0x5000054C)    // host channel 2 interrupt mask
#define OTGFS_HCTSIZ2   IO_REG32(0x50000550)    // host channel 2 transfer size
                                                //
#define OTGFS_HCCHAR3   IO_REG32(0x50000560)    // host channel 3 characteristics
#define OTGFS_HCINT3    IO_REG32(0x50000568)    // host channel 3 interrupt
#define OTGFS_HCINTMSK3 IO_REG32(0x5000056C)    // host channel 3 interrupt mask
#define OTGFS_HCTSIZ3   IO_REG32(0x50000570)    // host channel 3 transfer size
                                                //
#define OTGFS_HCCHAR4   IO_REG32(0x50000580)    // host channel 4 characteristics
#define OTGFS_HCINT4    IO_REG32(0x50000588)    // host channel 4 interrupt
#define OTGFS_HCINTMSK4 IO_REG32(0x5000058C)    // host channel 4 interrupt mask
#define OTGFS_HCTSIZ4   IO_REG32(0x50000590)    // host channel 4 transfer size
                                                //
#define OTGFS_HCCHAR5   IO_REG32(0x500005A0)    // host channel 5 characteristics
#define OTGFS_HCINT5    IO_REG32(0x500005A8)    // host channel 5 interrupt
#define OTGFS_HCINTMSK5 IO_REG32(0x500005AC)    // host channel 5 interrupt mask
#define OTGFS_HCTSIZ5   IO_REG32(0x500005B0)    // host channel 5 transfer size
                                                //
#define OTGFS_HCCHAR6   IO_REG32(0x500005C0)    // host channel 6 characteristics
#define OTGFS_HCINT6    IO_REG32(0x500005C8)    // host channel 6 interrupt
#define OTGFS_HCINTMSK6 IO_REG32(0x500005CC)    // host channel 6 interrupt mask
#define OTGFS_HCTSIZ6   IO_REG32(0x500005D0)    // host channel 6 transfer size
                                                //
#define OTGFS_HCCHAR7   IO_REG32(0x500005E0)    // host channel 7 characteristics
#define OTGFS_HCINT7    IO_REG32(0x500005E8)    // host channel 7 interrupt
#define OTGFS_HCINTMSK7 IO_REG32(0x500005EC)    // host channel 7 interrupt mask
#define OTGFS_HCTSIZ7   IO_REG32(0x500005F0)    // host channel 7 transfer size
                                                //
#define OTGFS_DCFG      IO_REG32(0x50000800)    // device configuration
#define OTGFS_DCTL      IO_REG32(0x50000804)    // device control
#define OTGFS_DSTS      IO_REG32(0x50000808)    // device status
#define OTGFS_DIEPMSK   IO_REG32(0x50000810)    // device IN endpoint common interrupt mask
#define OTGFS_DOEPMSK   IO_REG32(0x50000814)    // device OUT endpoint common interrupt mask
#define OTGFS_DAINT     IO_REG32(0x50000818)    // device all endpoints interrupt
#define OTGFS_DAINTMSK  IO_REG32(0x5000081C)    // all endpoints interrupt mask
#define OTGFS_DVBUSDIS   IO_REG32(0x50000828)   // device VBUS discharge time
#define OTGFS_DVBUSPULSE IO_REG32(0x5000082C)   // device VBUS pulsing time
#define OTGFS_DIEPEMPMSK IO_REG32(0x50000834)   // device IN endpoint FIFO empty interrupt mask
                                                //
#define OTGFS_DIEPCTL0  IO_REG32(0x50000900)    // device IN endpoint 0 control
#define OTGFS_DIEPINT0  IO_REG32(0x50000908)    // device IN endpoint 0 interrupt
#define OTGFS_DIEPTSIZ0 IO_REG32(0x50000910)    // device IN endpoint 0 transfer size
#define OTGFS_DTXFSTS0  IO_REG32(0x50000918)    // device IN endpoint 0 transmit status
                                                //
#define OTGFS_DIEPCTL1  IO_REG32(0x50000920)    // device IN endpoint 1 control
#define OTGFS_DIEPINT1  IO_REG32(0x50000928)    // device IN endpoint 1 interrupt
#define OTGFS_DIEPTSIZ1 IO_REG32(0x50000930)    // device IN endpoint 1 transfer size
#define OTGFS_DTXFSTS1  IO_REG32(0x50000938)    // device IN endpoint 1 transmit status
                                                //
#define OTGFS_DIEPCTL2  IO_REG32(0x50000940)    // device IN endpoint 2 control
#define OTGFS_DIEPINT2  IO_REG32(0x50000948)    // device IN endpoint 2 interrupt
#define OTGFS_DIEPTSIZ2 IO_REG32(0x50000950)    // device IN endpoint 2 transfer size
#define OTGFS_DTXFSTS2  IO_REG32(0x50000958)    // device IN endpoint 2 transmit status
                                                //
#define OTGFS_DIEPCTL3  IO_REG32(0x50000960)    // device IN endpoint 3 control
#define OTGFS_DIEPINT3  IO_REG32(0x50000968)    // device IN endpoint 3 interrupt
#define OTGFS_DIEPTSIZ3 IO_REG32(0x50000970)    // device IN endpoint 3 transfer size
#define OTGFS_DTXFSTS3  IO_REG32(0x50000978)    // device IN endpoint 3 transmit status
                                                //
#define OTGFS_DOEPCTL0  IO_REG32(0x50000B00)    // device OUT endpoint 0 control
#define OTGFS_DOEPINT0  IO_REG32(0x50000B08)    // device OUT endpoint 0 interrupt
#define OTGFS_DOEPTSIZ0 IO_REG32(0x50000B10)    // device OUT endpoint 0 transfer size
                                                //
#define OTGFS_DOEPCTL1  IO_REG32(0x50000B20)    // device OUT endpoint 1 control
#define OTGFS_DOEPINT1  IO_REG32(0x50000B28)    // device OUT endpoint 1 interrupt
#define OTGFS_DOEPTSIZ1 IO_REG32(0x50000B30)    // device OUT endpoint 1 transfer size
                                                //
#define OTGFS_DOEPCTL2  IO_REG32(0x50000B40)    // device OUT endpoint 2 control
#define OTGFS_DOEPINT2  IO_REG32(0x50000B48)    // device OUT endpoint 2 interrupt
#define OTGFS_DOEPTSIZ2 IO_REG32(0x50000B50)    // device OUT endpoint 2 transfer size
                                                //
#define OTGFS_DOEPCTL3  IO_REG32(0x50000B60)    // device OUT endpoint 3 control
#define OTGFS_DOEPINT3  IO_REG32(0x50000B68)    // device OUT endpoint 3 interrupt
#define OTGFS_DOEPTSIZ3 IO_REG32(0x50000B70)    // device OUT endpoint 3 transfer size
                                                //
#define OTGFS_PCGCR     IO_REG32(0x50000E00)    // power and clock gating control
                                                //
#define OTGFS_DFIFO     IO_REG32(0x50001000)    // data FIFO 0 access map
#define OTGFS_DFIFO0    IO_REG32(0x50001000)    // data FIFO 0 access map
#define OTGFS_DFIFO1    IO_REG32(0x50002000)    // data FIFO 1 access map
#define OTGFS_DFIFO2    IO_REG32(0x50003000)    // data FIFO 2 access map
#define OTGFS_DFIFO3    IO_REG32(0x50004000)    // data FIFO 3 access map
#define OTGFS_DFIFO4    IO_REG32(0x50005000)    // data FIFO 4 access map
#define OTGFS_DFIFO5    IO_REG32(0x50006000)    // data FIFO 5 access map
#define OTGFS_DFIFO6    IO_REG32(0x50007000)    // data FIFO 6 access map
#define OTGFS_DFIFO7    IO_REG32(0x50008000)    // data FIFO 7 access map
                                                //
#define     OTGFS_DFIFO_SIZE    0x1000          //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_OTGFS_HC           //
{                                               //
    STM_REG     sOTGFS_CHAR;                    //
    STM_REG     sReserved0;                     //
    STM_REG     sOTGFS_INT;                     //
    STM_REG     sOTGFS_INTMSK;                  //
    STM_REG     sOTGFS_HCTSIZ;                  //
    STM_REG     sReserved1[3];                  //
                                                //
} STM_OTGFS_HC, *PSTM_OTGFS_HC;                 //
#pragma pack(pop)                               //
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_OTGFS_EP           //
{                                               //
    STM_REG     sOTGFS_CTL;                     //
    STM_REG     sReserved0;                     //
    STM_REG     sOTGFS_INT;                     //
    STM_REG     sReserved1;                     //
    STM_REG     sOTGFS_TSIZ;                    //
    STM_REG     sReserved2;                     //
    STM_REG     sOTGFS_STS;                     //
    STM_REG     sReserved3;                     //
                                                //
} STM_OTGFS_EP, *PSTM_OTGFS_EP;                 //
#pragma pack(pop)                               //
                                                //
#define OTGFS_HC_BASE       ((PSTM_OTGFS_HC) 0x50000500)
#define OTGFS_HC0_BASE      ((PSTM_OTGFS_HC) 0x50000500)
#define OTGFS_HC1_BASE      ((PSTM_OTGFS_HC) 0x50000520)
#define OTGFS_HC2_BASE      ((PSTM_OTGFS_HC) 0x50000540)
#define OTGFS_HC3_BASE      ((PSTM_OTGFS_HC) 0x50000560)
#define OTGFS_HC4_BASE      ((PSTM_OTGFS_HC) 0x50000580)
#define OTGFS_HC5_BASE      ((PSTM_OTGFS_HC) 0x500005A0)
#define OTGFS_HC6_BASE      ((PSTM_OTGFS_HC) 0x500005C0)
#define OTGFS_HC7_BASE      ((PSTM_OTGFS_HC) 0x500005E0)

#define OTGFS_DIEP_BASE     ((PSTM_OTGFS_EP) 0x50000900)
#define OTGFS_DIEP0_BASE    ((PSTM_OTGFS_EP) 0x50000900)
#define OTGFS_DIEP1_BASE    ((PSTM_OTGFS_EP) 0x50000920)
#define OTGFS_DIEP2_BASE    ((PSTM_OTGFS_EP) 0x50000940)
#define OTGFS_DIEP3_BASE    ((PSTM_OTGFS_EP) 0x50000960)

#define OTGFS_DOEP_BASE     ((PSTM_OTGFS_EP) 0x50000B00)
#define OTGFS_DOEP0_BASE    ((PSTM_OTGFS_EP) 0x50000B00)
#define OTGFS_DOEP1_BASE    ((PSTM_OTGFS_EP) 0x50000B20)
#define OTGFS_DOEP2_BASE    ((PSTM_OTGFS_EP) 0x50000B40)
#define OTGFS_DOEP3_BASE    ((PSTM_OTGFS_EP) 0x50000B60)

                                                // OTGFS_GOTGCTL
#define bOTGFS_BSVLD            (1<<19)         // B-session valid
#define bOTGFS_ASVLD            (1<<18)         // A-session valid
#define bOTGFS_DBCT             (1<<17)         // debounce time detected
#define bOTGFS_CIDSTS           (1<<16)         // ID status
#define bOTGFS_DHNPEN           (1<<11)         // device HNP enabled
#define bOTGFS_HSHNPEN          (1<<10)         // host HNP enabled
#define bOTGFS_HNPRQ            (1<<9)          // HNP request
#define bOTGFS_HNGSCS           (1<<8)          // host negotiation success
#define bOTGFS_SRQ              (1<<1)          // session request
#define bOTGFS_ARQSCS           (1<<0)          // session resuest success
                                                //
                                                // OTGFS_GOTGINT
#define bOTGFS_DBCDNE           (1<<19)         // Debounce done
#define bOTGFS_ADTOCHG          (1<<18)         // A-device timeout change
#define bOTGFS_HNGDET           (1<<17)         // host negotiation detected
#define bOTGFS_HNSSCHG          (1<<9)          // host negotiation success status change
#define bOTGFS_SRSSCHG          (1<<8)          // session request success status change
#define bOTGFS_SEDET            (1<<2)          // Session end detected
                                                //
                                                // OTGFS_GAHBCFG
#define bOTGFS_PTXFELVL         (1<<8)          // host periodic TxFIFO empty level
#define bOTGFS_TXFELVL          (1<<7)          // TxFIFO empty level
#define bOTGFS_GINTMSK          (1<<0)          // Global interrupt mask
                                                //
                                                // OTGFS_GUSBCFG
#define bOTGFS_CTXPKT           (1u<<31)        // Corrupt Tx packet (debug)
#define bOTGFS_FDMOD            (1<<30)         // Force device mode
#define bOTGFS_FHMOD            (1<<29)         // Force host mode
#define bOTGFS_HNPCAP           (1<<9)          // HNP-capable
#define bOTGFS_SRPCAP           (1<<8)          // SRP-capable
#define bOTGFS_PHYSEL           (1<<7)          // Full Speed serial transceiver select
                                                //
#define bOTGFS_TRDT_SHIFT       10              // turnaround time
#define bOTGFS_TRDT_MASK        0x0F            //
#define bOTGFS_TOCAL_SHIFT      0               // FS timeout calibration
#define bOTGFS_TOCAL_MASK       0x07            //
                                                //
                                                // OTGFS_GRSTCTL
#define bOTGFS_AHBIDLE          (1u<<31)        // AHB master idle
#define bOTGFS_FTXFNUM_SHIFT    6               // TxFIFO number
#define bOTGFS_FTXFNUM_MASK     0x1F            //
#define bOTGFS_TXFNUM_ALL       0x10            //
#define bOTGFS_TXFFLSH          (1<<5)          // TxFIFO flush
#define bOTGFS_RXFFLSH          (1<<4)          // RxFIFO flush
#define bOTGFS_FCRST            (1<<2)          // host frame counter reset
#define bOTGFS_HSRST            (1<<1)          // HCLK soft reset
#define bOTGFS_CSRST            (1<<0)          // Core soft reset
                                                //
                                                // OTGFS_GINTSTS
#define bOTGFS_WKUPINT          (1u<<31)        // Resume/remote wakeup detected
#define bOTGFS_SRQINT           (1<<30)         // Session request/new session detected
#define bOTGFS_DISCINT          (1<<29)         // Disconnect detected
#define bOTGFS_CIDSCHG          (1<<28)         // Connector ID status change
#define bOTGFS_PTXFE            (1<<26)         // Periodic TxFIFO empty
#define bOTGFS_HCINT            (1<<25)         // Host channels interrupt
#define bOTGFS_HPRTINT          (1<<24)         // Host port interrupt
#define bOTGFS_IPXFR            (1<<21)         // Incomplete periodic transfer
#define bOTGFS_IISOOXFR         (1<<21)         // Incomplete isochronous OUT transfer
#define bOTGFS_IISOIXFR         (1<<20)         // Incomplete isochronous IN transfer
#define bOTGFS_OEPINT           (1<<19)         // OUT endpoint interrupt
#define bOTGFS_IEPINT           (1<<18)         // IN endpoint interrupt
#define bOTGFS_EOPF             (1<<15)         // End of periodic frame interrupt
#define bOTGFS_ISOODRP          (1<<14)         // Isochronous OUT packet dropped
#define bOTGFS_ENUMDNE          (1<<13)         // Enumeration done
#define bOTGFS_USBRST           (1<<12)         // USB reset
#define bOTGFS_USBSUSP          (1<<11)         // USB suspend
#define bOTGFS_ESUSP            (1<<10)         // Early suspend
#define bOTGFS_GONAKEFF         (1<<7)          // Global OUT NAK effective
#define bOTGFS_GINAKEFF         (1<<6)          // Global IN non-periodic NAK effective
#define bOTGFS_NPTXFE           (1<<5)          // Non-periodic TxFIFO empty
#define bOTGFS_RXFLVL           (1<<4)          // RxFIFO non-empty
#define bOTGFS_SOF              (1<<3)          // Start of frame
#define bOTGFS_OTGINT           (1<<2)          // OTG interrupt
#define bOTGFS_MMIS             (1<<1)          // Mode mismatch interrupt
#define bOTGFS_CMOD             (1<<0)          // Current mode of operation
                                                //
                                                // OTGFS_GINTMSK
#define bOTGFS_WKUPM            (1u<<31)        // Resume/remote wakeup detected
#define bOTGFS_SRQM             (1<<30)         // Session request/new session detected
#define bOTGFS_DISCM            (1<<29)         // Disconnect detected
#define bOTGFS_CIDSCHGM         (1<<28)         // Connector ID status change
#define bOTGFS_PTXFEM           (1<<26)         // Periodic TxFIFO empty
#define bOTGFS_HCIM             (1<<25)         // Host channels interrupt
#define bOTGFS_HPRTIM           (1<<24)         // Host port interrupt
#define bOTGFS_IPXFRM           (1<<21)         // Incomplete periodic transfer
#define bOTGFS_IISOOXFRM        (1<<21)         // Incomplete isochronous OUT transfer
#define bOTGFS_IISOIXFRM        (1<<20)         // Incomplete isochronous IN transfer
#define bOTGFS_OEPIM            (1<<19)         // OUT endpoint interrupt
#define bOTGFS_IEPIM            (1<<18)         // IN endpoint interrupt
#define bOTGFS_EPMISM           (1<<17)         // endpoint mismatch interrupt
#define bOTGFS_EOPFM            (1<<15)         // End of periodic frame interrupt
#define bOTGFS_ISOODRPM         (1<<14)         // Isochronous OUT packet dropped
#define bOTGFS_ENUMDNEM         (1<<13)         // Enumeration done
#define bOTGFS_USBRSTM          (1<<12)         // USB reset
#define bOTGFS_USBSUSPM         (1<<11)         // USB suspend
#define bOTGFS_ESUSPM           (1<<10)         // Early suspend
#define bOTGFS_GONAKEFFM        (1<<7)          // Global OUT NAK effective
#define bOTGFS_GINAKEFFM        (1<<6)          // Global IN non-periodic NAK effective
#define bOTGFS_NPTXFEM          (1<<5)          // Non-periodic TxFIFO empty
#define bOTGFS_RXFLVLM          (1<<4)          // RxFIFO non-empty
#define bOTGFS_SOFM             (1<<3)          // Start of frame
#define bOTGFS_OTGIM            (1<<2)          // OTG interrupt
#define bOTGFS_MMISM            (1<<1)          // Mode mismatch interrupt
                                                //
                                                // OTGFS_GRXSTSR
                                                // OTGFS_GRXSTSP
#define bOTGFS_FRMNUM_SHIFT         21          //
#define bOTGFS_FRMNUM_MASK          0x0F        //
                                                //
#define bOTGFS_PKTSTS_SHIFT         17          //
#define bOTGFS_PKTSTS_MASK          0x0F        //
#define bOTGFS_PKTSTS_GLBOUT        0x01        // Global OUT NAK
#define bOTGFS_PKTSTS_OUTDATA       0x02        // OUT data packet recieved
#define bOTGFS_PKTSTS_OUTDONE       0x03        // OUT transfer completed
#define bOTGFS_PKTSTS_SETUPDONE     0x04        // SETUP transfer completed
#define bOTGFS_PKTSTS_SETUP         0x06        // SETUP data packet recieved
                                                //
#define bOTGFS_DPID_SHIFT           15          //
#define bOTGFS_DPID_MASK            0x03        //
#define bOTGFS_DPID_DATA0           0           //
#define bOTGFS_DPID_DATA1           1           //
#define bOTGFS_DPID_DATA2           2           //
#define bOTGFS_DPID_DATA3           3           //
                                                //
#define bOTGFS_BCNT_SHIFT           4           //
#define bOTGFS_BCNT_MASK            0x7FF       //
                                                //
#define bOTGFS_EPNUM_SHIFT          0           //
#define bOTGFS_EPNUM_MASK           0x0F        //
                                                //
                                                // OTGFS_GCCFG
#define bOTGFS_NOVBUSSENS           (1<<21)     // VBUS sensing disable option
#define bOTGFS_SOFOUTEN             (1<<20)     // SOF output enable
#define bOTGFS_VBUSBSEN             (1<<19)     // Enable the VBUS sensing B-device
#define bOTGFS_VBUSASEN             (1<<18)     // Enable the VBUS sensing A-device
#define bOTGFS_PWRDWN               (1<<16)     // Power down negative
                                                //
                                                // OTGFS_DCFG
#define bOTGFS_PFIVL_SHIFT          11          //
#define bOTGFS_PFIVL_MASK           0x3         //
#define bOTGFS_PFIVL_80             0           //
#define bOTGFS_PFIVL_85             1           //
#define bOTGFS_PFIVL_90             2           //
#define bOTGFS_PFIVL_95             3           //
                                                //
#define bOTGFS_DAD_SHIFT            4           // device address
#define bOTGFS_DAD_MASK             0x7F        //
                                                //
#define bOTGFS_NZLSOHSK             (1<<2)      //
#define bOTGFS_DSPD_MASK            3           // Device speed
#define bOTGFS_DSPD_FS              3           // Device full speed
                                                //
                                                // OTGFS_DCTL
#define bOTGFS_POPRGDNE             (1<<11)     // Power-on programming done
#define bOTGFS_CGONAK               (1<<10)     // Clear global OUT NAK
#define bOTGFS_SGONAK               (1<<9)      // Set global OUT NAK
#define bOTGFS_CGINAK               (1<<8)      // Clear global IN NAK
#define bOTGFS_SGINAK               (1<<7)      // Set global IN NAK
#define bOTGFS_TCTL_NONE            (0<<4)      // Test control
#define bOTGFS_GONSTS               (1<<3)      // Global OUT NAK status
#define bOTGFS_GINSTS               (1<<2)      // Global IN NAK status
#define bOTGFS_SDIS                 (1<<1)      // Soft disconnect
#define bOTGFS_RWUSIG               (1<<0)      // Remote wakeup signaling
                                                //
                                                // OTGFS_DSTS
#define bOTGFS_FNSOF_SHIFT          8           //
#define bOTGFS_FNSOF_MASK           0x3FFF      //
#define bOTGFS_EERR                 (1<<3)      // erratic error
#define bOTGFS_ENUMSPD_SHIFT        1           // enumerated speed
#define bOTGFS_ENUMSPD_MASK         0x03        //
#define bOTGFS_ENUMSPD_FS           3           // full speed
                                                //
                                                // OTGFS_DIEPMSK/OTGFS_DOEPMSK
#define bOTGFS_INEPNEM              (1<<6)      // IN endpoint NAK effective mask
#define bOTGFS_INEPNMM              (1<<5)      // IN token received with EP mismatch mask
#define bOTGFS_ITTXFEMSK            (1<<4)      // IN token received when TxFIFO empty mask
#define bOTGFS_OTEPDM               (1<<4)      // OUT token received when endpoint disabled mask
#define bOTGFS_TOM                  (1<<3)      // Timeout condition mask (Non-isochronous ep)
#define bOTGFS_SETUPM               (1<<3)      // SETUP phase done mask
#define bOTGFS_EPDM                 (1<<1)      // Endpoint disabled interrupt mask
#define bOTGFS_XFRCM                (1<<0)      // Transfer completed interrupt mask
                                                //
                                                // OTGFS_DIEPCTLx/OTGFS_DOEPCTLx
#define bOTGFS_EPENA                (1u<<31)    // Endpoint enable
#define bOTGFS_EPDIS                (1<<30)     // Endpoint disable
#define bOTGFS_SODDFRM              (1<<29)     // Set odd frame
#define bOTGFS_SEVNFRM              (1<<28)     // Set even frame
#define bOTGFS_SD0PID               (1<<28)     // Set DATA0 PID
#define bOTGFS_SNAK                 (1<<27)     // Set NAK
#define bOTGFS_CNAK                 (1<<26)     // Clear NAK
                                                //
#define bOTGFS_TXFNUM_SHIFT         22          // TxFIFO number
#define bOTGFS_TXFNUM_MASK          0x0F        //
#define bOTGFS_TYPE_SHIFT           18          // EP type
#define bOTGFS_TYPE_MASK            0x03        // EP type
#define bOTGFS_TYPE_CTRL            0           // Control
#define bOTGFS_TYPE_ISO             1           // Isochronous
#define bOTGFS_TYPE_BULK            2           // Bulk
#define bOTGFS_TYPE_INT             3           // Interrupt
                                                //
#define bOTGFS_STALL                (1<<21)     // STALL handshake
#define bOTGFS_SNPM                 (1<<20)     // Snoop mode
#define bOTGFS_NAKSTS               (1<<17)     // NAK status
#define bOTGFS_EONUM                (1<<16)     // Even/odd frame
#define bOTGFS_DPID                 (1<<16)     // Endpoint data PID
#define bOTGFS_USBAEP               (1<<15)     // USB active endpoint
#define bOTGFS_MPSIZ                0x7FF       // maximum packet size
#define bOTGFS_MPSIZ0_64            0           // 64 byte for Control EP0
#define bOTGFS_MPSIZ0_32            1           // 32 byte for Control EP0
#define bOTGFS_MPSIZ0_16            2           // 16 byte for Control EP0
#define bOTGFS_MPSIZ0_8             3           // 8 byte for Control EP0
                                                //
                                                // OTGFS_DIEPINTx
#define bOTGFS_TXFE                 (1<<7)      // Transmit FIFO empty
#define bOTGFS_INEPNE               (1<<6)      // IN endpoint NAK effective
#define bOTGFS_ITTXFE               (1<<4)      // IN token received when TxFIFO is empty
#define bOTGFS_TOC                  (1<<3)      // Timeout condition
#define bOTGFS_EPDISD               (1<<1)      // Endpoint disabled interrupt
#define bOTGFS_XFRC                 (1<<0)      // Transfer completed interrupt
                                                //
                                                // OTGFS_DOEPINTx
#define bOTGFS_B2BSTUP              (1<<6)      // Back-to-back SETUP packets received
#define bOTGFS_OTEPDIS              (1<<4)      // OUT token received when endpoint disabled
#define bOTGFS_SETUP                (1<<3)      // SETUP phase done
                                                //
                                                // OTGFS_DIEPTSIZx/OTGFS_DOEPTSIZx
#define bOTGFS_MCNT_SHIFT           29          //
#define bOTGFS_MCNT_MASK            0x03        //
#define bOTGFS_SCNT_SHIFT           29          //
#define bOTGFS_SCNT_MASK            0x03        //
#define bOTGFS_PKTCNT_SHIFT         19          //
#define bOTGFS_PKTCNT_MASK          0x3FF       //
#define bOTGFS_XFRSIZ_MASK          0x7FFFF     //
                                                //
                                                // OTGFS_PCGCR
#define bOTGFS_PHYSUSP              (1<<4)      // PHY Suspended
#define bOTGFS_GATEHCLK             (1<<1)      // Gate HCLK
#define bOTGFS_STPPCLK              (1<<0)      // Stop PHY clock

//________________________________________________________________
//
//  USB OTG HS
//
#define OTGHS_BASE      ((PSTM_OTGFS)   0x40040000)

#define OTGHS_GOTGCTL   IO_REG32(0x40040000)    // control and status register
#define OTGHS_GOTGINT   IO_REG32(0x40040004)    // interrupt register
#define OTGHS_GAHBCFG   IO_REG32(0x40040008)    // AHB configuration register
#define OTGHS_GUSBCFG   IO_REG32(0x4004000C)    // USB configuration register
#define OTGHS_GRSTCTL   IO_REG32(0x40040010)    // reset register
#define OTGHS_GINTSTS   IO_REG32(0x40040014)    // core interrupt register
#define OTGHS_GINTMSK   IO_REG32(0x40040018)    // interrupt mask register
#define OTGHS_GRXSTSR   IO_REG32(0x4004001C)    // receive status debug read
#define OTGHS_GRXSTSP   IO_REG32(0x40040020)    // receive status read and pop
#define OTGHS_GRXFSIZ   IO_REG32(0x40040024)    // receive FIFO size register
                                                //
#define OTGHS_HNPTXFSIZ IO_REG32(0x40040028)    // Host non-periodic transmit FIFO size
#define OTGHS_DIEPTXF0  IO_REG32(0x40040028)    // Endpoint 0 Transmit FIFO size
#define OTGHS_HNPTXSTS  IO_REG32(0x4004002C)    // non-periodic transmit FIFO/queue status
#define OTGHS_GCCFG     IO_REG32(0x40040038)    // general core configuration register
#define OTGHS_CID       IO_REG32(0x4004003C)    // core ID register
#define OTGHS_HPTXFSIZ  IO_REG32(0x40040100)    // host periodic transmit FIFO size
                                                //
                                                //
#define OTGHS_DIEPTXF1  IO_REG32(0x40040104)    // device IN endpoint 1 transmit FIFO size
#define OTGHS_DIEPTXF2  IO_REG32(0x40040108)    // device IN endpoint 2 transmit FIFO size
#define OTGHS_DIEPTXF3  IO_REG32(0x4004010C)    // device IN endpoint 3 transmit FIFO size
#define OTGHS_DIEPTXF4  IO_REG32(0x40040110)    // device IN endpoint 4 transmit FIFO size
#define OTGHS_DIEPTXF5  IO_REG32(0x40040114)    // device IN endpoint 5 transmit FIFO size
#define OTGHS_DIEPTXF6  IO_REG32(0x40040118)    // device IN endpoint 6 transmit FIFO size
#define OTGHS_DIEPTXF7  IO_REG32(0x4004011C)    // device IN endpoint 7 transmit FIFO size
                                                //
#define OTGHS_HCFG      IO_REG32(0x40040400)    // host configuration register
#define OTGHS_HFIR      IO_REG32(0x40040404)    // host frame interval register
#define OTGHS_HFNUM     IO_REG32(0x40040408)    // host frame number/frame time remaining
#define OTGHS_HPTXSTS   IO_REG32(0x40040410)    // host periodic transmit FIFO/queue status
#define OTGHS_HAINT     IO_REG32(0x40040414)    // host all channels interrupt
#define OTGHS_HAINTMSK  IO_REG32(0x40040418)    // host all channels interrupt mask
#define OTGHS_HPRT      IO_REG32(0x40040440)    // host port control and status
                                                //
#define OTGHS_HCCHAR0   IO_REG32(0x40040500)    // host channel 0 characteristics
#define OTGHS_HCSPLT0   IO_REG32(0x40040504)    // host channel 0 split control
#define OTGHS_HCINT0    IO_REG32(0x40040508)    // host channel 0 interrupt
#define OTGHS_HCINTMSK0 IO_REG32(0x4004050C)    // host channel 0 interrupt mask
#define OTGHS_HCTSIZ0   IO_REG32(0x40040510)    // host channel 0 transfer size
#define OTGHS_HCDMA0    IO_REG32(0x40040514)    // host channel 0 transfer size
                                                //
#define OTGHS_HCCHAR1   IO_REG32(0x40040520)    // host channel 1 characteristics
#define OTGHS_HCSPLT1   IO_REG32(0x40040524)    // host channel 1 split control
#define OTGHS_HCINT1    IO_REG32(0x40040528)    // host channel 1 interrupt
#define OTGHS_HCINTMSK1 IO_REG32(0x4004052C)    // host channel 1 interrupt mask
#define OTGHS_HCTSIZ1   IO_REG32(0x40040530)    // host channel 1 transfer size
#define OTGHS_HCDMA1    IO_REG32(0x40040534)    // host channel 1 transfer size
                                                //
#define OTGHS_HCCHAR2   IO_REG32(0x40040540)    // host channel 2 characteristics
#define OTGHS_HCSPLT2   IO_REG32(0x40040544)    // host channel 2 split control
#define OTGHS_HCINT2    IO_REG32(0x40040548)    // host channel 2 interrupt
#define OTGHS_HCINTMSK2 IO_REG32(0x4004054C)    // host channel 2 interrupt mask
#define OTGHS_HCTSIZ2   IO_REG32(0x40040550)    // host channel 2 transfer size
#define OTGHS_HCDMA2    IO_REG32(0x40040554)    // host channel 2 transfer size
                                                //
#define OTGHS_HCCHAR3   IO_REG32(0x40040560)    // host channel 3 characteristics
#define OTGHS_HCSPLT3   IO_REG32(0x40040564)    // host channel 3 split control
#define OTGHS_HCINT3    IO_REG32(0x40040568)    // host channel 3 interrupt
#define OTGHS_HCINTMSK3 IO_REG32(0x4004056C)    // host channel 3 interrupt mask
#define OTGHS_HCTSIZ3   IO_REG32(0x40040570)    // host channel 3 transfer size
#define OTGHS_HCDMA3    IO_REG32(0x40040574)    // host channel 3 transfer size
                                                //
#define OTGHS_HCCHAR4   IO_REG32(0x40040580)    // host channel 4 characteristics
#define OTGHS_HCSPLT4   IO_REG32(0x40040584)    // host channel 4 split control
#define OTGHS_HCINT4    IO_REG32(0x40040588)    // host channel 4 interrupt
#define OTGHS_HCINTMSK4 IO_REG32(0x4004058C)    // host channel 4 interrupt mask
#define OTGHS_HCTSIZ4   IO_REG32(0x40040590)    // host channel 4 transfer size
#define OTGHS_HCDMA4    IO_REG32(0x40040594)    // host channel 4 transfer size
                                                //
#define OTGHS_HCCHAR5   IO_REG32(0x400405A0)    // host channel 5 characteristics
#define OTGHS_HCSPLT5   IO_REG32(0x400405A4)    // host channel 5 split control
#define OTGHS_HCINT5    IO_REG32(0x400405A8)    // host channel 5 interrupt
#define OTGHS_HCINTMSK5 IO_REG32(0x400405AC)    // host channel 5 interrupt mask
#define OTGHS_HCTSIZ5   IO_REG32(0x400405B0)    // host channel 5 transfer size
#define OTGHS_HCDMA5    IO_REG32(0x400405B4)    // host channel 5 transfer size
                                                //
#define OTGHS_HCCHAR6   IO_REG32(0x400405C0)    // host channel 6 characteristics
#define OTGHS_HCSPLT6   IO_REG32(0x400405C4)    // host channel 6 split control
#define OTGHS_HCINT6    IO_REG32(0x400405C8)    // host channel 6 interrupt
#define OTGHS_HCINTMSK6 IO_REG32(0x400405CC)    // host channel 6 interrupt mask
#define OTGHS_HCTSIZ6   IO_REG32(0x400405D0)    // host channel 6 transfer size
#define OTGHS_HCDMA6    IO_REG32(0x400405D4)    // host channel 6 transfer size
                                                //
#define OTGHS_HCCHAR7   IO_REG32(0x400405E0)    // host channel 7 characteristics
#define OTGHS_HCSPLT7   IO_REG32(0x400405E4)    // host channel 7 split control
#define OTGHS_HCINT7    IO_REG32(0x400405E8)    // host channel 7 interrupt
#define OTGHS_HCINTMSK7 IO_REG32(0x400405EC)    // host channel 7 interrupt mask
#define OTGHS_HCTSIZ7   IO_REG32(0x400405F0)    // host channel 7 transfer size
#define OTGHS_HCDMA7    IO_REG32(0x400405F4)    // host channel 7 transfer size
                                                //
#define OTGHS_HCCHAR8   IO_REG32(0x40040600)    // host channel 8 characteristics
#define OTGHS_HCSPLT8   IO_REG32(0x40040604)    // host channel 8 split control
#define OTGHS_HCINT8    IO_REG32(0x40040608)    // host channel 8 interrupt
#define OTGHS_HCINTMSK8 IO_REG32(0x4004060C)    // host channel 8 interrupt mask
#define OTGHS_HCTSIZ8   IO_REG32(0x40040610)    // host channel 8 transfer size
#define OTGHS_HCDMA8    IO_REG32(0x40040614)    // host channel 8 transfer size
                                                //
#define OTGHS_HCCHAR9   IO_REG32(0x40040620)    // host channel 9 characteristics
#define OTGHS_HCSPLT9   IO_REG32(0x40040624)    // host channel 9 split control
#define OTGHS_HCINT9    IO_REG32(0x40040628)    // host channel 9 interrupt
#define OTGHS_HCINTMSK9 IO_REG32(0x4004062C)    // host channel 9 interrupt mask
#define OTGHS_HCTSIZ9   IO_REG32(0x40040630)    // host channel 9 transfer size
#define OTGHS_HCDMA9    IO_REG32(0x40040634)    // host channel 9 transfer size
                                                //
#define OTGHS_HCCHAR10   IO_REG32(0x40040640)   // host channel 10 characteristics
#define OTGHS_HCSPLT10   IO_REG32(0x40040644)   // host channel 10 split control
#define OTGHS_HCINT10    IO_REG32(0x40040648)   // host channel 10 interrupt
#define OTGHS_HCINTMSK10 IO_REG32(0x4004064C)   // host channel 10 interrupt mask
#define OTGHS_HCTSIZ10   IO_REG32(0x40040650)   // host channel 10 transfer size
#define OTGHS_HCDMA10    IO_REG32(0x40040654)   // host channel 10 transfer size
                                                //
#define OTGHS_HCCHAR11   IO_REG32(0x40040660)   // host channel 11 characteristics
#define OTGHS_HCSPLT11   IO_REG32(0x40040664)   // host channel 11 split control
#define OTGHS_HCINT11    IO_REG32(0x40040668)   // host channel 11 interrupt
#define OTGHS_HCINTMSK11 IO_REG32(0x4004066C)   // host channel 11 interrupt mask
#define OTGHS_HCTSIZ11   IO_REG32(0x40040670)   // host channel 11 transfer size
#define OTGHS_HCDMA11    IO_REG32(0x40040674)   // host channel 11 transfer size
                                                //
#define OTGHS_DCFG      IO_REG32(0x40040800)    // device configuration
#define OTGHS_DCTL      IO_REG32(0x40040804)    // device control
#define OTGHS_DSTS      IO_REG32(0x40040808)    // device status
#define OTGHS_DIEPMSK   IO_REG32(0x40040810)    // device IN endpoint common interrupt mask
#define OTGHS_DOEPMSK   IO_REG32(0x40040814)    // device OUT endpoint common interrupt mask
#define OTGHS_DAINT     IO_REG32(0x40040818)    // device all endpoints interrupt
#define OTGHS_DAINTMSK  IO_REG32(0x4004081C)    // all endpoints interrupt mask
#define OTGHS_DVBUSDIS   IO_REG32(0x40040828)   // device VBUS discharge time
#define OTGHS_DVBUSPULSE IO_REG32(0x4004082C)   // device VBUS pulsing time
#define OTGHS_DIEPEMPMSK IO_REG32(0x40040834)   // device IN endpoint FIFO empty interrupt mask
                                                //
#define OTGHS_DEACHINT   IO_REG32(0x40040838)   // device each endpoint interrupt register
#define OTGHS_DEACHIMSK  IO_REG32(0x4004083C)   // device each endpoint interrupt mask register
#define OTGHS_DEACHINT1  IO_REG32(0x40040840)   // device each endpoint interrupt register 1
#define OTGHS_DEACHIMSK1 IO_REG32(0x40040880)   // device each endpoint interrupt mask register 1
                                                //
#define OTGHS_DIEPCTL0  IO_REG32(0x40040900)    // device IN endpoint 0 control
#define OTGHS_DIEPINT0  IO_REG32(0x40040908)    // device IN endpoint 0 interrupt
#define OTGHS_DIEPTSIZ0 IO_REG32(0x40040910)    // device IN endpoint 0 transfer size
#define OTGHS_DIEPDMA0  IO_REG32(0x40040914)    // device IN endpoint 0 DMA control
#define OTGHS_DTXFSTS0  IO_REG32(0x40040918)    // device IN endpoint 0 transmit status
                                                //
#define OTGHS_DIEPCTL1  IO_REG32(0x40040920)    // device IN endpoint 1 control
#define OTGHS_DIEPINT1  IO_REG32(0x40040928)    // device IN endpoint 1 interrupt
#define OTGHS_DIEPTSIZ1 IO_REG32(0x40040930)    // device IN endpoint 1 transfer size
#define OTGHS_DIEPDMA1  IO_REG32(0x40040934)    // device IN endpoint 1 DMA control
#define OTGHS_DTXFSTS1  IO_REG32(0x40040938)    // device IN endpoint 1 transmit status
                                                //
#define OTGHS_DIEPCTL2  IO_REG32(0x40040940)    // device IN endpoint 2 control
#define OTGHS_DIEPINT2  IO_REG32(0x40040948)    // device IN endpoint 2 interrupt
#define OTGHS_DIEPTSIZ2 IO_REG32(0x40040950)    // device IN endpoint 2 transfer size
#define OTGHS_DIEPDMA2  IO_REG32(0x40040954)    // device IN endpoint 2 DMA control
#define OTGHS_DTXFSTS2  IO_REG32(0x40040958)    // device IN endpoint 2 transmit status
                                                //
#define OTGHS_DIEPCTL3  IO_REG32(0x40040960)    // device IN endpoint 3 control
#define OTGHS_DIEPINT3  IO_REG32(0x40040968)    // device IN endpoint 3 interrupt
#define OTGHS_DIEPTSIZ3 IO_REG32(0x40040970)    // device IN endpoint 3 transfer size
#define OTGHS_DIEPDMA3  IO_REG32(0x40040974)    // device IN endpoint 3 DMA control
#define OTGHS_DTXFSTS3  IO_REG32(0x40040978)    // device IN endpoint 3 transmit status
                                                //
#define OTGHS_DIEPCTL4  IO_REG32(0x40040980)    // device IN endpoint 4 control
#define OTGHS_DIEPINT4  IO_REG32(0x40040988)    // device IN endpoint 4 interrupt
#define OTGHS_DIEPTSIZ4 IO_REG32(0x40040990)    // device IN endpoint 4 transfer size
#define OTGHS_DIEPDMA4  IO_REG32(0x40040994)    // device IN endpoint 4 DMA control
#define OTGHS_DTXFSTS4  IO_REG32(0x40040998)    // device IN endpoint 4 transmit status
                                                //
#define OTGHS_DIEPCTL5  IO_REG32(0x400409A0)    // device IN endpoint 5 control
#define OTGHS_DIEPINT5  IO_REG32(0x400409A8)    // device IN endpoint 5 interrupt
#define OTGHS_DIEPTSIZ5 IO_REG32(0x400409A0)    // device IN endpoint 5 transfer size
#define OTGHS_DIEPDMA5  IO_REG32(0x400409B4)    // device IN endpoint 5 DMA control
#define OTGHS_DTXFSTS5  IO_REG32(0x400409B8)    // device IN endpoint 5 transmit status
                                                //
#define OTGHS_DOEPCTL0  IO_REG32(0x40040B00)    // device OUT endpoint 0 control
#define OTGHS_DOEPINT0  IO_REG32(0x40040B08)    // device OUT endpoint 0 interrupt
#define OTGHS_DOEPTSIZ0 IO_REG32(0x40040B10)    // device OUT endpoint 0 transfer size
                                                //
#define OTGHS_DOEPCTL1  IO_REG32(0x40040B20)    // device OUT endpoint 1 control
#define OTGHS_DOEPINT1  IO_REG32(0x40040B28)    // device OUT endpoint 1 interrupt
#define OTGHS_DOEPTSIZ1 IO_REG32(0x40040B30)    // device OUT endpoint 1 transfer size
                                                //
#define OTGHS_DOEPCTL2  IO_REG32(0x40040B40)    // device OUT endpoint 2 control
#define OTGHS_DOEPINT2  IO_REG32(0x40040B48)    // device OUT endpoint 2 interrupt
#define OTGHS_DOEPTSIZ2 IO_REG32(0x40040B50)    // device OUT endpoint 2 transfer size
                                                //
#define OTGHS_DOEPCTL3  IO_REG32(0x40040B60)    // device OUT endpoint 3 control
#define OTGHS_DOEPINT3  IO_REG32(0x40040B68)    // device OUT endpoint 3 interrupt
#define OTGHS_DOEPTSIZ3 IO_REG32(0x40040B70)    // device OUT endpoint 3 transfer size
                                                //
#define OTGHS_DOEPCTL4  IO_REG32(0x40040B80)    // device OUT endpoint 4 control
#define OTGHS_DOEPINT4  IO_REG32(0x40040B88)    // device OUT endpoint 4 interrupt
#define OTGHS_DOEPTSIZ4 IO_REG32(0x40040B90)    // device OUT endpoint 4 transfer size
                                                //
#define OTGHS_DOEPCTL5  IO_REG32(0x40040BA0)    // device OUT endpoint 5 control
#define OTGHS_DOEPINT5  IO_REG32(0x40040BA8)    // device OUT endpoint 5 interrupt
#define OTGHS_DOEPTSIZ5 IO_REG32(0x40040BB0)    // device OUT endpoint 5 transfer size
                                                //
#define OTGHS_PCGCR     IO_REG32(0x40040E00)    // power and clock gating control
                                                //
#define OTGHS_DFIFO     IO_REG32(0x40040000)    // data FIFO 0 access map
#define OTGHS_DFIFO0    IO_REG32(0x40041000)    // data FIFO 0 access map
#define OTGHS_DFIFO1    IO_REG32(0x40042000)    // data FIFO 1 access map
#define OTGHS_DFIFO2    IO_REG32(0x40043000)    // data FIFO 2 access map
#define OTGHS_DFIFO3    IO_REG32(0x40044000)    // data FIFO 3 access map
#define OTGHS_DFIFO4    IO_REG32(0x40045000)    // data FIFO 4 access map
#define OTGHS_DFIFO5    IO_REG32(0x40046000)    // data FIFO 5 access map
#define OTGHS_DFIFO6    IO_REG32(0x40047000)    // data FIFO 6 access map
#define OTGHS_DFIFO7    IO_REG32(0x40048000)    // data FIFO 7 access map

//________________________________________________________________
//
//  System Configuration
//
#define SYSCFG_MEMRM    IO_REG32(0x40013800)    //
#define SYSCFG_PMC      IO_REG32(0x40013804)    //
#define SYSCFG_EXTICR1  IO_REG32(0x40013808)    //
#define SYSCFG_EXTICR2  IO_REG32(0x4001380C)    //
#define SYSCFG_EXTICR3  IO_REG32(0x40013810)    //
#define SYSCFG_EXTICR4  IO_REG32(0x40013814)    //
#define SYSCFG_CMPCR    IO_REG32(0x40013820)    //
                                                //
#pragma pack(push, 4)                           //
typedef volatile struct _STM_SYSCFG             //
{                                               //
    STM_REG     sSYSCFG_MEMRM;                  //
    STM_REG     sSYSCFG_PMC;                    //
    STM_REG     sSYSCFG_EXTICR1;                //
    STM_REG     sSYSCFG_EXTICR2;                //
    STM_REG     sSYSCFG_EXTICR3;                //
    STM_REG     sSYSCFG_EXTICR4;                //
    STM_REG     sResereved0[2];                 //
    STM_REG     sSYSCFG_CMPCR;                  //
                                                //
} STM_SYSCFG, *PSTM_SYSCFG;                     //
#pragma pack(pop)                               //

#define SYSCFG_BASE     ((PSTM_SYSCFG)  0x40013800)

                                                // SYSCFG_MEMRM
#define bSYSCGF_MAP_FLASH       (0<<0)          // Main Flash mapped at 0x00000000
#define bSYSCGF_MAP_SYSTEM      (1<<0)          // System Flash mapped at 0x00000000
#define bSYSCGF_MAP_FSMC        (2<<0)          // Static Memory Controler mapped at 0x00000000
#define bSYSCGF_MAP_SRAM        (3<<0)          // Embedded SRAM  mapped at 0x00000000
                                                //
                                                // SYSCFG_PMC
#define bSYSCFG_RMII_SEL        (1<<23)         // RMII selection
                                                //
                                                // SYSCFG_CMPCR
#define bSYSCFG_READY           (1<<8)          // Compensation cell ready
#define bSYSCFG_CMP_PD          (1<<0)          // Compensation cell enable
                                                //
//________________________________________________________________
//
#define DBGMCU_IDCODE   IO_REG32(0xE0042000)    // STM32 identification

#define STM32_ID_MASK       0xFFF
#define STM32_ID_411        0x411

//________________________________________________________________
//
// Memory mapping definitions for STM32F2xx
//
#define STM_XN_LIMIT        ((unsigned char*) 0x40000000)   // Non executive limit
#define STM_ISRAM           ((unsigned char*) 0x20000000)   // Internal SRAM base address
#define STM_ISRAM_BANK0     ((unsigned char*) 0x20000000)   // Internal SRAM bank 0
#define STM_ISRAM_BANK1     ((unsigned char*) 0x2001C000)   // Internal SRAM bank 0
#define STM_IFLASH          ((unsigned char*) 0x08000000)   // Internal FLASH base address
#define STM_BKUPSRAM        ((unsigned char*) 0x40024000)   // Backup SRAM base address
#define STM_BKUP_SIZE       ((unsigned int)   0x00001000)   // Backup SRAM size
#define STM_SYSMEM          ((unsigned char*) 0x1FFF0000)   // System memory base
#define STM_SYSMEM_SIZE     ((unsigned int)   30*0x400)     // System memory size 30К
#define STM_OPTMEM          ((unsigned char*) 0x1FFFC000)   //
#define STM_FLASH_SECTOR_SIZE   0x20000u
#endif
