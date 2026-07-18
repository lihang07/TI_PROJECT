/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                               160000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_4
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM17)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM17_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_CMPL_PORT                                            GPIOB
#define GPIO_PWM_0_C0_CMPL_PIN                                     DL_GPIO_PIN_6
#define GPIO_PWM_0_C0_CMPL_IOMUX                                 (IOMUX_PINCM23)
#define GPIO_PWM_0_C0_CMPL_IOMUX_FUNC           IOMUX_PINCM23_PF_TIMA1_CCP0_CMPL

/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_5
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM18)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM18_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_CMPL_PORT                                            GPIOB
#define GPIO_PWM_0_C1_CMPL_PIN                                    DL_GPIO_PIN_20
#define GPIO_PWM_0_C1_CMPL_IOMUX                                 (IOMUX_PINCM48)
#define GPIO_PWM_0_C1_CMPL_IOMUX_FUNC           IOMUX_PINCM48_PF_TIMA1_CCP1_CMPL





/* Defines for QEI_0 */
#define QEI_0_INST                                                         TIMG8
#define QEI_0_INST_IRQHandler                                   TIMG8_IRQHandler
#define QEI_0_INST_INT_IRQN                                     (TIMG8_INT_IRQn)
/* Pin configuration defines for QEI_0 PHA Pin */
#define GPIO_QEI_0_PHA_PORT                                                GPIOB
#define GPIO_QEI_0_PHA_PIN                                        DL_GPIO_PIN_21
#define GPIO_QEI_0_PHA_IOMUX                                     (IOMUX_PINCM49)
#define GPIO_QEI_0_PHA_IOMUX_FUNC                    IOMUX_PINCM49_PF_TIMG8_CCP0
/* Pin configuration defines for QEI_0 PHB Pin */
#define GPIO_QEI_0_PHB_PORT                                                GPIOA
#define GPIO_QEI_0_PHB_PIN                                        DL_GPIO_PIN_30
#define GPIO_QEI_0_PHB_IOMUX                                      (IOMUX_PINCM5)
#define GPIO_QEI_0_PHB_IOMUX_FUNC                     IOMUX_PINCM5_PF_TIMG8_CCP1


/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (199U)
/* Defines for TIMER_GB */
#define TIMER_GB_INST                                                    (TIMG6)
#define TIMER_GB_INST_IRQHandler                                TIMG6_IRQHandler
#define TIMER_GB_INST_INT_IRQN                                  (TIMG6_INT_IRQn)
#define TIMER_GB_INST_LOAD_VALUE                                         (1599U)
/* Defines for TIMER_count */
#define TIMER_count_INST                                                 (TIMG7)
#define TIMER_count_INST_IRQHandler                             TIMG7_IRQHandler
#define TIMER_count_INST_INT_IRQN                               (TIMG7_INT_IRQn)
#define TIMER_count_INST_LOAD_VALUE                                      (1999U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C1
#define I2C_0_INST_IRQHandler                                    I2C1_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C1_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                        DL_GPIO_PIN_18
#define GPIO_I2C_0_IOMUX_SDA                                     (IOMUX_PINCM40)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                      IOMUX_PINCM40_PF_I2C1_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                        DL_GPIO_PIN_15
#define GPIO_I2C_0_IOMUX_SCL                                     (IOMUX_PINCM37)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                      IOMUX_PINCM37_PF_I2C1_SCL

/* Defines for I2C_1 */
#define I2C_1_INST                                                          I2C0
#define I2C_1_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_1_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_1_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_1_SDA_PORT                                                GPIOA
#define GPIO_I2C_1_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_1_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_1_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_1_SCL_PORT                                                GPIOA
#define GPIO_I2C_1_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_1_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_1_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART_XG */
#define UART_XG_INST                                                       UART2
#define UART_XG_INST_FREQUENCY                                          32000000
#define UART_XG_INST_IRQHandler                                 UART2_IRQHandler
#define UART_XG_INST_INT_IRQN                                     UART2_INT_IRQn
#define GPIO_UART_XG_RX_PORT                                               GPIOB
#define GPIO_UART_XG_TX_PORT                                               GPIOB
#define GPIO_UART_XG_RX_PIN                                       DL_GPIO_PIN_16
#define GPIO_UART_XG_TX_PIN                                       DL_GPIO_PIN_15
#define GPIO_UART_XG_IOMUX_RX                                    (IOMUX_PINCM33)
#define GPIO_UART_XG_IOMUX_TX                                    (IOMUX_PINCM32)
#define GPIO_UART_XG_IOMUX_RX_FUNC                     IOMUX_PINCM33_PF_UART2_RX
#define GPIO_UART_XG_IOMUX_TX_FUNC                     IOMUX_PINCM32_PF_UART2_TX
#define UART_XG_BAUD_RATE                                               (115200)
#define UART_XG_IBRD_32_MHZ_115200_BAUD                                     (17)
#define UART_XG_FBRD_32_MHZ_115200_BAUD                                     (23)
/* Defines for UART_ICM */
#define UART_ICM_INST                                                      UART1
#define UART_ICM_INST_FREQUENCY                                          4000000
#define UART_ICM_INST_IRQHandler                                UART1_IRQHandler
#define UART_ICM_INST_INT_IRQN                                    UART1_INT_IRQn
#define GPIO_UART_ICM_RX_PORT                                              GPIOA
#define GPIO_UART_ICM_TX_PORT                                              GPIOA
#define GPIO_UART_ICM_RX_PIN                                       DL_GPIO_PIN_9
#define GPIO_UART_ICM_TX_PIN                                       DL_GPIO_PIN_8
#define GPIO_UART_ICM_IOMUX_RX                                   (IOMUX_PINCM20)
#define GPIO_UART_ICM_IOMUX_TX                                   (IOMUX_PINCM19)
#define GPIO_UART_ICM_IOMUX_RX_FUNC                    IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_ICM_IOMUX_TX_FUNC                    IOMUX_PINCM19_PF_UART1_TX
#define UART_ICM_BAUD_RATE                                              (115200)
#define UART_ICM_IBRD_4_MHZ_115200_BAUD                                      (2)
#define UART_ICM_FBRD_4_MHZ_115200_BAUD                                     (11)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for PIN22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED_PIN22_PIN                                           (DL_GPIO_PIN_22)
#define LED_PIN22_IOMUX                                          (IOMUX_PINCM50)
/* Defines for AIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define Motor_AIN1_PORT                                                  (GPIOA)
#define Motor_AIN1_PIN                                          (DL_GPIO_PIN_13)
#define Motor_AIN1_IOMUX                                         (IOMUX_PINCM35)
/* Defines for AIN2: GPIOB.9 with pinCMx 26 on package pin 61 */
#define Motor_AIN2_PORT                                                  (GPIOB)
#define Motor_AIN2_PIN                                           (DL_GPIO_PIN_9)
#define Motor_AIN2_IOMUX                                         (IOMUX_PINCM26)
/* Defines for BIN1: GPIOB.26 with pinCMx 57 on package pin 28 */
#define Motor_BIN1_PORT                                                  (GPIOB)
#define Motor_BIN1_PIN                                          (DL_GPIO_PIN_26)
#define Motor_BIN1_IOMUX                                         (IOMUX_PINCM57)
/* Defines for BIN2: GPIOA.29 with pinCMx 4 on package pin 36 */
#define Motor_BIN2_PORT                                                  (GPIOA)
#define Motor_BIN2_PIN                                          (DL_GPIO_PIN_29)
#define Motor_BIN2_IOMUX                                          (IOMUX_PINCM4)
/* Defines for STBY: GPIOB.7 with pinCMx 24 on package pin 59 */
#define Motor_STBY_PORT                                                  (GPIOB)
#define Motor_STBY_PIN                                           (DL_GPIO_PIN_7)
#define Motor_STBY_IOMUX                                         (IOMUX_PINCM24)
/* Defines for GB1: GPIOB.10 with pinCMx 27 on package pin 62 */
#define Motor_GB1_PORT                                                   (GPIOB)
// pins affected by this interrupt request:["GB1"]
#define Motor_INT_IRQN                                          (GPIOB_INT_IRQn)
#define Motor_INT_IIDX                          (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define Motor_GB1_IIDX                                      (DL_GPIO_IIDX_DIO10)
#define Motor_GB1_PIN                                           (DL_GPIO_PIN_10)
#define Motor_GB1_IOMUX                                          (IOMUX_PINCM27)
/* Defines for GB2: GPIOB.11 with pinCMx 28 on package pin 63 */
#define Motor_GB2_PORT                                                   (GPIOB)
#define Motor_GB2_PIN                                           (DL_GPIO_PIN_11)
#define Motor_GB2_IOMUX                                          (IOMUX_PINCM28)
/* Defines for KEY1: GPIOB.1 with pinCMx 13 on package pin 48 */
#define KEY_KEY1_PORT                                                    (GPIOB)
#define KEY_KEY1_PIN                                             (DL_GPIO_PIN_1)
#define KEY_KEY1_IOMUX                                           (IOMUX_PINCM13)
/* Defines for KEY2: GPIOA.21 with pinCMx 46 on package pin 17 */
#define KEY_KEY2_PORT                                                    (GPIOA)
#define KEY_KEY2_PIN                                            (DL_GPIO_PIN_21)
#define KEY_KEY2_IOMUX                                           (IOMUX_PINCM46)
/* Defines for KEY3: GPIOB.14 with pinCMx 31 on package pin 2 */
#define KEY_KEY3_PORT                                                    (GPIOB)
#define KEY_KEY3_PIN                                            (DL_GPIO_PIN_14)
#define KEY_KEY3_IOMUX                                           (IOMUX_PINCM31)
/* Defines for KEY4: GPIOA.23 with pinCMx 53 on package pin 24 */
#define KEY_KEY4_PORT                                                    (GPIOA)
#define KEY_KEY4_PIN                                            (DL_GPIO_PIN_23)
#define KEY_KEY4_IOMUX                                           (IOMUX_PINCM53)
/* Defines for G1: GPIOA.27 with pinCMx 60 on package pin 31 */
#define XG_G1_PORT                                                       (GPIOA)
#define XG_G1_PIN                                               (DL_GPIO_PIN_27)
#define XG_G1_IOMUX                                              (IOMUX_PINCM60)
/* Defines for G2: GPIOA.25 with pinCMx 55 on package pin 26 */
#define XG_G2_PORT                                                       (GPIOA)
#define XG_G2_PIN                                               (DL_GPIO_PIN_25)
#define XG_G2_IOMUX                                              (IOMUX_PINCM55)
/* Defines for G3: GPIOB.25 with pinCMx 56 on package pin 27 */
#define XG_G3_PORT                                                       (GPIOB)
#define XG_G3_PIN                                               (DL_GPIO_PIN_25)
#define XG_G3_IOMUX                                              (IOMUX_PINCM56)
/* Defines for G4: GPIOA.7 with pinCMx 14 on package pin 49 */
#define XG_G4_PORT                                                       (GPIOA)
#define XG_G4_PIN                                                (DL_GPIO_PIN_7)
#define XG_G4_IOMUX                                              (IOMUX_PINCM14)
/* Defines for G5: GPIOA.14 with pinCMx 36 on package pin 7 */
#define XG_G5_PORT                                                       (GPIOA)
#define XG_G5_PIN                                               (DL_GPIO_PIN_14)
#define XG_G5_IOMUX                                              (IOMUX_PINCM36)
/* Defines for G6: GPIOA.16 with pinCMx 38 on package pin 9 */
#define XG_G6_PORT                                                       (GPIOA)
#define XG_G6_PIN                                               (DL_GPIO_PIN_16)
#define XG_G6_IOMUX                                              (IOMUX_PINCM38)
/* Defines for G7: GPIOB.17 with pinCMx 43 on package pin 14 */
#define XG_G7_PORT                                                       (GPIOB)
#define XG_G7_PIN                                               (DL_GPIO_PIN_17)
#define XG_G7_IOMUX                                              (IOMUX_PINCM43)
/* Defines for G8: GPIOB.19 with pinCMx 45 on package pin 16 */
#define XG_G8_PORT                                                       (GPIOB)
#define XG_G8_PIN                                               (DL_GPIO_PIN_19)
#define XG_G8_IOMUX                                              (IOMUX_PINCM45)
/* Port definition for Pin Group I2C */
#define I2C_PORT                                                         (GPIOA)

/* Defines for SCL: GPIOA.31 with pinCMx 6 on package pin 39 */
#define I2C_SCL_PIN                                             (DL_GPIO_PIN_31)
#define I2C_SCL_IOMUX                                             (IOMUX_PINCM6)
/* Defines for SDA: GPIOA.28 with pinCMx 3 on package pin 35 */
#define I2C_SDA_PIN                                             (DL_GPIO_PIN_28)
#define I2C_SDA_IOMUX                                             (IOMUX_PINCM3)
#define GPIOB_EVENT_PUBLISHER_0_CHANNEL                                      (1)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_QEI_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_TIMER_GB_init(void);
void SYSCFG_DL_TIMER_count_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_UART_XG_init(void);
void SYSCFG_DL_UART_ICM_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
