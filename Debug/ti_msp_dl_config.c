/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_0Backup;
DL_TimerG_backupConfig gQEI_0Backup;
DL_TimerG_backupConfig gTIMER_GBBackup;
DL_TimerG_backupConfig gTIMER_countBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_0_init();
    SYSCFG_DL_QEI_0_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_TIMER_GB_init();
    SYSCFG_DL_TIMER_count_init();
    SYSCFG_DL_I2C_0_init();
    SYSCFG_DL_I2C_1_init();
    SYSCFG_DL_UART_XG_init();
    SYSCFG_DL_UART_ICM_init();
    /* Ensure backup structures have no valid state */
	gPWM_0Backup.backupRdy 	= false;
	gQEI_0Backup.backupRdy 	= false;
	gTIMER_GBBackup.backupRdy 	= false;
	gTIMER_countBackup.backupRdy 	= false;


}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_0_INST, &gPWM_0Backup);
	retStatus &= DL_TimerG_saveConfiguration(QEI_0_INST, &gQEI_0Backup);
	retStatus &= DL_TimerG_saveConfiguration(TIMER_GB_INST, &gTIMER_GBBackup);
	retStatus &= DL_TimerG_saveConfiguration(TIMER_count_INST, &gTIMER_countBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_0_INST, &gPWM_0Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(QEI_0_INST, &gQEI_0Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMER_GB_INST, &gTIMER_GBBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMER_count_INST, &gTIMER_countBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_0_INST);
    DL_TimerG_reset(QEI_0_INST);
    DL_TimerG_reset(TIMER_0_INST);
    DL_TimerG_reset(TIMER_GB_INST);
    DL_TimerG_reset(TIMER_count_INST);
    DL_I2C_reset(I2C_0_INST);
    DL_I2C_reset(I2C_1_INST);
    DL_UART_Main_reset(UART_XG_INST);
    DL_UART_Main_reset(UART_ICM_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_0_INST);
    DL_TimerG_enablePower(QEI_0_INST);
    DL_TimerG_enablePower(TIMER_0_INST);
    DL_TimerG_enablePower(TIMER_GB_INST);
    DL_TimerG_enablePower(TIMER_count_INST);
    DL_I2C_enablePower(I2C_0_INST);
    DL_I2C_enablePower(I2C_1_INST);
    DL_UART_Main_enablePower(UART_XG_INST);
    DL_UART_Main_enablePower(UART_ICM_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_IOMUX,GPIO_PWM_0_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_CMPL_IOMUX,GPIO_PWM_0_C0_CMPL_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C0_CMPL_PORT, GPIO_PWM_0_C0_CMPL_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_IOMUX,GPIO_PWM_0_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_CMPL_IOMUX,GPIO_PWM_0_C1_CMPL_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C1_CMPL_PORT, GPIO_PWM_0_C1_CMPL_PIN);
    
	DL_GPIO_initPeripheralOutputFunction(
		 GPIO_PWM_0_C0_IOMUX, GPIO_PWM_0_C0_IOMUX_FUNC);

    
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_QEI_0_PHA_IOMUX, GPIO_QEI_0_PHA_IOMUX_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_QEI_0_PHB_IOMUX, GPIO_QEI_0_PHB_IOMUX_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_1_IOMUX_SDA,
        GPIO_I2C_1_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_1_IOMUX_SCL,
        GPIO_I2C_1_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_1_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_1_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_XG_IOMUX_TX, GPIO_UART_XG_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_XG_IOMUX_RX, GPIO_UART_XG_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_ICM_IOMUX_TX, GPIO_UART_ICM_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_ICM_IOMUX_RX, GPIO_UART_ICM_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(LED_PIN22_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_BIN2_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_STBY_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Motor_GB1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Motor_GB2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(XG_G1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(XG_G2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(XG_G3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(XG_G4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(XG_G5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(XG_G6_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(XG_G7_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(XG_G8_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(I2C_SCL_IOMUX);

    DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);

    DL_GPIO_clearPins(GPIOA, Motor_AIN1_PIN |
		Motor_BIN2_PIN |
		I2C_SCL_PIN |
		I2C_SDA_PIN);
    DL_GPIO_enableOutput(GPIOA, Motor_AIN1_PIN |
		Motor_BIN2_PIN |
		I2C_SCL_PIN |
		I2C_SDA_PIN);
    DL_GPIO_clearPins(GPIOB, LED_PIN22_PIN |
		Motor_AIN2_PIN |
		Motor_BIN1_PIN |
		Motor_STBY_PIN);
    DL_GPIO_enableOutput(GPIOB, LED_PIN22_PIN |
		Motor_AIN2_PIN |
		Motor_BIN1_PIN |
		Motor_STBY_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_10_EDGE_RISE);
    DL_GPIO_setLowerPinsInputFilter(GPIOB, DL_GPIO_PIN_11_INPUT_FILTER_1_CYCLE);
    DL_GPIO_clearInterruptStatus(GPIOB, Motor_GB1_PIN);
    DL_GPIO_enableInterrupt(GPIOB, Motor_GB1_PIN);
    DL_GPIO_setPublisherChanID(GPIOB, DL_GPIO_PUBLISHER_INDEX_0, GPIOB_EVENT_PUBLISHER_0_CHANNEL);
    DL_GPIO_enableEvents(GPIOB, DL_GPIO_EVENT_ROUTE_1, Motor_GB1_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOB_INT_IRQn, 1);

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   160000 Hz = 32000000 Hz / (1 * (199 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_0ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 199U
};

static const DL_TimerA_PWMConfig gPWM_0Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 99,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_0_init(void) {

    DL_TimerA_setClockConfig(
        PWM_0_INST, (DL_TimerA_ClockConfig *) &gPWM_0ClockConfig);

    DL_TimerA_initPWMMode(
        PWM_0_INST, (DL_TimerA_PWMConfig *) &gPWM_0Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_0_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 29, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 29, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_0_INST);


    
    DL_TimerA_setCCPDirection(PWM_0_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );
    DL_TimerA_enableShadowFeatures(PWM_0_INST);


}


static const DL_TimerG_ClockConfig gQEI_0ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_5,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_QEI_0_init(void) {

    DL_TimerG_setClockConfig(
        QEI_0_INST, (DL_TimerG_ClockConfig *) &gQEI_0ClockConfig);

    DL_TimerG_configQEI(QEI_0_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_0_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_0_INST, 65535);
    DL_TimerG_enableEvent(QEI_0_INST, DL_TIMER_EVENT_ROUTE_1, DL_TIMER_EVENT_LOAD_EVENT);
    DL_TimerG_setPublisherChanID(QEI_0_INST, DL_TIMER_PUBLISHER_INDEX_0, 0);

    DL_TimerG_enableClock(QEI_0_INST);
    DL_TimerG_startCounter(QEI_0_INST);
}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000 Hz = 4000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (5ms * 40000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

    DL_TimerG_setClockConfig(TIMER_0_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_0ClockConfig);

    DL_TimerG_initTimerMode(TIMER_0_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerG_enableClock(TIMER_0_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   160000 Hz = 32000000 Hz / (1 * (199 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_GBClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 199U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_GB_INST_LOAD_VALUE = (10 ms * 160000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_GBTimerConfig = {
    .period     = TIMER_GB_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_GB_init(void) {

    DL_TimerG_setClockConfig(TIMER_GB_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_GBClockConfig);

    DL_TimerG_initTimerMode(TIMER_GB_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_GBTimerConfig);
    DL_TimerG_enableInterrupt(TIMER_GB_INST , DL_TIMERG_INTERRUPT_LOAD_EVENT);
	NVIC_SetPriority(TIMER_GB_INST_INT_IRQN, 2);
    DL_TimerG_enableShadowFeatures(TIMER_GB_INST);
    DL_TimerG_enableClock(TIMER_GB_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   200000 Hz = 4000000 Hz / (8 * (19 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_countClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 19U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_count_INST_LOAD_VALUE = (10 ms * 200000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_countTimerConfig = {
    .period     = TIMER_count_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_count_init(void) {

    DL_TimerG_setClockConfig(TIMER_count_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_countClockConfig);

    DL_TimerG_initTimerMode(TIMER_count_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_countTimerConfig);
    DL_TimerG_enableInterrupt(TIMER_count_INST , DL_TIMERG_INTERRUPT_LOAD_EVENT);
	NVIC_SetPriority(TIMER_count_INST_INT_IRQN, 0);
    DL_TimerG_enableClock(TIMER_count_INST);





}


static const DL_I2C_ClockConfig gI2C_0ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_0_init(void) {

    DL_I2C_setClockConfig(I2C_0_INST,
        (DL_I2C_ClockConfig *) &gI2C_0ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_0_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_0_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(I2C_0_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(I2C_0_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_0_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_0_INST);


}
static const DL_I2C_ClockConfig gI2C_1ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_1_init(void) {

    DL_I2C_setClockConfig(I2C_1_INST,
        (DL_I2C_ClockConfig *) &gI2C_1ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_1_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_1_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_1_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(I2C_1_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(I2C_1_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_1_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_1_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_1_INST);


}

static const DL_UART_Main_ClockConfig gUART_XGClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_XGConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_XG_init(void)
{
    DL_UART_Main_setClockConfig(UART_XG_INST, (DL_UART_Main_ClockConfig *) &gUART_XGClockConfig);

    DL_UART_Main_init(UART_XG_INST, (DL_UART_Main_Config *) &gUART_XGConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_XG_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_XG_INST, UART_XG_IBRD_32_MHZ_115200_BAUD, UART_XG_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(UART_XG_INST);
}
static const DL_UART_Main_ClockConfig gUART_ICMClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_8
};

static const DL_UART_Main_Config gUART_ICMConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_ICM_init(void)
{
    DL_UART_Main_setClockConfig(UART_ICM_INST, (DL_UART_Main_ClockConfig *) &gUART_ICMClockConfig);

    DL_UART_Main_init(UART_ICM_INST, (DL_UART_Main_Config *) &gUART_ICMConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115107.91
     */
    DL_UART_Main_setOversampling(UART_ICM_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_ICM_INST, UART_ICM_IBRD_4_MHZ_115200_BAUD, UART_ICM_FBRD_4_MHZ_115200_BAUD);



    DL_UART_Main_enable(UART_ICM_INST);
}

