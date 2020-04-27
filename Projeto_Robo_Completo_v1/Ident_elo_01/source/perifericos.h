/*
 * perifericos.h
 *
 *  Created on: 14 de fev de 2020
 *      Author: davia
 */

#ifndef PERIFERICOS_H_
#define PERIFERICOS_H_

#include "fsl_dac.h"
#include "fsl_pit.h"
#include "fsl_tpm.h"
#include "fsl_i2c.h"

void inicializar_dac(float tensao_max) {
	static dac_config_t DAC_CONFIG;
	DAC_CONFIG.enableLowPowerMode = false;
	DAC_CONFIG.referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2; //Ver qua eh
	DAC_Init(DAC0, &DAC_CONFIG);
	DAC_Enable(DAC0, true);
	DAC_SetBufferReadPointer(DAC0, 0U);
	// Valor de tensão entre 0 e 3,3 V
	uint16_t dacValue = (tensao_max * 4096.0f) / 3.31f;
	DAC_SetBufferValue(DAC0, 0U, dacValue);
}
void atualizar_dac(uint16_t dacValue) {
	if (dacValue < 0)
		dacValue = 0;
	if (dacValue > 4096)
		dacValue = 4096;
	// Valor de 12 bits entre 0 e 4096
	DAC_SetBufferValue(DAC0, 0U, dacValue);
}
void configurar_interrupcao_Hz(float Hz) {
	uint32_t usec = 1000000.0 / Hz;
	//	TPM_SetTimerPeriod(TPM1, USEC_TO_COUNT(usec, CLOCK_GetBusClkFreq()));
	//	TPM_EnableInterrupts(TPM1, kTPM_Chnl0InterruptEnable);
	//	TPM_StartTimer(TPM1, kTPM_SystemClock);

	/* Structure of initialize PIT */
	pit_config_t pitConfig;

	/* Board pin, clock, debug console init */
	//BOARD_InitHardware();
	PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);

	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0,
			USEC_TO_COUNT(usec, CLOCK_GetBusClkFreq()));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(PIT_IRQn);

	/* Start channel 0 */
	PRINTF("\r\nStarting channel No.0 ...");
	PIT_StartTimer(PIT, kPIT_Chnl_0);

}
void inicializar_PWMs() {
	tpm_config_t MeuPWM = { .enableDebugMode = false, .enableDoze = false,
			.enableReloadOnTrigger = false, .enableStartOnTrigger = false,
			.enableStopOnOverflow = false, .prescale = kTPM_Prescale_Divide_8,
			.useGlobalTimeBase =
			false, };

	//Inicializa TPM (Timer / PWM Module)
	TPM_Init(TPM0, &MeuPWM);
	//Configuração dos dois canais para o TPM2 (Vetor)
	tpm_chnl_pwm_signal_param_t MeuCanalPWM1[4] = { { .chnlNumber = 0,
			.dutyCyclePercent = 0, .level = kTPM_HighTrue }, { .chnlNumber = 1,
			.dutyCyclePercent = 0, .level = kTPM_HighTrue }, { .chnlNumber = 2,
			.dutyCyclePercent = 0, .level = kTPM_HighTrue }, { .chnlNumber = 3,
			.dutyCyclePercent = 0, .level = kTPM_HighTrue } };

	//Incializadao do PWM. Modulo TPM1, 2 Canais. Verificar que argumento 2 pode ser um vetor de estutura de canais (declarado acima)
	TPM_SetupPwm(TPM0, MeuCanalPWM1, 4, kTPM_EdgeAlignedPwm, 50,
			CLOCK_GetPllFllSelClkFreq());
	//Incializadao do PWM. Modulo TPM0, 1 Canal.

	TPM_StartTimer(TPM0, kTPM_SystemClock);

}


void inicializar_I2Cs() {

// Configuração Inicial de i2c0
	i2c_master_config_t I2C0_Config = { .baudRate_Bps = 400000, .enableMaster =
	true, .enableStopHold = false, };
	I2C_MasterInit(I2C0, &I2C0_Config, CLOCK_GetBusClkFreq());
// Configuração Inicial de i2c1
	i2c_master_config_t I2C1_Config = { .baudRate_Bps = 400000, .enableMaster =
	true, .enableStopHold = false, };
	I2C_MasterInit(I2C1, &I2C1_Config, CLOCK_GetBusClkFreq());

}
void delay(unsigned long n) {
	n = n * 1000;
	while (n > 0)
		n--;
}
void atualizar_motores(uint32_t teste_motor1, uint32_t teste_motor2,
		uint32_t teste_motor3) {

	TPM0->CONTROLS[0].CnV = teste_motor1;
	TPM0->CONTROLS[1].CnV = teste_motor2;
	TPM0->CONTROLS[2].CnV = teste_motor3;
}
void atualizar_motor_1 (float sinal_controle) {
	int valor_pwm = 1846.55 * sinal_controle + 1500;
	TPM0->CONTROLS[0].CnV = valor_pwm;
}
void atualizar_motor_2 (float sinal_controle) {
	int valor_pwm = 986.94 * sinal_controle + 1900;
	TPM0->CONTROLS[1].CnV = valor_pwm;
}
void atualizar_motor_3 (float sinal_controle) {
	int valor_pwm = 1368.99 * sinal_controle + 1100;
	TPM0->CONTROLS[2].CnV = valor_pwm;
}
void atualizar_motor_4 (float sinal_controle) {
	int valor_pwm = 2069.40 * sinal_controle + 1200;
	TPM0->CONTROLS[3].CnV = valor_pwm;
}

#endif /* PERIFERICOS_H_ */