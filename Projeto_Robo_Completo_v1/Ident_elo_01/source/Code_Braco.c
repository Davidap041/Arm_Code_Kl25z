#include <mpu_6050.h>
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
//#include "fsl_pit.h"
#include "fsl_gpio.h"
#include "function.h"
#include "perifericos.h"
#include "Kalman.h"
//#include "arm_math.h"

// Variáveis Globais
// Definição do endereçamento dos 4 sensores
acc_data_t acc_1 = { .address = 0x68, .I2C = I2C0_BASE, };
acc_data_t acc_2 = { .address = 0x69, .I2C = I2C0_BASE, };
acc_data_t acc_3 = { .address = 0x68, .I2C = I2C1_BASE, };
acc_data_t acc_4 = { .address = 0x69, .I2C = I2C1_BASE, };

// Variável do ângulo do acelerômetro
float acc_pitch = 0;
// Variável do ângulo da fusão sensorial

float posi_x = 0;
float posi_y = 0;
float posi_z = 0;
int valora = 0;

float fusedx = 0;
float fusedy = 0;
float fusedz = 0;
float aleatorio;

int fusao;
// definição de Variáveis sobre os ângulos
float alfa = 0.92f;
float seno = 0.003;

//Variáveis de Testes e DEBUGs
uint32_t teste_motor1 = 4500;
uint32_t teste_motor2 = 4000;
uint32_t teste_motor3 = 4000;
int valor_1;
int valor_2;
int valor_3;
int valor_4;
// Variáveis Relacionadas ao ensaio de prbs
int k = 100; 		// Duração do Ensaio
int amostra = 0; 	// Amostra do PRBS
const int tempo_ident = 100;
int valor_pwm;
int tempo;

float valor_prbs[100] = { 1.192, 1.0019, 3.0962, 2.2551, 1.2974, 0.3097, 2.3065,
		2.0011, 0.23186, 0.3784, 3.0822, 1.56, 0.070379, 0.16903, 0.44234,
		2.8055, 1.4627, 1.7611, 1.5526, 0.21285, 2.8186, 0.9061, 0.84481,
		1.8658, 1.4943, 1.1565, 2.0586, 2.9459, 1.9481, 0.88812, 0.64427,
		1.3789, 0.085566, 2.7512, 1.9157, 0.63928, 1.6325, 0.16901, 2.7073,
		1.3908, 1.7207, 1.7799, 2.1364, 1.1661, 0.24564, 1.4329, 0.15023,
		2.3181, 0.11932, 2.9963, 2.331, 2.9436, 1.612, 0.75644, 0.81629, 2.3832,
		3.1191, 1.1201, 2.364, 0.34555, 1.8747, 1.3521, 2.2945, 0.82009, 0.2977,
		1.416, 2.0098, 0.4146, 1.4219, 2.0479, 2.5968, 0.96736, 1.2634, 2.7765,
		2.1998, 0.75948, 2.3859, 0.91351, 0.87116, 0.019179, 1.1766, 1.372,
		0.9555, 0.9133, 0.7615, 2.9412, 2.701, 1.2473, 1.5054, 1.7741, 1.5374,
		0.84721, 3.1078, 0.57674, 2.7056, 0.10247, 1.0423, 2.3511, 2.0233,
		0.53141 };

float calcular_angulo(acc_data_t *acc_n) {
	ACC_ReadRaw(acc_n); // atualizar valores do sensor
	// velocidade angular do giroscopio
//	acc_n->v_ang_x = (acc_n->gx - acc_n->M_gx) * 1.3323e-04;
//	acc_n->v_ang_y = (acc_n->gy - acc_n->M_gy) * 1.3323e-04;
	acc_n->v_ang_z = (acc_n->gz - acc_n->M_gz) * 1.3323e-04;

	// calcula o ângulo com a integral da velocidade ângular
//	acc_n->angGx_atual = acc_n->angGx_anterior + Ts * acc_n->v_ang_x;
//	acc_n->angGx_anterior = acc_n->angGx_atual;
//	acc_n->angGy_atual = acc_n->angGy_anterior + Ts * acc_n->v_ang_y;
//	acc_n->angGy_anterior = acc_n->angGy_atual;
	acc_n->angGz_atual = acc_n->angGz_anterior + Ts * acc_n->v_ang_z;
	acc_n->angGz_anterior = acc_n->angGz_atual;

	// acelerações do acelerômetro
	acc_n->ang_pitch_x = GetAccPitch(acc_n, 1);
	acc_n->ang_pitch_y = GetAccPitch(acc_n, 2);
	acc_n->ang_pitch_z = GetAccPitch(acc_n, 3);

	// calcula angulo da fusão do giroscopio com acelerometro
	fusedx = alfa * (fusedx + acc_n->v_ang_z * Ts)
			+ (1 - alfa) * acc_n->ang_pitch_x;

	fusedy = (alfa) * (fusedy + acc_n->v_ang_z * Ts)
			+ ((1 - alfa) / 2) * acc_n->ang_pitch_y;

	fusedz = alfa * (fusedz + acc_n->v_ang_z * Ts)
			+ (1 - alfa) * acc_n->ang_pitch_z;

//	fused = alfa * (fused + acc_n->v_ang_z * Ts)
//				+ (1 - alfa) * acc_n->ang_pitch_y;

	return fusedx;
}

void inicializar_sensores(acc_data_t *acc_n) {
	// Instanciar Sensores
	bool acc_init = ACC_Init(acc_n);
	while (!acc_init) {
		printf("\nsensor nao respondeu.. ");
		inicializar_I2Cs();
		acc_init = ACC_Init(acc_n);
	}
	//printf("\nsensor respondeu :D ");
}

void rotina_interrupcao_1() {
	tempo++;
//	if (k > 0 && k < tempo_ident) {
//		k++;
//	} else {
//		atualizar_motor_1(valor_prbs[amostra]);
//		k = tempo_ident;
//		amostra++;
//	}

	if (tempo > 10) {
		atualizar_motores(teste_motor1, teste_motor2, teste_motor3);
		valora=2962;
	}
	calcular_angulo(&acc_1);
//	valor_1 = -10000 * acc_1.v_ang_z;
	valor_2 = 10000 * acc_1.angGz_atual;	// ****
//	valor_3 = 10000 * acc_1.ang_pitch_y;
//	valor_4 = -10000 * acc_1.angGz_atual;

//	printf("\n%d, %d, %d, %d", valor_1, valor_2, valor_3, valor_4);
	printf("\n %d %d", valora, valor_2);
	//int valor_da = 10000 * angGz_atual;
	atualizar_dac(valor_2);

}
void PIT_DriverIRQHandler() {
	if (PIT_GetStatusFlags(PIT, kPIT_Chnl_0)) {
		PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
		rotina_interrupcao_1();
	}
}
int main(void) {
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	inicializar_PWMs();
	atualizar_motores(1500, 4000, 4500);
	delay(10000);

	inicializar_I2Cs();
	printf("\nComunicacao I2C inicializada ");
	inicializar_sensores(&acc_1);

	printf("\nComunicacao Sensores inicializada ");
	calibrar_sensor(&acc_1);
	//	call_calibra(&acc_2);
	//	call_calibra(&acc_3);
	//	call_calibra(&acc_4);
	printf("\nsensores calibrados");
	inicializar_dac(3.14);
	configurar_interrupcao_Hz(100);

	//configurar_interrupcao_Hz(100);

	while (1) {

		__asm volatile ("nop");
	}
	return 0;
}
