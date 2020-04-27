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
// Variáveis para o cálculo do Ângulo do Giroscópio
float angGy = 0;
float angGy_aux = 0;
// Variável do ângulo do acelerômetro
float acc_pitch = 0;
// Variável do ângulo da fusão sensorial

float posi_x = 0;
float posi_y = 0;
float posi_z = 0;

float fused = 0;
float aleatorio;

int valor_1;
int valor_2;
int valor_3;
int fusao;
// definição de Variáveis sobre os ângulos
float alfa = 0.92f;
float seno = 0.003;

//Variáveis de Testes e DEBUGs
uint32_t teste_motor1 = 4500;
uint32_t teste_motor2 = 4000;
uint32_t teste_motor3 = 4000;
// Variáveis Relacionadas ao ensaio de prbs
int k = 0; 		// Duração do Ensaio
int amostra = 0; 	// Amostra do PRBS
const int tempo_ident = 150;
int valor_pwm;
int tempo;
// Variáveis relacionadas ao filtro de kalman
Kalman_data kalmanX;
Kalman_data kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

double pitch;
double gyroYrate;

float valor_prbs[100] = { -1.7108, -1.8392, -0.7411, -1.8499, -1.4536, -0.6995,
		-0.9547, -1.3331, -1.9121, -1.9225, -0.7842, -1.9305, -1.9116, -1.2464,
		-1.6904, -0.7621, -1.1567, -1.8532, -1.6790, -1.9149, -1.4866, -0.6124,
		-1.7593, -1.8789, -1.5190, -1.6304, -1.6098, -1.1150, -1.4862, -0.8034,
		-1.5575, -0.6069, -0.9525, -0.6271, -0.6990, -1.7231, -1.5417, -1.0091,
		-1.9018, -0.6106, -1.1806, -1.1000, -1.6414, -1.6832, -0.8255, -1.2526,
		-1.1903, -1.4733, -1.5622, -1.6261, -0.9512, -1.5204, -1.4857, -0.7913,
		-0.7298, -1.2647, -1.9152, -1.0419, -1.3872, -0.8776, -1.6213, -0.9217,
		-1.2754, -1.5477, -1.8182, -1.9146, -1.3336, -0.7575, -0.7725, -0.9251,
		-1.7474, -0.9205, -1.7101, -0.9054, -1.8723, -1.0555, -0.8392, -0.9160,
		-1.4306, -1.2293, -1.0578, -1.7335, -1.3872, -1.3371, -1.8552, -0.9650,
		-1.6297, -1.6248, -1.0984, -1.3626, -0.6690, -0.6381, -1.3104, -1.6606,
		-1.8790, -0.7452, -1.3640, -1.2238, -0.5788, -1.0373 };

void inicializar_Kalman() {
	accX = acc_1.ax * g * ACC_RESOLUTION;
	accY = acc_1.ay * g * ACC_RESOLUTION;
	accZ = acc_1.az * g * ACC_RESOLUTION;

	gyroX = acc_1.gx * 1.3323e-04;
	gyroY = acc_1.gy * 1.3323e-04;
	gyroZ = acc_1.gz * 1.3323e-04;

	pitch = atan2f(-accX, accZ);
	gyroYrate = gyroY;

	setAngle(&kalmanY, pitch);
	gyroYangle = pitch;
	compAngleY = pitch;

}

float calcular_angulo(acc_data_t *acc_n) {
	ACC_ReadRaw(acc_n); // atualizar valores do sensor

	accX = acc_1.ax * g * ACC_RESOLUTION;
	accY = acc_1.ay * g * ACC_RESOLUTION;
	accZ = acc_1.az * g * ACC_RESOLUTION;

	gyroX = acc_1.gx * 1.3323e-04;
	gyroY = acc_1.gy * 1.3323e-04;
	gyroZ = acc_1.gz * 1.3323e-04;

	pitch = atan2f(-accX, accZ);
	gyroYrate = gyroY;

	// Calculate the angle using a Kalman filter
	kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, Ts);

	return kalAngleY;
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
	if (k >= 0 && k < tempo_ident) {
		k++;
	} else {
		atualizar_motor_2(valor_prbs[amostra]);
		k = 0;
		amostra++;
	}
//	printf("\n%d,%d,%d", tempo, k, amostra);

	calcular_angulo(&acc_1);
	valor_1 = 10000 * valor_prbs[amostra-1];
	valor_2 = 10000 * kalAngleY;
//	valor_1 = 10000 * gyroYrate;
//	valor_2 = 10000 * pitch;
//	valor_3 = 10000 * kalAngleY;

	printf("\n%d %d %d %d", tempo, valor_1, valor_2, amostra);
	int valor_da = 1304.45 * kalAngleY + 2048;
	atualizar_dac(valor_da);
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
	TPM0->CONTROLS[0].CnV = 4500;
	TPM0->CONTROLS[1].CnV = 1900;
	TPM0->CONTROLS[2].CnV = 6000;
	delay(10000);

	inicializar_I2Cs();
	printf("\nComunicacao I2C inicializada ");
	inicializar_sensores(&acc_1);

	printf("\nComunicacao Sensores inicializada ");
//	calibrar_sensor(&acc_1);
	//	call_calibra(&acc_2);
	//	call_calibra(&acc_3);
	//	call_calibra(&acc_4);
	printf("\nsensores calibrados");
	inicializar_dac(3.14);
	inicializar_Kalman();
	configurar_interrupcao_Hz(100);

	//configurar_interrupcao_Hz(100);

	while (1) {
	}
	return 0;
}
