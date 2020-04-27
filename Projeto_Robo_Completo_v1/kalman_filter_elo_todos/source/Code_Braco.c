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
const int tempo_ident = 100;
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

float valor_prbs[100] = { .7247, 0.2208, 1.3012, -0.6668, 0.8955, 0.8729,
		0.4482, -0.4715, 0.5343, -0.1897, -0.5935, -0.4024, 1.2594, -0.7461,
		-0.3296, -0.7892, 0.1556, -0.8878, 1.2648, -0.4412, -0.6927, -0.1716,
		0.1905, -0.6725, 1.5040, -0.1114, -0.1960, -0.7690, -0.1938, -0.8072,
		0.3108, 0.9342, 0.6167, -0.7012, -0.7232, 0.9727, 1.2842, 0.3798,
		-0.6543, 1.0910, -0.0967, -0.2042, 0.8974, -0.8949, -0.8021, 0.7065,
		0.5495, 0.3611, 0.8570, 0.8023, 0.9828, -0.2188, 0.7664, 0.4356, 0.0455,
		-0.7701, 0.9799, -0.0980, 0.5602, 0.8851, -0.6648, -0.6087, 0.4182,
		0.2616, 1.2485, 1.0256, 0.8682, -0.7951, -0.7426, -0.7045, 1.0241,
		1.3764, 0.7450, -0.5984, 0.8400, -0.6514, -0.6340, 0.6402, -0.1193,
		0.6721, 0.9043, 0.5001, 0.8821, -0.3482, 0.8697, 1.4436, 1.1911,
		-0.7101, -0.0277, -0.0210, 0.7481, 0.5361, 1.0022, -0.0248, -0.4184,
		-0.7090, 0.9598, -0.4192, 0.0255, 0.4236 };
void inicializar_Kalman() {
	accX = acc_1.ax * g * ACC_RESOLUTION;
	accY = acc_1.ay * g * ACC_RESOLUTION;
	accZ = acc_1.az * g * ACC_RESOLUTION;

	gyroX = acc_1.gx * 1.3323e-04;
	gyroY = acc_1.gy * 1.3323e-04;
	gyroZ = acc_1.gz * 1.3323e-04;

	pitch = atan2f(accX, accY);
	gyroYrate = gyroZ;

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

	pitch = atan2f(accX, accY);
	gyroYrate = gyroZ; // Convert to deg/s

	kalAngleY = -getAngle(&kalmanY, pitch, gyroYrate, Ts); // Calculate the angle using a Kalman filter

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
		atualizar_motor_3(valor_prbs[amostra]);
		k = 0;
		amostra++;
	}
//	printf("\n%d,%d,%d",tempo,k,amostra);

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
	atualizar_motores(4500, 4800, 4500);
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

	while (1) {
//		atualizar_motores(teste_motor1, teste_motor2, teste_motor3);
	}
	return 0;
}
