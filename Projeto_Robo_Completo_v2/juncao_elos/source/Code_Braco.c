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
int valor_4;

int fusao;
// definição de Variáveis sobre os ângulos
float alfa = 0.92f;
float seno = 0.003;

//Variáveis de Testes e DEBUGs
uint32_t teste_motor1 = 4500;
uint32_t teste_motor2 = 3600;
uint32_t teste_motor3 = 6000;
// Variáveis Relacionadas ao ensaio de prbs
int time = 0; 		// Duração do Ensaio
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

float angGy;
float angGy1;

float v_ang;

float ang_z_ant;
float ang_z_agr;

int k = 6;
float y[5], u[2];

float ang_1;
float ang_motor;
float vetor_sensor[4];

int i_sensor = 1;
int j_sensor = 0;

float valor_prbs[100] = { -0.3941, 0.7818, 0.1792, 0.0920, 0.9937, -0.5556,
		0.6011, 0.5926, -0.3234, 0.1364, -1.0709, -1.1246, 0.0455, 0.6550,
		1.0350, -0.9382, 0.1389, -0.1051, -1.2278, -0.4297, -0.8590, 0.6921,
		-0.4933, 0.0400, -0.8505, 0.2202, -0.6117, 0.3481, 0.4343, 0.5789,
		-0.1514, -1.0513, -0.6951, 0.9843, -0.8831, 0.7695, 0.0641, 1.1874,
		-1.0652, -0.1707, -0.9953, 1.1034, -1.2456, 0.6446, 0.7486, 0.8747,
		-1.0498, -0.2760, -0.6193, 0.7063, -0.1983, 0.9777, -0.8108, -0.6096,
		-0.8999, -0.9231, 0.8762, 0.1656, 0.0923, -0.9013, 0.8363, 0.2695,
		-0.3958, 0.0025, -0.2710, -1.0706, -0.6683, -0.9544, -0.8057, -0.6682,
		-0.2331, -1.1352, 0.9582, 1.0614, -0.0525, -0.0564, -0.4283, 0.9517,
		-0.3509, -0.9841, 0.6577, -0.3006, -0.6639, -0.2658, -1.0203, -0.9331,
		1.0547, 1.0893, 0.1545, -1.1103, -0.6809, -0.3904, 0.7582, -1.2192,
		-1.1514, -0.8423, 0.3359, 0.5386, 0.3325, -0.1505 };
float estimar_posicao(float entrada) {

//    0.01931 z^-1
//-------------------------------------------------------------------
//1 - 3.635 z^-1 + 5.683 z^-2 - 4.721 z^-3 + 2.068 z^-4 - 0.3792 z^-5

	y[k - 5] = y[k - 4];
	y[k - 4] = y[k - 3];
	y[k - 3] = y[k - 2];
	y[k - 2] = y[k - 1];
	y[k - 1] = y[k];

	u[k - 1] = u[k];
	u[k] = entrada;

	y[k] = b0 * u[k] + b1 * u[k - 1] - a1 * y[k - 1] - a2 * y[k - 2]
			- a3 * y[k - 3] - a4 * y[k - 4] - a5 * y[k - 5];

	return y[k];
}
void selecionar_sensor(int sensor) {
	if (sensor == 1) {
		GPIO_ClearPinsOutput(BOARD_INITPINS_Sensor_1_GPIO,
				1U << BOARD_INITPINS_Sensor_1_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_2_GPIO,
				1U << BOARD_INITPINS_Sensor_2_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_3_GPIO,
				1U << BOARD_INITPINS_Sensor_3_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_4_GPIO,
				1U << BOARD_INITPINS_Sensor_4_PIN);
	}
	if (sensor == 2) {
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_1_GPIO,
				1U << BOARD_INITPINS_Sensor_1_PIN);
		GPIO_ClearPinsOutput(BOARD_INITPINS_Sensor_2_GPIO,
				1U << BOARD_INITPINS_Sensor_2_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_3_GPIO,
				1U << BOARD_INITPINS_Sensor_3_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_4_GPIO,
				1U << BOARD_INITPINS_Sensor_4_PIN);
	}
	if (sensor == 3) {
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_1_GPIO,
				1U << BOARD_INITPINS_Sensor_1_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_2_GPIO,
				1U << BOARD_INITPINS_Sensor_2_PIN);
		GPIO_ClearPinsOutput(BOARD_INITPINS_Sensor_3_GPIO,
				1U << BOARD_INITPINS_Sensor_3_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_4_GPIO,
				1U << BOARD_INITPINS_Sensor_4_PIN);
	}
	if (sensor == 4) {
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_1_GPIO,
				1U << BOARD_INITPINS_Sensor_1_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_2_GPIO,
				1U << BOARD_INITPINS_Sensor_2_PIN);
		GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_3_GPIO,
				1U << BOARD_INITPINS_Sensor_3_PIN);
		GPIO_ClearPinsOutput(BOARD_INITPINS_Sensor_4_GPIO,
				1U << BOARD_INITPINS_Sensor_4_PIN);
	}

}

float calcular_angulo(int sensor) {
	if (sensor == 1) { // Motor da Base
		selecionar_sensor(1);
		delay(1);
		ACC_ReadRaw(&acc_1); // atualizar valores do sensor
		gyroZ = acc_1.gz * 1.3323e-04;
		ang_1 = motor_angulo_1();
		ang_motor = estimar_posicao(ang_1);
		gyroYrate = -gyroZ;
		kalAngleY = getAngle(&kalmanY, ang_motor, gyroYrate, Ts); // Calculate the angle using a Kalman filter
	}
	if (sensor == 2) { // Motor do eixo
		selecionar_sensor(2);
		delay(1);
		ACC_ReadRaw(&acc_1); // atualizar valores do sensor
		accX = acc_1.ax * g * ACC_RESOLUTION;
		accZ = acc_1.az * g * ACC_RESOLUTION;
		gyroY = acc_1.gy * 1.3323e-04;

		pitch = atan2f(-accX, accZ);
		gyroYrate = gyroY;
		// Calculate the angle using a Kalman filter
		kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, Ts);
	}
	if (sensor == 3) {
		selecionar_sensor(3);
		delay(1);
		ACC_ReadRaw(&acc_1); // atualizar valores do sensor
		accX = acc_1.ax * g * ACC_RESOLUTION;
		accY = acc_1.ay * g * ACC_RESOLUTION;
		gyroZ = acc_1.gz * 1.3323e-04;

		pitch = atan2f(accX, accY);
		gyroYrate = gyroZ; // Convert to deg/s
		// Calculate the angle using a Kalman filter
		kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, Ts);
	}
	if (sensor == 4) {
		selecionar_sensor(4);
		delay(1);
		ACC_ReadRaw(&acc_1); // atualizar valores do sensor
		accX = acc_1.ax * g * ACC_RESOLUTION;
		accY = acc_1.ay * g * ACC_RESOLUTION;
		gyroZ = acc_1.gz * 1.3323e-04;

		pitch = atan2f(accX, accY);
		gyroYrate = gyroZ; // Convert to deg/s
		kalAngleY = -getAngle(&kalmanY, pitch, gyroYrate, Ts); // Calculate the angle using a Kalman filter
	}

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
}

void rotina_interrupcao_1() {
//	tempo++;
//	if (time >= 0 && time < tempo_ident) {
//		time++;
//	} else {
//		atualizar_motor_1(valor_prbs[amostra]);
//		time = 0;
//		amostra++;
//	}
	vetor_sensor[1] = calcular_angulo(1);
//	vetor_sensor[2] = calcular_angulo(2);
//	vetor_sensor[3] = calcular_angulo(3);
//	vetor_sensor[4] = calcular_angulo(4);

	valor_1 = 1000 * vetor_sensor[1];
	valor_2 = 1000 * vetor_sensor[2];
	valor_3 = 1000 * vetor_sensor[3];
	valor_4 = 1000 * vetor_sensor[4];

	printf("\n%d %d %d %d", valor_1, valor_2, valor_3, valor_4);

	if (j_sensor > 100) {
		if (i_sensor == 4) {
			i_sensor = 1;
		} else {
			i_sensor++;
		}
		j_sensor = 0;
	} else
		j_sensor++;

	int valor_da = 1304.45 * vetor_sensor[i_sensor] + 2048;
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
	atualizar_motores(4500, 3600, 6000);
	delay(10000);

	inicializar_I2Cs();
	printf("\nComunicacao I2C inicializada ");

	selecionar_sensor(1);
	inicializar_sensores(&acc_1);
	printf("\nComunicacao inicializada : Sensor 1 ");
	selecionar_sensor(2);
	inicializar_sensores(&acc_1);
	printf("\nComunicacao inicializada : Sensor 2 ");
	selecionar_sensor(3);
	inicializar_sensores(&acc_1);
	printf("\nComunicacao inicializada : Sensor 3 ");
	selecionar_sensor(4);
	inicializar_sensores(&acc_1);
	printf("\nComunicacao inicializada : Sensor 4 ");

	printf("\nTodos os Sensores okay");
	inicializar_dac(3.14);
//	inicializar_Kalman();
	configurar_interrupcao_Hz(100);

//configurar_interrupcao_Hz(100);

	while (1) {
//		atualizar_motores(teste_motor1, teste_motor2, teste_motor3);
	}
	return 0;
}
