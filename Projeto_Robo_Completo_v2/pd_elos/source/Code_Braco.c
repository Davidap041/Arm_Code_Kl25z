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
acc_data_t acc_2 = { .address = 0x68, .I2C = I2C0_BASE, };
acc_data_t acc_3 = { .address = 0x68, .I2C = I2C0_BASE, };
acc_data_t acc_4 = { .address = 0x68, .I2C = I2C0_BASE, };
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
uint32_t teste_motor2 = 3500;
uint32_t teste_motor3 = 4500;
uint32_t teste_motor4 = 6000;

// Variáveis Relacionadas ao ensaio de prbs
int time = 0; 		// Duração do Ensaio
int amostra = 0; 	// Amostra do PRBS
const int tempo_ident = 150;
int valor_pwm;
int tempo;
// Variáveis relacionadas ao filtro de kalman
Kalman_data kalman_0;
Kalman_data kalman_1;
Kalman_data kalman_2;
Kalman_data kalman_3;

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
int interrup = 0;

float angulos[4];

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
void inicializar_sensores(acc_data_t *acc_n) {
	// Instanciar Sensores
	bool acc_init = ACC_Init(acc_n);
	while (!acc_init) {
		printf("\nsensor nao respondeu.. ");
		inicializar_I2Cs();
		acc_init = ACC_Init(acc_n);
	}
}
void selecionar_sensor(int sensor) {
	GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_1_GPIO,
			1U << BOARD_INITPINS_Sensor_1_PIN);
	GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_2_GPIO,
			1U << BOARD_INITPINS_Sensor_2_PIN);
	GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_3_GPIO,
			1U << BOARD_INITPINS_Sensor_3_PIN);
	GPIO_SetPinsOutput(BOARD_INITPINS_Sensor_4_GPIO,
			1U << BOARD_INITPINS_Sensor_4_PIN);
	delay(10000);
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
	delay(10000);
}

void atualizar_angulo(int sensor) {
	if (sensor == 1) { // Motor da Base
		selecionar_sensor(1);
		inicializar_sensores(&acc_1);

		ACC_ReadRaw(&acc_1); // atualizar valores do sensor
		gyroZ = acc_1.gz * 1.3323e-04;
		ang_1 = motor_angulo_1();
		ang_motor = estimar_posicao(ang_1);
		gyroYrate = -gyroZ;
		kalAngleY = getAngle(&kalman_0, ang_motor, gyroYrate, Ts); // Calculate the angle using a Kalman filter
		angulos[0] = kalAngleY;
	}
	if (sensor == 2) { // Motor do eixo
		selecionar_sensor(2);
		inicializar_sensores(&acc_2);

		ACC_ReadRaw(&acc_2); // atualizar valores do sensor
		accX = acc_2.ax * g * ACC_RESOLUTION;
		accZ = acc_2.az * g * ACC_RESOLUTION;
		gyroY = acc_2.gy * 1.3323e-04;

		pitch = atan2f(-accX, accZ);
		gyroYrate = gyroY;
		// Calculate the angle using a Kalman filter
		kalAngleY = getAngle(&kalman_1, pitch, gyroYrate, Ts);
		angulos[1] = kalAngleY;
	}
	if (sensor == 3) {
		selecionar_sensor(3);
		inicializar_sensores(&acc_3);

		ACC_ReadRaw(&acc_3); // atualizar valores do sensor
		accX = acc_3.ax * g * ACC_RESOLUTION;
		accY = acc_3.ay * g * ACC_RESOLUTION;
		gyroZ = acc_3.gz * 1.3323e-04;

		pitch = atan2f(accX, accY);
		gyroYrate = gyroZ; // Convert to deg/s
		// Calculate the angle using a Kalman filter
		kalAngleY = getAngle(&kalman_2, pitch, gyroYrate, Ts);
		angulos[2] = kalAngleY;
	}
	if (sensor == 4) {
		selecionar_sensor(4);
		inicializar_sensores(&acc_4);

		ACC_ReadRaw(&acc_4); // atualizar valores do sensor
		accX = acc_4.ax * g * ACC_RESOLUTION;
		accY = acc_4.ay * g * ACC_RESOLUTION;
		gyroZ = acc_4.gz * 1.3323e-04;

		pitch = atan2f(accX, accY);
		gyroYrate = gyroZ; // Convert to deg/s
		kalAngleY = -getAngle(&kalman_3, pitch, gyroYrate, Ts); // Calculate the angle using a Kalman filter
		angulos[3] = kalAngleY;
	}
}

void rotina_interrupcao_1() {
	if (interrup == 0) {
		//	tempo++;
		//	if (time >= 0 && time < tempo_ident) {
		//		time++;
		//	} else {
		//		atualizar_motor_1(valor_prbs[amostra]);
		//		time = 0;
		//		amostra++;
		//	}
		valor_1 = 1000 * angulos[0];
		valor_2 = 1000 * angulos[1];
		valor_3 = 1000 * angulos[2];
		valor_4 = 1000 * angulos[3];

		printf("\n%d %d %d %d", valor_1, valor_2, valor_3, valor_4);

		if (j_sensor > 100) {
			if (i_sensor == 3) {
				i_sensor = 0;
			} else {
				i_sensor++;
			}
			j_sensor = 0;
		} else
			j_sensor++;

		int valor_da = 1304.45 * angulos[2] + 2048;
		atualizar_dac(valor_da);
		interrup = 1;

	}
	if (interrup == 1) {
		atualizar_angulo(2);
		atualizar_angulo(4);
		interrup = 0;
	} else {
		printf("Problema com a interrupção ......");
	}

}
void rotina_inter(int interrup_atual) {
	if (interrup_atual == 1) {
		valor_1 = 1000 * angulos[0];
		valor_2 = 1000 * angulos[1];
		valor_3 = 1000 * angulos[2];
		valor_4 = 1000 * angulos[3];

		printf("\n%d %d %d %d", valor_1, valor_2, valor_3, valor_4);

		if (j_sensor > 100) {
			if (i_sensor == 3) {
				i_sensor = 0;
			} else {
				i_sensor++;
			}
			j_sensor = 0;
		} else
			j_sensor++;

		int valor_da = 1304.45 * angulos[3] + 2048;
		atualizar_dac(valor_da);
	}
	if (interrup_atual == 2) {
		atualizar_angulo(3);
		atualizar_angulo(4);
	}
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
	atualizar_motores(4500, 3600, 6000, 6000);
	delay(10000);

	inicializar_I2Cs();
	printf("\nComunicacao I2C inicializada ");

	selecionar_sensor(1);
	inicializar_sensores(&acc_1);
	printf("\nComunicacao inicializada : Sensor 1 ");
	selecionar_sensor(2);
	inicializar_sensores(&acc_2);
	printf("\nComunicacao inicializada : Sensor 2 ");
	selecionar_sensor(3);
	inicializar_sensores(&acc_3);
	printf("\nComunicacao inicializada : Sensor 3 ");
	selecionar_sensor(3);
	inicializar_sensores(&acc_4);
	printf("\nComunicacao inicializada : Sensor 4 ");

	printf("\nTodos os Sensores okay");
	inicializar_dac(3.14);
//	inicializar_Kalman();
	printf("\nInterrupcao_configurada");
	configurar_interrupcao_Hz(100);

	while (1) {
		atualizar_motores(teste_motor1, teste_motor2, teste_motor3,
				teste_motor4);
//		rotina_inter(interrup);
	}
	return 0;
}
