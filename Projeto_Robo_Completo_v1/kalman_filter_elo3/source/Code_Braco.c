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

float valor_prbs[100] = { 1.3337, 1.5334, 1.0683, 0.6591, 1.4157, 0.6562,
		-0.5720, -0.3454, 1.2726, 0.4473, 1.2336, -0.1523, 0.5955, 0.7648,
		-0.5392, 0.7317, 0.1814, -0.5010, 0.4588, -0.1891, -0.3406, -0.1608,
		-0.2895, -0.1966, -0.5160, 0.7764, 0.0058, 0.5657, 0.9072, 0.4796,
		0.5596, 0.3619, -0.3387, 0.4605, 1.2514, 1.2970, -0.0195, -0.1543,
		0.6232, 0.7875, 0.3005, -0.1598, 1.4584, -0.4300, -0.3784, -0.2992,
		-0.2459, 0.7453, 0.6423, -0.4954, 1.4220, 0.9802, 1.0002, -0.4707,
		1.2676, 1.4289, 1.5380, 1.2643, 1.1043, 0.5107, -0.2216, 0.2603,
		-0.3169, -0.5416, 1.4393, 0.0481, 0.0356, 0.1171, 0.4097, 0.8047,
		-0.5540, 1.2279, 0.6102, 1.2538, 0.1497, 0.3638, -0.4907, -0.2227,
		0.8366, 0.1125, 1.3506, -0.3513, 1.5467, 0.5687, 0.9328, 1.5709, 0.0188,
		0.2951, 0.4048, 1.0572, 1.1755, -0.3904, -0.2205, 0.1754, -0.4853,
		0.5292, 0.1235, -0.2259, -0.1533, 1.3651 };
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
		atualizar_motor_3(valor_prbs[amostra]);
		k = 0;
		amostra++;
	}
//	printf("\n%d,%d,%d",tempo,k,amostra);

	calcular_angulo(&acc_1);
	valor_1 = 10000 * valor_prbs[amostra - 1];
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
	TPM0->CONTROLS[1].CnV = 4800;
	TPM0->CONTROLS[2].CnV = 4500;
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
//		atualizar_motores(teste_motor1, teste_motor2, teste_motor3);
	}
	return 0;
}
