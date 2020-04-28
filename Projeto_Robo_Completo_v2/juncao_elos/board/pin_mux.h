/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define SOPT5_UART0RXSRC_UART_RX 0x00u /*!<@brief UART0 receive data source select: UART0_RX pin */
#define SOPT5_UART0TXSRC_UART_TX 0x00u /*!<@brief UART0 transmit data source select: UART0_TX pin */

/*! @name PORTA2 (number 28), J1[4]/D1/UART0_TX
  @{ */
#define BOARD_INITPINS_DEBUG_UART_TX_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_INITPINS_DEBUG_UART_TX_PIN 2U     /*!<@brief PORTA pin index: 2 */
                                                /* @} */

/*! @name PORTA1 (number 27), J1[2]/D0/UART0_RX
  @{ */
#define BOARD_INITPINS_DEBUG_UART_RX_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_INITPINS_DEBUG_UART_RX_PIN 1U     /*!<@brief PORTA pin index: 1 */
                                                /* @} */

/*! @name PORTD0 (number 73), J2[6]/D10
  @{ */
#define BOARD_INITPINS_PWM1_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_PWM1_PIN 0U     /*!<@brief PORTD pin index: 0 */
                                       /* @} */

/*! @name PORTB0 (number 43), J10[2]/A0
  @{ */
#define BOARD_INITPINS_Sensor_1_e_2_SCL_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_Sensor_1_e_2_SCL_PIN 0U     /*!<@brief PORTB pin index: 0 */
                                                   /* @} */

/*! @name PORTC1 (number 56), J10[12]/U6[31]/A5
  @{ */
#define BOARD_INITPINS_Sensor_3_e_4_SCL_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_Sensor_3_e_4_SCL_PIN 1U     /*!<@brief PORTC pin index: 1 */
                                                   /* @} */

/*! @name PORTC2 (number 57), J10[10]/A4
  @{ */
#define BOARD_INITPINS_Sensor_3e_4_SDA_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_Sensor_3e_4_SDA_PIN 2U     /*!<@brief PORTC pin index: 2 */
                                                  /* @} */

/*! @name PORTE30 (number 22), J10[11]
  @{ */
#define BOARD_INITPINS_Saida_DAC_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_Saida_DAC_PIN 30U    /*!<@brief PORTE pin index: 30 */
                                            /* @} */

/*! @name PORTB1 (number 44), J10[4]/A1
  @{ */
#define BOARD_INITPINS_Sensor_1_e_2_SDA_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_Sensor_1_e_2_SDA_PIN 1U     /*!<@brief PORTB pin index: 1 */
                                                   /* @} */

/*! @name PORTA4 (number 30), J1[10]/D4
  @{ */
#define BOARD_INITPINS_PWM2_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_INITPINS_PWM2_PIN 4U     /*!<@brief PORTA pin index: 4 */
                                       /* @} */

/*! @name PORTA5 (number 31), J1[12]/D5
  @{ */
#define BOARD_INITPINS_PWM3_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_INITPINS_PWM3_PIN 5U     /*!<@brief PORTA pin index: 5 */
                                       /* @} */

/*! @name PORTD3 (number 76), J2[10]/D12
  @{ */
#define BOARD_INITPINS_PWM4_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_PWM4_PIN 3U     /*!<@brief PORTD pin index: 3 */
                                       /* @} */

/*! @name PORTD4 (number 77), J1[6]/D2
  @{ */
#define BOARD_INITPINS_PWM5_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_PWM5_PIN 4U     /*!<@brief PORTD pin index: 4 */
                                       /* @} */

/*! @name PORTD5 (number 78), J2[4]/D9
  @{ */
#define BOARD_INITPINS_PWM6_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_PWM6_PIN 5U     /*!<@brief PORTD pin index: 5 */
                                       /* @} */

/*! @name PORTC10 (number 67), J1[13]
  @{ */
#define BOARD_INITPINS_Sensor_2_FGPIO FGPIOC /*!<@brief FGPIO device name: FGPIOC */
#define BOARD_INITPINS_Sensor_2_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITPINS_Sensor_2_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_Sensor_2_PIN 10U    /*!<@brief PORTC pin index: 10 */
                                           /* @} */

/*! @name PORTC11 (number 68), J1[15]
  @{ */
#define BOARD_INITPINS_Sensor_1_FGPIO FGPIOC /*!<@brief FGPIO device name: FGPIOC */
#define BOARD_INITPINS_Sensor_1_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITPINS_Sensor_1_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_Sensor_1_PIN 11U    /*!<@brief PORTC pin index: 11 */
                                           /* @} */

/*! @name PORTC5 (number 62), J1[9]
  @{ */
#define BOARD_INITPINS_Sensor_4_FGPIO FGPIOC /*!<@brief FGPIO device name: FGPIOC */
#define BOARD_INITPINS_Sensor_4_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITPINS_Sensor_4_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_Sensor_4_PIN 5U     /*!<@brief PORTC pin index: 5 */
                                           /* @} */

/*! @name PORTC6 (number 63), J1[11]
  @{ */
#define BOARD_INITPINS_Sensor_3_FGPIO FGPIOC /*!<@brief FGPIO device name: FGPIOC */
#define BOARD_INITPINS_Sensor_3_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITPINS_Sensor_3_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_Sensor_3_PIN 6U     /*!<@brief PORTC pin index: 6 */
                                           /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
