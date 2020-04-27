################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS_driver/fsl_i2c_cmsis.c 

OBJS += \
./CMSIS_driver/fsl_i2c_cmsis.o 

C_DEPS += \
./CMSIS_driver/fsl_i2c_cmsis.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS_driver/%.o: ../CMSIS_driver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MKL25Z128VLK4 -DCPU_MKL25Z128VLK4_cm0plus -DSDK_OS_BAREMETAL -DFSL_RTOS_BM -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\drivers" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\board" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\CMSIS_driver" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\CMSIS" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\utilities" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\board" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\source" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\drivers" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\CMSIS" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\utilities" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\CMSIS_driver" -I"C:\Users\davia\OneDrive\Documentos\Mestrado_Softwares\Braco_Codigos\Projeto_Robo_Completo_v1\kalman_filter_elo1(n)\startup" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


