################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/motor/PI_control.c \
../Core/Src/motor/main.c 

OBJS += \
./Core/Src/motor/PI_control.o \
./Core/Src/motor/main.o 

C_DEPS += \
./Core/Src/motor/PI_control.d \
./Core/Src/motor/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/motor/%.o Core/Src/motor/%.su Core/Src/motor/%.cyclo: ../Core/Src/motor/%.c Core/Src/motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-motor

clean-Core-2f-Src-2f-motor:
	-$(RM) ./Core/Src/motor/PI_control.cyclo ./Core/Src/motor/PI_control.d ./Core/Src/motor/PI_control.o ./Core/Src/motor/PI_control.su ./Core/Src/motor/main.cyclo ./Core/Src/motor/main.d ./Core/Src/motor/main.o ./Core/Src/motor/main.su

.PHONY: clean-Core-2f-Src-2f-motor

