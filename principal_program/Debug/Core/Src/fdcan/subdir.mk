################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fdcan/main.c \
../Core/Src/fdcan/trcv_buffer.c 

OBJS += \
./Core/Src/fdcan/main.o \
./Core/Src/fdcan/trcv_buffer.o 

C_DEPS += \
./Core/Src/fdcan/main.d \
./Core/Src/fdcan/trcv_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fdcan/%.o Core/Src/fdcan/%.su Core/Src/fdcan/%.cyclo: ../Core/Src/fdcan/%.c Core/Src/fdcan/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fdcan

clean-Core-2f-Src-2f-fdcan:
	-$(RM) ./Core/Src/fdcan/main.cyclo ./Core/Src/fdcan/main.d ./Core/Src/fdcan/main.o ./Core/Src/fdcan/main.su ./Core/Src/fdcan/trcv_buffer.cyclo ./Core/Src/fdcan/trcv_buffer.d ./Core/Src/fdcan/trcv_buffer.o ./Core/Src/fdcan/trcv_buffer.su

.PHONY: clean-Core-2f-Src-2f-fdcan

