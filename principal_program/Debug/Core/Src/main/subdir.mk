################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main/adc.c \
../Core/Src/main/const_and_error.c \
../Core/Src/main/it.c \
../Core/Src/main/main.c \
../Core/Src/main/map.c \
../Core/Src/main/vec.c \
../Core/Src/main/vehicle.c \
../Core/Src/main/vehicle2.c 

OBJS += \
./Core/Src/main/adc.o \
./Core/Src/main/const_and_error.o \
./Core/Src/main/it.o \
./Core/Src/main/main.o \
./Core/Src/main/map.o \
./Core/Src/main/vec.o \
./Core/Src/main/vehicle.o \
./Core/Src/main/vehicle2.o 

C_DEPS += \
./Core/Src/main/adc.d \
./Core/Src/main/const_and_error.d \
./Core/Src/main/it.d \
./Core/Src/main/main.d \
./Core/Src/main/map.d \
./Core/Src/main/vec.d \
./Core/Src/main/vehicle.d \
./Core/Src/main/vehicle2.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/main/%.o Core/Src/main/%.su Core/Src/main/%.cyclo: ../Core/Src/main/%.c Core/Src/main/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-main

clean-Core-2f-Src-2f-main:
	-$(RM) ./Core/Src/main/adc.cyclo ./Core/Src/main/adc.d ./Core/Src/main/adc.o ./Core/Src/main/adc.su ./Core/Src/main/const_and_error.cyclo ./Core/Src/main/const_and_error.d ./Core/Src/main/const_and_error.o ./Core/Src/main/const_and_error.su ./Core/Src/main/it.cyclo ./Core/Src/main/it.d ./Core/Src/main/it.o ./Core/Src/main/it.su ./Core/Src/main/main.cyclo ./Core/Src/main/main.d ./Core/Src/main/main.o ./Core/Src/main/main.su ./Core/Src/main/map.cyclo ./Core/Src/main/map.d ./Core/Src/main/map.o ./Core/Src/main/map.su ./Core/Src/main/vec.cyclo ./Core/Src/main/vec.d ./Core/Src/main/vec.o ./Core/Src/main/vec.su ./Core/Src/main/vehicle.cyclo ./Core/Src/main/vehicle.d ./Core/Src/main/vehicle.o ./Core/Src/main/vehicle.su ./Core/Src/main/vehicle2.cyclo ./Core/Src/main/vehicle2.d ./Core/Src/main/vehicle2.o ./Core/Src/main/vehicle2.su

.PHONY: clean-Core-2f-Src-2f-main

