################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Functions/extra_motor_function.c \
../Src/Functions/mit_receiving.c \
../Src/Functions/mit_sending.c \
../Src/Functions/motor_control_demos.c \
../Src/Functions/servo_receiving.c \
../Src/Functions/servo_sending.c 

OBJS += \
./Src/Functions/extra_motor_function.o \
./Src/Functions/mit_receiving.o \
./Src/Functions/mit_sending.o \
./Src/Functions/motor_control_demos.o \
./Src/Functions/servo_receiving.o \
./Src/Functions/servo_sending.o 

C_DEPS += \
./Src/Functions/extra_motor_function.d \
./Src/Functions/mit_receiving.d \
./Src/Functions/mit_sending.d \
./Src/Functions/motor_control_demos.d \
./Src/Functions/servo_receiving.d \
./Src/Functions/servo_sending.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Functions/%.o Src/Functions/%.su: ../Src/Functions/%.c Src/Functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-Functions

clean-Src-2f-Functions:
	-$(RM) ./Src/Functions/extra_motor_function.d ./Src/Functions/extra_motor_function.o ./Src/Functions/extra_motor_function.su ./Src/Functions/mit_receiving.d ./Src/Functions/mit_receiving.o ./Src/Functions/mit_receiving.su ./Src/Functions/mit_sending.d ./Src/Functions/mit_sending.o ./Src/Functions/mit_sending.su ./Src/Functions/motor_control_demos.d ./Src/Functions/motor_control_demos.o ./Src/Functions/motor_control_demos.su ./Src/Functions/servo_receiving.d ./Src/Functions/servo_receiving.o ./Src/Functions/servo_receiving.su ./Src/Functions/servo_sending.d ./Src/Functions/servo_sending.o ./Src/Functions/servo_sending.su

.PHONY: clean-Src-2f-Functions

