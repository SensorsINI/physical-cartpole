################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/angle.c \
../Src/control.c \
../Src/encoder.c \
../Src/key.c \
../Src/led.c \
../Src/main.c \
../Src/motor.c \
../Src/sys.c \
../Src/timer.c \
../Src/usart.c 

OBJS += \
./Src/angle.o \
./Src/control.o \
./Src/encoder.o \
./Src/key.o \
./Src/led.o \
./Src/main.o \
./Src/motor.o \
./Src/sys.o \
./Src/timer.o \
./Src/usart.o 

C_DEPS += \
./Src/angle.d \
./Src/control.d \
./Src/encoder.d \
./Src/key.d \
./Src/led.d \
./Src/main.d \
./Src/motor.d \
./Src/sys.d \
./Src/timer.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

