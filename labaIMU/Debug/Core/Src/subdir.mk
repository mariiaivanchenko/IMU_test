################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/i3g4250d.c \
../Core/Src/l3gd20.c \
../Core/Src/lsm303agr.c \
../Core/Src/lsm303dlhc.c \
../Core/Src/main.c \
../Core/Src/stm32f411e_discovery.c \
../Core/Src/stm32f411e_discovery_accelerometer.c \
../Core/Src/stm32f411e_discovery_gyroscope.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/i3g4250d.o \
./Core/Src/l3gd20.o \
./Core/Src/lsm303agr.o \
./Core/Src/lsm303dlhc.o \
./Core/Src/main.o \
./Core/Src/stm32f411e_discovery.o \
./Core/Src/stm32f411e_discovery_accelerometer.o \
./Core/Src/stm32f411e_discovery_gyroscope.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/i3g4250d.d \
./Core/Src/l3gd20.d \
./Core/Src/lsm303agr.d \
./Core/Src/lsm303dlhc.d \
./Core/Src/main.d \
./Core/Src/stm32f411e_discovery.d \
./Core/Src/stm32f411e_discovery_accelerometer.d \
./Core/Src/stm32f411e_discovery_gyroscope.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/i3g4250d.cyclo ./Core/Src/i3g4250d.d ./Core/Src/i3g4250d.o ./Core/Src/i3g4250d.su ./Core/Src/l3gd20.cyclo ./Core/Src/l3gd20.d ./Core/Src/l3gd20.o ./Core/Src/l3gd20.su ./Core/Src/lsm303agr.cyclo ./Core/Src/lsm303agr.d ./Core/Src/lsm303agr.o ./Core/Src/lsm303agr.su ./Core/Src/lsm303dlhc.cyclo ./Core/Src/lsm303dlhc.d ./Core/Src/lsm303dlhc.o ./Core/Src/lsm303dlhc.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f411e_discovery.cyclo ./Core/Src/stm32f411e_discovery.d ./Core/Src/stm32f411e_discovery.o ./Core/Src/stm32f411e_discovery.su ./Core/Src/stm32f411e_discovery_accelerometer.cyclo ./Core/Src/stm32f411e_discovery_accelerometer.d ./Core/Src/stm32f411e_discovery_accelerometer.o ./Core/Src/stm32f411e_discovery_accelerometer.su ./Core/Src/stm32f411e_discovery_gyroscope.cyclo ./Core/Src/stm32f411e_discovery_gyroscope.d ./Core/Src/stm32f411e_discovery_gyroscope.o ./Core/Src/stm32f411e_discovery_gyroscope.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

