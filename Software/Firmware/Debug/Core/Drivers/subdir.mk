################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Drivers/hc-05_bluetooth.c \
../Core/Drivers/imu.c \
../Core/Drivers/lidar.c \
../Core/Drivers/motor.c \
../Core/Drivers/odometry.c \
../Core/Drivers/pid.c \
../Core/Drivers/tof.c 

OBJS += \
./Core/Drivers/hc-05_bluetooth.o \
./Core/Drivers/imu.o \
./Core/Drivers/lidar.o \
./Core/Drivers/motor.o \
./Core/Drivers/odometry.o \
./Core/Drivers/pid.o \
./Core/Drivers/tof.o 

C_DEPS += \
./Core/Drivers/hc-05_bluetooth.d \
./Core/Drivers/imu.d \
./Core/Drivers/lidar.d \
./Core/Drivers/motor.d \
./Core/Drivers/odometry.d \
./Core/Drivers/pid.d \
./Core/Drivers/tof.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Drivers/%.o Core/Drivers/%.su Core/Drivers/%.cyclo: ../Core/Drivers/%.c Core/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Aymen/Firmware/Core/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Drivers

clean-Core-2f-Drivers:
	-$(RM) ./Core/Drivers/hc-05_bluetooth.cyclo ./Core/Drivers/hc-05_bluetooth.d ./Core/Drivers/hc-05_bluetooth.o ./Core/Drivers/hc-05_bluetooth.su ./Core/Drivers/imu.cyclo ./Core/Drivers/imu.d ./Core/Drivers/imu.o ./Core/Drivers/imu.su ./Core/Drivers/lidar.cyclo ./Core/Drivers/lidar.d ./Core/Drivers/lidar.o ./Core/Drivers/lidar.su ./Core/Drivers/motor.cyclo ./Core/Drivers/motor.d ./Core/Drivers/motor.o ./Core/Drivers/motor.su ./Core/Drivers/odometry.cyclo ./Core/Drivers/odometry.d ./Core/Drivers/odometry.o ./Core/Drivers/odometry.su ./Core/Drivers/pid.cyclo ./Core/Drivers/pid.d ./Core/Drivers/pid.o ./Core/Drivers/pid.su ./Core/Drivers/tof.cyclo ./Core/Drivers/tof.d ./Core/Drivers/tof.o ./Core/Drivers/tof.su

.PHONY: clean-Core-2f-Drivers

