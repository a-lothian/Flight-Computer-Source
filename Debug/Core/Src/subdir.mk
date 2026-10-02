################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/AirbrakeController.cpp \
../Core/Src/ApogeePredictor.cpp \
../Core/Src/Atmosphere.cpp \
../Core/Src/AtmosphereData.cpp \
../Core/Src/CdTable.cpp \
../Core/Src/CdTableData.cpp \
../Core/Src/DeploymentCorrector.cpp \
../Core/Src/FaultMonitor.cpp \
../Core/Src/OvershootBias.cpp \
../Core/Src/RateLimiter.cpp \
../Core/Src/Validate.cpp \
../Core/Src/airbrake_api.cpp 

C_SRCS += \
../Core/Src/IMU.c \
../Core/Src/accel_select.c \
../Core/Src/airbrake_io.c \
../Core/Src/altitude_kf.c \
../Core/Src/attitude.c \
../Core/Src/bench.c \
../Core/Src/dps368.c \
../Core/Src/fsm.c \
../Core/Src/h3lis.c \
../Core/Src/main.c \
../Core/Src/mtr_control.c \
../Core/Src/sensor_fusion.c \
../Core/Src/sensor_mount.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/telemetry.c \
../Core/Src/telemetry_can.c \
../Core/Src/vertical_accel.c \
../Core/Src/w25n.c 

C_DEPS += \
./Core/Src/IMU.d \
./Core/Src/accel_select.d \
./Core/Src/airbrake_io.d \
./Core/Src/altitude_kf.d \
./Core/Src/attitude.d \
./Core/Src/bench.d \
./Core/Src/dps368.d \
./Core/Src/fsm.d \
./Core/Src/h3lis.d \
./Core/Src/main.d \
./Core/Src/mtr_control.d \
./Core/Src/sensor_fusion.d \
./Core/Src/sensor_mount.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/telemetry.d \
./Core/Src/telemetry_can.d \
./Core/Src/vertical_accel.d \
./Core/Src/w25n.d 

OBJS += \
./Core/Src/AirbrakeController.o \
./Core/Src/ApogeePredictor.o \
./Core/Src/Atmosphere.o \
./Core/Src/AtmosphereData.o \
./Core/Src/CdTable.o \
./Core/Src/CdTableData.o \
./Core/Src/DeploymentCorrector.o \
./Core/Src/FaultMonitor.o \
./Core/Src/IMU.o \
./Core/Src/OvershootBias.o \
./Core/Src/RateLimiter.o \
./Core/Src/Validate.o \
./Core/Src/accel_select.o \
./Core/Src/airbrake_api.o \
./Core/Src/airbrake_io.o \
./Core/Src/altitude_kf.o \
./Core/Src/attitude.o \
./Core/Src/bench.o \
./Core/Src/dps368.o \
./Core/Src/fsm.o \
./Core/Src/h3lis.o \
./Core/Src/main.o \
./Core/Src/mtr_control.o \
./Core/Src/sensor_fusion.o \
./Core/Src/sensor_mount.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/telemetry.o \
./Core/Src/telemetry_can.o \
./Core/Src/vertical_accel.o \
./Core/Src/w25n.o 

CPP_DEPS += \
./Core/Src/AirbrakeController.d \
./Core/Src/ApogeePredictor.d \
./Core/Src/Atmosphere.d \
./Core/Src/AtmosphereData.d \
./Core/Src/CdTable.d \
./Core/Src/CdTableData.d \
./Core/Src/DeploymentCorrector.d \
./Core/Src/FaultMonitor.d \
./Core/Src/OvershootBias.d \
./Core/Src/RateLimiter.d \
./Core/Src/Validate.d \
./Core/Src/airbrake_api.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++17 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -ffp-contract=off -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AirbrakeController.cyclo ./Core/Src/AirbrakeController.d ./Core/Src/AirbrakeController.o ./Core/Src/AirbrakeController.su ./Core/Src/ApogeePredictor.cyclo ./Core/Src/ApogeePredictor.d ./Core/Src/ApogeePredictor.o ./Core/Src/ApogeePredictor.su ./Core/Src/Atmosphere.cyclo ./Core/Src/Atmosphere.d ./Core/Src/Atmosphere.o ./Core/Src/Atmosphere.su ./Core/Src/AtmosphereData.cyclo ./Core/Src/AtmosphereData.d ./Core/Src/AtmosphereData.o ./Core/Src/AtmosphereData.su ./Core/Src/CdTable.cyclo ./Core/Src/CdTable.d ./Core/Src/CdTable.o ./Core/Src/CdTable.su ./Core/Src/CdTableData.cyclo ./Core/Src/CdTableData.d ./Core/Src/CdTableData.o ./Core/Src/CdTableData.su ./Core/Src/DeploymentCorrector.cyclo ./Core/Src/DeploymentCorrector.d ./Core/Src/DeploymentCorrector.o ./Core/Src/DeploymentCorrector.su ./Core/Src/FaultMonitor.cyclo ./Core/Src/FaultMonitor.d ./Core/Src/FaultMonitor.o ./Core/Src/FaultMonitor.su ./Core/Src/IMU.cyclo ./Core/Src/IMU.d ./Core/Src/IMU.o ./Core/Src/IMU.su ./Core/Src/OvershootBias.cyclo ./Core/Src/OvershootBias.d ./Core/Src/OvershootBias.o ./Core/Src/OvershootBias.su ./Core/Src/RateLimiter.cyclo ./Core/Src/RateLimiter.d ./Core/Src/RateLimiter.o ./Core/Src/RateLimiter.su ./Core/Src/Validate.cyclo ./Core/Src/Validate.d ./Core/Src/Validate.o ./Core/Src/Validate.su ./Core/Src/accel_select.cyclo ./Core/Src/accel_select.d ./Core/Src/accel_select.o ./Core/Src/accel_select.su ./Core/Src/airbrake_api.cyclo ./Core/Src/airbrake_api.d ./Core/Src/airbrake_api.o ./Core/Src/airbrake_api.su ./Core/Src/airbrake_io.cyclo ./Core/Src/airbrake_io.d ./Core/Src/airbrake_io.o ./Core/Src/airbrake_io.su ./Core/Src/altitude_kf.cyclo ./Core/Src/altitude_kf.d ./Core/Src/altitude_kf.o ./Core/Src/altitude_kf.su ./Core/Src/attitude.cyclo ./Core/Src/attitude.d ./Core/Src/attitude.o ./Core/Src/attitude.su ./Core/Src/bench.cyclo ./Core/Src/bench.d ./Core/Src/bench.o ./Core/Src/bench.su ./Core/Src/dps368.cyclo ./Core/Src/dps368.d ./Core/Src/dps368.o ./Core/Src/dps368.su ./Core/Src/fsm.cyclo ./Core/Src/fsm.d ./Core/Src/fsm.o ./Core/Src/fsm.su ./Core/Src/h3lis.cyclo ./Core/Src/h3lis.d ./Core/Src/h3lis.o ./Core/Src/h3lis.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mtr_control.cyclo ./Core/Src/mtr_control.d ./Core/Src/mtr_control.o ./Core/Src/mtr_control.su ./Core/Src/sensor_fusion.cyclo ./Core/Src/sensor_fusion.d ./Core/Src/sensor_fusion.o ./Core/Src/sensor_fusion.su ./Core/Src/sensor_mount.cyclo ./Core/Src/sensor_mount.d ./Core/Src/sensor_mount.o ./Core/Src/sensor_mount.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/telemetry.cyclo ./Core/Src/telemetry.d ./Core/Src/telemetry.o ./Core/Src/telemetry.su ./Core/Src/telemetry_can.cyclo ./Core/Src/telemetry_can.d ./Core/Src/telemetry_can.o ./Core/Src/telemetry_can.su ./Core/Src/vertical_accel.cyclo ./Core/Src/vertical_accel.d ./Core/Src/vertical_accel.o ./Core/Src/vertical_accel.su ./Core/Src/w25n.cyclo ./Core/Src/w25n.d ./Core/Src/w25n.o ./Core/Src/w25n.su

.PHONY: clean-Core-2f-Src

