################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/display_7seg.c \
../Core/Src/fsm_traffic3_4.c \
../Core/Src/led_control.c 

OBJS += \
./Core/Src/display_7seg.o \
./Core/Src/fsm_traffic3_4.o \
./Core/Src/led_control.o 

C_DEPS += \
./Core/Src/display_7seg.d \
./Core/Src/fsm_traffic3_4.d \
./Core/Src/led_control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103x6 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/display_7seg.cyclo ./Core/Src/display_7seg.d ./Core/Src/display_7seg.o ./Core/Src/display_7seg.su ./Core/Src/fsm_traffic3_4.cyclo ./Core/Src/fsm_traffic3_4.d ./Core/Src/fsm_traffic3_4.o ./Core/Src/fsm_traffic3_4.su ./Core/Src/led_control.cyclo ./Core/Src/led_control.d ./Core/Src/led_control.o ./Core/Src/led_control.su

.PHONY: clean-Core-2f-Src

