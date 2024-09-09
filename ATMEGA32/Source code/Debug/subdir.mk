################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../croutine.c \
../event_groups.c \
../heap_1.c \
../list.c \
../main.c \
../port.c \
../queue.c \
../softwareDelay.c \
../stream_buffer.c \
../tasks.c \
../timers.c 

OBJS += \
./croutine.o \
./event_groups.o \
./heap_1.o \
./list.o \
./main.o \
./port.o \
./queue.o \
./softwareDelay.o \
./stream_buffer.o \
./tasks.o \
./timers.o 

C_DEPS += \
./croutine.d \
./event_groups.d \
./heap_1.d \
./list.d \
./main.d \
./port.d \
./queue.d \
./softwareDelay.d \
./stream_buffer.d \
./tasks.d \
./timers.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega32 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


