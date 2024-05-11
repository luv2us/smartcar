################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/ENA.c \
../code/motor_servo_contrl.c \
../code/pid.c \
../code/servo_control.c \
../code/servo_pid.c 

OBJS += \
./code/ENA.o \
./code/motor_servo_contrl.o \
./code/pid.o \
./code/servo_control.o \
./code/servo_pid.o 

COMPILED_SRCS += \
./code/ENA.src \
./code/motor_servo_contrl.src \
./code/pid.src \
./code/servo_control.src \
./code/servo_pid.src 

C_DEPS += \
./code/ENA.d \
./code/motor_servo_contrl.d \
./code/pid.d \
./code/servo_control.d \
./code/servo_pid.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fC:/Users/ASUS/Desktop/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


