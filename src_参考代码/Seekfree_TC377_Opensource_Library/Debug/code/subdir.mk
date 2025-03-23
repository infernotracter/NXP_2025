################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/encoder.c \
../code/flash.c \
../code/gyro.c \
../code/image.c \
../code/key.c \
../code/menu.c \
../code/motor.c \
../code/pid.c \
../code/quater.c \
../code/tool.c \
../code/uart.c 

COMPILED_SRCS += \
./code/encoder.src \
./code/flash.src \
./code/gyro.src \
./code/image.src \
./code/key.src \
./code/menu.src \
./code/motor.src \
./code/pid.src \
./code/quater.src \
./code/tool.src \
./code/uart.src 

C_DEPS += \
./code/encoder.d \
./code/flash.d \
./code/gyro.d \
./code/image.d \
./code/key.d \
./code/menu.d \
./code/motor.d \
./code/pid.d \
./code/quater.d \
./code/tool.d \
./code/uart.d 

OBJS += \
./code/encoder.o \
./code/flash.o \
./code/gyro.o \
./code/image.o \
./code/key.o \
./code/menu.o \
./code/motor.o \
./code/pid.o \
./code/quater.o \
./code/tool.o \
./code/uart.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/workspace/up_camera/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/encoder.d ./code/encoder.o ./code/encoder.src ./code/flash.d ./code/flash.o ./code/flash.src ./code/gyro.d ./code/gyro.o ./code/gyro.src ./code/image.d ./code/image.o ./code/image.src ./code/key.d ./code/key.o ./code/key.src ./code/menu.d ./code/menu.o ./code/menu.src ./code/motor.d ./code/motor.o ./code/motor.src ./code/pid.d ./code/pid.o ./code/pid.src ./code/quater.d ./code/quater.o ./code/quater.src ./code/tool.d ./code/tool.o ./code/tool.src ./code/uart.d ./code/uart.o ./code/uart.src

.PHONY: clean-code

