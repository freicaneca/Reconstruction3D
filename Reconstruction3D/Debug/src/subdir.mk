################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/EpipolarGeometry.cpp \
../src/featureMatching.cpp \
../src/main.cpp 

OBJS += \
./src/EpipolarGeometry.o \
./src/featureMatching.o \
./src/main.o 

CPP_DEPS += \
./src/EpipolarGeometry.d \
./src/featureMatching.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/CIN/fdbo/local/include -I/home/felipe/opencv-3.4.0/include -I"/media/felipe/seagate_expansion/doutorado/faculdade/topicos_midia_interacao_1/projeto/eclipse/Reconstruction3D/include" -O0 -g3 -Wall -c -fmessage-length=0 `pkg-config --cflags --libs opencv` -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


