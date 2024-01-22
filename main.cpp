#include "Framebuffer.h"
extern "C"{
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "twi.h"
#include "mpu9250.h"
#include "uart.h"
#include "millis.h"
#include <math.h>
}

#ifndef main_declarations
#define main_declarations

//#define SerialDebug

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

int gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float ax,ay,az;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

unsigned long lastUpdate = 0;	// used to calculate integration interval
unsigned long Now = 0;	// used to calculate integration interval
double pitch = 0, yaw = 0, roll = 0;
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
uint8_t err_flag = 0;
uint8_t job = 1;
uint8_t oled_flag = 0;
float cubePosX= 32;
float cubePosY= 32;
float currentThetaX = 0.0f;
float currentThetaY = 0.0f;
#endif
char extractIntegerPartAsChar(float number, int digit);


int main(void) {
	
	// TIMSK2 &= ~(1 << OCIE2A);
    // sei();

	// millis_init();
	UART_Init(115200);
    // twi_init();                                // init TWI interface
  	
	// mpu9250_setup();
	// UART_Printf("Setup done! Purchasing gyroscopical data...\n\r");
	Framebuffer fb;
	fb.setCubePosition(cubePosX, cubePosY, 0);


	while(1){
		if(currentThetaX < 2*PI) currentThetaX += 0.01f * PI;
		else currentThetaX = 0.0f;
		if(currentThetaY < 2*PI) currentThetaY += 0.03f * (PI);
		else currentThetaY = 0.0f;
		fb.clear();
        fb.setCubePosition(-cubePosX, -cubePosY, 0);
        fb.rotateY3D(0.01f);
        fb.rotateX3D(0.03f);
        fb.setCubePosition(cubePosX, cubePosY, 0);
        fb.drawCube();
		//fb.printStringOnOLED(100,50,"TEST");
		fb.displayChar('X',64,6);
		fb.displayChar(':',70,6);
		fb.displayChar(extractIntegerPartAsChar(currentThetaX,0),80,6);
		fb.displayChar('.',88,6);
		fb.displayChar(extractIntegerPartAsChar(currentThetaX,1),96,6);
		fb.displayChar(extractIntegerPartAsChar(currentThetaX,2),104,6);	

		fb.displayChar('Y',64,22);
		fb.displayChar(':',70,22);
		fb.displayChar(extractIntegerPartAsChar(currentThetaY,0),80,22);
		fb.displayChar('.',88,22);
		fb.displayChar(extractIntegerPartAsChar(currentThetaY,1),96,22);
		fb.displayChar(extractIntegerPartAsChar(currentThetaY,2),104,22);				
        fb.show();
		
		
        _delay_us(10);


	}
    return 0;
}

char extractIntegerPartAsChar(float number, int digit) {
    int integerPart;
    char charPart;

    switch (digit) {
        case 0:
            integerPart = (int)floor(number);
            break;
        case 1:
            number *= 10;
            integerPart = (int)floor(number) % 10; // 소수점 첫째 자리
            break;
        case 2:
            number *= 100;
            integerPart = (int)floor(number) % 100; // 소수점 둘째 자리
            integerPart %= 10; // 십의 자리
            break;
        // Add more cases for additional digits if needed
        default:
            break;
    }

    // 정수 부분을 문자로 변환
    if (integerPart >= 0 && integerPart <= 9) {
        // 정수가 0부터 9 사이인 경우
        charPart = '0' + integerPart;
    } else {
        // 정수가 범위를 벗어난 경우, 여기에 필요한 처리 추가
        charPart = ' '; // 빈 문자 반환
    }

    return charPart;
}

