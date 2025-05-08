#include "RGB.h"
#include "WS2812.h"
#include "math.h"
#include "GPIO_PWM.h"
#include "DMA_PWM.h"
#include "WS2812.h"
#include "RGB.h"
#include "GPIO_PWM.h"

#include "BMI088driver.h"
#include "MahonyAHRS.h"
extern float Angle[3];
float AccelAndGyro[2];
float AccelXYZ[3];
extern fp32 gyro[3], accel[3], temp;

float Cal[3]={-0.088399902,-0.00070390763,-0.0429568365};

float gx;
float gy;
float gz;

// 去除重力后的
float ax_world;
float ay_world;
float az_world;
uint8_t RGB_buffer[NUM_LEDS][3]={
{0,222,0}
};
//设置单个颜色（底层）
void setLedColor(uint8_t(* RGB_buffer)[3], uint32_t led_index, uint8_t r, uint8_t g, uint8_t b) 
{
	RGB_buffer[led_index][0]= r;
	RGB_buffer[led_index][1]= g;
	RGB_buffer[led_index][2]= b;
}

//设置单个颜色,RGB
void setColor(uint32_t led_index, uint8_t r, uint8_t g, uint8_t b)
{
	setLedColor(RGB_buffer,led_index,r,g,b);
}
void clearColor(uint32_t start_index, uint32_t End_index) 
{
    for(uint32_t led = start_index; led <= End_index; led++) { 
            // 将RGB全部设为0（关闭）
            RGB_buffer[led][0] = 0;  // R
            RGB_buffer[led][1] = 0;  // G
            RGB_buffer[led][2] = 0;  // B   
    }
}

//用HSL数组填写RGB
void hsl_to_rgb(float h,float s,float l, uint8_t* RGB) //彩虹灯
{

//	float h = (int)fmod(i, 360.0);
//	float s=1.0f;//饱和度
//	float l=RGB_Light;//亮度
    float c = (1 - fabs(2 * l - 1)) * s;
    float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
    float m = l - c / 2;

    float r_prime, g_prime, b_prime;

    if (h >= 0 && h < 60) {
        r_prime = c;
        g_prime = x;
        b_prime = 0;
    } else if (h >= 60 && h < 120) {
        r_prime = x;
        g_prime = c;
        b_prime = 0;
    } else if (h >= 120 && h < 180) {
        r_prime = 0;
        g_prime = c;
        b_prime = x;
    } else if (h >= 180 && h < 240) {
        r_prime = 0;
        g_prime = x;
        b_prime = c;
    } else if (h >= 240 && h < 300) {
        r_prime = x;
        g_prime = 0;
        b_prime = c;
    } else { // h >= 300 && h < 360
        r_prime = c;
        g_prime = 0;
        b_prime = x;
    }

    RGB[0] = (int)((r_prime + m) * 255);
    RGB[1] = (int)((g_prime + m) * 255);
    RGB[2] = (int)((b_prime + m) * 255);
}



void RGB_Control()
{
	
}

//线性映射
float linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float DateCal[3];
float DateTemp[4];
void RGB_Task()
{
	static float i;
	static float j;
	static float k;
	static float l;
	static int temp;
	//计算陀螺仪的加速度和速度
	//加速度
////	AccelAndGyro[0] = sqrtf(accel[0]* accel[0]+  accel[1]* accel[1]+ accel[2]* accel[2]);
	//角速度
	AccelAndGyro[1] = sqrtf(gyro[0]* gyro[0]+  gyro[1]* gyro[1]+ gyro[2]* gyro[2]);
	
	
	 gx = sinf((Angle[1]/180.0f)*3.140f) * 9.8;
	 gy = -sinf((Angle[0]/180.0f)*3.140f) * cosf((Angle[1]/180.0f)*3.140f) * 9.8;
	 gz = -cosf((Angle[0]/180.0f)*3.140f) * cosf((Angle[1]/180.0f)*3.140f) * 9.8;

//	// 去除重力后的运动加速度
	 ax_world = (accel[0] + gx)*cosf((Angle[2]/180.0f)*3.140f);
	 ay_world = (accel[1] + gy)*sinf((Angle[2]/180.0f)*3.140f);
	 az_world = accel[2] + gz;


	AccelAndGyro[0] = sqrtf(ax_world* ax_world+ ay_world* ay_world+ az_world* az_world);
	
	
	clearColor(0,100-1);
	j=linear_map(Angle[0],-90,90,0,100-1);
	k=linear_map(fabsf(AccelAndGyro[0]),0,50,10,25-1);
	if(fabsf(AccelAndGyro[0])>15)
	{
		l=k;
	}
	else
	{
		l=0.01*k+0.99*l;
	}
	
	i+=0.5;
	if(i>360)
	{
		i=1;
	}
	for(int L=0;L<(int)l;L++)
	{
		hsl_to_rgb(180,1,0.1+0.5*fabsf((float)(((int)(j)-(int)(l/2)+L)-((int)(j))))/10,&RGB_buffer[(int)(j)-(int)(l/2)+L][0]);
	}

	
//	gyroCal(gyro[0],gyro[1],gyro[2],DateCal,DateTemp,1000);
	

}
