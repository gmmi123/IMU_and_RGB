#ifndef MyMahonyAHRS_h
#define MyMahonyAHRS_h

#include "dsp/fast_math_functions.h"    
#include "arm_math.h"

typedef struct {
    float twoKi;
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
    float invSampleFreq;
    float roll_mahony, pitch_mahony, yaw_mahony;
    char anglesComputed;
}MahonyAHRS_Typedef;

extern MahonyAHRS_Typedef MahonyAHRS;

//陀螺仪数据标准化
void MyMahony_CorrectDate(float ax,float ay,float az,
				 float gx,float gy,float gz,
			     float ACCrange,float GYROrange,float* Date);
//零飘校准
int MyMahony_gyroCal(float gx,float gy,float gz,float* DateCal,float*temp,int times);

//带零飘校准计算角度
void MyMahony_GetAngle(MahonyAHRS_Typedef *MahonyAHRS,float ax,float ay,float az,float gx,float gy,
	float gz,float mx, float my, float mz,float* gyro_corrects,float* Date);

//初始化对象
void MyMahony_Init(MahonyAHRS_Typedef*MahonyAHRS, float sampleFrequency);//频率参数要非常准确
//传入陀螺仪数值,初始更新数据
void MyMahonyAHRSinit(MahonyAHRS_Typedef* MahonyAHRSDate,float ax, float ay, float az, float mx, float my, float mz);
//更新四元数(带磁力计)
void MyMahony_update(MahonyAHRS_Typedef* MahonyAHRS, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
//更新四元数(无磁力计)
void MyMahony_updateIMU(MahonyAHRS_Typedef* MahonyAHRS, float gx, float gy, float gz, float ax, float ay, float az);
//计算角度
void MyMahony_computeAngles(MahonyAHRS_Typedef* MahonyAHRS);
#endif