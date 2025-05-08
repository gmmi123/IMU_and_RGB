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

//���������ݱ�׼��
void MyMahony_CorrectDate(float ax,float ay,float az,
				 float gx,float gy,float gz,
			     float ACCrange,float GYROrange,float* Date);
//��ƮУ׼
int MyMahony_gyroCal(float gx,float gy,float gz,float* DateCal,float*temp,int times);

//����ƮУ׼����Ƕ�
void MyMahony_GetAngle(MahonyAHRS_Typedef *MahonyAHRS,float ax,float ay,float az,float gx,float gy,
	float gz,float mx, float my, float mz,float* gyro_corrects,float* Date);

//��ʼ������
void MyMahony_Init(MahonyAHRS_Typedef*MahonyAHRS, float sampleFrequency);//Ƶ�ʲ���Ҫ�ǳ�׼ȷ
//������������ֵ,��ʼ��������
void MyMahonyAHRSinit(MahonyAHRS_Typedef* MahonyAHRSDate,float ax, float ay, float az, float mx, float my, float mz);
//������Ԫ��(��������)
void MyMahony_update(MahonyAHRS_Typedef* MahonyAHRS, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
//������Ԫ��(�޴�����)
void MyMahony_updateIMU(MahonyAHRS_Typedef* MahonyAHRS, float gx, float gy, float gz, float ax, float ay, float az);
//����Ƕ�
void MyMahony_computeAngles(MahonyAHRS_Typedef* MahonyAHRS);
#endif