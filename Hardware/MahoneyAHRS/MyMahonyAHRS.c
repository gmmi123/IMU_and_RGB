#include "MyMahonyAHRS.h"

MahonyAHRS_Typedef MahonyAHRS;

#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

float wrap_around(float value, float min, float max) {
	float range = max - min;
	float offset_value = value - min;  // 平移至以 min 为原点
	// 处理负数并取模
	float mod = fmodf(offset_value + range, range);
	return min + mod;
}

//圆型坐标轴数值修正为直线坐标轴数值
float InfiniteVal(float Val,float* Last_Val,int* NumberFlag,float min_Endpoint,float Max_Endpoint)
{
	if (Val - *Last_Val < -fabsf(Max_Endpoint - min_Endpoint) / 2)
	{
		(*NumberFlag)++;
	}
	else if (Val - *Last_Val > fabsf(Max_Endpoint - min_Endpoint) / 2)
	{
		(*NumberFlag)--;
	}
	*Last_Val = Val;

	if (Max_Endpoint - min_Endpoint > 0)
	{
		return Val + *NumberFlag * (Max_Endpoint - min_Endpoint);
	}
	else
	{
		return Val + *NumberFlag * -(Max_Endpoint - min_Endpoint);
	}
	return -1;
}
//CorrectDate函数:
/*陀螺仪原始数据转换函数*/
/*ax,ay,az,gx,gy,gz为陀螺仪的6轴数据*/
/*ACCrange为加速度的满量程，如16g满量程，填16*/   //详见:https://blog.csdn.net/cyj972628089/article/details/113293682
/*GYROrange为角速度的满量程，如2000满量程，填2000*/
/*Date为转化后的数据数组，依次对应为ax,ay,az,gx,gy,gz*/

void MyMahony_CorrectDate(float ax,float ay,float az,
				 float gx,float gy,float gz,
			     float ACCrange,float GYROrange,float* Date)
{
	Date[0]=ax*ACCrange*9.8/32768;
	Date[1]=ay*ACCrange*9.8/32768;
	Date[2]=az*ACCrange*9.8/32768;
	Date[3]=gx*GYROrange/32768*3.1415926/180;
	Date[4]=gy*GYROrange/32768*3.1415926/180;
	Date[5]=gz*GYROrange/32768*3.1415926/180;
//	Date[0]=ax*0.0047856934f;/*已确定量程可以提前算出来，减少计算*/
//	Date[1]=ay*0.0047856934f;
//	Date[2]=az*0.0047856934f;
//	Date[3]=gx*0.0010652644f;
//	Date[4]=gy*0.0010652644f;
//	Date[5]=gz*0.0010652644f;
}


/*
	角速度零漂校准函数
	自行创建2个数组:
	用来校准陀螺仪的零漂数组:float DateCal[3];
	用来存储缓存数据数组:float temp[4];
	传入xyz的角速度
*/
int MyMahony_gyroCal(float gx,float gy,float gz,float* DateCal,float*temp,int times)
{
	//temp数组共4为，第一位用来记录标志位,其余用来暂时存储累加的角速度
	//temp[0]
	if(temp[0]<=times)//数据累加校准零漂
	{
		temp[0]++;
		temp[1]+=gx;
		temp[2]+=gy;
		temp[3]+=gz;
		if(temp[0]==times)
		{
			DateCal[0]=temp[1]/(float)times;
			DateCal[1]=temp[2]/(float)times;
			DateCal[2]=temp[3]/(float)times;
			temp[0]++;
		}
		return 0;
	}
	else 
	{
		return 1;
	}
	return 0;
}

/*
传入标准化的陀螺仪数据，调用roll_mahony,pitch_mahony,yaw_mahony使用角度或Date数组来获取角度
Date[0]pitch角
Date[1]roll角
Date[2]yaw角
*/
void MyMahony_GetAngle(MahonyAHRS_Typedef *MahonyAHRS,float ax,float ay,float az,float gx,float gy,
	float gz,float mx, float my, float mz,float* gyro_corrects,float* Date)
{
	if(gyro_corrects!=NULL)
	{
		gx-=gyro_corrects[0];//角速度数据减去零飘
		gy-=gyro_corrects[1];
		gz-=gyro_corrects[2];
	}

	/*传入标准数据，进行数据更新*/
	MyMahony_update(MahonyAHRS,gx,gy,gz,ax,ay,az,0,0,0);
	MyMahony_computeAngles(MahonyAHRS);//计算角度
	
	Date[0] = MahonyAHRS->pitch_mahony;
	Date[1] = MahonyAHRS->roll_mahony;
	Date[2] = MahonyAHRS->yaw_mahony;
}


// 初始化函数
void MyMahony_Init(MahonyAHRS_Typedef *MahonyAHRS, float sampleFrequency)
{
   MahonyAHRS->twoKi = twoKiDef;
   MahonyAHRS->q0 = 1.0f;
   MahonyAHRS->q1 = 0.0f;
   MahonyAHRS->q2 = 0.0f;
   MahonyAHRS->q3 = 0.0f;
   MahonyAHRS->integralFBx = 0.0f;
   MahonyAHRS->integralFBy = 0.0f;
   MahonyAHRS->integralFBz = 0.0f;
   MahonyAHRS->anglesComputed = 0;
   MahonyAHRS->invSampleFreq = 1.0f / sampleFrequency;
}

float MyMahony_invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

static float MyinvSqrt(float x)  // if use other platform please use float Mahony_invSqrt(float x)
{
	volatile float tmp = 1.0f;
	tmp /= __sqrtf(x);
	return tmp;
}



void MyMahonyAHRSinit(MahonyAHRS_Typedef* MahonyAHRSDate,float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float init_yaw, init_pitch, init_roll;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float magX, magY;

    recipNorm = MyinvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) 
    {
	    recipNorm = MyinvSqrt(mx * mx + my * my + mz * mz);
	    mx *= recipNorm;
	    my *= recipNorm;
	    mz *= recipNorm;
	}

    init_pitch = atan2f(-ax, az);
    init_roll = atan2f(ay, az);

    sin_roll  = sinf(init_roll);
    cos_roll  = cosf(init_roll);
    cos_pitch = cosf(init_pitch);
    sin_pitch = sinf(init_pitch);

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
    {
    	magX = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
    	magY = my * cos_roll - mz * sin_roll;
    	init_yaw  = atan2f(-magY, magX);
    }
    else
    {
    	init_yaw=0.0f;
    }

    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    MahonyAHRSDate->q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    MahonyAHRSDate->q1= sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    MahonyAHRSDate->q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    MahonyAHRSDate->q3= cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // Normalise quaternion
    recipNorm = MyMahony_invSqrt(MahonyAHRSDate->q0 * MahonyAHRSDate->q0 + MahonyAHRSDate->q1 * MahonyAHRSDate->q1 + MahonyAHRSDate->q2 * MahonyAHRSDate->q2 + MahonyAHRSDate->q3 * MahonyAHRSDate->q3);
    MahonyAHRSDate->q0 *= recipNorm;
    MahonyAHRSDate->q1 *= recipNorm;
    MahonyAHRSDate->q2 *= recipNorm;
    MahonyAHRSDate->q3 *= recipNorm;
}

void MyMahony_update(MahonyAHRS_Typedef* MahonyAHRSDate, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// Convert gyroscope degrees/sec to radians/sec
//	gx *= 0.0174533f;
//	gy *= 0.0174533f;
//	gz *= 0.0174533f;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MyMahony_updateIMU(MahonyAHRSDate,gx, gy, gz, ax, ay, az);
        return;
    }

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = MyinvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = MyinvSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = MahonyAHRSDate->q0 * MahonyAHRSDate->q0;
		q0q1 = MahonyAHRSDate->q0 * MahonyAHRSDate->q1;
		q0q2 = MahonyAHRSDate->q0 * MahonyAHRSDate->q2;
		q0q3 = MahonyAHRSDate->q0 * MahonyAHRSDate->q3;
		q1q1 = MahonyAHRSDate->q1 * MahonyAHRSDate->q1;
		q1q2 = MahonyAHRSDate->q1 * MahonyAHRSDate->q2;
		q1q3 = MahonyAHRSDate->q1 * MahonyAHRSDate->q3;
		q2q2 = MahonyAHRSDate->q2 * MahonyAHRSDate->q2;
		q2q3 = MahonyAHRSDate->q2 * MahonyAHRSDate->q3;
		q3q3 = MahonyAHRSDate->q3 * MahonyAHRSDate->q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(MahonyAHRSDate->twoKi > 0.0f) {
			// integral error scaled by Ki
			MahonyAHRSDate->integralFBx += MahonyAHRSDate->twoKi * halfex * MahonyAHRSDate->invSampleFreq;
			MahonyAHRSDate->integralFBy += MahonyAHRSDate->twoKi * halfey * MahonyAHRSDate->invSampleFreq;
			MahonyAHRSDate->integralFBz += MahonyAHRSDate->twoKi * halfez * MahonyAHRSDate->invSampleFreq;
			gx += MahonyAHRSDate->integralFBx;	// apply integral feedback
			gy += MahonyAHRSDate->integralFBy;
			gz += MahonyAHRSDate->integralFBz;
		} else {
			MahonyAHRSDate->integralFBx = 0.0f;	// prevent integral windup
			MahonyAHRSDate->integralFBy = 0.0f;
			MahonyAHRSDate->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKpDef * halfex;
		gy += twoKpDef * halfey;
		gz += twoKpDef * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * MahonyAHRSDate->invSampleFreq);		// pre-multiply common factors
	gy *= (0.5f * MahonyAHRSDate->invSampleFreq);
	gz *= (0.5f * MahonyAHRSDate->invSampleFreq);
	qa = MahonyAHRSDate->q0;
	qb = MahonyAHRSDate->q1;
	qc = MahonyAHRSDate->q2;
	MahonyAHRSDate->q0 += (-qb * gx - qc * gy - MahonyAHRSDate->q3 * gz);
	MahonyAHRSDate->q1 += (qa * gx + qc * gz - MahonyAHRSDate->q3 * gy);
	MahonyAHRSDate->q2 += (qa * gy - qb * gz +MahonyAHRSDate->q3 * gx);
	MahonyAHRSDate->q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = MyinvSqrt(MahonyAHRSDate->q0 * MahonyAHRSDate->q0 + MahonyAHRSDate->q1 * MahonyAHRSDate->q1 + MahonyAHRSDate->q2 * MahonyAHRSDate->q2 + MahonyAHRSDate->q3 * MahonyAHRSDate->q3);
	MahonyAHRSDate->q0 *= recipNorm;
	MahonyAHRSDate->q1 *= recipNorm;
	MahonyAHRSDate->q2 *= recipNorm;
	MahonyAHRSDate->q3 *= recipNorm;
	MahonyAHRSDate->anglesComputed = 0;
}


void MyMahony_updateIMU(MahonyAHRS_Typedef* MahonyAHRS,float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = MyinvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = MahonyAHRS->q1 * MahonyAHRS->q3 - MahonyAHRS->q0 * MahonyAHRS->q2;
        halfvy = MahonyAHRS->q0 * MahonyAHRS->q1 + MahonyAHRS->q2 *MahonyAHRS->q3;
        halfvz = MahonyAHRS->q0 * MahonyAHRS->q0 - 0.5f + MahonyAHRS->q3 * MahonyAHRS->q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(MahonyAHRS->twoKi > 0.0f) {
            MahonyAHRS->integralFBx += MahonyAHRS->twoKi * halfex  * MahonyAHRS->invSampleFreq;	// integral error scaled by Ki
            MahonyAHRS->integralFBy += MahonyAHRS->twoKi * halfey  * MahonyAHRS->invSampleFreq;
            MahonyAHRS->integralFBz += MahonyAHRS->twoKi * halfez  * MahonyAHRS->invSampleFreq;
            gx += MahonyAHRS->integralFBx;	// apply integral feedback
            gy += MahonyAHRS->integralFBy;
            gz += MahonyAHRS->integralFBz;
        }
        else {
            MahonyAHRS->integralFBx = 0.0f;	// prevent integral windup
            MahonyAHRS->integralFBy = 0.0f;
            MahonyAHRS->integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f *   MahonyAHRS->invSampleFreq);		// pre-multiply common factors
    gy *= (0.5f  * MahonyAHRS->invSampleFreq);
    gz *= (0.5f  * MahonyAHRS->invSampleFreq);
    qa = MahonyAHRS->q0;
    qb = MahonyAHRS->q1;
    qc = MahonyAHRS->q2;
    MahonyAHRS->q0 += (-qb * gx - qc * gy - MahonyAHRS->q3 * gz);
    MahonyAHRS->q1 += (qa * gx + qc * gz - MahonyAHRS->q3 * gy);
    MahonyAHRS->q2 += (qa * gy - qb * gz + MahonyAHRS->q3 * gx);
    MahonyAHRS->q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = MyinvSqrt(MahonyAHRS->q0 * MahonyAHRS->q0 + MahonyAHRS->q1 * MahonyAHRS->q1 + MahonyAHRS->q2 * MahonyAHRS->q2 + MahonyAHRS->q3 * MahonyAHRS->q3);
    MahonyAHRS->q0 *= recipNorm;
    MahonyAHRS->q1 *= recipNorm;
    MahonyAHRS->q2 *= recipNorm;
    MahonyAHRS->q3 *= recipNorm;
}


void MyMahony_computeAngles(MahonyAHRS_Typedef* MahonyAHRS)
{
	arm_atan2_f32(MahonyAHRS->q0*MahonyAHRS->q1 + MahonyAHRS->q2*MahonyAHRS->q3, 0.5f - MahonyAHRS->q1*MahonyAHRS->q1 - MahonyAHRS->q2*MahonyAHRS->q2,&MahonyAHRS->roll_mahony);
	MahonyAHRS->roll_mahony *= 57.29578f;  
	MahonyAHRS->pitch_mahony =57.29578f * asinf(-2.0f * (MahonyAHRS->q1*MahonyAHRS->q3 - MahonyAHRS->q0*MahonyAHRS->q2));
	arm_atan2_f32(MahonyAHRS->q1*MahonyAHRS->q2 + MahonyAHRS->q0*MahonyAHRS->q3, 0.5f - MahonyAHRS->q2*MahonyAHRS->q2 - MahonyAHRS->q3*MahonyAHRS->q3,&MahonyAHRS->yaw_mahony); 
	MahonyAHRS->yaw_mahony *=57.29578f;
	MahonyAHRS->anglesComputed = 1;
}
