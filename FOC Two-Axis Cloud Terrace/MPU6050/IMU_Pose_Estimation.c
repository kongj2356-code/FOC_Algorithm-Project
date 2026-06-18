#include "IMU_Pose_Estimation.h"


void Gyro_InitBias(MPU6050_TypeDef *xzy)
{
	if(bias_done) return;
	gyro_bias_x_init+=xzy->Gyrof[x];
	gyro_bias_y_init+=xzy->Gyrof[y];
	gyro_bias_z_init+=xzy->Gyrof[z];
	bias_count++;
	if(bias_count>=GYRO_INIT_COUNT)
	{
		gyro_bias_x_init/=GYRO_INIT_COUNT;
		gyro_bias_y_init/=GYRO_INIT_COUNT;
		gyro_bias_z_init/=GYRO_INIT_COUNT;
		bias_done=1;
	}
}


void Gyro_Deadzone_Filter(MPU6050_TypeDef *xzy)
{
	if(fabsf(xzy->Gyrof[x])<GYRO_DEADZONE)xzy->Gyrof[x]=0;
	if(fabsf(xzy->Gyrof[y])<GYRO_DEADZONE)xzy->Gyrof[y]=0;
	if(fabsf(xzy->Gyrof[z])<GYRO_DEADZONE)xzy->Gyrof[z]=0;                  
}


static float fast_sqrt(float xx)
{
	float halfx=0.5f*xx;
	float yy=xx;
	long i=*(long*)&yy;
	i=0x5f3759df-(i >> 1);
	yy=*(float*)&i;
	yy=yy*(1.5f-(halfx*yy*yy));
	return yy;
}


void Mahony_Init(Mahony_TypeDef *mah) 
{
	mah->q0 =1; mah->q1=0; mah->q2=0; mah->q3=0;
	mah->exInt=mah->eyInt=mah->ezInt=0;
	mah->Kp=8.05f;
	mah->Ki=2.002f;
}


static inline int IMU_IsStable(float gx,float gy,float gz,float ax,float ay,float az)
{
	if (fabsf(gx)<GYRO_STABLE_TH&&fabsf(gy)<GYRO_STABLE_TH&&fabsf(gz)<GYRO_STABLE_TH)
	{
		float an=sqrtf(ax*ax+ay*ay+az*az);
		if(fabsf(an-1.0f)<ACC_NORM_TH)
		return 1;
	}
	return 0;
}



void GyroBias_Update(MPU6050_TypeDef *xzy,float dt)
{
	calibration_time+=dt;
	if (calibration_time<INIT_CALIBRATION_TIME)
	{
		gyro_bias_x=gyro_bias_x*0.9f+xzy->Gyrof[x]*0.1f;
		gyro_bias_y=gyro_bias_y*0.9f+xzy->Gyrof[y]*0.1f;
		gyro_bias_z=gyro_bias_z*0.9f+xzy->Gyrof[z]*0.1f;
		return;
	}
	if (IMU_IsStable(xzy->Gyrof[x],xzy->Gyrof[y],xzy->Gyrof[z],xzy->Accf[x],xzy->Accf[y],xzy->Accf[z]))
	{
		gyro_bias_x+=(0.0f-xzy->Gyrof[x])*BIAS_ALPHA;
		gyro_bias_y+=(0.0f-xzy->Gyrof[y])*BIAS_ALPHA;
		gyro_bias_z+=(0.0f-xzy->Gyrof[z])*BIAS_ALPHA;
	}
}     





void Mahony_Update(Mahony_TypeDef *mahony,MPU6050_TypeDef *xzy,QMC5883P_Calibration_t *qmc,float dt)
{
	float q0=mahony->q0,q1=mahony->q1,q2=mahony->q2,q3=mahony->q3;
	float *accf=xzy->Accf, *gyrof=xzy->Gyrof, norm;
	//
	float ax=accf[x],ay=accf[y],az=accf[z];
	norm=fast_sqrt(ax*ax+ay*ay+az*az);
	if(norm==0) return;
	ax*=norm; ay*=norm; az*=norm;    // 归一化加速度计
	//
	float mx= -qmc->m[0];   //  QMC5883P安装位置与MPU6050的X,Y,Z对齐
	float my= -qmc->m[1];   //
	float mz=  qmc->m[2];
	norm=fast_sqrt(mx*mx+my*my+mz*mz);
	if(norm==0) return;
	mx*=norm;my*=norm;mz*=norm;       //归一化磁力计

  /*四元数预运算处理*/
  float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;

  /*将载体坐标系下的磁力计数据转换至大地坐标系下*/
  float hx = (q0q0 + q1q1 - q2q2 - q3q3) * mx + 2 * (q1q2 - q0q3) * my + 2 * (q1q3 + q0q2) * mz;
  float hy = 2 * (q1q2 + q0q3) * mx + (q0q0 - q1q1 + q2q2 - q3q3) * my + 2 * (q2q3 - q0q1) * mz;
  float hz = 2 * (q1q3 - q0q2) * mx + 2 * (q2q3 + q0q1) * my + (q0q0 - q1q1 - q2q2 + q3q3) * mz;       

  /*获取大地坐标系下的理论磁场强度*/
  float bx = sqrt((hx*hx) + (hy*hy)); 
  float by = 0.0f;
  float bz = hz;

 
  /*将大地坐标系下的理论磁场强度转换至机体坐标系下*/
  float wx = (q0q0 + q1q1 - q2q2 - q3q3) * bx + 2*(q1q2 + q0q3)*by + 2*(q1q3 - q0q2)*bz;
  float wy = 2*(q1q2 - q0q3)*bx + (q0q0 - q1q1 + q2q2 - q3q3)*by + 2*(q2q3 + q0q1)*bz;
  float wz = 2*(q1q3 + q0q2)*bx + 2*(q2q3 - q0q1)*by + (q0q0 - q1q1 - q2q2 + q3q3)*bz;

  /*提取大地坐标系下的重力分量*/
  float vx = 2*(q1q3 - q0q2);                                     
  float vy = 2*(q0q1 + q2q3);
  float vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  /*误差累积*/
  float ex = (ay*vz - az*vy) + (my*wz - mz*wy); 
  float ey = (az*vx - ax*vz) + (mz*wx - mx*wz); 
  float ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	mahony->exInt+=mahony->Ki*ex*dt;
	mahony->eyInt+=mahony->Ki*ey*dt;
	mahony->ezInt+=mahony->Ki*ez*dt;
	//
	gyrof[x]+=mahony->Kp*ex+mahony->exInt;
	gyrof[y]+=mahony->Kp*ey+mahony->eyInt; 
	gyrof[z]+=mahony->Kp*ez+mahony->ezInt;
	//
	float halfT=0.5f*dt;
	float q0_dot=(-q1*gyrof[x]-q2*gyrof[y]-q3*gyrof[z])*halfT;
	float q1_dot=(q0*gyrof[x]+q2*gyrof[z]-q3*gyrof[y])*halfT;
	float q2_dot=(q0*gyrof[y]-q1*gyrof[z]+q3*gyrof[x])*halfT;
	float q3_dot=(q0*gyrof[z]+q1*gyrof[y]-q2*gyrof[x])*halfT;
        //
	q0+=q0_dot;q1+=q1_dot;
	q2+=q2_dot;q3+=q3_dot;
	//
	norm=fast_sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0*=norm;q1*=norm; 
	q2*=norm;q3*=norm;
	//
	mahony->q0=q0;mahony->q1=q1;
	mahony->q2=q2;mahony->q3=q3;
}




void Mahony_GetEuler(Mahony_TypeDef *mahony,float *roll,float *pitch,float *yaw,float gz_rad,float dt)
{
	float q0=mahony->q0,q1=mahony->q1,q2=mahony->q2,q3=mahony->q3;
	*roll=atan2f(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*57.295779513f;
	*pitch=asinf(-2*(q1*q3-q0*q2))*57.295779513f;
	*yaw=atan2f(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*57.295779513f;
}



//void Compass_GetYaw(MPU6050_TypeDef *mpu, QMC5883P_Calibration_t *qmc, float *yaw_deg)
//{
//	float mx=-qmc->m[1];
//	float my= qmc->m[0];
//	float mz= qmc->m[2];
//	// 
//	float ax=mpu->Accf[x];
//	float ay=mpu->Accf[y];
//	float az=mpu->Accf[z];
//	//
//	float roll=atan2f(ay,az);
//	float pitch=atan2f(-ax,sqrtf(ay*ay+az*az));
//	//
//	float cos_roll=cosf(roll);
//	float sin_roll=sinf(roll);
//	float cos_pitch=cosf(pitch);
//	float sin_pitch=sinf(pitch);
//	// 
//	float mx2=mx*cos_pitch+mz*sin_pitch;
//	float my2=mx*sin_roll*sin_pitch+my*cos_roll-mz*sin_roll*cos_pitch;
//	// 
//	*yaw_deg=atan2f(-my2,mx2)*57.295779513f;

//}

void Mahony_GetData(Mahony_TypeDef *mahony,MPU6050_TypeDef *mpu, QMC5883P_Calibration_t *qmc,float *roll,float *pitch,float *yaw,float *yaw1,float dt)
{

  MPU6050_GetData(mpu);
	QMC5883P_GetCalibData_uT(qmc);
	Gyro_InitBias(mpu);
	Gyro_Deadzone_Filter(mpu);
//	Compass_GetYaw(mpu,qmc,yaw1);
	Mahony_Update(mahony,mpu,qmc,dt);
	Mahony_GetEuler(mahony,roll,pitch,yaw,mpu->Gyrof[z],0.002f);
}


