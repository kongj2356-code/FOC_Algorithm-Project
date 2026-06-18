#include "stm32f10x.h"
#include "string.h"
#include "math.h" 
#include "IMU_I2C.h"
#include "QMC5883P.h"
#include "QMC5883P_Reg.h"
/*WRITE***********************************************************************/
/**
  * 函    数：QMC5883P写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考QMC5883P手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0x0B
  * 返 回 值：无
  */
void QMC5883P_WriteReg(uint8_t RegAddress, uint8_t Data)
{
   MyI2C_Start();					                	//MyI2C起始
	 MyI2C_SendByte(QMC5883P_ADDRESS);	      //发送从机地址，读写位为0，表示即将写入
	 MyI2C_ReceiveAck();					            //接收应答
	 MyI2C_SendByte(RegAddress);		        	//发送寄存器地址
   MyI2C_ReceiveAck();	                    //接收应答
	 MyI2C_SendByte(Data);				            //发送要写入寄存器的数据
	 MyI2C_ReceiveAck();				            	//接收应答
	 MyI2C_Stop();					                 	//MyI2C终止
}

// QMC5883P多字节写入寄存器
void QMC5883P_WriteRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length)
{
    MyI2C_Start();                      // MyI2C起始
    MyI2C_SendByte(QMC5883P_ADDRESS);               // 发送从机地址，读写位为0，表示即将写入
    MyI2C_ReceiveAck();                 // 接收应答
    MyI2C_SendByte(RegAddress);         // 发送寄存器起始地址
    MyI2C_ReceiveAck();                 // 接收应答
    
    for (uint8_t i = 0; i < Length; i++)
    {
        MyI2C_SendByte(Data[i]);        // 发送要写入寄存器的数据字节
        MyI2C_ReceiveAck();             // 接收应答
    }
    
    MyI2C_Stop();                       // MyI2C终止
}

/***********************************************************************WRITE*/


/*READ***********************************************************************/
/**
  * 函    数：QMC5883P读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考QMC5883P手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0x0B
  */
uint8_t QMC5883P_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();					         	//MyI2C起始
	MyI2C_SendByte(QMC5883P_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();				      	//接收应答
	MyI2C_SendByte(RegAddress);		  	//发送寄存器地址
	MyI2C_ReceiveAck();					      //接收应答
	
	MyI2C_Start();						                 //MyI2C重复起始
	MyI2C_SendByte(QMC5883P_ADDRESS | 0x01);	 //发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					               //接收应答
	Data = MyI2C_ReceiveByte();		           	//接收指定寄存器的数据
	MyI2C_SendAck(1);				                   //发送应答，给从机非应答，终止从机的数据输出
	MyI2C_Stop();					                  	//MyI2C终止
	
	return Data;
}



// QMC5883P读寄存器(多字节)
void QMC5883P_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint16_t Length)
{
    MyI2C_Start();                      // MyI2C起始
    MyI2C_SendByte( QMC5883P_ADDRESS);               // 发送从机地址，读写位为0，表示即将写入
    MyI2C_ReceiveAck();                 // 接收应答
    MyI2C_SendByte(RegAddress);         // 发送寄存器起始地址
    MyI2C_ReceiveAck();                 // 接收应答
    
    MyI2C_Start();                      // MyI2C重复起始
    MyI2C_SendByte( QMC5883P_ADDRESS| 0x01);        // 发送从机地址，读写位置1，表示即将读取
    MyI2C_ReceiveAck();                 // 接收应答
    
    // 循环读取多个字节
    for (uint16_t i = 0; i < Length; i++)
    {
        Data[i] = MyI2C_ReceiveByte();  // 接收从机发送的数据字节
        
        // 如果不是最后一个字节发送ACK，最后一个字节发送NACK
        if (i < Length - 1)
        {
            MyI2C_SendAck(0);           // ACK，继续读取
        }
        else
        {
            MyI2C_SendAck(1);           // NACK，停止读取
        }
    }
    
    MyI2C_Stop();                       // MyI2C终止
}
/***********************************************************************READ*/



uint8_t QMC5883P_GetID(void)
{
	return QMC5883P_ReadReg(QMC5883P_REG_CHIPID);		//返回CHIPID寄存器的值
}


float EMA(float rawValue, float filteredValue)
{
    return 0.2 * rawValue + (1 - 0.2) * filteredValue;
}




/* ================= 基础 ================= */

void QMC5883P_Init(void)
{
  MyI2C_Init();
 									
	QMC5883P_WriteReg(0x29,0x06);
	QMC5883P_WriteReg(0x0b,0x08);
	QMC5883P_WriteReg(0x0a,0xcd);
}


void QMC5883P_GetData(int16_t *X, int16_t *Y, int16_t *Z)
{
    uint8_t buf[6];
    QMC5883P_ReadRegs(0x01, buf, 6);

    *X = (buf[1] << 8) | buf[0];
    *Y = (buf[3] << 8) | buf[2];
    *Z = (buf[5] << 8) | buf[4];
}


void QMC5883P_GetData_uT(float *mx,float *my,float *mz)
{
	int16_t hx,hy,hz;
	QMC5883P_GetData(&hx,&hy,&hz);
	//
	*mx=hx*QMC5883P_LSB_UT;          
	*my=hy*QMC5883P_LSB_UT;
	*mz=hz*QMC5883P_LSB_UT;
}


/* ================= 校准 ================= */
// 采样点→拟合椭球→求中心偏置→求软铁矩阵→输出校准

// 开始做最小二乘拟合前，先把历史样本全部清空
void QMC5883P_StartCalibration(QMC5883P_Calibration_t *calib)
{
    memset(calib->A, 0, sizeof(calib->A));
    calib->sample_count = 0;
}


void QMC5883P_UpdateCalibration(QMC5883P_Calibration_t *calib)
{
    int16_t hx,hy,hz;
    QMC5883P_GetData(&hx,&hy,&hz);
   
    double xd = hx / 1200.0;
    double yd = hy / 1200.0;
    double zd = hz / 1200.0;

    double h[10] = {
        xd*xd, yd*yd, zd*zd,
        2*xd*yd, 2*xd*zd, 2*yd*zd,
        2*xd, 2*yd, 2*zd,
        1.0f
    };
    for(int i=0;i<10;i++)
        for(int j=i;j<10;j++)
        {
            calib->A[i][j]+=h[i]*h[j];
            calib->A[j][i]=calib->A[i][j];
        }
    calib->sample_count++;
}


/* ===== 高斯9x9 ===== */
static uint8_t GaussElimination(double A[9][9],double b[9],double x[9])
{
	double mat[9][10];
	for(int i=0;i<9;i++) 
	{
		for (int j=0;j<9;j++) mat[i][j]=A[i][j];
		mat[i][9]=b[i];
	}
	for(int col=0;col<9;col++) 
	{
		int pivot=col;
		for(int row=col+1;row<9;row++)
			if(fabs(mat[row][col])>fabs(mat[pivot][col]))pivot=row;
		for(int j=0;j<=9;j++) 
		{
			double tmp=mat[col][j];
			mat[col][j]=mat[pivot][j];
			mat[pivot][j]=tmp;
		}
		if(fabs(mat[col][col])<1e-20f) return 0; 
		for(int row=0;row<9;row++) 
		{
			if(row==col)continue;
			double factor=mat[row][col]/mat[col][col];
			for(int j=col;j<=9;j++)
				mat[row][j]-=factor*mat[col][j];
		}
	}
	for(int i=0;i<9;i++)x[i]=mat[i][9]/mat[i][i];
	return 1;
}



static void mat3x3_mul(double A[3][3], double B[3][3], double C[3][3])
{
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++) 
		{
			C[i][j]=0;
			for(int k=0;k<3;k++)
			{
				C[i][j]+=A[i][k]*B[k][j];
			}
		}
	}
}



static void eig3x3(double A[3][3], double eigvec[3][3], double eigval[3])
{
	double S[3][3];
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) S[i][j]=A[i][j];
	//
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) eigvec[i][j]=(i==j)?1.0:0.0;
	// 
	for(int iter=0;iter<100;iter++) 
	{
		int p=0, q=1;
		double maxval = fabs(S[0][1]);
		if(fabs(S[0][2])>maxval) {maxval=fabs(S[0][2]);p=0;q=2;}
		if(fabs(S[1][2])>maxval) {maxval=fabs(S[1][2]);p=1;q=2;}
		if(maxval<1e-12) break;
		// 
		double theta=0.5*atan2(2*S[p][q],S[p][p]-S[q][q]);
		double c = cos(theta), s=sin(theta);
		// 
		double G[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
		G[p][p]=c;G[q][q]=c;G[p][q]=-s;G[q][p]=s;
		// 
		double tmp[3][3];
		// 
		for(int i=0;i<3;i++) for(int j=0;j<3;j++) 
		{
			tmp[i][j]=0;
			for(int k=0;k<3;k++) 
				tmp[i][j]+=G[k][i]*S[k][j];
		}
		// 
		for(int i=0;i<3;i++) for(int j=0;j<3;j++) 
		{
			S[i][j]=0;
			for(int k=0;k<3;k++)
				S[i][j]+=tmp[i][k]*G[k][j];
		}
		// 
		for(int i=0;i<3;i++) 
			for(int j=0;j<3;j++) 
			{
				tmp[i][j]=0;
				for(int k=0;k<3;k++) 
					tmp[i][j]+=eigvec[i][k]*G[k][j];
			}
		for(int i=0;i<3;i++) for(int j=0;j<3;j++) eigvec[i][j]=tmp[i][j];
	}
	eigval[0]=S[0][0]; eigval[1]=S[1][1]; eigval[2]=S[2][2];
}




/* ===== 结束校准 ===== */

void QMC5883P_EndCalibration(QMC5883P_Calibration_t *calib)
{
  static double A9[9][9];  
	static double mat3[3][4];
	static double TA[4][4];
	static double R[4][4];
	static double Q[3][3];
	static double U[3][3];
	static double sqrtD[3][3];
	static double UsqrtD[3][3];
	static double Mmat[3][3];
	double b9[9];
	//
	for(int i=0;i<4;i++) for(int j=0;j<4;j++) TA[i][j]=0;
	for(int i=0;i<4;i++) for(int j=0;j<4;j++) R[i][j]=0;
	for(int i=0;i<9;i++) 
	{
		b9[i]=-calib->A[i][9];
		for(int j=0;j<9;j++)
			A9[i][j]=calib->A[i][j];
	}
	
	double v[10];
	double theta[9];
	if(!GaussElimination(A9,b9,theta)) return;
	for(int i =0;i<9;i++) 
		v[i]=theta[i];
	v[9]=1.0;
	//
	if(v[0]<0) for(int i=0;i<10;i++) v[i]=-v[i];
	//
	double Amat[4][4]={
		{v[0],v[3],v[4],v[6]},
		{v[3],v[1],v[5],v[7]},
		{v[4],v[5],v[2],v[8]},
		{v[6],v[7],v[8],v[9]}
	};
	
	double M[3][3] = {
		{Amat[0][0],Amat[0][1],Amat[0][2]},
		{Amat[1][0],Amat[1][1],Amat[1][2]},
		{Amat[2][0],Amat[2][1],Amat[2][2]}
	};
	double rhs[3]={-v[6],-v[7],-v[8]};
	//
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++) 
			mat3[i][j]=M[i][j];
		mat3[i][3]=rhs[i];
	}
	for(int col=0;col<3;col++)
	{
		int pivot=col;
		for(int row=col+1;row<3;row++)
			if(fabs(mat3[row][col])>fabs(mat3[pivot][col])) pivot=row;
		for(int j=0;j<=3;j++)
		{
			double tmp=mat3[col][j]; 
			mat3[col][j]=mat3[pivot][j];
			mat3[pivot][j]=tmp;
		}
		for(int row=0;row<3;row++)
		{
			if(row==col) continue;
			double f=mat3[row][col]/mat3[col][col];
			for(int j=col;j<=3;j++) 
				mat3[row][j]-=f*mat3[col][j];
		}
	}
	double center[3];
	for(int i=0;i<3;i++) 
		center[i]=mat3[i][3]/mat3[i][i];
	//
	calib->x_offset=(float)(center[0]*1200.0);
	calib->y_offset=(float)(center[1]*1200.0);
	calib->z_offset=(float)(center[2]*1200.0);
	//
	double T[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	T[3][0]=center[0];
	T[3][1]=center[1];
	T[3][2]=center[2];
	//
	for(int i=0;i<4;i++) 
		for(int j=0;j<4;j++)
			for(int k=0;k<4;k++) 
				TA[i][j]+=T[i][k]*Amat[k][j];
	//
	for(int i=0;i<4;i++) 
		for(int j=0;j<4;j++)
			for(int k=0;k<4;k++) 
				R[i][j]+=TA[i][k]*T[j][k]; 
	//
	for(int i=0;i<3;i++) 
		for(int j=0;j<3;j++)
			Q[i][j]=R[i][j]/(-R[3][3]);
	//
	double  eigval[3];
	eig3x3(Q, U, eigval);
	//
	for(int i=0;i<3;i++) 
		sqrtD[i][i]=sqrt(fabs(eigval[i]));
	//
	mat3x3_mul(U, sqrtD, UsqrtD);
	//
	for(int i=0;i<3;i++) 
		for(int j=0;j<3;j++)
		{
			Mmat[i][j]=0;
			for(int k=0;k<3;k++) 
				Mmat[i][j]+=UsqrtD[i][k]*U[j][k];
		}
	//
	for(int i=0;i<3;i++) 
		for(int j=0;j<3;j++)
			calib->W[i][j]=(float)Mmat[i][j];
}


/* ================= 输出 ================= */

void QMC5883P_GetCalibData(QMC5883P_Calibration_t *cal)                             
{
	int16_t hx, hy, hz;
	QMC5883P_GetData(&hx,&hy,&hz);
	//
	float dx=hx-cal->x_offset;
	float dy=hy-cal->y_offset;
	float dz=hz-cal->z_offset;
	//
	cal->m[0]=(cal->W[0][0]*dx+cal->W[0][1]*dy+cal->W[0][2]*dz);
	cal->m[1]=(cal->W[1][0]*dx+cal->W[1][1]*dy+cal->W[1][2]*dz);
	cal->m[2]=(cal->W[2][0]*dx+cal->W[2][1]*dy+cal->W[2][2]*dz);
}



void QMC5883P_GetCalibData_uT(QMC5883P_Calibration_t *cal)                             
{
	int16_t hx, hy, hz;
	QMC5883P_GetData(&hx,&hy,&hz);
	//
	float dx=hx-cal->x_offset;
	float dy=hy-cal->y_offset;
	float dz=hz-cal->z_offset;
	//
	cal->m[0]=(cal->W[0][0]*dx+cal->W[0][1]*dy+cal->W[0][2]*dz)*QMC5883P_LSB_UT;
	cal->m[1]=(cal->W[1][0]*dx+cal->W[1][1]*dy+cal->W[1][2]*dz)*QMC5883P_LSB_UT;
	cal->m[2]=(cal->W[2][0]*dx+cal->W[2][1]*dy+cal->W[2][2]*dz)*QMC5883P_LSB_UT;
}


