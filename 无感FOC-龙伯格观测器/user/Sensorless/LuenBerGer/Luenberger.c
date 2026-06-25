 #include "Luenberger.h"

LuenBerGer_Observer  Luenberger_member={


  .Forecast_Ialpha_dt=0,   //预估Ialpha电流微分
  .Forecast_Ibeta_dt=0,    //预估Ibeta电流微分
  .Forecast_Ialpha=0,      //Ialpha电流
  .Forecast_Ibeta=0,       //Ibeta电流
  .V_Alpha=0,              //预估Alpha反电势
 	.V_Beta=0,               //预估Beta反电势
 	.V_Alpha_LPF=0,          //滤波后的预估Alpha反电势
 	.V_Beta_LPF=0,           //滤波后的预估Beta反电势
  .V_Alpha_dt=0,          //预估Alpha反电势微分
 	.V_Beta_dt=0,           //预估Beta反电势微分
	.K1=50.0f, 	
  .K2=600, 	
	.Err_Ialpha=0,
	.Err_Ibeta=0,
	
	
};
Luenberger_pll Luenberger_PLL={
     .PLL_Kp= 20.0f,
     .PLL_Ki= 0.1f,
 
   
     .Err=0,
	   .sum=0,
 	
     .out=0,
     .outMax=1256,

     .We_fore=0,
	   .We_fore_lpf=0,
	   .Theta_fore_now=0,
	   .Theta_fore_Last=0,

};

    float  add_luenberger_angle(float current_angle)
 {
	 static float last_angle=0 ,angle=0;
	 float i = current_angle - last_angle;
if (i < -_PI)  
{
    i += _2PI;
}
else if (i > _PI) 
	{
    i -= _2PI;
}
angle += i;
if(angle>=pair*_2PI)
angle=fmod(angle ,pair*_2PI);
else if(angle<=-pair*_2PI)
angle=fmod(angle ,-pair*_2PI);
last_angle  = current_angle;

	 return angle;
	 
 }
  float Luenberger_atan_angle(void)
{
   static float angle=0;
	 angle=atan2f( -Luenberger_member.V_Alpha_LPF ,Luenberger_member.V_Beta_LPF);  
   return add_luenberger_angle(angle);//处理角度，可以一直累加
}


 
   void LuenBerGer_PLL(LuenBerGer_Observer*Luenberger_member,Luenberger_pll* Luenberger_PLL)
{
  Luenberger_PLL->Err = -cosf(Luenberger_PLL->Theta_fore_Last)*Luenberger_member->V_Alpha_LPF - sinf(Luenberger_PLL->Theta_fore_Last)*Luenberger_member->V_Beta_LPF;
 	Luenberger_PLL->I_Partern +=Luenberger_PLL-> PLL_Ki *  Luenberger_PLL->Err;
	Luenberger_PLL->We_fore = Luenberger_PLL->PLL_Kp * Luenberger_PLL->Err + Luenberger_PLL->I_Partern;
	Luenberger_PLL->We_fore = Limit(Luenberger_PLL->We_fore , Luenberger_PLL->outMax);//限幅，额定转速为3000rpm
	Luenberger_PLL->We_fore = Filter_func(&we_fore_filter, Luenberger_PLL->We_fore);
	Luenberger_PLL->Theta_fore_now  += Luenberger_PLL->We_fore * FOC_dt;
	
	if(Luenberger_PLL->Theta_fore_now>0)
	Luenberger_PLL->Theta_fore_now = fmod(Luenberger_PLL->Theta_fore_now ,pair*_2PI);
	else  if(Luenberger_PLL->Theta_fore_now<0)
	Luenberger_PLL->Theta_fore_now = fmod(Luenberger_PLL->Theta_fore_now ,-pair*_2PI);
	
		Luenberger_PLL->Theta_fore_Last = Luenberger_PLL->Theta_fore_now;
	
	
}
 void Luenberger_Observer(X_FOC *FOC,LuenBerGer_Observer*Luenberger)
 {
 
 Luenberger->Err_Ialpha=Luenberger->Forecast_Ialpha - FOC->I_uAlpha;
 Luenberger->Err_Ibeta =Luenberger->Forecast_Ibeta  - FOC->I_ubeta;
	 
 Luenberger->Forecast_Ialpha_dt=-(motor_R*Luenberger->Forecast_Ialpha+ Luenberger->V_Alpha_LPF -FOC->uAlpha)/motor_L+Luenberger->K1*Luenberger->Err_Ialpha;
 Luenberger->Forecast_Ibeta_dt =-(motor_R*Luenberger->Forecast_Ibeta + Luenberger->V_Beta_LPF -FOC->uBeta)/motor_L+Luenberger->K1*Luenberger->Err_Ibeta;
 Luenberger->V_Alpha_dt= -Luenberger_PLL.We_fore*Luenberger->V_Beta_LPF+Luenberger->K2*Luenberger->Err_Ialpha;
 Luenberger->V_Beta_dt=	 Luenberger_PLL.We_fore* Luenberger->V_Alpha_LPF+Luenberger->K2*Luenberger->Err_Ibeta;
 
 Luenberger->Forecast_Ialpha+=Luenberger->Forecast_Ialpha_dt*FOC_dt;
 Luenberger->Forecast_Ibeta+=Luenberger->Forecast_Ibeta_dt*FOC_dt;
 Luenberger->V_Alpha+=Luenberger->V_Alpha_dt*FOC_dt;
 Luenberger->V_Beta +=Luenberger->V_Beta_dt*FOC_dt;
//	 
 Luenberger->V_Alpha_LPF=	Filter_func(&Luenberger_VAlpha_filter,Luenberger->V_Alpha);
 Luenberger->V_Beta_LPF= Filter_func(&Luenberger_VBeta_filter,Luenberger->V_Beta);
	 
	LuenBerGer_PLL( &Luenberger_member, &Luenberger_PLL);
  //Luenberger_PLL.Theta_fore_now=Luenberger_atan_angle();
  
//    Luenberger_PLL.We_fore=-Luenberger->V_Alpha/FLUX/sinf(Luenberger_PLL.Theta_fore_now);
//// // Luenberger_PLL.We_fore=Luenberger->V_Beta_LPF/FLUX/cosf(Luenberger_PLL.Theta_fore_now);


 }

