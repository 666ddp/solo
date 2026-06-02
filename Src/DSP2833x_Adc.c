// TI File $Revision: /main/5 $
// Checkin $Date: October 23, 2007   13:34:09 $
//###########################################################################
//
// FILE:	DSP2833x_Adc.c
//
// TITLE:	DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#pragma CODE_SECTION(Ad_CaiJi, "ramfuncs");
#pragma CODE_SECTION(JiSuan_Dl, "ramfuncs");

extern _iq ia;
extern _iq ib;
extern _iq ic;


#define ADC_usDELAY  5000L

int32 AD_BUF[8] = {0,0,0,0,0,0,0,0};
int16 *AdcResult = (int16 *)0x4001;	//read AD convst result 

Uint16 I_A=0;
Uint16 I_B=0;
Uint16 I_C=0;

float32 IA_MAX=0;
float32 IB_MAX=0;
float32 IC_MAX=0;

Uint16 I_A_2=0;
Uint16 I_B_2=0;
Uint16 I_C_2=0;

float32 IA_2_MAX=0;
float32 IB_2_MAX=0;
float32 IC_2_MAX=0;

int16 HALL_U=0; //U相霍尔的零点
int16 HALL_W=0; //W相霍尔的零点

int16 HALL_U_2=0; //U相霍尔的零点
int16 HALL_W_2=0; //W相霍尔的零点

float32 U_dc=0;
float32 U_dc_2=0;
Uint16 U_dc_dis=0;
Uint16 U_dc_dis_2=0;
Uint16 GuoliuZhi=0;
Uint16 GuoliuZhi_2=0;
Uint16 O_Current=0;
Uint16 O_Current_2=0;
Uint16 DC_ON_flag=0;
Uint16 DC_ON_OPEN=0;
Uint32 DC_ON_CNT=0;

Uint16 DC_ON_flag_2=0;
Uint16 DC_ON_OPEN_2=0;
Uint32 DC_ON_CNT_2=0;

_iq E_Ding_DianLiu_Q=0;
_iq E_Ding_DianLiu_2_Q=0;

void DelayUS(Uint16 N_US) //1US延时 
{
    Uint16 i=0;  

	for(i=0;i<N_US;i++)
	{
	  asm("	NOP");
	
	}

} 
 
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);


    // *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

	    EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
		ADC_cal();
		EDIS;




    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
	// after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
	// CPU_RATE define statement in the DSP2833x_Examples.h file must
	// contain the correct CPU clock period in nanoseconds.

    AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
}


void Ad_CaiJi(void)
{	
    Uint16 i=0,j=0; 


       
     

     while(GpioDataRegs.GPBDAT.bit.GPIO52==1)  //置1表示ADC正忙
     {
	 
	 j++;
	 if(j==200)//防止死循环，计数超过200强制退出
	 {break;}

     
     
     };

     EALLOW; 
     GpioDataRegs.GPBDAT.bit.GPIO48=0;//ad cs B48置0表示芯片被选中，可以通信
     EDIS;

	

     for(i=0;i<8;i++)  //启动读取
     {
        EALLOW;
        GpioDataRegs.GPADAT.bit.GPIO27=0;//ad rd A27置0 启动读取
        EDIS;
        

        AD_BUF[i] = (*AdcResult);//读取结果
       AD_BUF[i]=AD_BUF[i]*0.1525;//这里是真实值 ad7606 单位 mv （转换为实际电压值）

        EALLOW;
        GpioDataRegs.GPADAT.bit.GPIO27=1;//A27置1结束读取
        EDIS;


     }
     if(Run_PMSM==2)//记录电机停止时的零点偏移，用于运行时补偿
     {    //轴1停止
          HALL_U=AD_BUF[2];
          HALL_W=AD_BUF[3];

     }

     if(Run_PMSM_2==2)
     {   
          //轴2停止
          HALL_U_2=AD_BUF[0];
          HALL_W_2=AD_BUF[1];

     }

    //轴1电流计算 U V W
     ic=_IQdiv(_IQ(AD_BUF[3])-_IQ(HALL_W),E_Ding_DianLiu_Q);
     ic=_IQmpy(ic, _IQ(0.025));//W相电流
     ia=_IQdiv(_IQ(AD_BUF[2])-_IQ(HALL_U),E_Ding_DianLiu_Q);
     ia=_IQmpy(ia, _IQ(0.025));//U相电流
     ib=-ia-ic;    // V相电流Compute phase-c current

    //轴2电流计算
     ic_2=_IQdiv(_IQ(AD_BUF[1])-_IQ(HALL_W_2),E_Ding_DianLiu_2_Q);
     ic_2=_IQmpy(ic_2, _IQ(0.025));//W相电流
     ia_2=_IQdiv(_IQ(AD_BUF[0])-_IQ(HALL_U_2),E_Ding_DianLiu_2_Q);
     ia_2=_IQmpy(ia_2, _IQ(0.025));//V相电流
     ib_2=-ia_2-ic_2;    // V相电流 Compute phase-c current

     

      EALLOW; 
     GpioDataRegs.GPBDAT.bit.GPIO48=1;//ad cs B48置1禁用片选
     EDIS;

}


void DC_Link(void)//计算母线电压
{
   
      U_dc=0.2951*AD_BUF[5];//轴1直流母线电压采样
 
    U_dc_dis=U_dc; //储存显示值
   
   

    if((Run_PMSM==1)&&(DC_ON_flag==0))//轴1运行&&掉电 则停机
    {
        if(U_dc_dis<5)//执行停机
        {
                DC_ON_1; //关闭轴1电机
        DC_ON_flag=1;//设置欠压标志
        DC_ON_OPEN=2;//显示故障类型：欠压


        }

    }

  if(U_dc_dis>360)//过压，执行停机
   {
       DC_ON_1; //关闭轴1电机
       DC_ON_flag=1;//设置欠、过压标志
       DC_ON_OPEN=3;//显示故障类型：过压

   }



    U_dc_2=0.2951*AD_BUF[4];//轴2直流母线电压采样
    U_dc_dis_2=U_dc_2;//轴2储存显示值

        if((Run_PMSM_2==1)&&(DC_ON_flag_2==0))//运行中掉电 则停机
    {
        if(U_dc_dis_2<5)//执行停机
        {
                DC_ON2_1;
        DC_ON_flag_2=1;
        DC_ON_OPEN_2=2;


        }

    }

  if(U_dc_dis_2>360)//过压，执行停机
   {
       DC_ON2_1; 
       DC_ON_flag_2=1;
       DC_ON_OPEN_2=3;

   } 

    

}




void JiSuan_Dl(void)//计算电流
{
       float32 IA,IB,IC,IA_2,IB_2,IC_2;
    static Uint16 i=0;
    static Uint16 j=0;
    
    IC=(AD_BUF[3]-HALL_W)*0.25;//放大10倍  轴1电流计算
    IA=(AD_BUF[2]-HALL_U)*0.25;//放大10倍 轴1电流计算
    IB=0-IA-IC;//轴1电流计算
    if(IB<0.0)//取绝对值
    {IB=-IB;}
    if(IA<0.0)
    {IA=-IA;}
     if(IC<0.0)
    {IC=-IC;}

       if((IA>200)||(IB>200)||(IC>200))//过流保护（第一级保护）
    {   DC_ON_1;//断开电源
        Pwm_EN_1;//禁止PWM输出
        eva_close();//过流保护， 停机 停止PWM生成
        O_Current=2;//故障类型：过流
       
           Run_PMSM=2;//停止运行
            

    }

    if(IA>IA_MAX)//峰值检测
    {
        IA_MAX=IA;
    }
    if(IB>IB_MAX)
    {
        IB_MAX=IB;
    }
      if(IC>IC_MAX)
    {
        IC_MAX=IC;
    }
    i++;//计算峰值有效值 /根号2
    if(i==300)
    {
    I_A=IA_MAX/1.414;//U相有效值
    I_B=IB_MAX/1.414;//V相有效值
    I_C=IC_MAX/1.414;//W相有效值
    i=0;
    IB_MAX=0;
    IA_MAX=0;
    IC_MAX=0;

    if((I_A>GuoliuZhi)||(I_B>GuoliuZhi)||(I_C>GuoliuZhi))//过流保护（相同于瞬时保护）（第二级保护）
    {   DC_ON_1;//断直流电
        Pwm_EN_1;
        eva_close();//过流保护， 停机
        O_Current=1;//故障类型：有效值过流
       
           Run_PMSM=2;
            

    }

    }
 

//轴2
    IC_2=(AD_BUF[1]-HALL_W_2)*0.25;//放大10倍 轴2电流计算
    IA_2=(AD_BUF[0]-HALL_U_2)*0.25;//放大10倍 轴2电流计算
    IB_2=0-IA_2-IC_2;//轴2电流计算
    if(IB_2<0.0)
    {IB_2=-IB_2;}
    if(IA_2<0.0)
    {IA_2=-IA_2;}
     if(IC_2<0.0)
    {IC_2=-IC_2;}

       if((IA_2>200)||(IB_2>200)||(IC_2>200))
    {  DC_ON2_1;
        Pwm_EN2_1;
        eva_close_2();//过流保护， 停机
        O_Current_2=2;
       
        Run_PMSM_2=2;
            

    }

    if(IA_2>IA_2_MAX)
    {
        IA_2_MAX=IA_2;
    }
    if(IB_2>IB_2_MAX)
    {
        IB_2_MAX=IB_2;
    }
      if(IC_2>IC_2_MAX)
    {
        IC_2_MAX=IC_2;
    }
    j++;
    if(j==300)
    {
    I_A_2=IA_2_MAX/1.414;//有效值
    I_B_2=IB_2_MAX/1.414;//有效值
    I_C_2=IC_2_MAX/1.414;//有效值
    j=0;
    IB_2_MAX=0;
    IA_2_MAX=0;
    IC_2_MAX=0;

    if((I_A_2>GuoliuZhi_2)||(I_B_2>GuoliuZhi_2)||(I_C_2>GuoliuZhi_2))
    {   DC_ON2_1;
        Pwm_EN2_1;
        eva_close_2();//过流保护， 停机
        O_Current_2=1;
       
           Run_PMSM_2=2;
            

    }
   
    }
 

} 


