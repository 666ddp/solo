// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:25 $
//###########################################################################
//
// FILE:	DSP2833x_Gpio.c
//
// TITLE:	DSP2833x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################
#include "IQmathLib.h"       // IQmath 库头文件，定义了 iq 类型
#include "DSP28x_Project.h"  // 项目主头文件
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#pragma CODE_SECTION(IPM_BaoHu, "ramfuncs");



Uint16 IPM_Fault=0;
Uint16 IPM_Fault_2=0;
Uint16 *HallAngle1_rd = (Uint16 *)0x4008;	//read AD convst result 
Uint16 *HallAngle2_rd = (Uint16 *)0x4007;	//read AD convst result 


//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example. 
void InitGpio(void)
{
   EALLOW;
   
   // Each GPIO pin can be: 
   // a) a GPIO input/output
   // b) peripheral function 1
   // c) peripheral function 2
   // d) peripheral function 3
   // By default, all are GPIO Inputs 
   GpioCtrlRegs.GPAMUX1.all = 0x0000;     // GPIO functionality GPIO0-GPIO15
   GpioCtrlRegs.GPAMUX2.all = 0x0000;     // GPIO functionality GPIO16-GPIO31
   GpioCtrlRegs.GPBMUX1.all = 0x0000;     // GPIO functionality GPIO32-GPIO39
   GpioCtrlRegs.GPBMUX2.all = 0x0000;     // GPIO functionality GPIO48-GPIO63
   GpioCtrlRegs.GPCMUX1.all = 0x0000;     // GPIO functionality GPIO64-GPIO79
   GpioCtrlRegs.GPCMUX2.all = 0x0000;     // GPIO functionality GPIO80-GPIO95

      GpioCtrlRegs.GPADIR.all = 0x0000;      // GPIO0-GPIO31 are inputs
   GpioCtrlRegs.GPBDIR.all = 0x0000;      // GPIO32-GPIO63 are inputs   
   GpioCtrlRegs.GPCDIR.all = 0x0000;      // GPI064-GPIO95 are inputs

   // Each input can have different qualification
   // a) input synchronized to SYSCLKOUT
   // b) input qualified by a sampling window
   // c) input sent asynchronously (valid for peripheral inputs only)
   GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // GPIO0-GPIO15 Synch to SYSCLKOUT 
   GpioCtrlRegs.GPAQSEL2.all = 0x0000;    // GPIO16-GPIO31 Synch to SYSCLKOUT
   GpioCtrlRegs.GPBQSEL1.all = 0x0000;    // GPIO32-GPIO39 Synch to SYSCLKOUT 
   GpioCtrlRegs.GPBQSEL2.all = 0x0000;    // GPIO48-GPIO63 Synch to SYSCLKOUT 

   // Pull-ups can be enabled or disabled. 
   GpioCtrlRegs.GPAPUD.all = 0x0000;      // Pullup's enabled GPIO0-GPIO31
   GpioCtrlRegs.GPBPUD.all = 0x0000;      // Pullup's enabled GPIO32-GPIO63
   GpioCtrlRegs.GPCPUD.all = 0x0000;      // Pullup's enabled GPIO64-GPIO79



    //ad760x
   
   GpioCtrlRegs.GPAMUX2.bit.GPIO21=0;//AD760x RESTAD 引脚 设置为GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO49=0;//AD760x CONVST 引脚 设置为GPIO
   GpioCtrlRegs.GPAMUX2.bit.GPIO27=0;//AD760x AD_RD 引脚 设置为GPIO
   GpioCtrlRegs.GPBMUX2.bit.GPIO52=0;//busy ad
   GpioCtrlRegs.GPBMUX2.bit.GPIO48=0;//AD760x AD_CS 引脚 设置为GPIO

   GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;      // AD760x RESTAD 引脚 设置为输出
   GpioCtrlRegs.GPBDIR.bit.GPIO49= 1;      // AD760x CONVST 引脚 设置为输出
   GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;      // AD760x AD_RD 引脚 设置为输出
   GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0;      // busy ad
   GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;      // AD760x AD_CS 引脚 设置为输出

   GpioDataRegs.GPADAT.bit.GPIO21=0;//AD760x RESTAD 引脚 输出低电平
   GpioDataRegs.GPBDAT.bit.GPIO49=1;//AD760x CONVST 引脚 输出高电平
   GpioDataRegs.GPADAT.bit.GPIO27=1;//AD760x AD_RD 引脚 输出高电平
   GpioDataRegs.GPBDAT.bit.GPIO48=1;//AD760x AD_CS 引脚 输出高电平

 

   //pwm_en1
   
   GpioCtrlRegs.GPAMUX2.bit.GPIO16=0;//pwm_en 为gpio
   Pwm_EN_1;
   GpioCtrlRegs.GPADIR.bit.GPIO16= 1 ;//PEM_EN为输出
//pwm_en2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14=0;//pwm_en2 为gpio
   Pwm_EN2_1;
   GpioCtrlRegs.GPADIR.bit.GPIO14= 1 ;//PEM_EN2为输出
GpioDataRegs.GPADAT.bit.GPIO21=1;
      //alarm 1
   GpioCtrlRegs.GPAMUX1.bit.GPIO12=0;//设置 alarm为gpio
   GpioCtrlRegs.GPADIR.bit.GPIO12=0;//设置alarm为输入

   
   //alarm 2
   GpioCtrlRegs.GPAMUX1.bit.GPIO13=0;//设置 alarm为gpio
   GpioCtrlRegs.GPADIR.bit.GPIO13=0;//设置alarm为输入

   //DC_ON 1

   GpioCtrlRegs.GPAMUX2.bit.GPIO17=0;//设置DC_ON 脚为GPIO 
   DC_ON_1;
    GpioCtrlRegs.GPADIR.bit.GPIO17= 1; // 设置DC_ON为输出
    DC_ON_1;
       //DC_ON 2

   GpioCtrlRegs.GPAMUX1.bit.GPIO15=0;//设置DC_ON 脚为GPIO 
   DC_ON2_1;
    GpioCtrlRegs.GPADIR.bit.GPIO15= 1; // 设置DC_ON为输出
  DC_ON2_1;
     

    //SPICS
   
   GpioCtrlRegs.GPBMUX2.bit.GPIO57=0;//设置spics脚为GPIO 
   GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;      
    GpioDataRegs.GPBDAT.bit.GPIO57=1;
  //uvw hall cs
   GpioCtrlRegs.GPBMUX2.bit.GPIO61=0;//设置为GPIo
  

   GpioCtrlRegs.GPBDIR.bit.GPIO61=1;//输入 
 
   GpioDataRegs.GPBDAT.bit.GPIO61=1;

 //485_set
 GpioCtrlRegs.GPAMUX2.bit.GPIO20=0;//设置485_set 脚为GPIO 
 GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;      // 485_set  引脚 设置为输出
    GpioDataRegs.GPADAT.bit.GPIO20=1;//设置 485_set输出高电平 

    //INPUT

   GpioCtrlRegs.GPBMUX2.bit.GPIO60=0;//设置INPUT1 脚为GPIO 
   
   GpioCtrlRegs.GPBDIR.bit.GPIO60= 0;      // INPUT1 引脚 设置为输入
   
//output
   GpioCtrlRegs.GPBMUX2.bit.GPIO59=0;//设置OUTPU1 脚为GPIO
   GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;      // OUTPU1  引脚 设置为输出
 
   
    
 GpioDataRegs.GPADAT.bit.GPIO21=0;
//abc

    GpioCtrlRegs.GPBMUX2.bit.GPIO50=1;// 编码器 a
    GpioCtrlRegs.GPBMUX2.bit.GPIO51=1;// 编码器 b
     GpioCtrlRegs.GPBMUX2.bit.GPIO53=1;// 编码器 c

        GpioCtrlRegs.GPAMUX2.bit.GPIO24=2;// 编码器 a_2
    GpioCtrlRegs.GPAMUX2.bit.GPIO25=2;// 编码器 b_2
     GpioCtrlRegs.GPAMUX2.bit.GPIO26=2;// 编码器 c_2
//外部中断
      GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL=53;//gpio53 作为外部中断3输入引脚
    XIntruptRegs.XINT3CR.bit.ENABLE=1;//外部中断打开
    XIntruptRegs.XINT3CR.bit.POLARITY=3;//上升沿有效

   EDIS;

   Init_Scib_Gpio();
   Init_Scic_Gpio();
   InitEPwmGpio();
   InitSpiaGpio();

  
   

}

void eva_close(void)
{
     EALLOW;
       //  1.3.5强制高，2.4.6有效

       GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;   // Configure GPIO0 as EPWM1A

     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;   // Configure GPIO2 as EPWM2A

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;   // Configure GPIO4 as EPWM3A


    GpioCtrlRegs.GPADIR.bit.GPIO0=1;
    GpioCtrlRegs.GPADIR.bit.GPIO2=1;
    GpioCtrlRegs.GPADIR.bit.GPIO4=1;

    GpioDataRegs.GPASET.bit.GPIO0=1;
    GpioDataRegs.GPASET.bit.GPIO2=1;
    GpioDataRegs.GPASET.bit.GPIO4=1;





    EPwm1Regs.CMPA.half.CMPA =3375; //
EPwm2Regs.CMPA.half.CMPA = 3375; //
EPwm3Regs.CMPA.half.CMPA = 3375; //
   EDIS;
     Run_PMSM=2;
   LocationFlag=1;
   Speed_Ui=0;
   ID_Ui=0;
   IQ_Ui=0;
   Position=1;
   j=0;
   speed_dis=0;
   IQ_Given=0;
   OldRawTheta=0;
   SpeedRef=0;
   speed_give=0;
   Modulation=0.25;    // 调制比
   O_Current=0;
   PosCount=0;
   OldRawThetaPos=0;
   Hall_Fault=0;
   Speed_run=0;
}


void Init_Scib_Gpio(void)//232
{
   EALLOW;
	
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;     // Enable pull-up for GPIO9  (SCITXDB) 

	
  GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up for GPIO11 (SCIRXDB) 

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO11 (SCIRXDB)
//  GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (SCIRXDB)
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDB)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)

/* Configure SCI-B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
// Comment out other unwanted lines.

  GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;    // Configure GPIO9 for SCITXDB operation 
	
  GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO11 for SCIRXDB operation 
	
    EDIS;
}

void Init_Scic_Gpio(void)//485
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

/* Configure SCI-C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation
	
    EDIS;
}





void eva_open(void)
{
     EALLOW;
       GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B       
   
   EDIS;


}


void eva_open_2(void)//轴2 pwm允许输出
{
     EALLOW;
       GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO0 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO1 as EPWM4B
     GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO2 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO3 as EPWM5B
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO4 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO5 as EPWM6B       
   
   EDIS;


}


void Get_HallAngle(void)
{   Uint16  temp=0;
    EALLOW;
    GpioDataRegs.GPBCLEAR.bit.GPIO61=1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;  // XA3
    GpioCtrlRegs.GPBDIR.bit.GPIO43=1;
    GpioDataRegs.GPBSET.bit.GPIO43=1;//hall2
    EDIS;

	 asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP"); 
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
      
	 temp=(*HallAngle2_rd);
     

              HallAngle_2=0;
        if(temp&4) //W
        {
		HallAngle_2+=1;
        }
		if(temp&2)//V
		{
            HallAngle_2+=2;
            }
		
		if(temp&1)//U
		{
            HallAngle_2+=4;

        }
    
    EALLOW;
    GpioDataRegs.GPBCLEAR.bit.GPIO43=1;//hall1
    EDIS;
   asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP"); 
      asm(" NOP");
       asm(" NOP");
       asm(" NOP");
      asm(" NOP");
       asm(" NOP");
	    asm(" NOP");
      asm(" NOP");
     
	   temp=(*HallAngle1_rd);
	

              HallAngle=0;
        if(temp&4) //W
        {
		HallAngle+=1;
        }
		if(temp&2)//V
		{
            HallAngle+=2;
            }
		
		if(temp&1)//U
		{
            HallAngle+=4;

        }
    EALLOW;
    GpioDataRegs.GPBSET.bit.GPIO61=1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3;  // XA3
    EDIS;




}















void  IPM_BaoHu(void)
{
    static Uint32 i=0,ii=0;
    Uint16 j=0,jj=0;//读取GPIO状态

    j=GpioDataRegs.GPADAT.bit.GPIO12;//读取故障位（故障时输出低）
    if(j==0)
    {
        i++;
        if(i==2)
        {      DC_ON_1;
            Pwm_EN_1;
            eva_close();
            IPM_Fault=1;
            	

        }

    }
    else
    {i=0;
	
    
    }

       jj=GpioDataRegs.GPADAT.bit.GPIO13;//读取故障位
    if(jj==0)
    {
        ii++;
        if(ii==2)
        {
              DC_ON2_1;
            Pwm_EN2_1;
            eva_close_2();
            IPM_Fault_2=1;
            	

        }

    }
    else
    {ii=0;
	
    
    }

    



}
	
//===========================================================================
// End of file.
//===========================================================================
