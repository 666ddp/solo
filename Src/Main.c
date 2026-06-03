//*****************************************************************************************************
//Flash和RAM软件版本切换说明(程序默认为ram版本)
//
//一.切换为Flash烧写版本方法
//1.将主程序中的:MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//               InitFlash();
//  两个函数取消注释
//2.将工程中28335_RAM_lnk.cmd从工程中删除，添加CMD文件夹下的F28335.cmd文件，全编译一次即可烧写。
//
//二.切换为RAM在线仿真版本方法
//1.将主程序中的:MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//               InitFlash();
//  两个函数注释掉
//2.将工程中的F28335.cmd从工程中删除，添加CMD文件夹下28335_RAM_lnk.cmd文件，全编译一次即可。
//
//*****************************************************************************************************

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <math.h>
#include <stdio.h>  // 必须包含，用于sprintf

#define SPEED_SMC_LAMBDA   _IQ(0.02)   // 积分系数
#define SPEED_SMC_KP       _IQ(0.5)    // 比例增益
#define SPEED_SMC_KSW      _IQ(1.0)    // 切换增益
#define SPEED_SMC_PHI      _IQ(0.05)   // 边界层厚度

// ================= 电机物理参数=================
#define MOTOR_J         (1.2e-4f)   // 转动惯量 J = 1.2×10⁻⁴ kg·m²
#define MOTOR_NP        (4.0f)      // 极对数pp = 4
#define MOTOR_PSIF      (0.1592f)   // 永磁磁链 ψf = 0.1592 Wb
#define MOTOR_B         (0.0001f)   // 黏滞摩擦系数 B

// ================= 预定义时间收敛参数================
float32 P1  = 0.6f;      // 0 < p1 < 1
float32 P2  = 0.6f;      // 0 < p2 < 1
float32 A11 = 1.0f;      // a1 > 0
float32 B11 = 1.0f;      // b1 > 0
float32 A22 = 1.0f;      // a2 > 0
float32 B22 = 1.0f;      // b2 > 0
//#define TC1             (0.05f)     // 滑模面收敛时间T_c1 (s)
//#define TC2             (0.05f)     // 趋近率收敛时间T_c2 (s)
//#define C3              (10.0f)     // 线性项系数 c3 > 0
// 替换为全局变量（赋初值，以便开机时 Init_SiShu 能算出一组默认参数）
float32 TC1 = 0.05f;
float32 TC2 = 0.05f;
float32 C3  = 10.0f;
float32 TC3 = 0.02f;
float32 C8  = 50.0f;
// ================= HIL 贝叶斯优化全局变量 =================
#define BO_RECORD_LENGTH 2000
#define STEP_RECORD_LENGTH 4000
#define RECORD_LENGTH STEP_RECORD_LENGTH
#define DUAL_RECORD_LENGTH BO_RECORD_LENGTH
#define STEP1_END_INDEX 1000
#define STEP2_END_INDEX 2500
#define HIL_STEP_SPEED_REF_40 (0.0133333f)
int16 Log_Speed_x10[RECORD_LENGTH];
int16 Log_Speed_2_x10[DUAL_RECORD_LENGTH];
Uint16 Record_Index = 0;           // 记录索引
Uint16 HIL_RecordLength = BO_RECORD_LENGTH;
Uint16 HIL_StepMode = 0;
char Rx_Buffer[192];
Uint16 Rx_Index = 0;
Uint16 Rx_LineReady = 0;  // ISR置1表示收到完整行
Uint16 New_Params_Flag = 0;
Uint16 New_PI2_Params_Flag = 0;
Uint16 New_Dual_Params_Flag = 0;
Uint16 Eval_State = 0;             // 0:空闲, 1:发车, 2:录制中, 3:准备发送
Uint16 StartDelay = 0;             // 启动前等电压爬升的计数器
Uint16 HIL_FailCode = 0;
Uint16 HIL_BoostDelay = 0;
Uint16 HIL_ForceIq = 0;
Uint16 HIL_MotionSeen = 0;
Uint16 HIL_StartupCheckTicks = 0;
Uint16 HIL_DcDropTicks = 0;
Uint16 Dual_Axis2Started = 0;
Uint16 Dual_Mode = 0;
float32 HIL_LastValidSpeed = 0.0f;
float32 HIL_LastValidSpeed_2 = 0.0f;
float32 HIL_ForceIqPu = 0.0f;
Uint16 HIL_Speed2SpikeCount = 0;

#define HIL_TARGET_SPEED_REF (0.0267f)
#define HIL_LOG_MAX_X10 32767
#define HIL_LOG_MIN_X10 (-32768)
#define HIL_BOOST_IQ_PU_LOW  (0.15f)
#define HIL_BOOST_IQ_PU_HIGH (0.20f)
#define HIL_BOOST_TICKS      (500)
#define HIL_BOOST_STEP_TICKS (250)
#define HIL_MOTION_RPM       (5.0f)
#define HIL_RECORD_MIN_RPM   (40.0f)
#define HIL_MOTION_TIMEOUT  (300)
#define HIL_STARTUP_CHECK_TICKS (300)
#define HIL_DC_DROP_CONFIRM_TICKS (100)
#define HIL_SPEED2_MAX_ABS_RPM (300.0f)
#define HIL_SPEED2_MAX_STEP_RPM (150.0f)
#define HIL_SPEED2_MAX_SPIKES (3)
#define DUAL_AXIS2_START_INDEX (300)
//================= PTDO 观测器专用参数================
float32 P3  = 0.6f;      // 0 < p3 < 1
float32 A33 = 1.0f;      // a3 > 0
float32 B33 = 1.0f;      // b3 > 0
//#define TC3             (0.02f)     // 观测器收敛时间T_c3 (s)，建议比 TC1 快
//#define C8              (50.0f)     // 积分项增益c8 > 0
#define PHI_SIGMA       (5.0f)      // 观测器误差sigma 的边界层（防抖振）
// 抖振抑制（饱和函数边界层）
#define PHI_E           (2.0f)     // 误差 e 的边界层
#define PHI_S           (2.0f)     // 滑模面 s 的边界层
// 前馈使能：1=启用，0=禁用
#define USE_FEEDFORWARD (0)
#define ALIGN_MODULATION (0.35f)
#define RUN_MODULATION   (0.95f)
#pragma CODE_SECTION(EPWM_1_INT, "ramfuncs");

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);
interrupt void EPWM_1_INT(void);
interrupt void SCIBRX_ISR(void);
interrupt void INT3_ISR(void);

void Init_SiShu(void);
void TX232_String(char *str);
void HIL_Evaluation_Task(void);
void HIL_Poll_Rx(void);
float32 HIL_StepSpeedRef(Uint16 index);

static int16 HIL_SpeedRpmToX10(float32 speed_rpm)
{
    float32 scaled = speed_rpm * 10.0f;
    if(scaled > (float32)HIL_LOG_MAX_X10)
    {
        return HIL_LOG_MAX_X10;
    }
    if(scaled < (float32)HIL_LOG_MIN_X10)
    {
        return HIL_LOG_MIN_X10;
    }
    return (int16)scaled;
}

//*****************************************************************************************************
//全局变量定义与初始化
//*****************************************************************************************************
char Matlab_TxBuffer[64];
Uint16 Serial_Tick = 0;      // 添加这一行
float32 Log_Spd1 = 0.0f;
float32 Log_Spd2 = 0.0f;
float32 Log_Iq1 = 0.0f;
float32 Log_Iq2 = 0.0f;
float32 d_hat_filtered = 0.0f;

float32 i=0;
float32 j=0;
float32 k=0;
Uint16 IsrTicker = 0;
Uint16 BackTicker = 0; //用于次数计数
Uint16 T1Period=0;     // T1定时器周期(Q0)
Uint16 T1Period_2=0;     // T1定时器周期(Q0)
Uint16 T3Period = 0;
float32 Modulation=0.25;    // 调制比
float32 Modulation_2=0.25;    // 调制比
int16 MPeriod=0;
int32 Tmp=0;
int16 MPeriod_2=0;
int32 Tmp_2=0;

// 预定义时间滑模控制变量（轴1）
float32 C1, C2, C4, C5;            // 预定义时间系数
float32 KT_INV;                     // 2J/(3npψf)
float32 S_smc = 0.0f;               // 滑模面 s
float32 Integral_f1 = 0.0f;         // ∫f1(e) dt
float32 Omega_dot_ref = 0.0f;       // 参考加速度 d(ω_ref)/dt
float32 SpeedRef_prev = 0.0f;       // 上一拍速度给定

// ----- 新增：PTDO 观测器变量-----
float32 C6, C7, Kt, B_J;           // 观测器常系数
float32 omega_hat = 0.0f;          // 观测器估计转速(rad/s)
float32 d_hat = 0.0f;              // 估计的集总扰动(rad/s^2)
float32 integral_sigma = 0.0f;     // 观测器内部积分项

//:::::::::::::::::::::::::::位置环变量定义::::::::::::::::::::::::::
long PlaceError=0,Place_now=0, Now_P=0,//圈数
              OutPreSat_Place=0;//位置变量值定义

long PlaceError_2=0,Place_now_2=0, Now_P_2=0,//圈数
              OutPreSat_Place_2=0;//位置变量值定义
Uint16 PlaceSetBit=0;  //位置设定标志位
Uint16 PlaceSetBit_2=0;  //位置设定标志位
int32   PosCount = 0;
int32   PosCount_2 = 0;

float32 MfuncF1=0;
float32 MfuncF2=0;
float32 MfuncF3=0;
//===============转子初始位置定位=============================
Uint16 LocationFlag=1;
Uint16 LocationEnd=0;
Uint16 Position=1;
Uint16 LocationFlag_2=1;
Uint16 LocationEnd_2=0;
Uint16 Position_2=1;
Uint16 PositionPhase60=1;
Uint16 PositionPhase120=2;
Uint16 PositionPhase180=3;
Uint16 PositionPhase240=4;
Uint16 PositionPhase300=5;
Uint16 PositionPhase360=6;

//===============DAC模拟=====================================
_iq DACTemp0=0;
_iq DACTemp1=0;
_iq DACTemp2=0;

_iq MfuncC1=0;
_iq MfuncC2=0;
_iq MfuncC3=0;

//===============转子速度计算=====================================
Uint16 SpeedLoopPrescaler = 10;     // 速度环定标
Uint16 SpeedLoopCount = 1;          // 速度环计数
_iq NewRawTheta=0;
_iq OldRawTheta=0;
_iq SpeedRpm=0;                     //速度，单位：转每分钟
Uint16 Hall_Fault=0;
_iq RawThetaTmp=0;
float32 SpeedRef=0;
_iq Speed=0;                        //速度，标幺值
_iq Speed_SMC_Integral = 0;


Uint16 SpeedLoopPrescaler_2 = 10;     // 速度环定标
Uint16 SpeedLoopCount_2 = 1;          // 速度环计数
_iq NewRawTheta_2=0;
_iq OldRawTheta_2=0;
_iq SpeedRpm_2=0;                     //速度，单位：转每分钟
Uint16 Hall_Fault_2=0;
_iq RawThetaTmp_2=0;
float32 SpeedRef_2=0;
_iq Speed_2=0;                        //速度，标幺值


//===============转子角度计算===================================
Uint16 DirectionQep=0;               //转子旋转方向
_iq RawTheta=0;
_iq OldRawThetaPos = 0;

Uint16 DirectionQep_2=0;               //转子旋转方向
_iq RawTheta_2=0;
_iq OldRawThetaPos_2 = 0;

_iq TotalPulse=0;
_iq TotalPulse_2=0;

_iq MechTheta = 0;                   //机械角度，单位：度
_iq ElecTheta = 0;                   //电气角度，单位：度
_iq AnglePU=0;                       //角度标幺值
_iq Cosine=0;
_iq Sine=0;

_iq MechTheta_2 = 0;                   //机械角度，单位：度
_iq ElecTheta_2 = 0;                   //电气角度，单位：度
_iq AnglePU_2=0;                       //角度标幺值
_iq Cosine_2=0;
_iq Sine_2=0;

//===============控制绕组电流计算============================
_iq ia=0;
_iq ib=0;
_iq ic=0;
_iq ia_2=0;
_iq ib_2=0;
_iq ic_2=0;
_iq ialfa=0;
_iq ibeta=0;
_iq id=0;
_iq iq=0;

_iq ialfa_2=0;
_iq ibeta_2=0;
_iq id_2=0;
_iq iq_2=0;

//===============PI控制器参数计算===========================
_iq ID_Given=0;
_iq ID_Ref=0;
_iq ID_Fdb=0;
_iq ID_Error=0;

_iq ID_Up=0;
_iq ID_Up1=0;
_iq ID_Ui=0;
_iq ID_OutPreSat=0;
_iq ID_SatError=0;
_iq ID_OutMax=_IQ(1);
_iq ID_OutMin=_IQ(-1);
_iq ID_Out=0;

_iq IQ_Given=0;
_iq IQ_Ref=0;
_iq IQ_Fdb=0;
_iq IQ_Error=0;

_iq IQ_Up=0;
_iq IQ_Up1=0;
_iq IQ_Ui=0;
_iq IQ_OutPreSat=0;
_iq IQ_SatError=0;
_iq IQ_OutMax=_IQ(1);
_iq IQ_OutMin=_IQ(-1);
_iq IQ_Out=0;

_iq Speed_Given=_IQ(0.2); //速度给定    标幺值0.2==>600RPM，最高转速1.0==>3000RPM
_iq Speed_Ref=0;
_iq Speed_Fdb=0;
_iq Speed_Error=0;

_iq Speed_Up=0;
_iq Speed_Ui=0;
_iq Speed_OutPreSat=0;
_iq Speed_SatError=0;
_iq Speed_OutMax=_IQ(0.99999);
_iq Speed_OutMin=-_IQ(0.99999);
_iq Speed_Out=0;
Uint16 Speed_run=0;

_iq ID_2Given=0;
_iq ID_2Ref=0;
_iq ID_2Fdb=0;
_iq ID_2Error=0;

_iq ID_2Up=0;
_iq ID_2Ui=0;
_iq ID_2OutPreSat=0;
_iq ID_2SatError=0;
_iq ID_2OutMax=_IQ(1);
_iq ID_2OutMin=_IQ(-1);
_iq ID_2Out=0;

_iq IQ_2Given=0;
_iq IQ_2Ref=0;
_iq IQ_2Fdb=0;
_iq IQ_2Error=0;

_iq IQ_2Up=0;
_iq IQ_2Ui=0;
_iq IQ_2OutPreSat=0;
_iq IQ_2SatError=0;
_iq IQ_2OutMax=_IQ(1);
_iq IQ_2OutMin=_IQ(-1);
_iq IQ_2Out=0;

_iq Speed_2Given=_IQ(0.2); //速度给定    标幺值0.2==>600RPM，最高转速1.0==>3000RPM
_iq Speed_2Ref=0;
_iq Speed_2Fdb=0;
_iq Speed_2Error=0;

_iq Speed_2Up=0;
_iq Speed_2Ui=0;
_iq Speed_2OutPreSat=0;
_iq Speed_2SatError=0;
_iq Speed_2OutMax=_IQ(0.99999);
_iq Speed_2OutMin=-_IQ(0.99999);
_iq Speed_2Out=0;
Uint16 Speed_2run=0;

//===============SVPWM计算====================================
Uint16 Sector = 0;
_iq Ualfa=0;
_iq Ubeta=0;
_iq Ud=0;
_iq Uq=0;
_iq B0=0;
_iq B1=0;
_iq B2=0;
_iq X=0;
_iq Y=0;
_iq Z=0;
_iq t1=0;
_iq t2=0;
_iq Ta=0;
_iq Tb=0;
_iq Tc=0;
_iq MfuncD1=0;
_iq MfuncD2=0;
_iq MfuncD3=0;
Uint16 ZhengFan_2=1;
Uint16 ZhengFan=1;
Uint16 Sector_2 = 0;
_iq Ualfa_2=0;
_iq Ubeta_2=0;
_iq Ud_2=0;
_iq Uq_2=0;
_iq B0_2=0;
_iq B1_2=0;
_iq B2_2=0;
_iq X_2=0;
_iq Y_2=0;
_iq Z_2=0;
_iq t1_2=0;
_iq t2_2=0;
_iq Ta_2=0;
_iq Tb_2=0;
_iq Tc_2=0;
_iq MfuncD1_2=0;
_iq MfuncD2_2=0;
_iq MfuncD3_2=0;
//===================================================================
Uint16 Run_PMSM=2;
Uint16 Run_PMSM_2=2;
Uint16 speed_give=0;
Uint16 HallAngle=0;
Uint16 BuChang=416;
int16 TotalCnt=0;
Uint16 ShangDian_Err=0;

Uint16 speed_give_2=0;
Uint16 HallAngle_2=0;
Uint16 BuChang_2=416;
int16 TotalCnt_2=0;
Uint16 ShangDian_Err_2=0;
_iq BaseRpm=0;
_iq BaseRpm_2=0;
Uint16 KEY_Type=1;

//========================速度环PI参数=================================
_iq Speed_Kp=_IQ(8);
_iq Speed_Ki=_IQ(0.005);
//========================Q轴电流环PI参数==============================
_iq IQ_Kp=_IQ(0.3);
_iq IQ_Ki=_IQ(0.002);
//========================D轴电流环PI参数==============================
_iq ID_Kp=_IQ(0.3);
_iq ID_Ki=_IQ(0.002);

//轴2
//========================速度环PI参数=================================
_iq Speed_2Kp=_IQ(8);
_iq Speed_2Ki=_IQ(0.005);
//========================Q轴电流环PI参数==============================
_iq IQ_2Kp=_IQ(0.3);
_iq IQ_2Ki=_IQ(0.002);
//========================D轴电流环PI参数==============================
_iq ID_2Kp=_IQ(0.3);
_iq ID_2Ki=_IQ(0.002);

long PlaceSet=1000000;//位置环脉冲数
Uint16 PlaceEnable=0;//位置环使能 1 使能 ;  0 禁止

long PlaceSet_2=1000000;//位置环脉冲数
Uint16 PlaceEnable_2=0;//位置环使能 1 使能 ;  0 禁止

//=====================轴1电机 参数设置========================================
float32 E_Ding_DianLiu=4.2;        //设置电机的额定电流单位A
Uint16 BaseSpeed=3000;              //设置电机额定转速与BaseRpm相等

//=====================轴2电机 参数设置========================================
float32 E_Ding_DianLiu_2=4.2;        //设置电机的额定电流单位A
Uint16 BaseSpeed_2=3000;              //设置电机额定转速与BaseRpm相等

float32 E_Ding_DianLiu_Rated=4.2;
float32 E_Ding_DianLiu_2_Rated=4.2;

void main(void)
{
   InitSysCtrl();//禁用看门狗避免复位，设置cpu（外部晶振10/2得到cpu时钟）和外设时钟频率

   InitGpio();
   Pwm_EN_1;//已经宏定义过的：pwm禁止
   Pwm_EN2_1;
    InitXintf();//外部接口初始化


   DINT;//关中断

   InitPieCtrl(); //场景还原
   IER = 0x0000;//一次性关闭所有中断
   IFR = 0x0000;//清除所有挂起的中断标志

   InitPieVectTable();

   EALLOW;  // This is needed to write to EALLOW protected registers
  // PieVectTable.TINT0 = &cpu_timer0_isr;
   //中断入口
   PieVectTable.EPWM1_INT=&EPWM_1_INT;//设置PWM1中断服务程序
   PieVectTable.SCIRXINTB= &SCIBRX_ISR;   //设置串口B中断服务程序
   PieVectTable.XINT3=&INT3_ISR;
   EDIS;    // This is needed to disable write to EALLOW protected registers
 // InitCpuTimers();
   //InitSci_C();
   InitSci_B();              // 115200 初始化已确认TX正常)
   TX232_String("READY\n");  // 开机自报
   //InitSpi();
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);//将flash复制到RAM中执行
   InitFlash();
   InitEPwm_1_2_3();// 轴1 轴2 pwm初始化
   QEP_Init(); //轴1 轴2 qep 初始化
   Init_ch454();//数码管显示
   Init_SiShu();//四则运算
   eva_close(); //轴1停机
   eva_close_2();//轴2停机
    DELAY_US(1000000);//等待硬件稳定
      EALLOW;
     GpioDataRegs.GPBDAT.bit.GPIO49=0;//convst ad 产生下降沿
    asm("   NOP");//保持五个nop周期延迟，保证adc正确识别
    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
     GpioDataRegs.GPBDAT.bit.GPIO49=1;//convst ad//拉高完成启动信号
     EDIS;

    //启动信号完成 adc开始转换
    Ad_CaiJi(); //电流电压采集
    if(AD_BUF[5]<150)//轴1 上电安全检测（直流母线电压是否正常）
   {
    Pwm_EN_0;//允许PWM使能
    }
   else
   {
    Pwm_EN_1;//禁止PWM使能
    ShangDian_Err=1;//记录故障
   }
      if(AD_BUF[4]<150) //轴2
   {
    Pwm_EN2_0;//允许PWM使能
    }
   else
   {
    Pwm_EN2_1;//禁止PWM使能
    ShangDian_Err_2=1;
   }


   IER |= M_INT3;//使能对应的中断组
   IER |= M_INT9;
   IER |= M_INT12;
   //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//timer0
   //使能PIE组中的具体中断
   PieCtrlRegs.PIEIER3.bit.INTx1=1;//epwm1int   int3.1
   PieCtrlRegs.PIEIER9.bit.INTx3=1;//scib       int9.3
   PieCtrlRegs.PIEIER12.bit.INTx1=1;//xint3     int12.1
   Init_lcd();
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM




       for(;;)//循环，进入永不退出
     {
        HIL_Poll_Rx();           // 最先收数据，防FIFO溢出
        DC_Link();//直流母线电压检测
         Read_key();//读取按键
        deal_key();//处理按键
        LCD_DIS(); //LCD显示
        HIL_Poll_Rx();           // 再收一次，兜底
        HIL_Evaluation_Task();
      }




}
   interrupt void EPWM_1_INT(void)
   {
          _iq t_01,t_02,t_01_2,t_02_2;//临时变量
              EALLOW;
              GpioDataRegs.GPBDAT.bit.GPIO49=0;//convst ad
    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
     GpioDataRegs.GPBDAT.bit.GPIO49=1;//convst ad
     EDIS;
       IPM_BaoHu();
    if(Eval_State == 2 || Eval_State == 22)
    {
        if(IPM_Fault || Hall_Fault)
        {
            HIL_FailCode = 1;
        }
    }
    Show_time2++;
       if(Show_time2==1000)//1秒
    {
        Show_time2=0;
        lcd_dis_flag=1;//触发显示更新
    }
    //void Lubo(_iq Ch1,_iq Ch2,_iq Ch3,_iq Ch4)//参数都是标幺值Q15格式,知道怎么用了
   //----ch1--ch2-ch3-ch4 4个通道
     Lubo(ia, ib, ic,Speed);
  Ad_CaiJi();
  JiSuan_Dl();



if(Run_PMSM==1&&IPM_Fault==0)//初始位置定位算法
{
    if(LocationFlag!=LocationEnd)//尚未完成初始位置定位    flag初始化 END结束0
    { Get_HallAngle();
            Modulation=ALIGN_MODULATION;//设置调制比强制定位到最近的扇区中心
           Get_HallAngle();//获得霍尔传感器状态
        switch(HallAngle)
            {
                case 5:
                    Position=PositionPhase60;//设置60度位置
                    LocationFlag=LocationEnd;//定位结束
                EQep1Regs.QPOSCNT =BuChang*0+BuChang/2;  //QEP计数器设置为 起始位置+区间的一半
                 OldRawTheta=_IQ(EQep1Regs.QPOSCNT);//保存为角度计算的基准

                break;
                case 1:
                    Position=PositionPhase360;
                     LocationFlag=LocationEnd;//定位结束
                EQep1Regs.QPOSCNT =BuChang*5+BuChang/2;
               OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                break;
                case 3:
                    Position=PositionPhase300;
                     LocationFlag=LocationEnd;//定位结束
                EQep1Regs.QPOSCNT =BuChang*4+BuChang/2;
                 OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                break;
                case 2:
                    Position=PositionPhase240;
                     LocationFlag=LocationEnd;//定位结束
                EQep1Regs.QPOSCNT =BuChang*3+BuChang/2;
                  OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                break;
                case 6:
                    Position=PositionPhase180;
                     LocationFlag=LocationEnd;//定位结束
                EQep1Regs.QPOSCNT =BuChang*2+BuChang/2;
                  OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                break;
                case 4:
                    Position=PositionPhase120;
                     LocationFlag=LocationEnd;//定位结束
                    EQep1Regs.QPOSCNT =BuChang*1+BuChang/2;
                     OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                break;
                default:
                     DC_ON_1;
                    Run_PMSM=2;//霍尔信号错误启动停止
                    eva_close();
                    Hall_Fault=1;
                    break;
            }
    }
//=====================================================================================================
//初始位置定位结束，开始闭环控制
//=====================================================================================================
    else if(LocationFlag==LocationEnd)
    {
        Modulation = RUN_MODULATION;
//======================================================================================================
//QEP角度计算
//======================================================================================================

// 旋转方向判定
        DirectionQep = EQep1Regs.QEPSTS.bit.QDF;//编码器方向    1=顺时针；0=逆时针

        RawTheta = _IQ(EQep1Regs.QPOSCNT);//读取编码器的当前位置计数

        if(DirectionQep ==1) //递增计数，代表顺时针
        {

         //编码器正向溢出检测 上一周期的编码器原始计数值 当前编码器的原始计数值
              if((OldRawThetaPos> 324403200) && (RawTheta<_IQ(900)))//从最大值回到零的情况
            {
                PosCount += TotalCnt;//增加一个总计数
            }
            Place_now= _IQtoF(RawTheta)+PosCount;//绝对位置=编码器的原始位置＋累计计数
            OldRawThetaPos = RawTheta;//
        }
        else if(DirectionQep ==0)//递减计数，代表逆时针
        {
              if((RawTheta> 294912000) && (OldRawThetaPos<_IQ(1000)))
            {
                PosCount -= TotalCnt;//减少一个总计数
            }
            Place_now = _IQtoF(RawTheta)+PosCount;
            OldRawThetaPos = RawTheta;
        }
        MechTheta = _IQmpy(1179,RawTheta);//编码器原始计数值转机械角度  1179转换系数
         if(MechTheta>_IQ(360))//标幺化处理
        {MechTheta=MechTheta-_IQ(360);}
         if(MechTheta<_IQ(-360))
        {MechTheta=MechTheta+_IQ(360);}
        ElecTheta = _IQmpy(_IQ(4),MechTheta);   //机械角度转电角度 _IQmpy×的意义

        AnglePU=_IQdiv(ElecTheta,_IQ(360))+14876;//角度标幺值（归一化的值）为后期准备的值，IQDIV相除的意思，前后
        Sine = _IQsinPU(AnglePU);//计算值用于park变换
        Cosine = _IQcosPU(AnglePU);

//======================================================================================================
//QEP速度计算 (已补全：移植自轴2脉冲差计算逻辑)
//======================================================================================================
//速度计算器每中断加一，到达速度计算预分频值才开始计算速度
                if (SpeedLoopCount >= SpeedLoopPrescaler)
                {
                    // ================= 1. 找回丢失的速度反馈计算（至关重要！=================
                    DirectionQep = EQep1Regs.QEPSTS.bit.QDF; // 读取旋转方向
                    NewRawTheta = _IQ(EQep1Regs.QPOSCNT);    // 读取当前编码器位置

                    if(DirectionQep == 1) // 顺时针
                    {
                        RawThetaTmp = OldRawTheta - NewRawTheta;
                        if(RawThetaTmp > _IQ(0))
                        {
                            RawThetaTmp = RawThetaTmp - TotalPulse;
                        }
                    }
                    else if(DirectionQep == 0) // 逆时针
                    {
                        RawThetaTmp = OldRawTheta - NewRawTheta;
                        if(RawThetaTmp < _IQ(0))
                        {
                            RawThetaTmp = RawThetaTmp + TotalPulse;
                        }
                    }

                    // 计算出真实的反馈速度标幺值(一定要有这一步，滑模才能闭环)
                    Speed = _IQmpy(RawThetaTmp, 65);
                    OldRawTheta = NewRawTheta;
                    // ==================================================================
                    // ================= 2. 预定义时间复合滑模速度控制 =================
                    // 确定速度环周期 T_s（秒）
                    float32 T_s = (float32)SpeedLoopPrescaler / 10000.0f;

                    // --- 量纲转换模块 ---
                    float32 actual_speed_rpm = _IQtoF(Speed) * (float32)BaseSpeed; // 现在 Speed 有真实数据了
                    if(Eval_State == 2 && HIL_StepMode != 0)
                    {
                        SpeedRef = HIL_StepSpeedRef(Record_Index);
                    }
                    float32 target_speed_rpm = SpeedRef * (float32)BaseSpeed;
                    float32 speed_abs_rpm;
                    float32 speed_limit_rpm = 1.2f * (float32)BaseSpeed;

                    if(actual_speed_rpm > speed_limit_rpm || actual_speed_rpm < -speed_limit_rpm)
                    {
                        actual_speed_rpm = HIL_LastValidSpeed;
                    }
                    else
                    {
                        HIL_LastValidSpeed = actual_speed_rpm;
                    }

                    speed_abs_rpm = actual_speed_rpm;
                    if(speed_abs_rpm < 0.0f) speed_abs_rpm = -speed_abs_rpm;

                    // 转速(RPM) 转角速度(rad/s)
                    float32 omega_fdb = actual_speed_rpm * 0.104719755f;
                    float32 omega_ref = target_speed_rpm * 0.104719755f;

                    float32 e_spd = omega_ref - omega_fdb; // 真实的角速度误差


// =========================================================================
// HIL 在线评估数据录制 (仅限记录，不发串口
// =========================================================================
                    if(Eval_State == 2)
                      {
                      if(speed_abs_rpm >= HIL_MOTION_RPM)
                        {
                        HIL_MotionSeen = 1;
                        }

                      if(Record_Index < HIL_STARTUP_CHECK_TICKS &&
                         HIL_StartupCheckTicks < HIL_STARTUP_CHECK_TICKS)
                        {
                        if(speed_abs_rpm >= HIL_RECORD_MIN_RPM)
                          {
                          HIL_StartupCheckTicks = HIL_STARTUP_CHECK_TICKS;
                          }
                        else
                          {
                          HIL_StartupCheckTicks++;
                          if(HIL_StartupCheckTicks >= HIL_STARTUP_CHECK_TICKS)
                            {
                            HIL_StartupCheckTicks = HIL_STARTUP_CHECK_TICKS;
                            }
                          }
                        }

                      if(HIL_FailCode == 0)
                        {
                        if(Record_Index < HIL_RecordLength)
                          {
                          Log_Speed_x10[Record_Index] = HIL_SpeedRpmToX10(actual_speed_rpm);
                          Record_Index++;
                          }
                         else
                          {
                          Eval_State = 3;
                          }
                        }
                       }
                    else if(Eval_State == 31)
                      {
                      if(HIL_FailCode == 0)
                        {
                        if(Record_Index < HIL_RecordLength)
                          {
                          Log_Speed_x10[Record_Index] = HIL_SpeedRpmToX10(actual_speed_rpm);
                          Log_Speed_2_x10[Record_Index] = HIL_SpeedRpmToX10(Log_Spd2);
                          Record_Index++;
                          }
                         else
                          {
                          Eval_State = 33;
                          }
                        }
                       }
// =========================================================================
//预定义时间干扰观测器 (PTDO)
// =========================================================================
                    if (Run_PMSM == 1 && IPM_Fault == 0)
                    {
                        // 【核心修复1：输入端抗噪滤波】对原始速度进行极其轻微的滤波，抹平离散量化阶跃
                        static float32 omega_fdb_fil = 0.0f;
                        omega_fdb_fil = 0.85f * omega_fdb_fil + 0.15f * omega_fdb;

                        // 计算观测误差
                        float32 sigma = omega_fdb_fil - omega_hat;

                        // 暴力切断大动态尖峰（保持原有的安全冗余）
                        if (sigma > 30.0f) sigma = 30.0f;
                        if (sigma < -30.0f) sigma = -30.0f;

                        float32 abs_sigma = fabsf(sigma);
                        float32 sign_sigma = (sigma > 0.0f) ? 1.0f : ((sigma < 0.0f) ? -1.0f : 0.0f);

                        // 【核心修复2：彻底重构边界层，消除零点附近非线性项激发的剧烈抽动。
                        if (abs_sigma >= PHI_SIGMA)
                        {
                            // 外部非线性区
                            float32 term_d1 = C6 * pow(abs_sigma, 1.0f - P3);
                            float32 term_d2 = C7 * pow(abs_sigma, 1.0f + P3);

                            // 标准非线性切换项 + 积分项
                            integral_sigma += C8 * sign_sigma * T_s;
                            d_hat = (term_d1 + term_d2) * sign_sigma + integral_sigma;
                        }
                        else
                        {
                            // 内部线性区：误差小于 PHI_SIGMA 时，强制转为线性增益，使零点导数平滑，彻底消除抖振振荡
                            float32 d_hat_boundary = C6 * pow(PHI_SIGMA, 1.0f - P3)
                                                   + C7 * pow(PHI_SIGMA, 1.0f + P3);

                            // 边界层内线性插值+ 积分项软化
                            integral_sigma += C8 * (sigma / PHI_SIGMA) * T_s;
                            d_hat = d_hat_boundary * (sigma / PHI_SIGMA) + integral_sigma;
                        }

                        // 积分器抗饱和限幅
                        if(integral_sigma > 2000.0f) integral_sigma = 2000.0f;
                        if(integral_sigma < -2000.0f) integral_sigma = -2000.0f;

                        // 观测器输出绝对限幅（你的两道防飞车防火墙非常完美，予以保留）
                        if(d_hat > 5000.0f) d_hat = 5000.0f;
                        if(d_hat < -5000.0f) d_hat = -5000.0f;

                        // 一阶低通滤波平滑输出
                        d_hat_filtered = 0.95f * d_hat_filtered + 0.05f * d_hat;

                        // 3. 更新估计转速（使用干净的理想指令 IQ_Given，保留你的优秀纠错逻辑）
                        float32 iq_cmd_amps = _IQtoF(IQ_Given) * E_Ding_DianLiu;
                        float32 omega_hat_dot = Kt * iq_cmd_amps - B_J * omega_fdb_fil + d_hat; // 采用滤波后的速度作状态反馈
                        omega_hat += omega_hat_dot * T_s;
                    }
                    else
                    {
                        // 停机复位
                        omega_hat = omega_fdb;
                        integral_sigma = 0.0f;
                        d_hat = 0.0f;
                        d_hat_filtered = 0.0f;
                    }
// =========================================================================                                        // =========================================================================                            // =========================================================================
            // 3. 计算 f1(e) = c1*sign(e)|e|^{1-p1} + c2*sign(e)|e|^{1+p1} + c3*sign(e)
            float32 abs_e = fabsf(e_spd);
            float32 sign_e = (e_spd > 0.0f) ? 1.0f : ((e_spd < 0.0f) ? -1.0f : 0.0f);
            float32 sat_e = (abs_e < PHI_E) ? (e_spd / PHI_E) : sign_e;   // 饱和函数抑制抖振

            float32 term1 = C1 * sat_e * pow(abs_e, 1.0 - P1);
            float32 term2 = C2 * sat_e * pow(abs_e, 1.0 + P1);
            float32 term3 = C3 * sat_e;

            float32 f1 = term1 + term2 + term3;

            // 4. 更新滑模面 s = e + ∫f1 dt
            if (Run_PMSM == 1 && IPM_Fault == 0) // 仅在启动状态下积分
            {
                Integral_f1 += f1 * T_s;
            }
            else
            {
                Integral_f1 = 0.0f; // 停机时强行清零，防止噪声累积
            }

            // 抗积分饱和限制(建议根据实际需要的最大角加速度重新评估这个 10.0f 的阈值
            if (Integral_f1 > 50.0f) Integral_f1 = 50.0f;
            if (Integral_f1 < -50.0f) Integral_f1 = -50.0f;

            float32 S_smc = e_spd + Integral_f1;

            // 5. 计算 f2(s) = (c4|s|^{1-p2} + c5|s|^{1+p2}) * sign(s)
            float32 abs_s = fabs(S_smc);
            float32 sign_s = (S_smc > 0.0f) ? 1.0f : ((S_smc < 0.0f) ? -1.0f : 0.0f);
            float32 sat_s = (abs_s < PHI_S) ? (S_smc / PHI_S) : sign_s;
            float32 f2 = (C4 * pow(abs_s, 1.0 - P2) + C5 * pow(abs_s, 1.0 + P2)) * sat_s;

            // 6. 参考加速度前馈（可选）
            float32 omega_dot_ref = 0.0f;
            #if USE_FEEDFORWARD
                omega_dot_ref = (SpeedRef - SpeedRef_prev) / T_s;
                SpeedRef_prev = SpeedRef;
            #endif

            // 7. 控制律合成
            // iqref = 1/Kt * [omega_dot_ref + f1(e) + f2(s) - d_hat + B/J*omega_m]
                float32 iq_ref_raw = omega_dot_ref + f1 + f2 - d_hat_filtered + B_J * omega_fdb;
                float32 iq_ref_amps = KT_INV * iq_ref_raw; // 这里算出的是真实物理电流 (A)

                // --- 新增：安培转标幺值---
                // E_Ding_DianLiu 是 1.414 * 额定电流（峰值），代表标幺值的 1.0
                float32 iq_ref_pu = iq_ref_amps / E_Ding_DianLiu;
                // --------------------------

             // 8. 限幅输出
                if (iq_ref_pu > _IQtoF(Speed_OutMax)) iq_ref_pu = _IQtoF(Speed_OutMax);
                if (iq_ref_pu < _IQtoF(Speed_OutMin)) iq_ref_pu = _IQtoF(Speed_OutMin);

                if(HIL_ForceIq != 0)
                {
                    Integral_f1 = 0.0f;
                    omega_hat = omega_fdb;
                    integral_sigma = 0.0f;
                    d_hat = 0.0f;
                    d_hat_filtered = 0.0f;
                    IQ_Given = _IQ(HIL_ForceIqPu);
                    ID_Given = 0;
                }
                else
                {
                    IQ_Given = _IQ(iq_ref_pu);
                }

              // 9. 重置速度环计数器
            SpeedLoopCount = 1;
            RawThetaTmp = 0;
        }
        else
        {
            SpeedLoopCount++;
        }
        Speed_run = 1;

//======================================================================================================
//IQ电流PID调节控制
//======================================================================================================
        IQ_Ref=IQ_Given;
        IQ_Fdb=iq;

        IQ_Error=IQ_Ref-IQ_Fdb;

        IQ_Up=_IQmpy(IQ_Kp,IQ_Error);//PI控制器
        IQ_Ui=IQ_Ui + _IQmpy(IQ_Ki,IQ_Up) + _IQmpy(IQ_Ki,IQ_SatError);

        IQ_OutPreSat=IQ_Up+IQ_Ui;//限幅前的输出

        if(IQ_OutPreSat>IQ_OutMax)//限幅
            IQ_Out=IQ_OutMax;
        else if(IQ_OutPreSat<IQ_OutMin)
            IQ_Out=IQ_OutMin;
        else
            IQ_Out=IQ_OutPreSat;

        IQ_SatError=IQ_Out-IQ_OutPreSat;  //饱和误差（用于积分反馈）

        Uq=IQ_Out;//Q轴电压输出

//======================================================================================================
//ID电流PID调节控制
//======================================================================================================
        ID_Ref=ID_Given;
        ID_Fdb=id;

        ID_Error=ID_Ref-ID_Fdb;

        ID_Up=_IQmpy(ID_Kp,ID_Error);  //PI控制器
        ID_Ui=ID_Ui+_IQmpy(ID_Ki,ID_Up)+_IQmpy(ID_Ki,ID_SatError);

        ID_OutPreSat=ID_Up+ID_Ui;

        if(ID_OutPreSat>ID_OutMax)   //限幅
            ID_Out=ID_OutMax;
        else if(ID_OutPreSat<ID_OutMin)
            ID_Out=ID_OutMin;
        else
            ID_Out=ID_OutPreSat;

        ID_SatError=ID_Out-ID_OutPreSat;   //饱和误差（用于积分反馈）

        Ud=ID_Out;//D轴电压输出

//======================================================================================================
//IPark变换
//======================================================================================================
        Ualfa = _IQmpy(Ud,Cosine) - _IQmpy(Uq,Sine);
        Ubeta = _IQmpy(Uq,Cosine) + _IQmpy(Ud,Sine);

//======================================================================================================
//SVPWM实现
//======================================================================================================
        B0=Ubeta;//计算三个参考电压
        B1=_IQmpy(_IQ(0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2
        B2=_IQmpy(_IQ(-0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2


        Sector=0;//计算扇区编码
        if(B0>_IQ(0)) Sector =1;
        if(B1>_IQ(0)) Sector =Sector +2;
        if(B2>_IQ(0)) Sector =Sector +4;

        X=Ubeta;//va  电压在三个轴上的投影  αβ转三相坐标系
        Y=_IQmpy(_IQ(0.8660254),Ualfa)+ _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2 vb
        Z=_IQmpy(_IQ(-0.8660254),Ualfa)+ _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2 vc


     if(Sector==1)
        {
            t_01=Z;//作用时间
            t_02=Y;

       if((t_01+t_02)>_IQ(1))//过调制处理，按比例缩放
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));//矢量方向不变  幅度缩小到六边形边界
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }
                 //计算三相占空比
            Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));// V
            Ta=Tb+t1;//U
            Tc=Ta+t2;//W
        }
        else if(Sector==2)
        {
            t_01=Y;
            t_02=-X;

             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

            Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Tc=Ta+t1;
            Tb=Tc+t2;
        }
        else if(Sector==3)
        {
            t_01=-Z;
            t_02=X;

             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

            Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Tb=Ta+t1;
            Tc=Tb+t2;
        }
        else if(Sector==4)
        {
            t_01=-X;
            t_02=Z;
             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

            Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Tb=Tc+t1;
            Ta=Tb+t2;
        }
        else if(Sector==5)
        {
            t_01=X;
            t_02=-Y;
             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

            Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Tc=Tb+t1;
            Ta=Tc+t2;
        }
        else if(Sector==6)
        {
            t_01=-Y;
            t_02=-Z;
             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

            Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Ta=Tc+t1;
            Tb=Ta+t2;
        }

        MfuncD1=_IQmpy(_IQ(2),(_IQ(0.5)-Ta));//生成对应扇区的UVW的PWM占空比
        MfuncD2=_IQmpy(_IQ(2),(_IQ(0.5)-Tb));
        MfuncD3=_IQmpy(_IQ(2),(_IQ(0.5)-Tc));
//======================================================================================================
//EVA全比较器参数赋值，用于驱动电机
//======================================================================================================
        //SVPWM结果转换为实际PWM寄存器值
        MPeriod = (int16)(T1Period * Modulation);              // Q0 = (Q0 * Q0)

    Tmp = (int32)MPeriod * (int32)MfuncD1;                    // Q15 = Q0*Q15，计算全比较器CMPR1赋值
     EPwm1Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    Tmp = (int32)MPeriod * (int32)MfuncD2;                    // Q15 = Q0*Q15，计算全比较器CMPR2赋值
     EPwm2Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    Tmp = (int32)MPeriod * (int32)MfuncD3;                    // Q15 = Q0*Q15，计算全比较器CMPR3赋值
     EPwm3Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)
     //右移几位就是除以2几次方

    }
    }

//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

if(Run_PMSM_2==1&&IPM_Fault_2==0)//轴2
{
    if(LocationFlag_2!=LocationEnd_2)
    {
            Modulation_2=0.95;
            Get_HallAngle();
        switch(HallAngle_2)
            {
                case 5:
                    Position_2=PositionPhase60;
                    LocationFlag_2=LocationEnd_2;//定位结束
                EQep2Regs.QPOSCNT =BuChang_2*0+BuChang_2/2;
                 OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);

                break;
                case 1:
                    Position_2=PositionPhase360;
                     LocationFlag_2=LocationEnd_2;//定位结束
                EQep2Regs.QPOSCNT =BuChang_2*5+BuChang_2/2;
               OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
                break;
                case 3:
                    Position_2=PositionPhase300;
                     LocationFlag_2=LocationEnd_2;//定位结束
                EQep2Regs.QPOSCNT =BuChang_2*4+BuChang_2/2;
                 OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
                break;
                case 2:
                    Position_2=PositionPhase240;
                     LocationFlag_2=LocationEnd_2;//定位结束
                EQep2Regs.QPOSCNT =BuChang_2*3+BuChang_2/2;
                  OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
                break;
                case 6:
                    Position_2=PositionPhase180;
                     LocationFlag_2=LocationEnd_2;//定位结束
                EQep2Regs.QPOSCNT =BuChang_2*2+BuChang_2/2;
                  OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
                break;
                case 4:
                    Position_2=PositionPhase120;
                     LocationFlag_2=LocationEnd_2;//定位结束
                    EQep2Regs.QPOSCNT =BuChang_2*1+BuChang_2/2;
                     OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
                break;
                default:
                     DC_ON2_1;
                    Run_PMSM_2=2;//霍尔信号错误启动停止
                    eva_close_2();
                    Hall_Fault_2=1;
                    break;
            }



    }

        else if(LocationFlag_2==LocationEnd_2)
    {


//======================================================================================================
//QEP角度计算
//======================================================================================================

// 旋转方向判定
        DirectionQep_2 = EQep2Regs.QEPSTS.bit.QDF;

        RawTheta_2 = _IQ(EQep2Regs.QPOSCNT);

        if(DirectionQep_2 ==1) //递增计数，代表顺时针
        {


              if((OldRawThetaPos_2> 324403200) && (RawTheta_2<_IQ(900)))
            {
                PosCount_2 += TotalCnt_2;
            }


            Place_now_2= _IQtoF(RawTheta_2)+PosCount_2;
            OldRawThetaPos_2 = RawTheta_2;




        }
        else if(DirectionQep_2 ==0)//递减计数，代表逆时针
        {


              if((RawTheta_2> 294912000) && (OldRawThetaPos_2<_IQ(1000)))
            {
                PosCount_2 -= TotalCnt_2;
            }
            Place_now_2 = _IQtoF(RawTheta_2)+PosCount_2;
            OldRawThetaPos_2 = RawTheta_2;




        }
        MechTheta_2 = _IQmpy(1179,RawTheta_2);
         if(MechTheta_2>_IQ(360))
        {MechTheta_2=MechTheta_2-_IQ(360);}
         if(MechTheta_2<_IQ(-360))
        {MechTheta_2=MechTheta_2+_IQ(360);}
        ElecTheta_2 = _IQmpy(_IQ(4),MechTheta_2);

        AnglePU_2=_IQdiv(ElecTheta_2,_IQ(360))+14876;
        Sine_2 = _IQsinPU(AnglePU_2);
        Cosine_2 = _IQcosPU(AnglePU_2);
//======================================================================================================
//QEP速度计算
//======================================================================================================

        if (SpeedLoopCount_2>=SpeedLoopPrescaler_2)
        {
// 旋转方向判定
            DirectionQep_2 = EQep2Regs.QEPSTS.bit.QDF;
            NewRawTheta_2 =_IQ(EQep2Regs.QPOSCNT);
// 计算机械角度
            if(DirectionQep_2 ==1) //递增计数
            {


                RawThetaTmp_2 =  OldRawTheta_2-NewRawTheta_2 ;
                if(RawThetaTmp_2 > _IQ(0))
                {
                 RawThetaTmp_2 = RawThetaTmp_2 - TotalPulse_2;
                }



            }
            else if(DirectionQep_2 ==0) //递减计数
            {



                RawThetaTmp_2 =OldRawTheta_2-NewRawTheta_2;
                if(RawThetaTmp_2 < _IQ(0))
                {
                 RawThetaTmp_2 = RawThetaTmp_2 + TotalPulse_2;
                }




            }
            Speed_2 = _IQmpy(RawThetaTmp_2,65);
            SpeedRpm_2 = _IQmpy(BaseRpm_2,Speed_2);
            OldRawTheta_2 = NewRawTheta_2;
               if(Speed_2<0)
            {Speed_dis_2=_IQtoF(_IQmpy(Speed_2, _IQ(-100)));}
            else{
            Speed_dis_2=_IQtoF(_IQmpy(Speed_2, _IQ(100)));}

            if(Eval_State == 22)
            {
                float32 axis2_speed_rpm = _IQtoF(Speed_2) * (float32)BaseSpeed_2;
                float32 axis2_delta_rpm = axis2_speed_rpm - HIL_LastValidSpeed_2;
if(axis2_delta_rpm < 0.0f) axis2_delta_rpm = -axis2_delta_rpm;
                if(axis2_speed_rpm > HIL_SPEED2_MAX_ABS_RPM ||
                   axis2_speed_rpm < -HIL_SPEED2_MAX_ABS_RPM ||
                   axis2_delta_rpm > HIL_SPEED2_MAX_STEP_RPM)
                {
                    axis2_speed_rpm = HIL_LastValidSpeed_2;
                    HIL_Speed2SpikeCount++;
                    if(HIL_Speed2SpikeCount >= HIL_SPEED2_MAX_SPIKES)
                    {
                        HIL_FailCode = 4;
                    }
                }
                else
                {
                    HIL_LastValidSpeed_2 = axis2_speed_rpm;
                    HIL_Speed2SpikeCount = 0;
                }
                Speed_2 = _IQ(axis2_speed_rpm / (float32)BaseSpeed_2);

                if(HIL_FailCode == 0)
                {
                    if(Record_Index < HIL_RecordLength)
                    {
                        Log_Speed_x10[Record_Index] = HIL_SpeedRpmToX10(axis2_speed_rpm);
                        Record_Index++;
                    }
                    else
                    {
                        Eval_State = 23;
                    }
                }
            }

            SpeedLoopCount_2=1;
            RawThetaTmp_2=0;


//=================位置环控制==================================
  if(PlaceEnable_2 ==1)
    {
        PlaceError_2 = PlaceSet_2 + Place_now_2;

        OutPreSat_Place_2 = PlaceError_2;
        if((PlaceError_2<=10000)&&(PlaceError_2>=-10000))
        {
           OutPreSat_Place_2 = PlaceError_2/3;
        }


        if (OutPreSat_Place_2> 2000)
        {
          SpeedRef_2 =  0.5;
        }
        else if (OutPreSat_Place_2< -2000)
        {
          SpeedRef_2 =  -0.5;
        }
        else
        {
          SpeedRef_2 = OutPreSat_Place_2/(float32)BaseSpeed_2;
        }


    }
  //=================速度环PI===================================
            if(Eval_State == 22 && HIL_StepMode != 0)
            {
                SpeedRef_2 = HIL_StepSpeedRef(Record_Index);
            }
            if(Eval_State == 31 || Eval_State == 22)
            {
                Speed_2Ref=_IQ(SpeedRef_2);
            }
            else
            {
                Speed_2Ref=_IQ(SpeedRef);
            }
            Speed_2Fdb=Speed_2;

            Speed_2Error=Speed_2Ref - Speed_2Fdb;

            Speed_2Up=_IQmpy(Speed_2Kp,Speed_2Error);
            Speed_2Ui=Speed_2Ui + _IQmpy(Speed_2Ki,Speed_2Up) + _IQmpy(Speed_2Ki,Speed_2SatError);

            Speed_2OutPreSat=Speed_2Up+Speed_2Ui;

            if(Speed_2OutPreSat>Speed_2OutMax)
                Speed_2Out=Speed_2OutMax;
            else if(Speed_2OutPreSat<Speed_2OutMin)
                Speed_2Out=Speed_2OutMin;
            else
                Speed_2Out=Speed_2OutPreSat;

            Speed_2SatError=Speed_2Out-Speed_2OutPreSat;

            IQ_2Given=Speed_2Out;
            Speed_2run=1;
        }
        else
            {
                SpeedLoopCount_2++;
        }
         if(Speed_2run==1)
        {


            ialfa_2=ia_2;
        ibeta_2=_IQmpy(ia_2,_IQ(0.57735026918963))+_IQmpy(ib_2,_IQ(1.15470053837926));

        id_2 = _IQmpy(ialfa_2,Cosine_2) +_IQmpy(ibeta_2,Sine_2);
        iq_2 = _IQmpy(ibeta_2,Cosine_2)- _IQmpy(ialfa_2,Sine_2) ;

//======================================================================================================
//IQ电流PID调节控制
//======================================================================================================
        IQ_2Ref=IQ_2Given;
        IQ_2Fdb=iq_2;

        IQ_2Error=IQ_2Ref-IQ_2Fdb;

        IQ_2Up=_IQmpy(IQ_2Kp,IQ_2Error);
        IQ_2Ui=IQ_2Ui + _IQmpy(IQ_2Ki,IQ_2Up) + _IQmpy(IQ_2Ki,IQ_2SatError);

        IQ_2OutPreSat=IQ_2Up+IQ_2Ui;

        if(IQ_2OutPreSat>IQ_2OutMax)
            IQ_2Out=IQ_2OutMax;
        else if(IQ_2OutPreSat<IQ_2OutMin)
            IQ_2Out=IQ_2OutMin;
        else
            IQ_2Out=IQ_2OutPreSat;

        IQ_2SatError=IQ_2Out-IQ_2OutPreSat;

        Uq_2=IQ_2Out;

//======================================================================================================
//ID电流PID调节控制
//======================================================================================================
        ID_2Ref=ID_2Given;
        ID_2Fdb=id_2;

        ID_2Error=ID_2Ref-ID_2Fdb;

        ID_2Up=_IQmpy(ID_2Kp,ID_2Error);
        ID_2Ui=ID_2Ui+_IQmpy(ID_2Ki,ID_2Up)+_IQmpy(ID_2Ki,ID_2SatError);

        ID_2OutPreSat=ID_2Up+ID_2Ui;

        if(ID_2OutPreSat>ID_2OutMax)
            ID_2Out=ID_2OutMax;
        else if(ID_2OutPreSat<ID_2OutMin)
            ID_2Out=ID_2OutMin;
        else
            ID_2Out=ID_2OutPreSat;

        ID_2SatError=ID_2Out-ID_2OutPreSat;

        Ud_2=ID_2Out;

//======================================================================================================
//IPark变换
//======================================================================================================
        Ualfa_2 = _IQmpy(Ud_2,Cosine_2) - _IQmpy(Uq_2,Sine_2);
        Ubeta_2 = _IQmpy(Uq_2,Cosine_2) + _IQmpy(Ud_2,Sine_2);
//======================================================================================================
//SVPWM实现
//======================================================================================================
        B0_2=Ubeta_2;
        B1_2=_IQmpy(_IQ(0.8660254),Ualfa_2)- _IQmpy(_IQ(0.5),Ubeta_2);// 0.8660254 = sqrt(3)/2
        B2_2=_IQmpy(_IQ(-0.8660254),Ualfa_2)- _IQmpy(_IQ(0.5),Ubeta_2); // 0.8660254 = sqrt(3)/2

        Sector_2=0;
        if(B0_2>_IQ(0)) Sector_2 =1;
        if(B1_2>_IQ(0)) Sector_2 =Sector_2 +2;
        if(B2_2>_IQ(0)) Sector_2 =Sector_2 +4;

        X_2=Ubeta_2;//va
        Y_2=_IQmpy(_IQ(0.8660254),Ualfa_2)+ _IQmpy(_IQ(0.5),Ubeta_2);// 0.8660254 = sqrt(3)/2 vb
        Z_2=_IQmpy(_IQ(-0.8660254),Ualfa_2)+ _IQmpy(_IQ(0.5),Ubeta_2); // 0.8660254 = sqrt(3)/2 vc


     if(Sector_2==1)
        {
            t_01_2=Z_2;
            t_02_2=Y_2;

       if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

            Tb_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
            Ta_2=Tb_2+t1_2;
            Tc_2=Ta_2+t2_2;
        }
        else if(Sector_2==2)
        {
            t_01_2=Y_2;
            t_02_2=-X_2;

             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

            Ta_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
            Tc_2=Ta_2+t1_2;
            Tb_2=Tc_2+t2_2;
        }
        else if(Sector_2==3)
        {
            t_01_2=-Z_2;
            t_02_2=X_2;

             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

            Ta_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
            Tb_2=Ta_2+t1_2;
            Tc_2=Tb_2+t2_2;
        }
        else if(Sector_2==4)
        {
            t_01_2=-X_2;
            t_02_2=Z_2;
             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

            Tc_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
            Tb_2=Tc_2+t1_2;
            Ta_2=Tb_2+t2_2;
        }
        else if(Sector_2==5)
        {
            t_01_2=X_2;
            t_02_2=-Y_2;
             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

            Tb_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
            Tc_2=Tb_2+t1_2;
            Ta_2=Tc_2+t2_2;
        }
        else if(Sector_2==6)
        {
            t_01_2=-Y_2;
            t_02_2=-Z_2;
             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

            Tc_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
            Ta_2=Tc_2+t1_2;
            Tb_2=Ta_2+t2_2;
        }

        MfuncD1_2=_IQmpy(_IQ(2),(_IQ(0.5)-Ta_2));
        MfuncD2_2=_IQmpy(_IQ(2),(_IQ(0.5)-Tb_2));
        MfuncD3_2=_IQmpy(_IQ(2),(_IQ(0.5)-Tc_2));
//======================================================================================================
//EVA全比较器参数赋值，用于驱动电机
//======================================================================================================
    MPeriod_2 = (int16)(T1Period_2 * Modulation_2);              // Q0 = (Q0 * Q0)

    Tmp_2 = (int32)MPeriod_2 * (int32)MfuncD1_2;                    // Q15 = Q0*Q15，计算全比较器CMPR1赋值
     EPwm4Regs.CMPA.half.CMPA = (int16)(Tmp_2>>16) + (int16)(T1Period_2>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    Tmp_2 = (int32)MPeriod_2 * (int32)MfuncD2_2;                    // Q15 = Q0*Q15，计算全比较器CMPR2赋值
     EPwm5Regs.CMPA.half.CMPA = (int16)(Tmp_2>>16) + (int16)(T1Period_2>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    Tmp_2 = (int32)MPeriod_2 * (int32)MfuncD3_2;                    // Q15 = Q0*Q15，计算全比较器CMPR3赋值
     EPwm6Regs.CMPA.half.CMPA = (int16)(Tmp_2>>16) + (int16)(T1Period_2>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)


    }
    }

}

if(DC_ON_flag==1)
{

        if(U_dc_dis<10)//执行停机命令
        {
        eva_close();
        Run_PMSM=2;
        DC_ON_flag=0;

        }
}


if(DC_ON_flag_2==1)
{
        if(U_dc_dis_2<10)//执行停机命令
        {
        eva_close_2();
        Run_PMSM_2=2;
        DC_ON_flag_2=0;

        }
}

// 这一处是完美的！它位于所有算法执行完毕、退出中断之前
// ========== 新增：中断末尾快照数据==========
Log_Spd1 = _IQtoF(Speed) * (float32)BaseSpeed;
Log_Spd2 = _IQtoF(Speed_2) * (float32)BaseSpeed_2;
Log_Iq1  = _IQtoF(IQ_Given);
Log_Iq2  = _IQtoF(IQ_2Given);

EPwm1Regs.ETCLR.bit.INT=1; // 清除中断标志
PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}



// SCI-B receive interrupt
interrupt void SCIBRX_ISR(void)
{
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}


// 主循环中调用的轮询接收函数
void HIL_Poll_Rx(void)
{
    Uint16 ch;
    float32 p[13];
    Uint16 seg;
    char tmp[16];
    Uint16 ti;
    Uint16 i;
    Uint16 j;
    Uint16 s;
    Uint16 parse_start;
    float32 frac;
    Uint16 has_dot;

    while(ScibRegs.SCIFFRX.bit.RXFFST > 0)
    {
        ch = ScibRegs.SCIRXBUF.all & 0xFF;

        if(ch == '\n' || ch == '\r')
        {
            if(Rx_Index > 0)
            {
                Rx_Buffer[Rx_Index] = '\0';

                if((Rx_Buffer[0] == 'P' && Rx_Buffer[1] == ',') ||
                   (Rx_Buffer[0] == 'S' && Rx_Buffer[1] == 'T' && Rx_Buffer[2] == 'E' &&
                    Rx_Buffer[3] == 'P' && Rx_Buffer[4] == '1' && Rx_Buffer[5] == ','))
                {
                    if(Eval_State != 0)
                    {
                        TX232_String("BUSY\n");
                    }
                    else
                    {
                        parse_start = 2;
                        if(Rx_Buffer[0] == 'S') parse_start = 6;
                        seg = 0; ti = 0;
                        for(i = parse_start; i < 192; i++)
                        {
                            if(Rx_Buffer[i] == ',' || Rx_Buffer[i] == '\0')
                            {
                                tmp[ti] = '\0';
                                p[seg] = 0.0f; frac = 0.1f; has_dot = 0; s = 0;
                                if(tmp[0] == '-') { s = 1; }
                                for(j = s; j < ti; j++)
                                {
                                    if(tmp[j] == '.') { has_dot = 1; continue; }
                                    if(!has_dot) p[seg] = p[seg] * 10.0f + (tmp[j] - '0');
                                    else { p[seg] += (tmp[j] - '0') * frac; frac *= 0.1f; }
                                }
                                if(tmp[0] == '-') p[seg] = -p[seg];
                                seg++; ti = 0;
                                if(seg >= 11 || Rx_Buffer[i] == '\0') break;
                            }
                            else { tmp[ti++] = Rx_Buffer[i]; if(ti >= 15) ti = 15; }
                        }
                        if(seg >= 11)
                        {
                            if(p[0]  > 0.001f && p[0]  <= 1.000f)  P1  = p[0];
                            if(p[1]  > 0.001f && p[1]  <= 1.000f)  P2  = p[1];
                            if(p[2]  > 0.001f && p[2]  < 2000.0f)  A11 = p[2];
                            if(p[3]  > 0.001f && p[3]  < 2000.0f)  B11 = p[3];
                            if(p[4]  > 0.001f && p[4]  < 2000.0f)  A22 = p[4];
                            if(p[5]  > 0.001f && p[5]  < 2000.0f)  B22 = p[5];
                            if(p[6]  > 0.1f   && p[6]  < 2000.0f)  C3  = p[6];
                            if(p[7]  > 0.001f && p[7]  <= 1.000f)  P3  = p[7];
                            if(p[8]  > 0.001f && p[8]  < 2000.0f)  A33 = p[8];
                            if(p[9]  > 0.001f && p[9]  < 2000.0f)  B33 = p[9];
                            if(p[10] > 0.1f   && p[10] < 2000.0f)  C8  = p[10];
                            HIL_StepMode = (parse_start == 6) ? 1 : 0;
                            New_Params_Flag = 1;
                        }
                    }
                }
                else if(Rx_Buffer[0] == 'D' && Rx_Buffer[1] == 'U' && Rx_Buffer[2] == 'A' &&
                        Rx_Buffer[3] == 'L' && Rx_Buffer[4] == ',')
                {
                    if(Eval_State != 0)
                    {
                        TX232_String("BUSY\n");
                    }
                    else
                    {
                        seg = 0; ti = 0;
                        for(i = 5; i < 192; i++)
                        {
                            if(Rx_Buffer[i] == ',' || Rx_Buffer[i] == '\0')
                            {
                                tmp[ti] = '\0';
                                p[seg] = 0.0f; frac = 0.1f; has_dot = 0; s = 0;
                                if(tmp[0] == '-') { s = 1; }
                                for(j = s; j < ti; j++)
                                {
                                    if(tmp[j] == '.') { has_dot = 1; continue; }
                                    if(!has_dot) p[seg] = p[seg] * 10.0f + (tmp[j] - '0');
                                    else { p[seg] += (tmp[j] - '0') * frac; frac *= 0.1f; }
                                }
                                if(tmp[0] == '-') p[seg] = -p[seg];
                                seg++; ti = 0;
                                if(seg >= 13 || Rx_Buffer[i] == '\0') break;
                            }
                            else { tmp[ti++] = Rx_Buffer[i]; if(ti >= 15) ti = 15; }
                        }
                        if(seg >= 13)
                        {
                            if(p[0]  > 0.001f && p[0]  <= 1.000f)  P1  = p[0];
                            if(p[1]  > 0.001f && p[1]  <= 1.000f)  P2  = p[1];
                            if(p[2]  > 0.001f && p[2]  < 2000.0f)  A11 = p[2];
                            if(p[3]  > 0.001f && p[3]  < 2000.0f)  B11 = p[3];
                            if(p[4]  > 0.001f && p[4]  < 2000.0f)  A22 = p[4];
                            if(p[5]  > 0.001f && p[5]  < 2000.0f)  B22 = p[5];
                            if(p[6]  > 0.1f   && p[6]  < 2000.0f)  C3  = p[6];
                            if(p[7]  > 0.001f && p[7]  <= 1.000f)  P3  = p[7];
                            if(p[8]  > 0.001f && p[8]  < 2000.0f)  A33 = p[8];
                            if(p[9]  > 0.001f && p[9]  < 2000.0f)  B33 = p[9];
                            if(p[10] > 0.1f   && p[10] < 2000.0f)  C8  = p[10];
                            if(p[11] > 0.001f && p[11] < 200.0f)   Speed_2Kp = _IQ(p[11]);
                            if(p[12] > 0.000001f && p[12] < 1.0f)  Speed_2Ki = _IQ(p[12]);
                            New_Dual_Params_Flag = 1;
                        }
                    }
                }
                else if((Rx_Buffer[0] == 'P' && Rx_Buffer[1] == 'I' && Rx_Buffer[2] == '2' &&
                         Rx_Buffer[3] == ',') ||
                        (Rx_Buffer[0] == 'S' && Rx_Buffer[1] == 'T' && Rx_Buffer[2] == 'E' &&
                         Rx_Buffer[3] == 'P' && Rx_Buffer[4] == '2' && Rx_Buffer[5] == ','))
                {
                    if(Eval_State != 0)
                    {
                        TX232_String("BUSY\n");
                    }
                    else
                    {
                        parse_start = 4;
                        if(Rx_Buffer[0] == 'S') parse_start = 6;
                        seg = 0; ti = 0;
                        for(i = parse_start; i < 192; i++)
                        {
                            if(Rx_Buffer[i] == ',' || Rx_Buffer[i] == '\0')
                            {
                                tmp[ti] = '\0';
                                p[seg] = 0.0f; frac = 0.1f; has_dot = 0; s = 0;
                                if(tmp[0] == '-') { s = 1; }
                                for(j = s; j < ti; j++)
                                {
                                    if(tmp[j] == '.') { has_dot = 1; continue; }
                                    if(!has_dot) p[seg] = p[seg] * 10.0f + (tmp[j] - '0');
                                    else { p[seg] += (tmp[j] - '0') * frac; frac *= 0.1f; }
                                }
                                if(tmp[0] == '-') p[seg] = -p[seg];
                                seg++; ti = 0;
                                if(seg >= 2 || Rx_Buffer[i] == '\0') break;
                            }
                            else { tmp[ti++] = Rx_Buffer[i]; if(ti >= 15) ti = 15; }
                        }
                        if(seg >= 2)
                        {
                            if(p[0] > 0.001f && p[0] < 200.0f) Speed_2Kp = _IQ(p[0]);
                            if(p[1] > 0.000001f && p[1] < 1.0f) Speed_2Ki = _IQ(p[1]);
                            HIL_StepMode = (parse_start == 6) ? 1 : 0;
                            New_PI2_Params_Flag = 1;
                        }
                    }
                }
            }
            Rx_Index = 0;
        }
        else if(Rx_Index < 191)
        {
            Rx_Buffer[Rx_Index++] = (char)ch;
        }
    }
}

void Init_SiShu(void)
{
    // 转变转速基准值格式便于计算（Uint16整数 _iqQ24定点数）
    BaseRpm = _IQ(BaseSpeed);
    BaseRpm_2 = _IQ(BaseSpeed_2);

    // 过流保护阈值计算（15倍额定电流作为过流保护阈值）
    E_Ding_DianLiu = E_Ding_DianLiu_Rated;
    E_Ding_DianLiu_2 = E_Ding_DianLiu_2_Rated;

    GuoliuZhi = 15 * E_Ding_DianLiu;
    GuoliuZhi_2 = 15 * E_Ding_DianLiu_2;

    // 有效电流转电流峰值
    E_Ding_DianLiu = 1.414 * E_Ding_DianLiu;
    E_Ding_DianLiu_2 = 1.414 * E_Ding_DianLiu_2;

    // 转格式
    E_Ding_DianLiu_Q = _IQ(E_Ding_DianLiu);
    E_Ding_DianLiu_2_Q = _IQ(E_Ding_DianLiu_2);

    // ========== 预定义时间及观测器系数计算（同一作用域内变量仅声明一次） ==========
    float32 pi = 3.14159265358979f;
    float32 sqrt_a1b1 = sqrt(A11 * B11);
    float32 sqrt_a2b2 = sqrt(A22 * B22);
    float32 sqrt_a3b3 = sqrt(A33 * B33);

    // 轴1 速度环无控滑模系数
    C1 = (pi * A11) / (P1 * TC1 * sqrt_a1b1);
    C2 = (pi * B11) / (P1 * TC1 * sqrt_a1b1);
    C4 = (pi * A22) / (2.0f * P2 * TC2 * sqrt_a2b2);
    C5 = (pi * B22) / (2.0f * P2 * TC2 * sqrt_a2b2);

    // ----- PTDO 观测器系数初始化 -----
    C6 = (pi * A33) / (2.0f * P3 * TC3 * sqrt_a3b3);
    C7 = (pi * B33) / (2.0f * P3 * TC3 * sqrt_a3b3);

    // 电机物理常数计算
    Kt = (3.0f * MOTOR_NP * MOTOR_PSIF) / (2.0f * MOTOR_J); // 算力矩系数Kt
    KT_INV = 1.0f / Kt;
    B_J = MOTOR_B / MOTOR_J;                               // 算阻尼惯量比 B/J
}


interrupt void INT3_ISR(void)
{

PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;//清除第十二组的中断标志
}


float32 HIL_StepSpeedRef(Uint16 index)
{
    if(index < STEP1_END_INDEX)
    {
        return 0.0f;
    }
    if(index < STEP2_END_INDEX)
    {
        return HIL_STEP_SPEED_REF_40;
    }
    return HIL_TARGET_SPEED_REF;
}


void HIL_Evaluation_Task(void)
{
    HIL_Poll_Rx();

    if(HIL_FailCode != 0)//故障处理入口
    {
        Run_PMSM = 2;
        eva_close();
        Run_PMSM_2 = 2;
        eva_close_2();
        DC_ON_1;
        DC_ON2_1;
        Record_Index = 0;
        New_Params_Flag = 0;
        New_PI2_Params_Flag = 0;
        New_Dual_Params_Flag = 0;
        Eval_State = 0;
        HIL_RecordLength = BO_RECORD_LENGTH;
        HIL_StepMode = 0;
        HIL_BoostDelay = 0;
        HIL_ForceIq = 0;
        HIL_ForceIqPu = 0.0f;
        HIL_MotionSeen = 0;
        HIL_StartupCheckTicks = 0;
        HIL_LastValidSpeed = 0.0f;
        HIL_LastValidSpeed_2 = 0.0f;
        HIL_Speed2SpikeCount = 0;
        Speed = 0;
        Speed_2 = 0;
        Log_Spd1 = 0.0f;
        Log_Spd2 = 0.0f;

        if(HIL_FailCode == 1) TX232_String("RUN_FAIL\n");
        else if(HIL_FailCode == 2) TX232_String("NO_DC\n");
        else if(HIL_FailCode == 3) TX232_String("HALL_FAIL\n");
        else if(HIL_FailCode == 4) TX232_String("STALL\n");
        else if(HIL_FailCode == 5) TX232_String("DC_DROP\n");
        else if(HIL_FailCode == 6) TX232_String("USER_STOP\n");

        HIL_FailCode = 0;
        TX232_String("READY\n");
        return;
    }

    if(New_Params_Flag == 1 && Eval_State == 0)
    {
        New_Params_Flag = 0;
        HIL_RecordLength = (HIL_StepMode != 0) ? STEP_RECORD_LENGTH : BO_RECORD_LENGTH;
        HIL_BoostDelay = 0;
        HIL_ForceIq = 0;
        HIL_ForceIqPu = 0.0f;
        HIL_MotionSeen = 0;
        HIL_StartupCheckTicks = 0;
        HIL_LastValidSpeed = 0.0f;
        Init_SiShu();
        DC_ON_0;
        Pwm_EN_0;
        TX232_String("AK\n");
        Eval_State = 1;
        StartDelay = 5000;
    }

    if(New_PI2_Params_Flag == 1 && Eval_State == 0)
    {
        New_PI2_Params_Flag = 0;
        HIL_RecordLength = (HIL_StepMode != 0) ? STEP_RECORD_LENGTH : BO_RECORD_LENGTH;
        HIL_BoostDelay = 0;
        HIL_ForceIq = 0;
        HIL_ForceIqPu = 0.0f;
        HIL_MotionSeen = 0;
        HIL_StartupCheckTicks = 0;
        HIL_DcDropTicks = 0;
        HIL_LastValidSpeed = 0.0f;
        HIL_LastValidSpeed_2 = 0.0f;
        HIL_Speed2SpikeCount = 0;
        DC_ON2_0;
        Pwm_EN2_0;
        TX232_String("AK\n");
        Eval_State = 21;
        StartDelay = 5000;
    }

    if(Eval_State == 21)
    {
        if(StartDelay > 0) { StartDelay--; return; }
        if(U_dc_dis_2 <= 15)
        {
            HIL_FailCode = 2;
            return;
        }

        eva_close();
        Run_PMSM = 2;
        Speed_2Ui = 0;
        IQ_2Ui = 0;
        ID_2Ui = 0;
        IQ_2Given = 0;
        ID_2Given = 0;
        Speed_2run = 0;
        Speed_2 = 0;
        Log_Spd2 = 0.0f;
        Record_Index = 0;
        SpeedRef_2 = HIL_TARGET_SPEED_REF;
        Modulation_2 = RUN_MODULATION;
        LocationFlag_2 = 1;
        Position_2 = 1;
        PosCount_2 = 0;
        OldRawTheta_2 = 0;
        OldRawThetaPos_2 = 0;
        eva_open_2();
        Run_PMSM_2 = 1;
        SpeedLoopCount_2 = SpeedLoopPrescaler_2;
        Eval_State = 22;
    }

    if(Eval_State == 1)//启动延时状态
    {
        if(StartDelay > 0) { StartDelay--; return; }
        Eval_State = 11;
    }

    if(New_Dual_Params_Flag == 1 && Eval_State == 0)
    {
        New_Dual_Params_Flag = 0;
        HIL_RecordLength = BO_RECORD_LENGTH;
        HIL_StepMode = 0;
        HIL_BoostDelay = 0;
        HIL_ForceIq = 0;
        HIL_ForceIqPu = 0.0f;
        HIL_MotionSeen = 0;
        HIL_StartupCheckTicks = 0;
        HIL_DcDropTicks = 0;
        Dual_Axis2Started = 0;
        Dual_Mode = 1;
        HIL_LastValidSpeed = 0.0f;
        HIL_LastValidSpeed_2 = 0.0f;
        HIL_Speed2SpikeCount = 0;
        Init_SiShu();
        DC_ON_0;
        DC_ON2_0;
        Pwm_EN_0;
        Pwm_EN2_0;
        TX232_String("AK\n");
        Eval_State = 30;
        StartDelay = 5000;
    }

    if(Eval_State == 30)
    {
        if(StartDelay > 0) { StartDelay--; return; }
        if(U_dc_dis <= 15)
        {
            HIL_FailCode = 2;
            return;
        }

        Hall_Fault = 0;
        Hall_Fault_2 = 0;
        IPM_Fault = 0;
        IPM_Fault_2 = 0;
        LocationFlag = 1;
        LocationFlag_2 = 1;
        Position = 1;
        Position_2 = 1;
        PosCount = 0;
        PosCount_2 = 0;
        OldRawTheta = 0;
        OldRawTheta_2 = 0;
        OldRawThetaPos = 0;
        OldRawThetaPos_2 = 0;
        Integral_f1 = 0.0f;
        omega_hat = 0.0f;
        integral_sigma = 0.0f;
        d_hat = 0.0f;
        d_hat_filtered = 0.0f;
        IQ_Given = 0;
        IQ_2Given = 0;
        IQ_Ui = 0;
        IQ_2Ui = 0;
        ID_Ui = 0;
        ID_2Ui = 0;
        Speed_Ui = 0;
        Speed_2Ui = 0;
        Record_Index = 0;
        Speed = 0;
        Speed_2 = 0;
        Log_Spd1 = 0.0f;
        Log_Spd2 = 0.0f;
        SpeedRef = HIL_TARGET_SPEED_REF;
        SpeedRef_2 = HIL_TARGET_SPEED_REF;
        Modulation = RUN_MODULATION;
        Modulation_2 = RUN_MODULATION;
        eva_open();
        Run_PMSM = 1;
        HIL_ForceIq = 0;
        HIL_ForceIqPu = 0.0f;
        HIL_BoostDelay = 0;
        Run_PMSM_2 = 2;
        eva_close_2();
        Dual_Axis2Started = 0;
        SpeedLoopCount = SpeedLoopPrescaler;
        SpeedLoopCount_2 = SpeedLoopPrescaler_2;
        Eval_State = 31;
    }

    if(Eval_State == 11)//预启动检查状态
    {
        if(U_dc_dis <= 15)
        {
            HIL_FailCode = 2;
            return;
        }

        Get_HallAngle();
        if(HallAngle == 7)
        {
            HIL_FailCode = 3;
            return;
        }

        Hall_Fault = 0;
        IPM_Fault = 0;
        LocationFlag = 1;
        Position = 1;
        PosCount = 0;
        OldRawTheta = 0;
        OldRawThetaPos = 0;
        eva_close();
        Run_PMSM_2 = 2;
        eva_close_2();
        Integral_f1 = 0.0f;
        omega_hat = 0.0f;
        integral_sigma = 0.0f;
        d_hat = 0.0f;
        d_hat_filtered = 0.0f;
        IQ_Given = 0;
        IQ_Ui = 0;
        ID_Ui = 0;
        Speed_Ui = 0;
        Record_Index = 0;
        Speed = 0;
        Log_Spd1 = 0.0f;
        HIL_LastValidSpeed = 0.0f;
        HIL_MotionSeen = 0;
        SpeedRef = HIL_TARGET_SPEED_REF;
        Modulation = RUN_MODULATION;
        eva_open();
        Run_PMSM = 1;
        HIL_ForceIq = 0;
        HIL_ForceIqPu = 0.0f;
        HIL_BoostDelay = 0;
        SpeedLoopCount = SpeedLoopPrescaler;
        Eval_State = 2;
    }

    if(Eval_State == 12)//Boost强拖状态
    {
        float32 abs_speed = Log_Spd1;

        if(U_dc_dis <= 15)
        {
            HIL_FailCode = 5;
            return;
        }

        if(abs_speed < 0.0f) abs_speed = -abs_speed;
        if(abs_speed >= HIL_MOTION_RPM)
        {
            HIL_MotionSeen = 1;
        }

        if(HIL_MotionSeen == 0 && HIL_BoostDelay <= HIL_BOOST_STEP_TICKS)
        {
            HIL_ForceIqPu = HIL_BOOST_IQ_PU_HIGH;
        }

        if(HIL_BoostDelay > 0)
        {
            HIL_BoostDelay--;
            return;
        }

        HIL_ForceIq = 0;
        HIL_ForceIqPu = 0.0f;
        Integral_f1 = 0.0f;
        omega_hat = abs_speed * 0.104719755f;
        integral_sigma = 0.0f;
        d_hat = 0.0f;
        d_hat_filtered = 0.0f;
        IQ_Ui = 0;
        ID_Ui = 0;
        Speed_Ui = 0;

        Record_Index = 0;
        HIL_MotionSeen = 0;
        HIL_StartupCheckTicks = 0;
        SpeedRef = HIL_TARGET_SPEED_REF;
        if(Dual_Mode != 0)
        {
            Eval_State = 31;
        }
        else
        {
            Eval_State = 2;
        }
    }

    if(Eval_State == 2)
    {
if(U_dc_dis <= 15)
        {
            HIL_FailCode = 5;
            return;
        }

    }

    if(Eval_State == 31)
    {
        if(Dual_Axis2Started == 0 && Record_Index >= DUAL_AXIS2_START_INDEX)
        {
            DC_ON2_0;
            Pwm_EN2_0;
            Speed_2Ui = 0;
            IQ_2Ui = 0;
            ID_2Ui = 0;
            IQ_2Given = 0;
            ID_2Given = 0;
            Speed_2 = 0;
            Log_Spd2 = 0.0f;
            OldRawTheta_2 = 0;
            OldRawThetaPos_2 = 0;
            LocationFlag_2 = 1;
            Position_2 = 1;
            PosCount_2 = 0;
            eva_open_2();
            SpeedRef_2 = HIL_TARGET_SPEED_REF;
            Run_PMSM_2 = 1;
            SpeedLoopCount_2 = SpeedLoopPrescaler_2;
            Dual_Axis2Started = 1;
        }

        if(U_dc_dis <= 15)
        {
            HIL_DcDropTicks++;
            if(HIL_DcDropTicks >= HIL_DC_DROP_CONFIRM_TICKS)
            {
                HIL_FailCode = 5;
                return;
            }
        }
        else
        {
            HIL_DcDropTicks = 0;
        }
    }

    if(Eval_State == 33)
    {
        char dual_tx_buf[64];
        Uint16 i;

        Run_PMSM = 2;
        Run_PMSM_2 = 2;
        eva_close();
        eva_close_2();
        TX232_String("DUAL_START\n");

        DINT;
        for(i = 0; i < HIL_RecordLength; i++)
        {
            float32 vals[2];
            Uint16 kk;
            Uint16 idx = 0;

            vals[0] = ((float32)Log_Speed_x10[i]) * 0.1f;
            vals[1] = ((float32)Log_Speed_2_x10[i]) * 0.1f;

            for(kk = 0; kk < 2; kk++)
            {
                float32 val = vals[kk];
                Uint16 int_part, frac_part;

                if(val != val) { val = 0.0f; }
                if(val > 9999.0f) val = 9999.0f;
                if(val < -999.0f) val = -999.0f;
                if(val < 0.0f) { val = -val; dual_tx_buf[idx++] = '-'; }

                int_part = (Uint16)val;
                frac_part = (Uint16)((val - (float32)int_part) * 100.0f + 0.5f);
                if(frac_part >= 100) { frac_part = 99; }
                if(int_part >= 1000) dual_tx_buf[idx++] = '0' + int_part/1000;
                if(int_part >= 100)  dual_tx_buf[idx++] = '0' + (int_part/100)%10;
                if(int_part >= 10)   dual_tx_buf[idx++] = '0' + (int_part/10)%10;
                dual_tx_buf[idx++] = '0' + (int_part % 10);
                dual_tx_buf[idx++] = '.';
                dual_tx_buf[idx++] = '0' + (frac_part / 10);
                dual_tx_buf[idx++] = '0' + (frac_part % 10);
                if(kk == 0) dual_tx_buf[idx++] = ',';
            }

            dual_tx_buf[idx++] = '\n';
            dual_tx_buf[idx] = '\0';
            TX232_String(dual_tx_buf);
        }

        while(ScibRegs.SCIFFRX.bit.RXFFST > 0) { ScibRegs.SCIRXBUF.all; }
        EINT;
        Eval_State = 0;
        Dual_Mode = 0;
        TX232_String("DUAL_END\n");
        TX232_String("READY\n");
    }

    if(Eval_State == 3 || Eval_State == 23)
    {
        char tx_buf[32];
        Uint16 i;

        if(Eval_State == 23)
        {
            Run_PMSM_2 = 2;
            eva_close_2();
        }
        else
        {
            Run_PMSM = 2;
            eva_close();
        }
        TX232_String("DATA_START\n");

        DINT;
        for(i = 0; i < HIL_RecordLength; i++)
        {
            float32 val = ((float32)Log_Speed_x10[i]) * 0.1f;
            Uint16 int_part, frac_part, idx = 0;

            if(val != val) { val = 0.0f; }
            if(val > 9999.0f) val = 9999.0f;
            if(val < -999.0f) val = -999.0f;
            if(val < 0.0f) { val = -val; tx_buf[idx++] = '-'; }

            int_part = (Uint16)val;
            frac_part = (Uint16)((val - (float32)int_part) * 100.0f + 0.5f);
            if(frac_part >= 100) { frac_part = 99; }
            if(int_part >= 1000) tx_buf[idx++] = '0' + int_part/1000;
            if(int_part >= 100)  tx_buf[idx++] = '0' + (int_part/100)%10;
            if(int_part >= 10)   tx_buf[idx++] = '0' + (int_part/10)%10;
            tx_buf[idx++] = '0' + (int_part % 10);
            tx_buf[idx++] = '.';
            tx_buf[idx++] = '0' + (frac_part / 10);
            tx_buf[idx++] = '0' + (frac_part % 10);
            tx_buf[idx++] = '\n';
            tx_buf[idx] = '\0';

            TX232_String(tx_buf);
        }

        while(ScibRegs.SCIFFRX.bit.RXFFST > 0) { ScibRegs.SCIRXBUF.all; }
        EINT;
        Eval_State = 0;
        TX232_String("DATA_END\n");
        TX232_String("READY\n");
    }
}

void TX232_String(char *str)
{
    Uint16 i = 0;
    while(str[i] != '\0')
    {
        // 等待 SCIB 发送FIFO有空位(避免溢出丢失)
        // TXFFST 表示当前 FIFO 里排队等待发送的字符个数，最大是 16
        while(ScibRegs.SCIFFTX.bit.TXFFST >= 16) {;}

        // 【核心代码！】把单个字符塞进发送缓冲区，硬件会自动通过串口发出
        ScibRegs.SCITXBUF = str[i];

        i++;
    }
}



void eva_close_2(void)//轴2 关闭PWM输出
{
     EALLOW;
       //  1.3.5强制高，2.4.6有效

       GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;   // Configure GPIO0 as EPWM4A

     GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;   // Configure GPIO2 as EPWM5A

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;   // Configure GPIO4 as EPWM6A


    GpioCtrlRegs.GPADIR.bit.GPIO6=1;
    GpioCtrlRegs.GPADIR.bit.GPIO8=1;
    GpioCtrlRegs.GPADIR.bit.GPIO10=1;

    GpioDataRegs.GPASET.bit.GPIO6=1;
    GpioDataRegs.GPASET.bit.GPIO8=1;
    GpioDataRegs.GPASET.bit.GPIO10=1;





    EPwm4Regs.CMPA.half.CMPA =3375; //
EPwm5Regs.CMPA.half.CMPA = 3375; //
EPwm6Regs.CMPA.half.CMPA = 3375; //
   EDIS;
    Run_PMSM_2=2;
   LocationFlag_2=1;
   Speed_2Ui=0;
   ID_2Ui=0;
   IQ_2Ui=0;
   Position_2=1;
   j=0;
   Speed_dis_2=0;
   IQ_2Given=0;
   OldRawTheta_2=0;
   SpeedRef_2=0;
   speed_give_2=0;
   Modulation_2=0.25;    // 调制比
   O_Current_2=0;
   PosCount_2=0;
   OldRawThetaPos_2=0;
   Hall_Fault_2=0;
   Speed_2run=0;


}

//===========================================================================
// No more.
//===========================================================================





