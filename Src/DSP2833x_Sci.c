// TI File $Revision: /main/2 $
// Checkin $Date: March 1, 2007   16:06:07 $
//###########################################################################
//
// FILE:	DSP2833x_Sci.c
//
// TITLE:	DSP2833x SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
Uint16 speed_dis=0;
 
Uint16 Speed_dis_2=0;

void InitSci_B(void)
{
    EALLOW;
    ScibRegs.SCICCR.all = 0x0007;   // 1ֹͣλ, ��У��, 8λ����
    ScibRegs.SCICTL1.all = 0x0003;  // ʹ�� TX, RX
    ScibRegs.SCICTL2.all = 0x0000;  // ���������жϣ������ò�ѯ���������жϵ��ڿ����߼��⣩

    // ���Ӿ�ȷ�� 115200 ������ (���� LSPCLK = 37.5MHz)
    ScibRegs.SCIHBAUD = 0x00;
    ScibRegs.SCILBAUD = 0x28; // ���Ը�Ϊ 40 (0x28)����ʱ�� 39 ����

    ScibRegs.SCIFFTX.all = 0xE040;  // 闁革妇鍎搁弫鍫遍嚋閸楀啿FIFO閿涘苯顦╅崚鍡欑矓TXFIFO锟藉箚uml;鐣玊XFFINT
    ScibRegs.SCIFFRX.all = 0x202C;  // RXFFIL=12, 平衡中断频率和溢出的最优值
    ScibRegs.SCIFFCT.all = 0x00;
   
    ScibRegs.SCICTL1.bit.SWRESET = 1;
    EDIS;
}


void InitSci_C(void)//485
{    
    
 	ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback 
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK, 
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =0x0003; 
	ScicRegs.SCICTL2.bit.RXBKINTENA =1;
    ScicRegs.SCIHBAUD    =0x0001;//9600
    ScicRegs.SCILBAUD    =0x00e7;
	ScicRegs.SCICCR.bit.LOOPBKENA =0; // disable loop back  
	ScicRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset 
	ScicRegs.SCIFFRX.all=0x204f;
    ScicRegs.SCIFFCT.all=0x0;
	ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
    
}

        
#pragma CODE_SECTION(RS232_SendBytes, "ramfuncs");  
void RS232_SendBytes(Uint16 DATA[],Uint16 N)//���ڷ���N���ֽ�����
{
    Uint16 i=0;
 for(i=0;i<N;i++)
    {
      while(ScibRegs.SCICTL2.bit.TXRDY!=1);
         
      ScibRegs.SCITXBUF=DATA[i];
	    
	   
	   while(ScibRegs.SCICTL2.bit.TXRDY!=1);
        

    }
}


 

 
 

	
//===========================================================================
// End of file.
//===========================================================================
