#include<stm32f10x.h>
#include<stdio.h>
#include"delay.h"
#include"LCD.h"

//------------------------------------------------------------------------------

/* USART_1 & printf ���� define*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ascii)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ascii, FILE *f)
#endif 
#define BUFFER_SIZE 200

//------------------------------------------------------------------------------

// BlueTooth ���� ������ ���� ����----------------------------------------------
volatile char control_data='a';
u8 RxBuf[BUFFER_SIZE];                    
u8 TxBuf[BUFFER_SIZE]; 
u8 TxInCnt=0;   
u8 TxOutCnt=0;     
u8 RxCnt=0;


// �������� ���ŵ����� ���� ����
int Dust_value[16]={9,};

int Measured_value=0;
int _Value;
volatile u16 a=0;

//------------------------------------------------------------------------------
void Rcc_Initialize(void) ;
void Gpio_Initialize(void);
void Nvic_Initialize(void);
void UART1_Initialize(void);
void USART1_IRQHandler(void);
void UART2_Initialize(void);

void Delay_us(unsigned int Count)
{
   Count *= 12;
   for(; Count!=0; Count--);
}
void Delay_ms(unsigned int Count)
{
   Count *= 12000;
   for(; Count!=0; Count--);
}

void main(void)
{
  Rcc_Initialize( );
  Gpio_Initialize( );
  Nvic_Initialize( );
  UART1_Initialize( );
  USART1_IRQHandler( );
  UART2_Initialize( );



  while(1)
 {
  Delay_us(2000);

  //to �̼����� ���� , send
  USART_SendData(USART2,0x11);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  
  USART_SendData(USART2,0x01);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  
  USART_SendData(USART2,0x01);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  
  USART_SendData(USART2,0xED);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  
  Delay_us(3000);
  //------- 
  //from �̼����� ���� , recieve 
    for(int temp=0; temp<16;temp++) //���� ��Ŷ 00 16 0d 01
    {
      Dust_value[temp]= USART_ReceiveData(USART2);
      Delay_ms(1);
    }
  
  USART_ClearITPendingBit(USART2, USART_IT_RXNE); 
  
  //�̼������� ���.
  Measured_value=Dust_value[3]*256*256*256+Dust_value[4]*256*256+Dust_value[5]*256+Dust_value[6];
  _Value=(Measured_value*3528)/100000;
  
  //------
  //to ������� send.
  printf("{%d}",_Value); 
  //------
  
  //LED ������
  if(_Value<=30)
   GPIOD->ODR = 0x0001; 
 
  else if((_Value<=80)&&(_Value>30))
     GPIOD->ODR = 0x0002; 
   
  else if((_Value<=150)&&(_Value>80))
       GPIOD->ODR = 0x0004;
     
  else if(_Value>150)
       GPIOD->ODR = 0x000C; 
     
  //------
 
 }

}

//RCC ����----------------------------------------------------------------------
void Rcc_Initialize(void)
{
  ErrorStatus HSEStartUpStatus;
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if (HSEStartUpStatus == SUCCESS)
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//HCLK = SYSCLK
    
    RCC_PCLK2Config(RCC_HCLK_Div1); // 72Mhz
    RCC_PCLK1Config(RCC_HCLK_Div4); // 72/4 x2 = 36Mhz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLL ����8Mhz*9=72Mhz
    RCC_PLLCmd(ENABLE);
    
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);
  }
 
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //GPIO A�� clock����Ѵ�. 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE); //GPIO D�� clock����Ѵ�. 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//������� UART1 clock���.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);   // �̼����� ����. 

}

//GPIO ����---------------------------------------------------------------------
void Gpio_Initialize(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   
   //������� TX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//�� 9�߰� 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   //������� RX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//�� 10�߰� 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   //�̼����� ���� TX 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //�̼����� ���� RX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//�� 10�߰� 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
      //LED
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;// �� 0,1,2,3
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   


} 

// Nvic �Լ�����----------------------------------------------------------
void Nvic_Initialize(void)
{ 
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
   
   //USART1_IRQ enable
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
 
}





//Printf�� ����ϱ� ���� ����---------------------------------------------------
PUTCHAR_PROTOTYPE
{
    TxBuf[TxInCnt] = ascii;
    if(TxInCnt<BUFFER_SIZE-1) TxInCnt++;
    else TxInCnt = 0;     
    
    //Enable the USART1 Transmit interrupt     
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

    return ascii; 
}
  
//-----------------------------------------------------------------------------

void UART1_Initialize(void){ //PCLK2�� ����Ѵ�. 
  
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   
   
   USART_InitStructure.USART_BaudRate = 9600; 
   // baud rate= ������ ��ſ��� ���� ������ ���� �ӵ��� 1�ʰ��� ���۵Ǵ� ��ȣ�� ���� ��Ÿ�� ��.
   
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   //8��Ʈ �Ǵ� 9��Ʈ�� ���� �� ���ִ�. 
   
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   //�ϳ��� �ܾ�(word)�� ���� ǥ���ϱ� ���� ���Ŀ� �ΰ��ϴ� 2��Ʈ(1 1.5 2 �� ���� ����)
   
   USART_InitStructure.USART_Parity = USART_Parity_No;
   //1���� ��Ʈ�� ������ ¦������ Ȧ�������� ����

   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   // ���������� ���źҰ� ������ ��� �۽������� ������ �������� �ʵ��� �ϰ� �ٽ� ���Ű��� ���°� �Ǿ��� ���� ������ �����ϴ� ���. 
   
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   //USART_CR1 �������Ϳ��� bit3 TE Transmitter enable �� Bit 2 RE Receiver enable �� �����ϱ� ����. �ش��Ʈ��ġ�� 1�� �ǵ�����. 
   
   USART_Init(USART1, &USART_InitStructure);
   //���������ϰ� 0���� �ʱ�ȭ. 
   
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //enable receive interrupt
   
   USART_Cmd(USART1, ENABLE); // ����Ϸ��� ENABLE ����ߵ� <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
   //USART1 Enable
   
   
   NVIC_InitStructure.NVIC_IRQChannel = 37; //USART1_IRQChannel
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
}


//(BLUETOOTH ���)USART1 ���ͷ�Ʈ ����------------------------------------------
void USART1_IRQHandler(void)
//UART1���� ���ŵ� ���� �д� ��. 
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)// ��Ʈ�� 0�� �ƴ� ��쿡 �����Ͱ� ����.
    {         
       
        control_data  = USART_ReceiveData(USART1); //0�� �ƴ� ��쿡 �����͸� �д´�. control_data= ���ۿ� ���� �����ϴ� ������ �ϴ� ����.
        
        /*Buffer�� �����ؾ��ϴ� ����.
        
        ���Žð������� ó���ð��� �� ���ͷ�Ʈ �߻��κп����� ���ۿ� �ܼ��� ���常�ϰ� ó���� �ٸ������� �Ѵ�.
        */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
        // pending bit�� clear.(�����ָ� ���ͷ�Ʈ�� ó���Ǿ����� �˼����� �ٽ� ���ͷ�Ʈ�� �߻��Ѱ����� �����ؼ� ��� �ڵ鷯 ȣ��) 
       
    }
    
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)//��Ʈ�� 0�� �ƴ� ��쿡 �����Ͱ� �۽� 
    {         
     
        USART_SendData(USART1,TxBuf[TxOutCnt]);
        
        if(TxOutCnt<BUFFER_SIZE-1) TxOutCnt++; // Txoutcount�� buffer size ���� ������ +1�� ����. 
        else TxOutCnt = 0;      
            
        if(TxOutCnt == TxInCnt)// Txoutcount�� Txincount�� �Ǹ� tx ���� RX Ų��. 
        {
          USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
          USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        }         
    }        
   
}

//------------------------------------------------------------------------------
//�̼����� ����.
void UART2_Initialize(void)
{
 
   USART_InitTypeDef USART_InitStructure;
   USART_InitStructure.USART_BaudRate = 9600 ;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode =USART_Mode_Rx |USART_Mode_Tx;
   USART_Init(USART2, &USART_InitStructure);
   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
   USART_Cmd(USART2, ENABLE);  
}

