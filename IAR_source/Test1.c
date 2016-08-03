#include<stm32f10x.h>
#include<stdio.h>
#include"delay.h"
#include"LCD.h"

//------------------------------------------------------------------------------

/* USART_1 & printf 관련 define*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ascii)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ascii, FILE *f)
#endif 
#define BUFFER_SIZE 200

//------------------------------------------------------------------------------

// BlueTooth 수신 데이터 저장 변수----------------------------------------------
volatile char control_data='a';
u8 RxBuf[BUFFER_SIZE];                    
u8 TxBuf[BUFFER_SIZE]; 
u8 TxInCnt=0;   
u8 TxOutCnt=0;     
u8 RxCnt=0;


// 먼지센서 수신데이터 저장 변수
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

  //to 미세먼지 센서 , send
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
  //from 미세먼지 센서 , recieve 
    for(int temp=0; temp<16;temp++) //시작 패킷 00 16 0d 01
    {
      Dust_value[temp]= USART_ReceiveData(USART2);
      Delay_ms(1);
    }
  
  USART_ClearITPendingBit(USART2, USART_IT_RXNE); 
  
  //미세먼지값 계산.
  Measured_value=Dust_value[3]*256*256*256+Dust_value[4]*256*256+Dust_value[5]*256+Dust_value[6];
  _Value=(Measured_value*3528)/100000;
  
  //------
  //to 블루투스 send.
  printf("{%d}",_Value); 
  //------
  
  //LED 색변경
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

//RCC 설정----------------------------------------------------------------------
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
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLL 설정8Mhz*9=72Mhz
    RCC_PLLCmd(ENABLE);
    
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);
  }
 
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //GPIO A에 clock허용한다. 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE); //GPIO D에 clock허용한다. 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//블루투스 UART1 clock허용.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);   // 미세먼지 센서. 

}

//GPIO 설정---------------------------------------------------------------------
void Gpio_Initialize(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   
   //블루투스 TX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//핀 9추가 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   //블루투스 RX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//핀 10추가 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   //미세먼지 센서 TX 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //미세먼지 센서 RX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//핀 10추가 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
      //LED
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;// 핀 0,1,2,3
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   


} 

// Nvic 함수선언----------------------------------------------------------
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





//Printf를 사용하기 위한 정의---------------------------------------------------
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

void UART1_Initialize(void){ //PCLK2를 사용한다. 
  
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   
   
   USART_InitStructure.USART_BaudRate = 9600; 
   // baud rate= 데이터 통신에서 직렬 전송의 변조 속도를 1초간에 전송되는 신호의 수로 나타낸 값.
   
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   //8비트 또는 9비트로 설정 할 수있다. 
   
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   //하나의 단어(word)의 끝을 표시하기 위해 최후에 부가하는 2비트(1 1.5 2 로 설정 가능)
   
   USART_InitStructure.USART_Parity = USART_Parity_No;
   //1”의 비트의 개수가 짝수인지 홀수인지를 결정

   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   // 수신측에서 수신불가 상태인 경우 송신측에서 데이터 전송하지 않도록 하고 다시 수신가능 상태가 되었을 때만 데이터 전송하는 방식. 
   
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   //USART_CR1 레지스터에서 bit3 TE Transmitter enable 과 Bit 2 RE Receiver enable 를 설정하기 위함. 해당비트위치가 1이 되도록함. 
   
   USART_Init(USART1, &USART_InitStructure);
   //변수선언하고 0으로 초기화. 
   
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //enable receive interrupt
   
   USART_Cmd(USART1, ENABLE); // 사용하려면 ENABLE 해줘야됨 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
   //USART1 Enable
   
   
   NVIC_InitStructure.NVIC_IRQChannel = 37; //USART1_IRQChannel
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
}


//(BLUETOOTH 통신)USART1 인터럽트 설정------------------------------------------
void USART1_IRQHandler(void)
//UART1으로 수신된 값을 읽는 것. 
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)// 비트에 0이 아닐 경우에 데이터가 수신.
    {         
       
        control_data  = USART_ReceiveData(USART1); //0이 아닐 경우에 데이터를 읽는다. control_data= 버퍼에 값을 저장하는 역할을 하는 변수.
        
        /*Buffer룰 구현해야하는 이유.
        
        수신시간에비해 처리시간이 길어서 인터럽트 발생부분에서는 버퍼에 단순히 저장만하고 처리는 다른곳에서 한다.
        */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
        // pending bit를 clear.(안해주면 인터럽트가 처리되었는지 알수없고 다시 인터럽트가 발생한것으로 인지해서 계속 핸들러 호출) 
       
    }
    
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)//비트에 0이 아닐 경우에 데이터가 송신 
    {         
     
        USART_SendData(USART1,TxBuf[TxOutCnt]);
        
        if(TxOutCnt<BUFFER_SIZE-1) TxOutCnt++; // Txoutcount가 buffer size 보다 작으면 +1씩 샌다. 
        else TxOutCnt = 0;      
            
        if(TxOutCnt == TxInCnt)// Txoutcount가 Txincount가 되면 tx 끄고 RX 킨다. 
        {
          USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
          USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        }         
    }        
   
}

//------------------------------------------------------------------------------
//미세먼지 센서.
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

