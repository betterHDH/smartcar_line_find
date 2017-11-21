#include  "common.h"
#include  "camera.h"
#include  "gpio.h"
#include  "exti.h"
#include  "uart.h"
#include  "roadJudge.h"
#include  "arm_math.h"
#include  "FTM.h"
#include  "delay.h"
#include  "isr.h"

//变量定义
unsigned char threshold;
unsigned char Is_SendPhoto=0;  
unsigned char Pix_data[V][H]={0},Bina_data[V][H]={0};
int  LeftBorder[V]={0},RightBorder[V]={0}, CenterLine[V]={0};
unsigned char CCD_V,CCD_H;
unsigned char top_line=40,top_line1=50,top_line2=48;
long count,WhiteNum;
unsigned char width_count=50;
unsigned char width_min=50;
unsigned char V_count=0;          //采集行计数
unsigned char jump_point;

unsigned char last_top;
unsigned char top_top_line=49;

unsigned char THRE[20];
long  thre_sum=0;
unsigned char thread_count;
unsigned char center_min=30;
unsigned char left_line,right_line;

unsigned int left_count,right_count;

/*    unsigned char left_tiaobian=0;
    unsigned char  right_tiaobian=0;
    unsigned char left_guaidian=0;
    unsigned char  right_guaidian=0; 
     unsigned char left_tiaobian_row=0;
     unsigned char right_tiaobian_row=0;
    unsigned char left_guaidian_row=0;
    unsigned char right_guaidian_row=0;
*/
//摄像头初始化
void CameraInit()
{  
    exti_init(PORTC,1,rising_down);  //行中断，PORTC1 端口外部中断初始化 ，上升沿触发中断，内部下拉
    disable_irq(89);                 //行中断关闭
    disable_irq(90);                 //场中断关闭
    exti_init(PORTD,1,falling_down); //场中断，PORTD1 端口外部中断初始化 ，下降沿触发中断，内部下拉
}

//二值化函数
void BinaData()
{
  int i=0,j=0;
  //count=0;
  unsigned char *p;
  for(i=0;i<V;i++)
  {
    p=Pix_data[i];
    for(j=0;j<H;j++)
    {
      if(*(p+j)>=threshold)     
      {
        Bina_data[i][j]=1;
      //  count++;
      }
      else                            Bina_data[i][j]=0;
    }
  }
}


//整场去燥 
void AllFilt()
{ 
  count=0;
  //unsigned char sum;
  for(CCD_V=1;CCD_V<V-1;CCD_V++)
  {
    for(CCD_H=1;CCD_H<H-1;CCD_H++)
    {
      if(Bina_data[CCD_V][CCD_H]==1)
      {  if((Bina_data[CCD_V-1][CCD_H]==1 || Bina_data[CCD_V+1][CCD_H]==1) && (Bina_data[CCD_V][CCD_H-1]==1 || Bina_data[CCD_V][CCD_H+1]==1))
          {
               Bina_data[CCD_V][CCD_H]=1;  
               count++;
          }
         else  
               Bina_data[CCD_V][CCD_H]=0;
     /*   sum=0;
        sum=sum + Bina_data[CCD_V-1][CCD_H] + Bina_data[CCD_V-1][CCD_H-1] + Bina_data[CCD_V-1][CCD_H+1]
             + Bina_data[CCD_V][CCD_H-1] + Bina_data[CCD_V][CCD_H+1] + Bina_data[CCD_V+1][CCD_H+1]
             + Bina_data[CCD_V+1][CCD_H-1] + Bina_data[CCD_V+1][CCD_H];
        if(sum>=4)        Bina_data[CCD_V][CCD_H]=1;
        else              Bina_data[CCD_V][CCD_H]=0;
        */
      }
      else if(Bina_data[CCD_V][CCD_H]==0)
      {
         if((Bina_data[CCD_V-1][CCD_H]==0 || Bina_data[CCD_V+1][CCD_H]==0) && (Bina_data[CCD_V][CCD_H-1]==0 || Bina_data[CCD_V][CCD_H+1]==0))
          {
               Bina_data[CCD_V][CCD_H]=0; 
          }
         else  
         {
               Bina_data[CCD_V][CCD_H]=1;
               count++;
         }
        /*sum=0;
        sum=sum + Bina_data[CCD_V-1][CCD_H] + Bina_data[CCD_V-1][CCD_H-1] + Bina_data[CCD_V-1][CCD_H+1]
             + Bina_data[CCD_V][CCD_H-1] + Bina_data[CCD_V][CCD_H+1] + Bina_data[CCD_V+1][CCD_H+1]
             + Bina_data[CCD_V+1][CCD_H-1] + Bina_data[CCD_V+1][CCD_H];
        if(sum<=6)        Bina_data[CCD_V][CCD_H]=0;
        else              Bina_data[CCD_V][CCD_H]=1;*/
      }
    }
  }
}

//统计图像下半部分白点数
void WhiteCount()
{
  WhiteNum=0;
  for(CCD_V=0;CCD_V<25;CCD_V++)
    for(CCD_H=0;CCD_H<H;CCD_H++)
      if(Bina_data[CCD_V][CCD_H]==1)
        WhiteNum++;
}


//中心线提取
void get_center()
{
  
  unsigned char *p;
  static unsigned char last_center=75;
  for(CCD_V=0;CCD_V<LINE;CCD_V++)
  {
    p=Bina_data[CCD_V];
    for(CCD_H=75;CCD_H>1;CCD_H--)//中间往右
    {
      if((*(p+CCD_H)==0) && (*(p+CCD_H-1)==0) && (*(p+CCD_H-2)==0)) //右边黑线检测
      {
        RightBorder[CCD_V]=CCD_H;
        break;
      }
      else     RightBorder[CCD_V]=0;      //如果右边没有检测到有黑线
     
    }
      
   for(CCD_H=75;CCD_H<H-2;CCD_H++)  //中间往左
   {
      if((*(p+CCD_H)==0) && (*(p+CCD_H+1)==0) && (*(p+CCD_H+2)==0))    //左边黑线检测
      {
        LeftBorder[CCD_V]=CCD_H;
        break;
       }
      else    LeftBorder[CCD_V]=149;    //如果左边没有检测到有黑线
     
   }
    CenterLine[CCD_V]=(LeftBorder[CCD_V]+RightBorder[CCD_V])/2; //计算中心线
    
    if(fabsf(CenterLine[4]-last_center)>=8)  //基准
      CenterLine[4]=last_center;
    last_center=CenterLine[4];         
    
    if((LeftBorder[CCD_V]-RightBorder[CCD_V])>width_count)  //判断赛道宽度
       width_count=LeftBorder[CCD_V]-RightBorder[CCD_V];
    
  }  
  
  for(CCD_V=LINE;CCD_V<V;CCD_V++)
  {
    p=Bina_data[CCD_V];
    for(CCD_H=(unsigned char )CenterLine[CCD_V-1];CCD_H>1;CCD_H--)//中间往右
    {
      if((*(p+CCD_H)==0) && (*(p+CCD_H-1)==0) && (*(p+CCD_H-2)==0)) //右边黑线检测
      {
        RightBorder[CCD_V]=CCD_H;
        break;
      }
      else   RightBorder[CCD_V]=0;      
      
    }
     
    for(CCD_H=(unsigned char )CenterLine[CCD_V-1];CCD_H<H-2;CCD_H++)  //中间往左
    {
      if((*(p+CCD_H)==0) && (*(p+CCD_H+1)==0) && (*(p+CCD_H+2)==0))    //左边黑线检测
      {
        LeftBorder[CCD_V]=CCD_H;
        break;
      }
      else   LeftBorder[CCD_V]=149;
     
    }
    CenterLine[CCD_V]=(LeftBorder[CCD_V]+RightBorder[CCD_V])/2;
       
    
    if((LeftBorder[CCD_V]-RightBorder[CCD_V])>width_count) ////判断赛道宽度
       width_count=LeftBorder[CCD_V]-RightBorder[CCD_V];
   
  }
}

void centeradjust()
{
  for(CCD_V=top_line-1;CCD_V<V;CCD_V++)
  {
    if(LeftBorder[top_line-2]>=145 || LeftBorder[top_line-3]>=145)
    {//
      CenterLine[CCD_V]= (int)((147-left_offset_row)*1.0/(V-1-top_line)*(CCD_V+2-top_line)+0.5)+
        left_offset_row;
      if(CenterLine[CCD_V]>=147)    CenterLine[CCD_V]=147;
    }
    
    if(RightBorder[top_line-2]<=5 || RightBorder[top_line-3]<=5)
    {
      CenterLine[CCD_V]=(int)((2-right_offset_row)*1.0/(V-1-top_line)*(CCD_V+2-top_line)+0.5)+right_offset_row;
      if(CenterLine[CCD_V]<=3)      CenterLine[CCD_V]=2;
    }
  }
  
        
}
void stop_line()
{
  Right_dot=0;
  left_dot=0;
  
  if(stop_flag==1)
  {
      for(CCD_V=10;CCD_V<15;CCD_V++)
      {
        for(CCD_H=RightBorder[20];CCD_H<=CenterLine[20];CCD_H++)
        {
          if(Bina_data[CCD_V][CCD_H]==0)
          Right_dot++;    
        }
        for(CCD_H=CenterLine[20];CCD_H<=LeftBorder[20];CCD_H++)
        {
          if(Bina_data[CCD_V][CCD_H]==0)
          left_dot++;   
        }
      if((LeftBorder[CCD_V]-RightBorder[CCD_V])<width_min)
             width_min=LeftBorder[CCD_V]-RightBorder[CCD_V]; 
      }
      if(Right_dot>=30 && Right_dot<=90 && left_dot<=90 && left_dot>=30 && top_line1>=45)
      {
         FTM_PWM_Duty(FTM0,CH4,1000);
         FTM_PWM_Duty(FTM0,CH5,0);
         if(gpio_get(PORTA,13)==0)    delayms(50);
         if(gpio_get(PORTA,14)==0)    delayms(50);
         if(gpio_get(PORTA,15)==0)    delayms(50);
         if(gpio_get(PORTA,16)==0)    delayms(50);
         if(gpio_get(PORTA,12)==0)    delayms(50);
         gpio_set(PORTC,19,0);
      }
  }
}


//中心线判断与矫正  顶端行求取
void TopGet()
{
  
  for(CCD_V=8;CCD_V<V-2;CCD_V++)
  {
    if((CenterLine[CCD_V]-CenterLine[CCD_V+1])>center_error || (CenterLine[CCD_V+1]-CenterLine[CCD_V])>center_error)
    {
      if((CenterLine[CCD_V+1]-CenterLine[CCD_V+2])>center_error || (CenterLine[CCD_V+2]-CenterLine[CCD_V+1])>center_error)
      {
        top_line1=CCD_V;
        break;
      }
      else ;
    }
    else 
       top_line1=48;
    if(LeftBorder[CCD_V]-RightBorder[CCD_V]<=30)
    {
      if(LeftBorder[CCD_V+1]-RightBorder[CCD_V+1]<=30)
      {
        top_line2=CCD_V;
        break;
      }
      else ;
    }
  }
  if(top_line1>=30 && top_line2>=30)
        top_line=(top_line2 < top_line1) ? top_line2 : top_line1;
  
  /*
  if(count>=5100)
     last_top=top_line;*/
}

void line_count()
{
  left_line=0;
  right_line=0;
  for(CCD_V=0;CCD_V<49;CCD_V++)
  {
      if(LeftBorder[CCD_V]==149)
        left_line++;
      if(RightBorder[CCD_V]==0)
        right_line++;
  }
}

//十字交叉
void CrossJudge()
{
  if(count>=5000 && count<=5800 && right_line>=10 && left_line>=10)// && right_line<=18 && left_line<=18)
  {   
    for(CCD_V=8;CCD_V<49;CCD_V++)
    {
      if(CenterLine[8]+CenterLine[7]-CenterLine[1]-CenterLine[0]<0)
      {
        if(LeftBorder[CCD_V] - LeftBorder[CCD_V-1]>= 3) 
        {
          LeftBorder[CCD_V]=LeftBorder[CCD_V-1]-1;
          RightBorder[CCD_V]=RightBorder[5];
        }
        else if(LeftBorder[CCD_V]>=145)
          LeftBorder[CCD_V]=LeftBorder[CCD_V-1];
        else;
      }
      
      else if(CenterLine[8]+CenterLine[7]-CenterLine[1]-CenterLine[0]>0)
      {
        if(RightBorder[CCD_V] - RightBorder[CCD_V-1]<= -3)
        {
          RightBorder[CCD_V]=RightBorder[CCD_V-1]+1;
          LeftBorder[CCD_V]=LeftBorder[5];
        }
        else if(RightBorder[CCD_V]<= 5) 
          RightBorder[CCD_V]=RightBorder[CCD_V-1];
        else;
      }
      CenterLine[CCD_V]=(LeftBorder[CCD_V]+RightBorder[CCD_V])/2; 
    }
    
    for(CCD_V=49;CCD_V>top_line;CCD_V--)
      {
        for(CCD_H=CenterLine[top_line];CCD_H>RightBorder[top_line];CCD_H--)
        {
          if(Bina_data[CCD_V][CCD_H+1]==1  && Bina_data[CCD_V][CCD_H]==1) 
          {
           if(Bina_data[CCD_V][CCD_H-1]==0 && Bina_data[CCD_V][CCD_H-2]==0)
            {
              RightBorder[CCD_V]=CCD_H;
              break;
            }
          }
         else     RightBorder[CCD_V]=RightBorder[top_line];  
        }
        for(CCD_H=CenterLine[top_line];CCD_H<LeftBorder[top_line];CCD_H++)
        {
          if(Bina_data[CCD_V][CCD_H+1]==1  && Bina_data[CCD_V][CCD_H]==1)
          {
            if(Bina_data[CCD_V][CCD_H+3]==0 && Bina_data[CCD_V][CCD_H+2]==0)
            {
              LeftBorder[CCD_V]=CCD_H;
              break;
            }
          }
                else     LeftBorder[CCD_V]=LeftBorder[top_line];   
        }
        CenterLine[CCD_V]=(LeftBorder[CCD_V]+RightBorder[CCD_V])/2;  
      }
    
   for(CCD_V=48;CCD_V>=top_line;CCD_V--)
   {
    if((CenterLine[CCD_V]-CenterLine[CCD_V-1])>center_error || (CenterLine[CCD_V-1]-CenterLine[CCD_V])>center_error)
    {
      if((CenterLine[CCD_V]-CenterLine[CCD_V-2])>center_error || (CenterLine[CCD_V-2]-CenterLine[CCD_V])>center_error)
      {
        top_top_line=CCD_V;
        break;
      }
      else ;
    }     
   }
   
   //中心线修正
   
  }
}


  
//=====================================
//||                                 ||
//||          十字道处理             ||
//||                                 ||
//===================================== 
/* crossJudge()
{
    char i,j;
    char left_tiaobian=0;
    char right_tiaobian=0;
    char left_guaidian=0;
    char right_guaidian=0; 
    char left_tiaobian_row=0;
    char right_tiaobian_row=0;
    char left_guaidian_row=0;
    char right_guaidian_row=0;
    
    
    if(count>=5000 && count<=5800 && right_line>=10 && left_line>=10)  //十字道判断条件1，整场黑点数2，白条宽度  3，斜十字能否准确判断！
    {
  
       //==========十字道处理==========  //搜索左下边沿
       for(i=1;i<V;i++) //注意不要让数组溢出
       {
          //====左边线处理==
             //搜索拐点  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&7?
             //左右要写的一样
          if(LeftBorder[i]>LeftBorder[i-1] || LeftBorder[i]>140)  //抓住十字边界的延生特点左突变
              {
		left_guaidian= LeftBorder[i-3];
                left_guaidian_row=i-3;
                break;
              }
         
       }
          //搜索跳变点  //搜索左上边沿
       for(i=1;i<V;i++) 
       {    
         if(LeftBorder[i]-LeftBorder[i-1]<-10)  
		 {
             
              left_tiaobian= LeftBorder[i+2];
              left_tiaobian_row=i+2;
              break;
          }
       }
            //搜索完后再补线。 
         //边线补线
         if((left_guaidian!=0)&&(left_tiaobian!=0))   //通过直线去补线效果更好
            {
               for(i=left_guaidian_row;i<=left_tiaobian_row;i++) 
                  LeftBorder[i]=(char)((left_tiaobian- left_guaidian)*1.0/( left_tiaobian_row-left_guaidian_row)*(i-left_guaidian_row)+0.5)+ left_guaidian;
                 
                
             }
       
      //====右边线处理==
         //搜索拐点
        for(i=1;i<V;i++) //注意不要让数组溢出
       {
          //====右边线处理==
             //搜索拐点  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&7?
             //左右要写的一样
          if(RighttBorder[i]<RightBorder[i-1] || RightBorder[i]<10)  //抓住十字边界的延生特点左突变
             {
				right_guaidian= RightBorder[i-3];
                right_guaidian_row=i-3;
                break;
              }
         
       }
          //搜索跳变点  //搜索右上边沿
       for(i=1;i<V;i++) 
       {    
         if( RightBorder[i]- RightBorderr[i-1]>10)  
		 {
             
              right_tiaobian= RightBorder[i+2];
              right_tiaobian_row=i+2;
              break;
          }
       }
        //边线补线
       if((right_guaidian!=0)&&(right_tiaobian!=0))  
         {
               for(i=right_guaidian_row;i<right_tiaobian_row-1;i++) 
                  RightBorder[i]=(char)(( right_tiaobian-right_guaidian)*1.0/( right_tiaobian_row- right_guaidian_row)*(i-right_guaidian_row)+0.5)+right_guaidian;
         }
            
         
       
	}
		  //=================中心线重提取============
			  	 
       
			  for(i=0;i<V;i++) 
	
			  CenterLine[i]=(LeftBorder[i]+RightBorder[i])/2;
			 
}*/


//小S
void S_road()
{
  long center_sum=0;
  //long left_sum=0,right_sum=0;
 // unsigned char left_ave=0,right_ave=0;
  if(count>5800)
  {
    for(CCD_V=5;CCD_V<=40;CCD_V++)
    {
      if(fabsf((LeftBorder[CCD_V]-LeftBorder[CCD_V+1])>=3))    LeftBorder[CCD_V]=LeftBorder[CCD_V-1];
      if(fabsf(RightBorder[CCD_V]-RightBorder[CCD_V+1])>=3)    RightBorder[CCD_V]=RightBorder[CCD_V-1];
      CenterLine[CCD_V]=(LeftBorder[CCD_V]+RightBorder[CCD_V])/2;
      center_sum+=CenterLine[CCD_V];
      //right_sum+=RightBorder[CCD_V];
      //left_sum+=LeftBorder[CCD_V];
    }
    //left_ave=left_sum/19;
    //right_ave=right_sum/19;
    
    
    for(CCD_V=5;CCD_V<=44;CCD_V++)
    {
      CenterLine[CCD_V]=center_sum/35;
     /* if((right_line-left_line)>5)
        CenterLine[CCD_V]=CenterLine[CCD_V]-10;
      if((left_line-right_line)>5)
        CenterLine[CCD_V]=CenterLine[CCD_V]+10;*/
    }
      
  }
  
}

//发送图像到上位机
void send_photo()
{     
  int i=0,j=0;
  //以下四句话是固定格式，是串口和上位机软件之间的协议
  uart_putchar(UART2,0);
  uart_putchar(UART2,255);
  uart_putchar(UART2,1);
  uart_putchar(UART2,0);
  for(i=V-1;i>=0;i--)
    for(j=H-1;j>=0;j--)
    {
      if(j==CenterLine[i])     uart_putchar(UART2,0);              //中心线
      else if(j==RightBorder[i])     uart_putchar(UART2,0);
      else if(j==LeftBorder[i])     uart_putchar(UART2,0);
      else                     uart_putchar(UART2,Bina_data[i][j]);
    }
}

void SendImg()
{
  disable_irq(89);
  disable_irq(90);                     //关闭场中断,防止串口发送图像数据进程被打乱
  if(gpio_get(PORTA,11)==0)    thread();
  else     threshold=135;
  BinaData();
  
 /* if(gpio_get(PORTA,11)==0)            //发送二值化数据
  {
    AllFilt();                         //滤波
    get_center();   //远处中心提取
    line_count();
    TopGet();     
    centeradjust();
    //顶端行判断
    
    CrossJudge();
    
    S_road();
    send_photo();
  }*/
  
  enable_irq(89);                     //打开场中断,开始读图
  enable_irq(90);        
      
}

char get_thread()
{
 // unsigned char n=0;
  unsigned char num[2];
  unsigned int gray[26]={0};

  for(CCD_V=0;CCD_V<V;CCD_V++)
    for(CCD_H=0;CCD_H<H;CCD_H++)
       gray[Pix_data[CCD_V][CCD_H]/10]++;
  for(CCD_V=25;CCD_V>0;CCD_V--)
  {
    if(gray[CCD_V]>=330)
    {
      num[0]=CCD_V;
      break;
    }
  }
  for(CCD_V=0;CCD_V<26;CCD_V++)
  {
    if(gray[CCD_V]>=330)
    {
      num[1]=CCD_V;
      break;
    }
  }
  return(10*(num[0]+num[1])/2+2);
}

void thread()
{
  if(thread_count<20)
  {
    THRE[thread_count]=get_thread();
    thre_sum=thre_sum+THRE[thread_count];
    thread_count++;
  }
  if(thread_count==20)
  {
    threshold=thre_sum/20+28;
  }
}




