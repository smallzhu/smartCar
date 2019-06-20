
#include "common.h"
#include "include.h"


#define MOTOR1_IO   PTA5
#define MOTOR2_IO   PTD5
#define MOTOR3_IO   PTD5
#define MOTOR4_IO   PTA6

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH2
#define MOTOR2_PWM  FTM_CH5
#define MOTOR3_PWM  FTM_CH6
#define MOTOR4_PWM  FTM_CH3

#define MOTOR1_PWM_IO  FTM0_CH2_PIN
#define MOTOR2_PWM_IO  FTM0_CH5_PIN
#define MOTOR3_PWM_IO  FTM0_CH6_PIN
#define MOTOR4_PWM_IO  FTM0_CH3_PIN

#define MOTOR_HZ    (1000)

struct PID{
float SetSpeed;
float ActualSpeed;
float Err;
float Err_Last;
float Kp,Ki,Kd;
float Voltage;
float Integral;
}*MOTOR_1,*MOTOR_2;

struct VAR_t
{
  int16 cam_hight;
  float maxZanKong;//130
  int16 threasholds;  //0x57
  int16 last_threasholds;
}var_t;


int16 LeftLine[60]; 
int16 RightLine[60]; 
int16 MidLine[60]; 
int16 MidLineP = 39;
int8 LeftLineFlag[60];
int8 RightLineFlag[60];

int16 Topoint; 
int16 Leftflog; 
int16 Rightflog; 
int16 leftpoint1; 
int16 rightpoint1;
float error[2];
float speed_1,speed_2;
//float zhankongbi_1=0,zhankongbi_2=0;


int16 zhicha[60]={0,0,1,1,1,2,2,3,3,3,3,4,4,4,4,5,5,5,6,7,
                  8,9,10,11,11,12,12,13,14,14,15,15,16,16,17,17,18,18,19,20,
                  20,21,21,22,23,23,24,24,25,26,26,27,27,28,29,29,30,30,31,32};
//原码缺失
int16 TopointFlag;
int16 leftpoint2=0;  // leftpoint1 代表flag 2代表行号
int16 rightpoint2=0;
int16 L;
int16 R;
int16 Left1,Left2,Right1,Right2;

//按键
int UI_POS = 2;
int UI_choose_flag = 0;

Site_t site;
Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
Size_t size;                   //显示区域图像大小

uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
uint8 img[CAMERA_H][CAMERA_W]; 

flash_data_t data;

void VAR_init();

void PORTA_IRQHandler();
void DMA0_IRQHandler();

void PORTD_IRQHandler(void);        //PORTD端口中断服务函数
void key_handler(void);             //按键按下的测试中断服务函数

void xunxian1(void);
void xunxian2(void);
void angle_PID(void);
void PID_Init(struct PID *MOTOR_1);
float PID_Cal(float Speed,struct PID *PIDprt);

void main()
{
    MOTOR_1 = (struct PID*)malloc(sizeof(struct PID));
    MOTOR_2 = (struct PID*)malloc(sizeof(struct PID));
    PID_Init(MOTOR_1);
    PID_Init(MOTOR_2);
    
    gpio_init(PTD9,GPO,1);
    
    VAR_init();  //定义会修改变量的初始值;
    
    data.sectornum_start    = FLASH_SECTOR_NUM - 3;     //起始扇区      用最后的3个扇区来作为保存参数
    data.sectornum_end    = FLASH_SECTOR_NUM - 1;       //结束扇区
    
    data.data_addr      = &var_t;                          //数据的地址
    data.data_size      = sizeof(var_t);                  //数据的大小
    
    flash_data_init(&data);
    flash_data_load(&data);
    
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,100,MOTOR1_PWM_IO);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,100,MOTOR2_PWM_IO);      //初始化 电机 PWM
    lcd_init(GBLUE);

    size.H=CAMERA_H;
    size.W=CAMERA_W;
    
    camera_init(imgbuff);
    //配置中断服务函数
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置LPTMR的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_DMA16_VECTORn , DMA0_IRQHandler); 
    
    key_init(KEY_U);
    key_init(KEY_D);
    key_init(KEY_L);
    key_init(KEY_R);
    
    port_init(PTD13, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTD7 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    set_vector_handler(PORTD_VECTORn ,PORTD_IRQHandler);    //设置PORTE的中断服务函数为 PORTE_IRQHandler
    enable_irq (PORTD_IRQn);                                //使能PORTE中断
    
    while(1)
    {
        camera_get_img();                                   //摄像头获取图像
        img_extract(img,imgbuff,CAMERA_SIZE);//解压为二维数组
        site.x = 0;site.y=0;
        lcd_img_binary_z(site, size, imgbuff, imgsize,BLACK,WHITE);
        xunxian1(); 
        xunxian2();
       angle_PID();
    }
}
void VAR_init()
{
  var_t.cam_hight = 27;
  var_t.maxZanKong = 130;
  var_t.threasholds = 0x57;
  var_t.last_threasholds = 0x57;
}
void xunxian1(void)
{
  int16 i,j,k;
  for(i=59;i>0;i=i-1)
  {
    j=MidLineP;
    k=MidLineP;

    LeftLineFlag[i]=0;
    RightLineFlag[i]=0;
    do
    {
      if(img[i][j]==0xff)
      {
        if(img[i][j-1]==0xff)
        {
          if(img[i][j-2]==0x00)
          {
            if(img[i][j-3]==0x00)
            {
              LeftLine[i]=j-1;
              LeftLineFlag[i]=1;
              //img[i][k+1]=150;  //用于显示边线？
              /*showLeftLine.x = LeftLine[i];
              showLeftLine.y = i;
              lcd_point(showLeftLine,RED);*/
              break;
            }
          }
        }
      }
      j=j-1;
    }while(j>2);

    do
    {
      if(img[i][k]==0xff)
      {
        if(img[i][k+1]==0xff)
        {
          if(img[i][k+2]==0x00)
          {
            if(img[i][k+3]==0x00)
            {
              RightLine[i]=k+1;
              RightLineFlag[i]=1;
              //img[i][k+1]=150;  //用于显示边线？
              site.x = RightLine[i];
              site.y = i;
              lcd_point(site,BLUE);
              break;
            }
          }
        }
      }
      k=k+1;
    }while(k<79);

    if(LeftLineFlag[i]==0&&RightLineFlag[i]==1)//左边线没有找到
    {
      LeftLine[i]=0;
      MidLine[i]=(LeftLine[i]+RightLine[i])/2;
      if(RightLine[i]<=RightLine[i+1]&&LeftLineFlag[i+10]==0&&RightLineFlag[i+10]==0)
      {
        MidLine[i]=MidLine[i+1];
      }
    }

    if(LeftLineFlag[i]==1&&RightLineFlag[i]==0)//右边线没有找到
    {
      RightLine[i]=79;
      MidLine[i]=(LeftLine[i]+RightLine[i])/2;
      if(LeftLine[i]>=LeftLine[i+1]&&LeftLineFlag[i+10]==0&&RightLineFlag[i+10]==0)
      {
        MidLine[i]=MidLine[i+1];
        
      }
    }

    if(LeftLineFlag[i]==0&&RightLineFlag[i]==0)//左右边线都没有找到
    {
      LeftLine[i]=0;
      RightLine[i]=79;
      if(i==59)MidLine[i]=39;
      else MidLine[i]= MidLine[i+1];
    }

    if(RightLineFlag[i]==1&&LeftLineFlag[i]==1)//左右边线都找到
    {
      MidLine[i]=(LeftLine[i]+RightLine[i])/2;
    }
    MidLineP=MidLine[i];//重新切换采集行中点
    /*showMidLine.x = MidLine[i];
      showMidLine.y = i;
      lcd_point(showMidLine,RED);*/
    site.x = 0;
    site.y = 80;
    lcd_num_c(site,MidLine[i],RED,GBLUE);
  }
}
//判断左边线是否有丢线
int16 left(int16 a)
{
  int16 i,Flag=1;
  for(i=a;i>Topoint;i--)
  {
    if(LeftLineFlag[i]==0)
    {
      Flag=0;
      break;
    }
  }
  return Flag;
}
//判断右边线是否有丢线
int16 right(int16 a)
{
  int16 i,Flag=1;
  for(i=a;i>Topoint;i--)
  {
    if(RightLineFlag[i]==0)
    {
      Flag=0;
      break;
    }
  }
  return Flag;
}
//扫描边线全在
void bianxian(void)
{
  int16 i,Flag=1;
  for(i=Left2+2;i>=Left2-10;i--)
  {
    if(LeftLineFlag[i]==0||RightLineFlag[i]==0)
    {
      Flag=0;
      break;
    }
  }
  if(Flag==1)
  {
    Leftflog=0;
  }
  Flag=1;
  for(i=Right2+2;i>=Right2-10;i--)
  {
    if(LeftLineFlag[i]==0||RightLineFlag[i]==0)
    {
      Flag=0;
      break;
    }
  }
  if(Flag==1)
  {
    Rightflog=0;
  }
}
//左弯2
void L_2(void)
{
  int16 Flag=1,i=0;
  if(Topoint<45)
  {
    for(i=50;i>Topoint+5;i--)
    {
      if(LeftLineFlag[i]!=0||RightLineFlag[i]==0)
      {
        Flag=0;
        break;
      }
    }
    if(Flag==1&&leftpoint1==0&&rightpoint1==0) 
    {
      leftpoint1=2; 
    }
  }
}
//右弯2
void R_2(void)
{
  int16 Flag=1,i=0;
  if(Topoint<45)
  {
    for(i=50;i>Topoint+5;i--)
    {
      if(LeftLineFlag[i]==0||RightLineFlag[i]!=0)
      {
        Flag=0;
        break;
      }
    }
    if(Flag==1&&leftpoint1==0&&rightpoint1==0) 
    {
      rightpoint1=2; 
    }
  }
}

int16 max(int16 l,int16 r)
{
  if(l>r) return l;
  if(l<r) return r;
  return l;
}

//赛道寻线2
void xunxian2(void)
{
  int16 i;
  Topoint=0;//顶点初始值
  TopointFlag=0; 
  Leftflog=0;
  Rightflog=0;
  leftpoint1=0;
  rightpoint1=0;
  leftpoint2=0;
  rightpoint2=0;
  
  for(i=50;i>2;i=i-1)
  {
    if(TopointFlag==0)//顶点
    {
      //用定点来判定是否出线  连续三行都是
      if(img[i-1][MidLine[i]]==0x00||img[i-2][MidLine[i]]==0x00||img[i-3][MidLine[i]]==0x00)
      {
        Topoint=i;
        TopointFlag=1;
      }
    }
    else
    {
      break;
    }
  }
  for(i=29;i>0;i=i-1)
  {
    if(TopointFlag==0)//顶点
    {
      if(MidLine[i]<5||MidLine[i]>75)
      {
        Topoint=i;
        TopointFlag=1;
      }
    }
    else
    {
      break;
    }
  }
  
  if(Topoint!=0) //代表中线往图像两边偏出
  {
    for(i=55;i>Topoint;i--)
    {
      if(LeftLineFlag[i]==0)//左边线没有找到
      {
        if(leftpoint1==0&&rightpoint1==0&&i<59&&LeftLineFlag[i+1]==1&&LeftLineFlag[i+2]==1&&LeftLineFlag[i-1]==0)//是否有左边线断点
        {
          leftpoint1=1;//打左标志点
          leftpoint2=i;
          L=Topoint;
          for(i=leftpoint2-10;i>Topoint;i--)
          {
            if((LeftLineFlag[i]==1&&RightLineFlag[i]==1&&RightLine[i]-RightLine[i+1]>0)||RightLineFlag[i]==0)//右弯 或者右边有丢线
            {
              leftpoint1=2;
              break;
            }
          }
        }
      }
      if(RightLineFlag[i]==0)//右边线没有找到
      {
        if(leftpoint1==0&&rightpoint1==0&&i<59&&RightLineFlag[i+1]==1&&RightLineFlag[i+2]==1&&RightLineFlag[i-1]==0)//是否有右边线断点
        {
          rightpoint1=1;//打右标志点
          rightpoint2=i;
          R=Topoint;
          for(i=rightpoint2-10;i>Topoint;i--)
          {
            if((LeftLineFlag[i]==1&&RightLineFlag[i]==1&&LeftLine[i]-LeftLine[i+1]<0)||LeftLineFlag[i]==0)//左弯 或者左边有丢线
            {
              rightpoint1=2;
              break;
            }
          }
        }
      }
    }
    L_2();
    R_2();
  }
  
  for(i=50;i>Topoint+2;i=i-1)//拐点的标定
  {
    if(Leftflog==0)
    {
      //往左偏，是右凸
      if(LeftLine[i]-LeftLine[i+3]>=0&&LeftLine[i]-LeftLine[i-3]>=0&&LeftLine[i-3]<LeftLine[i+3]&&((LeftLine[i]-LeftLine[i+3])+(LeftLine[i]-LeftLine[i-3])>6)) 
      {
        Left1=LeftLine[i+1];//左拐点   //Left1 代表列，Left2代表行
        Left2=i+1; 
        Leftflog=1;//打左拐点标志点
        
        if(Left1>70)   //LeftLine[i+1] >70
        {
          Leftflog=0;
          rightpoint1=1;
        }
        else if(right(Left2))//右边线没有丢线  有丢线返回0 无丢线返回1
        {
          Leftflog=0;
        }
        else
        {
          leftpoint1=0; 
          rightpoint1=0;
        }
      }
    }
    if(Rightflog==0)
    {
      if(RightLine[i+3]-RightLine[i]>=0&&RightLine[i-3]-RightLine[i]>=0&&RightLine[i-3]>RightLine[i+3]&&RightLine[i+3]-RightLine[i]+RightLine[i-3]-RightLine[i]>6)//先找到一个右拐点
      {
        Right1=RightLine[i+1];//右拐点   Right1 代表列   Right2  代表行
        Right2=i+1; 
        Rightflog=1;//打右拐点标志点
        
        if(Right1<10)   //RightLine[i+] <10
        {
          Rightflog=0;
          leftpoint1=1;
        }
        else if(left(Right2))//左边没有丢线 有丢线是0 无丢线是1
        {
          Rightflog=0;
        }
        else
        {
          leftpoint1=0; 
          rightpoint1=0; 
        }
      }
    }
  }
  if(Leftflog==1&&Rightflog==1)  //左右无丢线  且都找到拐点
  {
    if((Left2>Right2&&Left1>Right1)||Left2-Right2>40)   //Left : i+1   > Right  : i+1 且两边相差40
    {
      Rightflog=0;
    }
    if((Right2>Left2&&Left1>Right1)||Right2-Left2>40)
    {
      Leftflog=0;
    }
  }
  
  bianxian();  
  if(Leftflog==1&&Rightflog==1)//左右边线都找到拐点
  {
    for(i=Left2;i>=Topoint;i--)
    {
      LeftLineFlag[i]=1;
      LeftLine[i]=(int16)(LeftLine[Left2]+(Left2-i)*(LeftLine[Left2]-LeftLine[Left2+8])/8.0);
      if(LeftLine[i]<0)LeftLine[i]=0;
      if(LeftLine[i]>79)LeftLine[i]=79;
    }
    for(i=Right2;i>=Topoint;i--)
    {
      RightLineFlag[i]=1;
      RightLine[i]=(int16)(RightLine[Right2]+(Right2-i)*(RightLine[Right2]-RightLine[Right2+8])/8.0);
      if(RightLine[i]<0)RightLine[i]=0;
      if(RightLine[i]>79)RightLine[i]=79;
    }
    for(i=max(Left2,Right2);i>=Topoint;i--)
    {
      MidLine[i]=(LeftLine[i]+RightLine[i])/2;
      //这段是有丢线才显示
      //显示补线
      site.x = MidLine[i];
      site.y = i;
      lcd_point(site,RED);
      
      site.x = RightLine[i];
      site.y = i;
      lcd_point(site,BLUE);
      
      site.x = LeftLine[i];
      site.y = i;
      lcd_point(site,RED);
    }
  }
  if(Leftflog==1&&Rightflog==0)//只有左拐点
  {
    for(i=Left2;i>=Topoint;i--)
    {
      LeftLineFlag[i]=1;
      RightLineFlag[i]=1;
      LeftLine[i]=(int16)(LeftLine[Left2]+(Left2-i)*(LeftLine[Left2]-LeftLine[Left2+8])/8.0);
      if(LeftLine[i]<0)LeftLine[i]=0;//限幅
      if(LeftLine[i]>79)LeftLine[i]=79;
    }
    for(i=59;i>=Topoint;i--)
    {
      MidLine[i]=LeftLine[i]+(int16)(1.3*(float)(zhicha[i]));
      if(MidLine[i]>79)MidLine[i]=79;//限幅
      //这部分都是有丢线才显示
      site.x = MidLine[i];
      site.y = i;
      lcd_point(site,RED);
      
      site.x = RightLine[i];
      site.y = i;
      lcd_point(site,BLUE);
      
      site.x = LeftLine[i];
      site.y = i;
      lcd_point(site,RED);
      //img[i][LeftLine[i]]=0; 
      //img[i][MidLine[i]]=0;
    }
  }
  if(Leftflog==0&&Rightflog==1)//只有右拐点
  {
    for(i=Right2;i>=Topoint;i--)
    {
      LeftLineFlag[i]=1;
      RightLineFlag[i]=1;
      RightLine[i]=(int16)(RightLine[Right2]+(Right2-i)*(RightLine[Right2]-RightLine[Right2+8])/8.0);
      if(RightLine[i]<0)RightLine[i]=0;
      if(RightLine[i]>79)RightLine[i]=79;
    }
    for(i=59;i>=Topoint;i--)
    {
      MidLine[i]=RightLine[i]-(int16)(1.3*(float)(zhicha[i]));
      
      if(MidLine[i]<0)MidLine[i]=0;
      //有丢线才显示
      site.x = MidLine[i];
      site.y = i;
      lcd_point(site,RED);
      
      site.x = RightLine[i];
      site.y = i;
      lcd_point(site,BLUE);
      
      site.x = LeftLine[i];
      site.y = i;
      lcd_point(site,RED);
      //img[i][RightLine[i]]=0; 
      //img[i][MidLine[i]]=0; 
    }
  }
  
  if(leftpoint1==1)//过左弯
  {
    for(i=L;i>0;i--)//处理图像外部的左右中线
    {
      LeftLine[i]=0;
      LeftLineFlag[i]=1;
      RightLine[i]=0;
      RightLineFlag[i]=1;
      MidLine[i]=0;
    }
    for(i=leftpoint2;i>L;i--)
    {
      if(RightLine[i]==79)
      {
        MidLine[i]=MidLine[i+1]-1;
      }
      else
      {
        MidLine[i]=MidLine[i+1]-(RightLine[i+1]-RightLine[i]); 
      }
      if(MidLine[i]<0)MidLine[i]=0;
    }
  }
  if(leftpoint1==2)//过左弯
  {
    for(i=59;i>0;i--)
    {
      MidLine[i]=RightLine[i]-40;
      if(MidLine[i]<0)
      {
        MidLine[i]=0;
        L=i;
        break;
      }
    }
    for(i=L;i>0;i--)//处理图像外部的左右中线
    {
      LeftLine[i]=0;
      LeftLineFlag[i]=1;
      RightLine[i]=0;
      RightLineFlag[i]=1;
      MidLine[i]=0;
    }
  }
  
  if(rightpoint1==1)//过右弯
  {
    for(i=R;i>0;i--)//处理图像外部的左右中线
    {
      LeftLine[i]=79;
      LeftLineFlag[i]=1;
      RightLine[i]=79;
      RightLineFlag[i]=1;
      MidLine[i]=79;
    }
    for(i=rightpoint2;i>R;i--)
    {
      if(LeftLine[i]==0)
      {
        MidLine[i]=MidLine[i+1]+1;
      }
      else
      {
          MidLine[i]=MidLine[i+1]+(LeftLine[i]-LeftLine[i+1]);
      }
      if(MidLine[i]>79)MidLine[i]=79;
    }
  }
  if(rightpoint1==2)//过右弯
  {
    for(i=59;i>0;i--)
    {
      MidLine[i]=LeftLine[i]+40;
      if(MidLine[i]>79)
      {
        MidLine[i]=79;
        R=i;
        break;
      }
    }
    for(i=R;i>0;i--)//处理图像外部的左右中线
    {
      LeftLine[i]=79;
      LeftLineFlag[i]=1;
      RightLine[i]=79;
      RightLineFlag[i]=1;
      MidLine[i]=79;
    }
  }

  /*for(i=78;i>1;i=i-1)//再次寻找顶点Top
  {
    if(TopointFlag==0)
    {
      if(((img[i-1][MidLine[i]])==0x00&&MidLine[i-1]!=MidLine[i])||MidLine[i]==0||MidLine[i]==79)
      {
        Topoint=i;
        TopointFlag=1;
      }
    }
    else
    {
      break;
    }
  }*/
  
  site.x = 80;
  site.y = 0;
  lcd_str(site,"Topint",RED,GBLUE);
  site.y = 20;
  lcd_num(site,Topoint,RED,GBLUE);
  
  site.x = MidLine[i];
  site.y = i;
  lcd_point(site,RED);
      
  site.x = RightLine[i];
  site.y = i;
  lcd_point(site,BLUE);
  
  site.x = LeftLine[i];
  site.y = i;
  lcd_point(site,RED);
     
  MidLineP=MidLine[59];//重新切换采集场中点
  //使用Topint来修改cam_height.
  
}

void PID_Init(struct PID *MOTOR)
{
  printf("PID_Init begin! \n");
  MOTOR->SetSpeed = 0;
  MOTOR->ActualSpeed = 0;
  MOTOR->Err = 0;
  MOTOR->Err_Last = 0;
  MOTOR->Kp = 0.2;
  MOTOR->Ki = 0.2; //增大了积分系数
  MOTOR->Kd = 0.2;
  MOTOR->Voltage = 0;
  MOTOR->Integral = 0;
  printf("PID_Init end! \n");
}

float myabs(float number)
{
    if(number<0) return -number;
    else return number;
}

float PID_Cal(float Speed,struct PID *PIDprt)
{
    unsigned char index;
    float temp;
    PIDprt->SetSpeed = Speed;
    PIDprt->Err = PIDprt->SetSpeed - PIDprt->ActualSpeed;
    temp = myabs(PIDprt->Err);
    if(temp>200) //变积分处理控制
    {
    index = 0;
    }
    else if(temp<180)
    {
    index =1;
    PIDprt->Integral += PIDprt->Err;
    }
    else
    {
    index = (200-temp)/20;
    PIDprt->Integral += PIDprt->Err;
    }
    PIDprt->Voltage = (PIDprt->Kp)*(PIDprt->Err)+index*(PIDprt->Ki)*(PIDprt->Integral) + PIDprt->Kd*(PIDprt->Err-PIDprt->Err_Last);
    PIDprt->Err_Last = PIDprt->Err;
    PIDprt->ActualSpeed = PIDprt->Voltage*1.0;
    return PIDprt->ActualSpeed;
}

void angle_PID(void)
{
  if(Topoint>var_t.cam_hight)
  {
    error[0]=(3*(MidLine[var_t.cam_hight]-40)+2*(MidLine[var_t.cam_hight+1]-40)+(MidLine[var_t.cam_hight+2]-40))/6;
  }
  else if(Topoint<=var_t.cam_hight)
  {
    error[0]=(3*(MidLine[Topoint]-40)+2*(MidLine[Topoint+1]-40)+(MidLine[Topoint+2]-40))/6;
  }
          
  if(error[0]>0)
  {
     if(error[0]<20)
    {
      speed_1 = PID_Cal(var_t.maxZanKong-6*error[0],MOTOR_1);
      speed_2 = PID_Cal(var_t.maxZanKong,MOTOR_2);
    }
    else if(error[0]>=20&&error[0]<30)
    {
      speed_1 = PID_Cal(var_t.maxZanKong-5*error[0],MOTOR_1);
      speed_2 = PID_Cal(var_t.maxZanKong,MOTOR_2);
    }
    else if(error[0]>=30&&error[0]<40)
    {     
      speed_1 = PID_Cal(var_t.maxZanKong-4*error[0],MOTOR_1);
      speed_2 = PID_Cal(var_t.maxZanKong,MOTOR_2);
    }
  }
   else
   {
     if(error[0]>-20)
     {
      speed_1 = PID_Cal(var_t.maxZanKong,MOTOR_1);
      speed_2 = PID_Cal(var_t.maxZanKong-6*(-error[0]),MOTOR_2);
    }
    else if(error[0]>-30&&error[0]<=-20)
    {
      speed_1 = PID_Cal(var_t.maxZanKong,MOTOR_1);
      speed_2 = PID_Cal(var_t.maxZanKong-5*(-error[0]),MOTOR_2);
    }
    else if(error[0]>-40&&error[0]<=-30)
    {
      speed_1 = PID_Cal(var_t.maxZanKong,MOTOR_1);
      speed_2 = PID_Cal(var_t.maxZanKong-4*(-error[0]),MOTOR_2);//speed_2是右边的电机，speed_1是左边的电机
    }

   }
  if(Topoint<50)
  {
    if(speed_1>var_t.maxZanKong) speed_1 = var_t.maxZanKong;
    if(speed_1<0) speed_1 = 1;
    if(speed_2>var_t.maxZanKong) speed_2 = var_t.maxZanKong;
    if(speed_2<0)speed_2 = 1;
  }
  else if(Topoint>=50) 
  {
    speed_1 = 0;
    speed_2 = 0;
  }
  
  ftm_pwm_duty(MOTOR_FTM,MOTOR1_PWM,speed_1);
  ftm_pwm_duty(MOTOR_FTM,MOTOR2_PWM,speed_2);
  site.x = 0;
  site.y = 100;
  lcd_str(site,"l",RED,GBLUE);
  site.x += 20;
  lcd_num_c(site,(int)speed_1,RED,GBLUE);
  site.x += 30;
  lcd_str(site,"r",RED,GBLUE);
  site.x += 20;
  lcd_num_c(site,(int)speed_2,RED,GBLUE);
  site.y = 60;
  site.x = 0;
  lcd_str(site,"error:",RED,GBLUE);
  site.x = 50;
  lcd_num_c(site,error[0],RED,GBLUE);
}

void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}


//按键中断服务函数
void PORTD_IRQHandler(void)
{

#if 0       // 条件编译，两种方法可供选择

    uint8  n = 0;    //引脚号
    n = 13;
    if(PORTD_ISFR & (1 << n))           //PTD7 触发中断
    {
        PORTD_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */

        key_handler();

        /*  以上为用户任务  */
    }
#else
    PORT_FUNC(D,13,key_handler);
#endif
}

void key_handler(void)
{
  UI_POS = 1;
  UI_choose_flag = 0;
  lcd_clear(GBLUE);
    while(UI_POS != 0)
    {
      if(key_check(KEY_U) == KEY_DOWN&&UI_POS ==1)
      {
        UI_choose_flag--;
        if( UI_choose_flag < 0) UI_choose_flag = 5;
        gpio_set(PTD9,1);
        DELAY_MS(100);  
        gpio_set(PTD9,0);
        DELAY_MS(100); 
        gpio_set(PTD9,1);
      }
      if(key_check(KEY_D) == KEY_DOWN&&UI_POS ==1) //检测key状态（带延时消抖）
      {
          UI_choose_flag++;
          if(UI_choose_flag > 5) UI_choose_flag=5;
        gpio_set(PTD9,1);
        DELAY_MS(100);  
        gpio_set(PTD9,0);
        DELAY_MS(100); 
        gpio_set(PTD9,1);
      }
      if(key_check(KEY_R) == KEY_DOWN&&UI_POS ==1)
      {
        UI_POS++;
        if(UI_POS>2) UI_POS = 2;
        else lcd_clear(GBLUE);
        gpio_set(PTD9,1);
        DELAY_MS(100);  
        gpio_set(PTD9,0);
        DELAY_MS(100); 
        gpio_set(PTD9,1);
      }
      if(key_check(KEY_L) == KEY_DOWN)
      {
        UI_POS--;
        if(UI_POS<0) UI_POS = 0;
        else lcd_clear(GBLUE);
        gpio_set(PTD9,1);
        DELAY_MS(100);  
        gpio_set(PTD9,0);
        DELAY_MS(100); 
        gpio_set(PTD9,1);
      }
      if(key_check(KEY_B) == KEY_DOWN)
      {
        flash_data_save(&data);
        gpio_set(PTD9,1);
        DELAY_MS(50);
        gpio_set(PTD9,0);
        DELAY_MS(50);
        gpio_set(PTD9,1);
        DELAY_MS(50);
        gpio_set(PTD9,0);
        DELAY_MS(50);
        gpio_set(PTD9,1);
      }
      
      
      if(UI_choose_flag == 0)  //摄像头
      {
        if(UI_POS == 1)
        {site.y = 0;site.x = 0;
        lcd_str(site,"show_camera",RED,YELLOW);
        site.y+=20;
        lcd_str(site,"cam_thresholds",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"Kd",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"Ki",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"speed",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"cam_hight",RED,GBLUE);
        }
        else if(UI_POS == 2)
        {
          site.x = 0;site.y = 0;
          lcd_str(site,"show_camera",RED,YELLOW);
        }
        
      }
      else if(UI_choose_flag == 1)  //修改阈值
      {
        if(UI_POS == 1)
        {site.y = 0;site.x = 0;
        lcd_str(site,"show_camera",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"cam_thresholds",RED,YELLOW);
        site.y+=20;
        lcd_str(site,"Kd",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"Ki",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"speed",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"cam_hight",RED,GBLUE);
        }
        else if(UI_POS == 2)
        {
          if(key_check(KEY_U) == KEY_DOWN)
          {
            var_t.threasholds ++;
            SCCB_WriteByte(OV7725_CNST,var_t.threasholds);
            DELAY_MS(200);
          }
          if(key_check(KEY_D) == KEY_DOWN)
          {
            var_t.threasholds --;
            SCCB_WriteByte(OV7725_CNST,var_t.threasholds);
            DELAY_MS(200);
          }
          
          site.y = 0;site.x = 0;
          lcd_str(site,"cam_threasholds:",RED,GBLUE);
          site.y = 20;
          lcd_num(site,var_t.threasholds,RED,GBLUE);
          site.y = 40;
          lcd_str(site,"last modifi:",RED,GBLUE);
          site.y = 60;
          lcd_num(site,var_t.last_threasholds,RED,GBLUE);
        }
      }
      else if(UI_choose_flag == 2)  //KD
      {
        if(UI_POS == 1)
        {
        site.y = 0;site.x = 0;
        lcd_str(site,"show_camera",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"cam_thresholds",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"Kd",RED,YELLOW);
        site.y+=20;
        lcd_str(site,"Ki",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"speed",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"cam_hight",RED,GBLUE);
        }
        else if(UI_POS == 2)
        {
          site.y = 0;site.x = 0;
          lcd_str(site,"Kd",RED,GBLUE);
        }
      }
      else if(UI_choose_flag == 3)  //KI
      {
        if(UI_POS == 1)
        {site.y = 0;site.x = 0;
        lcd_str(site,"show_camera",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"cam_thresholds",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"Kd",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"Ki",RED,YELLOW);
        site.y+=20;
        lcd_str(site,"speed",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"cam_hight",RED,GBLUE);
        }
        else if(UI_POS == 2)
        {
          site.x = 0;site.y = 0;
          lcd_str(site,"Ki config",RED,GBLUE);
        }
      }
      else if(UI_choose_flag == 4)  //SPEED
      {
        if(UI_POS == 1)
        {
        site.y = 0;site.x = 0;
        lcd_str(site,"show_camera",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"cam_thresholds",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"Kd",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"Ki",RED,GBLUE);
        site.y+=20;
        lcd_str(site,"speed",RED,YELLOW);
        site.y+=20;
        lcd_str(site,"cam_hight",RED,GBLUE);
        }
        else if(UI_POS == 2)
        {
          if(key_check(KEY_U) == KEY_DOWN)
          {
            var_t.maxZanKong++;
            DELAY_MS(200);
          }
          if(key_check(KEY_D) == KEY_DOWN)
          {
            var_t.maxZanKong--;
            DELAY_MS(200);
          }
          site.y = 0;site.x = 0;
          lcd_str(site,"speed config",RED,GBLUE);
          site.y = 20;
          lcd_num(site,(int)var_t.maxZanKong,RED,GBLUE);
        }
      }
      else if(UI_choose_flag == 5)   //CAM_HIGHT
      {
        if(UI_POS == 1)
        {
           site.y = 0;site.x = 0;
          lcd_str(site,"show_camera",RED,GBLUE);
          site.y+=20;
          lcd_str(site,"cam_thresholds",RED,GBLUE);
          site.y+=20;
          lcd_str(site,"Kd",RED,GBLUE);
          site.y+=20;
          lcd_str(site,"Ki",RED,GBLUE);
          site.y+=20;
          lcd_str(site,"speed",RED,GBLUE);
          site.y+=20;
          lcd_str(site,"cam_hight",RED,YELLOW);
        }
        else if(UI_POS == 2)
        {
          if(key_check(KEY_U) == KEY_DOWN)
          {
              var_t.cam_hight++;
              if( UI_choose_flag >100) UI_choose_flag = 100;
              DELAY_MS(200);
          }
          if(key_check(KEY_D) == KEY_DOWN) //检测key状态（带延时消抖）
          {
              var_t.cam_hight--;
          if(UI_choose_flag < 5) UI_choose_flag=5;
                DELAY_MS(200);
          }
          site.y = 0;site.x = 0;
          lcd_str(site,"max:100,min:5",RED,GBLUE);
          site.y = 20;
          lcd_num_c(site,var_t.cam_hight,RED,GBLUE);
        }
      }
    }
                   //通过串口助手查看，提示按键按下
    //调整这里的时间，会发现，时间越长，
                                        //快速双击，就没法识别第二次采集
    site.y = 0;site.x = 0;
    lcd_clear(GBLUE);
    
}
