/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�STC16���İ�
����    д��chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2021��1��23��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��IDE��STC16
��Target �� C251Keil5.6������
��SYS PLL�� 30MHzʹ���ڲ�����
=================================================================
STC16���������Ƶ��
����Ƽ�Bվ��ַ��https://space.bilibili.com/95313236
STC16�������زο���Ƶ�� https://www.bilibili.com/video/BV1gy4y1p7T1/
STC16һ����ӽ�����Ƶ�� https://www.bilibili.com/video/BV1Jy4y1e7R4/
=================================================================
����ʱ, ѡ��ʱ�� 30MHZ (�û��������޸�Ƶ��).
STC16F��������:����IRCBND=0��ISP��������Ϊ24M��
Ȼ��IRCBND=0������Ƶ��Ϊ30M��
����IRCBND=1������Ƶ��Ϊ30M������Ϊ׼
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "LQ_dianci.h"
#include "include.h"
#include "LQ_PID.h"

extern pid_param_t Speed_PID;
extern pid_param_t Direc_PID;

long bmq_juli=0;      //����������
long tly_jifen=0;     //�����ǻ���

short leftP=0,leftV=0,rightV=0,rightP=0,zhongjian=0;//���
long TempAngle = 0;           // ��ֵ
short MotorDuty1=0; 				  // ��������ȫ�ֱ���
short MotorDuty2=0;						// �����ҵ��ȫ�ֱ���
short ECPULSE1 = 0;           // �����������ȫ�ֱ��� 
short ECPULSE2 = 0;           // �����ұ�����ȫ�ֱ���

volatile short Target_Speed = 0;          // �ٶ�ȫ�ֱ���

int time = 0;  //ʱ���־            
int chazhi_sancha=0; //��·��ֵ
int chazhi_huandao=0; //������ֵ

int hdcha=0,huandao=0,huandao1=0; //������־λ    

int yxy=0;
int Forkchu=0;//����·��־
int count = 0;//������־                     
int Fork=0,Fork_L=0,Fork_R=0,Forking=0;               // ����·��־
int Round=0,Rounding=0,Round_Out=0,Detection=0;       //������־

int Out_dw=0;//�����⻷
int Out_dn=0;//�����ڻ�
int Out_s=0;//�ٶȻ�

void timer0_int (void) interrupt 1
{
	  ICM_Get_Raw_data(&aacx1,&aacy1,&aacz1,&gyrox1,&gyroy1,&gyroz1);//��ȡ�����ǵ�����
	  //tly_jifen+=gyroz1;

		ECPULSE1 = Read_Encoder(1); 			        // ���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
		ECPULSE2 = Read_Encoder(2); 	          	// �ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ
	
	  InductorNormal();

	if(Go == 1)
	{
		Go=2;
		Round = 0;
		Fork = 0;				
		count = 0;
		Forkchu = 1;
	}
	if(Go==2)
	{
		time++;
	}
	  
		Control();
	
	//TempAngle=(leftV-rightV)*100/(leftV+rightV);	
	if(Round==0&&Fork==0) TempAngle=(leftV-rightV);//����ѭ��
	
	if(time==3)
	{
		Out_dw=dircontrol(TempAngle); //�⻷
		time=0;
	}
		
	  //Out_dw=((int)dircontrol(TempAngle));	
	    Out_dn=diranglecontrol(Out_dw); //�ڻ�

	if((Round ==4)||(Round ==5)&&huandao==0)
	{
		Out_dn=diranglecontrol(TempAngle);//�����ڻ�
	}	
	
	if(Forkchu==1&&yxy==1)
	{
		Target_Speed=22;
	}
	
//  if(Out_d>1000)Out_d=1000;	
//  if(Out_d<-1000)Out_d=-1000;
    Out_s=speedcontrol((int)((ECPULSE1-ECPULSE2)/2),Target_Speed);//�ٶȻ�
//	if(Out_s>150)Out_s=150;
//	if(Out_s<-150)Out_s=-150;

	
    MotorDuty1 =Out_s - Out_dn;//���1���
    MotorDuty2 =Out_s + Out_dn;//���2���

       //����޷�
    /*if(MotorDuty1 > 8000)MotorDuty1 = 8000;else if(MotorDuty1 < -8000)MotorDuty1 = -8000;
    if(Speed_PID.out > 8000)Speed_PID.out = 8000;else if(Speed_PID.out < -8000)Speed_PID.out = -8000;

    if(MotorDuty2 > 8000)MotorDuty2 = 8000;else if(MotorDuty2 < -8000)MotorDuty2 = -8000;
    if(Direc_PID.out > 8000)Direc_PID.out = 8000;else if(Direc_PID.out < -8000)Direc_PID.out = -8000;*/
		
		if(Go == 0||Go == 3)
	{
		MotorDuty1=0;
		MotorDuty2=0;
	}
	
	  MotorCtrl(MotorDuty1,MotorDuty2);
}

 
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������void InductorNormal (void)
����  �ܡ���ų���ʾ����
����  �ߡ�chiusir
�������¡�2021��1��22�� 
������汾��V1.0
/QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned int data diangan[5];
void InductorNormal (void)
{
    diangan[0]=ADC_Read_Average(8,5);    	// ��ȡADC8ͨ��ֵ   P00
    diangan[1]=ADC_Read_Average(9,5);			// ��ȡADC9ͨ��ֵ	  P01
    diangan[2]=ADC_Read_Average(10,5); 		// ��ȡADC10ͨ��ֵ	P02
    diangan[3]=ADC_Read_Average(11,5);		// ��ȡADC11ͨ��ֵ	P03
	  diangan[4]=ADC_Read_Average(0,5);		  // ��ȡADC0ͨ��ֵ	  P10
		
		leftV =  (float)((diangan[0] - 0.0) / (1700.0 - 0.0)) * 100.0;		// ��й�һ��
		leftP =  (float)(diangan[1] - 100.0) / (4095.0 - 100.0) * 100.0;		// ��й�һ��
		rightP = (float)(diangan[2] - 100.0) / (4095.0 - 100.0) * 100.0;		// ��й�һ��
		rightV = (float)((diangan[3] - 0.0) / (1900.0 - 0.0)) * 100.0;    // ��й�һ��	
	  zhongjian = (float)(diangan[4] - 100.0) / (4095.0 - 100.0) * 100.0;
		
	  leftP = (leftP < 0) ? 0 : leftP;            //��һ���������������
		leftP = (leftP > 100) ? 100 : leftP;				//��һ���������������
		rightV = (rightV < 0) ? 0 : rightV;					//��һ���������������
		rightV = (rightV > 100) ? 100 : rightV;			//��һ���������������
	  leftV = (leftV < 0) ? 0 : leftV;						//��һ���������������
		leftV = (leftV > 100) ? 100 : leftV;				//��һ���������������
		rightP = (rightP < 0) ? 0 : rightP;					//��һ���������������
		rightP = (rightP > 100) ? 100 : rightP;			//��һ���������������
    zhongjian = (zhongjian < 0) ? 0 : zhongjian;
		zhongjian = (zhongjian > 100) ? 100 : zhongjian;	
}

void Control(void)
{
	int i;
	if(Forkchu==1)
	{
		TempAngle=(leftV-rightV);
		if(diangan[4]>1600)
		{
			Forkchu=0;
		}
	}	
//����һ
//����һ
//����һ
	if(P32==0&&P33==0)
{
   if(Round==1)
    {
      Discern4();	  //��������		
    }
  else  if(Fork==1)
	  {
		  sancha();     //������
	  }
else  
  {
          // ʶ�𻷵�
       if(((Detection==0)&&(leftV>68)&&(rightV>78)&&(diangan[4]>2100)&&(diangan[1]+diangan[2])>3200)&&Round==0&&Fork==0) 
       {
           Detection=1;				  
				   LED_Ctrl(LED2,ON);
       }
       else if((bmq_juli>5500)&&Detection==1)      
       {
           Round = 1;
       }
			    //ʶ���·
		   //if(diangan[4]<1050&&(diangan[1]+diangan[2])<1650&&(diangan[1]+diangan[2])>1250&&leftV>50&&rightV>50&&Round==0&&Fork==0&&Forkchu==0)
			 if(diangan[4]<1000&&(diangan[1]+diangan[2])<500&&(diangan[1]+diangan[2])>200&&Round==0&&Fork==0&&Forkchu==0)
            {
							  Fork = 1;
								LED_Ctrl(LED2,ON);				
            }
					//����ѭ��Ŀ���ٶ�
			 if(Forkchu==1)
			 {
				 Target_Speed =15;
			 }
      if(Forkchu==0)	
		  {				 
			 if(Round == 1&&Fork == 0)
			 {
				 Target_Speed =25;
			 }
       if(Round == 0&&Fork == 0)
			 {
				 Target_Speed =25;
			 }
		  }
	}
		if(Detection==1)
			{
		    bmq_juli+=(ECPULSE1-ECPULSE2);
			}
	  if(Fork==1&&Forking==0)
			{
		    tly_jifen+=gyroz1;
			}
		if(Round==1)
			{
		    tly_jifen+=gyroz1;
			}
}	
//{
//	if((Round ==4)||(Round ==5))
//    {
//      Discern();	  //��������		
//    }
//  else  if(Fork==1)
//	  {
//		  sancha();     //������
//	  }
//else  
//  {
//          // ʶ�𻷵�		
//		if(((Detection==0)&&(leftV>70)&&(rightV>70)&&(diangan[4]+diangan[3]>3350)&&(diangan[1]+diangan[2])>3100)&&Round==0&&Fork==0)  
//      {
//				Detection=1;
//				hdcha=(diangan[1]-diangan[2]);
//			}			
//		if(hdcha>-50&&Detection==1&&(leftV>rightV))
//		 {
//			Detection=2;
//		 }
//	  if(hdcha<-100&&Detection==1&&(leftV<rightV))
//		 {
//			Detection=3;
//		 }
//		if(Detection==2&&bmq_juli>2000)
//			{
//				Round=5;
//				LED_Ctrl(LED2,ON);
//				Detection=4;
//				bmq_juli=0;
//			}
//		if(Detection==3&&bmq_juli>2000)
//			{
//				Round=4;
//				LED_Ctrl(LED2,ON);
//				Detection=4;
//				bmq_juli=0;
//			}
//			    //ʶ���·
//		 //if(diangan[4]<1050&&(diangan[1]+diangan[2])<1650&&(diangan[1]+diangan[2])>1250&&leftV>50&&rightV>50&&Round==0&&Fork==0&&Forkchu==0)
//			if(diangan[4]<1000&&(diangan[1]+diangan[2])<500&&(diangan[1]+diangan[2])>200&&Round==0&&Fork==0&&Forkchu==0)
//      {
//				Fork = 1;
//				LED_Ctrl(LED2,ON);				
//      }
//					//����ѭ��Ŀ���ٶ�
//     if(Round == 0&&Fork == 0)
//         {
//					if((leftV<5)&&(rightV<5))
//					{
//						Target_Speed =30;
//					}
//					else if((leftV>60)&&(rightV>60))
//					{
//						Target_Speed =30;
//					}
//					else if((leftV>5)&&(leftV<60)&&(rightV<60)&&(rightV>5))
//					{
//						Target_Speed =30;
//					}
//         }
//	}
//		if(Detection==2||Detection==3)
//			{
//		    bmq_juli+=(ECPULSE1-ECPULSE2);
//			}
//	  if(Fork==1&&Forking==0)
//			{
//		    tly_jifen+=gyroz1;
//			}
//		if((Round ==4)||(Round ==5)&&(huandao==0))
//		{
//			bmq_juli+=(ECPULSE1-ECPULSE2);
//			tly_jifen+=gyroz1;
//		}
//}
//������
//������
//������	
		if(P32==1&&P33==0)      
{
	if(Round ==4)
    {
      Discern();	  //��������		
    }
  else  if(Fork==1)
	  {
		  sancha();     //������
	  }
else  
  {
          // ʶ�𻷵�		
		if(((Detection==0)&&(leftV>70)&&(rightV>70)&&(diangan[4]+diangan[3]>3350)&&(diangan[1]+diangan[2])>3100)&&Round==0&&Fork==0)  
      {
				Detection=1;
			}	
		if(Detection==1)
		{
			hdcha=(diangan[1]-diangan[2]);
		}
	  if(hdcha<-100&&Detection==1)
		 {
			Detection=2;
		 }
		if(Detection==2&&bmq_juli>4200)
			{
				Round=4;
				LED_Ctrl(LED2,ON);
				Detection=3;
				bmq_juli=0;
			}
			    //ʶ���·
		 //if(diangan[4]<1050&&(diangan[1]+diangan[2])<1650&&(diangan[1]+diangan[2])>1250&&leftV>50&&rightV>50&&Round==0&&Fork==0&&Forkchu==0)
			if(diangan[4]<1000&&(diangan[1]+diangan[2])<500&&(diangan[1]+diangan[2])>200&&Round==0&&Fork==0&&Forkchu==0)
      {
				Fork = 1;
				LED_Ctrl(LED2,ON);				
      }
					//����ѭ��Ŀ���ٶ�
     if(Round == 0&&Fork == 0)
         {
					if((leftV<5)&&(rightV<5))
					{
						Target_Speed =30;
					}
					else if((leftV>60)&&(rightV>60))
					{
						Target_Speed =30;
					}
					else if((leftV>5)&&(leftV<60)&&(rightV<60)&&(rightV>5))
					{
						Target_Speed =30;
					}
         }
	}
		if(Detection==2)
			{
		    bmq_juli+=(ECPULSE1-ECPULSE2);
			}
	  if(Fork==1&&Forking==0)
			{
		    tly_jifen+=gyroz1;
			}
		if((Round ==4)&&(huandao==0))
		{
			bmq_juli+=(ECPULSE1-ECPULSE2);
			tly_jifen+=gyroz1;
		}
}
//������
//������
//������	
	if(P33==1&&P32==0)        
{
	if(Round ==4)
    {
      Discern3();	  //��������		
    }
  else  if(Fork==1)
	  {
		  sancha();     //������
	  }
else  
  {
          // ʶ�𻷵�		
		if(((Detection==0)&&(leftV>70)&&(rightV>70)&&(diangan[4]+diangan[3]>3350)&&(diangan[1]+diangan[2])>3100)&&Round==0&&Fork==0)  
      {
				Detection=1;
			}	
		if(Detection==1)
		{
			hdcha=(diangan[1]-diangan[2]);
		}
	  if(hdcha<-100&&Detection==1)
		 {
			Detection=2;
		 }
		if(Detection==2&&bmq_juli>4600)
			{
				Round=4;
				LED_Ctrl(LED2,ON);
				Detection=3;
				bmq_juli=0;
			}
			    //ʶ���·
		 //if(diangan[4]<1050&&(diangan[1]+diangan[2])<1650&&(diangan[1]+diangan[2])>1250&&leftV>50&&rightV>50&&Round==0&&Fork==0&&Forkchu==0)
			if(diangan[4]<1000&&(diangan[1]+diangan[2])<500&&(diangan[1]+diangan[2])>200&&Round==0&&Fork==0&&Forkchu==0)
      {
				Fork = 1;
				LED_Ctrl(LED2,ON);				
      }
					//����ѭ��Ŀ���ٶ�
     if(Round == 0&&Fork == 0)
        {
				 if(Forkchu==1)
				 {
						Target_Speed =20;
				 }
				 if(Forkchu==0)
				 {
					if((leftV<5)&&(rightV<5))
					{
						Target_Speed =25;
					}
					else if((leftV>60)&&(rightV>60))
					{
						Target_Speed =25;
					}
					else if((leftV>5)&&(leftV<60)&&(rightV<60)&&(rightV>5))
					{
						Target_Speed =25;
					}
				 }
        }
	}
		if(Detection==2)
			{
		    bmq_juli+=(ECPULSE1-ECPULSE2);
			}
	  if(Fork==1&&Forking==0)
			{
		    tly_jifen+=gyroz1;
			}
		if((Round ==4)&&(huandao==0))
		{
			bmq_juli+=(ECPULSE1-ECPULSE2);
			tly_jifen+=gyroz1;
		}
}
//������
//������
//������	
	if(P33==1&&P32==1) 
{
	 yxy=1;
   if(Round==1)
    {
      Discern4();	  //��������		
    }
  else  if(Fork==1)
	  {
		  sancha();     //������
	  }
else  
  {
          // ʶ�𻷵�
       if(((Detection==0)&&(leftV>68)&&(rightV>78)&&(diangan[4]>2100)&&(diangan[1]+diangan[2])>3200)&&Round==0&&Fork==0) 
       {
           Detection=1;				  
				   LED_Ctrl(LED2,ON);
       }
       else if((bmq_juli>4700)&&Detection==1)      //2600     //4000
       {
           Round = 1;
       }
			    //ʶ���·
		   //if(diangan[4]<1050&&(diangan[1]+diangan[2])<1650&&(diangan[1]+diangan[2])>1250&&leftV>50&&rightV>50&&Round==0&&Fork==0&&Forkchu==0)
			 if(diangan[4]<1000&&(diangan[1]+diangan[2])<500&&(diangan[1]+diangan[2])>200&&Round==0&&Fork==0&&Forkchu==0)
            {
							  Fork = 1;
								LED_Ctrl(LED2,ON);				
            }
					//����ѭ��Ŀ���ٶ�
			 if(Round == 1&&Fork == 0)
			 {
				 Target_Speed =20;
			 }
       if(Round == 0&&Fork == 0)
			 {
				 Target_Speed =31;
			 }
	}
		if(Detection==1)
			{
		    bmq_juli+=(ECPULSE1-ECPULSE2);
			}
	  if(Fork==1&&Forking==0)
			{
		    tly_jifen+=gyroz1;
			}
		if(Round==1)
			{
		    tly_jifen+=gyroz1;
			}
}	
}

void sancha(void)		
{
	if(Fork==1&&count==0&&(tly_jifen>>10)>-150)	//�Ҳ�·      //�������ǻ���-300����ֵ-40//  
	 {
		 chazhi_sancha-=30;
		 TempAngle=chazhi_sancha;
		 if(TempAngle<-70)TempAngle=-70;
		 Fork_R = 1;
	 }
	else if(Fork_R = 1&&(tly_jifen>>10)<-150)
	 {
		 Forking=1;
		 TempAngle=(leftV-rightV);
     if(Fork_R==1&&Forking==1)
		 {
       TempAngle=(leftV-rightV);
		   bmq_juli+=(ECPULSE1-ECPULSE2);			 
		 }
		 if(bmq_juli>=10000)
		 {
			 if(Go==2)Go=3;
		   LED_Ctrl(LED2,OFF);
			 bmq_juli=0;
		 }
	 }
 /*if(Fork==1&&count==0&&time1<110)	//�Ҳ�·
	 {
		 TempAngle=-20;
		 time1++;
		 Fork_R = 1;
		 Target_Speed =0;
	 }
	 else if(Fork_R==1&&time1>=110)
	 {
		 Target_Speed =15; 
		 TempAngle=(leftV-rightV);
     if(Fork_R==1)
		 {
		   bmq_juli+=(ECPULSE1-ECPULSE2);
		 }
		 if(bmq_juli>=12000)
		 {
			 if(Go==2)Go=3;
		   LED_Ctrl(LED2,OFF);
			 bmq_juli=0;
		 }
	 }
		 if(diangan[4]>1400)
		 {
			 Fork=0;
			 Fork_R=0;
			 count=1;
	     time1=0;
			 LED_Ctrl(LED2,OFF);
		 }
	 else if(Fork==1&&count==1&&time2<110) //���·
	 {
		  TempAngle=20;
      time2++;
      Fork_L = 1;
      Target_Speed =0;
	 }
	  else if(Fork_L==1&&time2>=110)
		{
      Target_Speed =10; 
		  TempAngle=(leftV-rightV);
			if(diangan[4]>1200)
			{
			 Fork=0;
			 Fork_L=0;
			 count=0;
			 time2=0;
			 LED_Ctrl(LED2,OFF);
		  }
		}*/
}
void sancha3(void)		
{
	if(Fork==1&&count==0&&(tly_jifen>>10)>-400)	//�Ҳ�·      //�������ǻ���-300����ֵ-40//  
	 {
		 Target_Speed =15;
		 chazhi_sancha-=30;
		 TempAngle=chazhi_sancha;
		 if(TempAngle<-70)TempAngle=-70;
		 Fork_R = 1;
	 }
	else if(Fork_R = 1&&(tly_jifen>>10)<-400)
	 {
		 Forking=1;
		 TempAngle=(leftV-rightV);
     if(Fork_R==1&&Forking==1)
		 {
       TempAngle=(leftV-rightV);
		   bmq_juli+=(ECPULSE1-ECPULSE2);			 
		 }
		 if(bmq_juli>=11500)
		 {
			 if(Go==2)Go=3;
		   LED_Ctrl(LED2,OFF);
			 bmq_juli=0;
		 }
	 }
 }
void Discern(void) // ��������               //�������ǻ���200����ֵ40
{	

}
void Discern3(void) // ��������               //�������ǻ���200����ֵ40
{	

}
void Discern4(void) 
{
    if(Round==1&&(tly_jifen>>10)>-200)         
    {
		 chazhi_huandao-=30;
		 TempAngle=chazhi_huandao;
		 if(TempAngle<-70)TempAngle=-70;
    }
    if((tly_jifen>>10)<-200)
    {
      Rounding=1;
			TempAngle=(leftV-rightV);
    }
    if(Rounding==1&&(tly_jifen>>10)<-2230)      //1260
    {
      TempAngle=0;
			Round_Out = 1;
    }
    if(Round_Out==1&&bmq_juli>37500)           //39500 
    {  
			      TempAngle=(leftV-rightV);
			      Detection=0;
            Round = 0;
            Rounding=0;      
            Round_Out = 0;
		    	  bmq_juli=0;
			      tly_jifen=0;			      
			      LED_Ctrl(LED2,OFF);	        
    }
}
             

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������void OLED_show (void)
����  �ܡ�OLED��ʾ����
����  �ߡ�chiusir
�������¡�2021��12��22�� 
������汾��V1.0
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void OLED_show(void)
{
	  char txt[30];
		sprintf(txt,"LV:%04d RV:%04d",leftV,rightV);		              // ��ʾ�������ADC��ֵ
    OLED_P6x8Str(0,0,(u8*)txt);
    sprintf(txt,"LP:%04d RP:%04d",diangan[1],diangan[2]);		      // ��ʾ�����м�ADC��ֵ
    OLED_P6x8Str(0,1,(u8*)txt);
	  sprintf(txt,"ZJ:%04d T:%04d",diangan[4],TempAngle);		         // ��ʾ�м���ֵ�Ͳ�ֵ
    OLED_P6x8Str(0,2,(u8*)txt);
	  sprintf(txt,"E:%04d %04d",ECPULSE1,ECPULSE2);		              // ��ʾ������
    OLED_P6x8Str(0,3,(u8*)txt);
		sprintf(txt,"M:%04d Mt:%04d",MotorDuty1,MotorDuty2);	        // ��ʾռ�ձ�
    OLED_P6x8Str(0,4,(u8*)txt);	
	  sprintf(txt,"LY:%04d JF:%04d",Forkchu,Go);	                // 
    OLED_P6x8Str(0,5,(u8*)txt);	
//	  sprintf(txt,"LY:%05d JF:%05d",(int)(speed_i*100),tly_jifen>>10);	                // ��ʾ����RX�ͻ���
//    OLED_P6x8Str(0,5,(u8*)txt);		
	  sprintf(txt,"P32:%02d P33:%02d",P32,P33);	
    OLED_P6x8Str(0,6,(u8*)txt);
//	  sprintf(txt,"P:%04d D:%04d",(int)(dir_p*10),(int)(dir_d*10));	// ��ʾpid��ֵ
//    OLED_P6x8Str(0,6,(u8*)txt);
	
    /*sprintf(txt,"LV:%06d RV:%06d",aacx1,aacy1);		         
    OLED_P6x8Str(0,0,(u8*)txt);
    sprintf(txt,"LP:%06d RP:%06d",aacz1,gyrox1);		     
    OLED_P6x8Str(0,1,(u8*)txt);
		sprintf(txt,"ZJ:%06d %06d",gyroy1,gyroz1);		                
    OLED_P6x8Str(0,2,(u8*)txt);
	  sprintf(txt,"R:%01d %01d",Round,Rounding);	                
    OLED_P6x8Str(0,6,(u8*)txt);*/
		
	  //(0xF1,leftV,diangan[0],rightV,diangan[3],diangan[4],diangan[1],diangan[2],Out_s);
}











