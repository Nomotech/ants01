#include "mbed.h"
#include <math.h>

DigitalIn sw(D8);

Serial pc(USBTX, USBRX); // tx, rx
Serial bt(D1,D0); // tx, rx
PwmOut  motorA1(PA_12);//A...right
PwmOut  motorA2(PB_0);
PwmOut  motorB1(PB_5); //B...left
PwmOut  motorB2(PA_11);
PwmOut  servo(PA_8);

AnalogIn encoderA(PA_0);
AnalogIn encoderB(PA_1);
AnalogIn sensor1 (PA_4);
//AnalogIn sensor1 (PA_4);
Ticker sendT;

struct Point{
    double x;
    double y;
};
struct Buf {
    unsigned char xhigh;
    unsigned char xlow;
    unsigned char yhigh;
    unsigned char ylow;
};
int Mx,My;
double a,b;
int ae,be;
int t=0;
int countA =0;
int countB =0;
int old_countA=0;
int old_countB=0;
double d_A;
double d_B;
int nowA;
int nowB;
int oldA=0;
int oldB=0;
double x=0.0,y=0.0,theta=3.1415926535/2.0;
double d_x=0.0,d_y=0.0,d_theta=0.0;
double theta_m=90,theta_t,theta_d;
double delta;
double s;
int count_D;
int start=0;
int go;
int data;
int goal;
int ts=0;//theta_sensor
int ts_=1;
//theta = 3.1415926535/2.0;//初期角度

int d=27;//タイヤ間距離
double r=0.0;//回転半径

Buf buf[100];
Point target[100];

void move(int m,double s);
void read(void);
void send(void);

int main() {
    pc.baud(115200);
    bt.baud(115200);
    
     sw.mode(PullUp);
     
    
    bt.attach(read,Serial::RxIrq);
    sendT.attach(&send,0.05);
    
    for(ts=0;ts<80;ts+=5){
        servo.pulsewidth_us(700+ts*20);
        wait(0.3);
    }
    while(1){    
        if(sw.read()==0||go==1)break;
    }
    while(sw.read()!=0) {
        
        a=encoderA.read();
        b=encoderB.read();
        if(a<0.5f)nowA=1;
        else nowA =0;
        if(b<0.5f)nowB=1;
        else nowB =0;
        if(motorA1==1&&motorA2!=1)ae=1;
        else if(motorA1!=1&&motorA2==1)ae=-1;
        if(motorB1==1&&motorB2!=1)be=1;
        else if(motorB1!=1&&motorB2==1)be=-1;    
        if(oldA!=nowA)countA += ae;
        if(oldB!=nowB)countB += be;
        oldA=nowA;
        oldB=nowB;
      
        
        d_A = (countA-old_countA)*5.0*3.1415926535/4.0; //差分をとる
        d_B = (countB-old_countB)*5.0*3.1415926535/4.0; 
        old_countA=countA;              //oldに保存
        old_countB=countB;
        
        r=(double)(d_A+d_B)*d/(d_A-d_B);//回転半径
        d_theta=(double)(d_A-d_B)/(2.0*d);
        d_x=r*sin(d_theta);
        d_y=r*(1.0-cos(d_theta));
        
        theta += d_theta;
        x += d_x*cos(theta) - d_y*sin(theta);
        y += d_x*sin(theta) + d_y*cos(theta);
        
        Mx = (int)x;
        My = (int)y;
        
        //bt.printf("%d %d \n",countA,countB);
        pc.printf(":B:%d:%d:%d:\n",(int)x,(int)y,(int)(theta*180.0/3.1415926535));
        //bt.printf(":R:%d:%d:%d:\n",(int)target[t].x*5,(int)target[t].y*5,(int)theta_t);
        //send();
        
        theta_t = atan2(target[t].y-y,target[t].x-x)*180.0/3.1415926535;  //targetの角度
        theta_m = (theta*180.0/3.1415926535);                        //マシーンの角度0~360
        while(theta_m>360.0)theta_m-=360.0;
        while(theta_m<0.0)theta_m+=360.0;
        theta_d = theta_t - theta_m;                //角度の差
        if(theta_d>180.0) theta_d = theta_d-360.0;    
        if(theta_d<-180.0)theta_d = 360.0+theta_d;    
        delta = pow(target[t].x-x,2)+pow(target[t].y-y,2);
        
        //pc.printf(":%d %d: \t%f\n",countA,countB,theta_d);
        if(delta>20.0)s=1.0;
        //else s = delta/20.0;
        s=1.0;
        if(delta>2.0){
            if(theta_d>30.0)                move(8,s);//左急旋回 
            if(theta_d<30.0&&theta_d>10.0)  move(7,s);//左旋回 
            if(theta_d<10.0&&theta_d>2.0)   move(6,s);//左 
            if(theta_d>-2.0&&theta_d<2.0)   move(1,s);//前進
            if(theta_d>-10.0&&theta_d<-2.0) move(3,s);//右 
            if(theta_d>-30.0&&theta_d<-10.0)move(4,s);//右旋回 
            if(theta_d<-30.0)               move(5,s);//右急旋回 
        }else {
            move(0,1.0);
            wait_ms(200);
            if(t<goal)t++;
            else {
                move(0,1.0);
                while(1){    
                    if(go==1){break;t=0;}
                    if(sw.read()==0) NVIC_SystemReset();
                }
            }
        }
    }
    NVIC_SystemReset();
}


void read(void){
    data = bt.getc();
    pc.printf("data:%d\n",data);
    int i;
    if(data==170){
       
        count_D=-2;
        start=1;
        go=0;
        for(i=0;i<100;i++){
            target[i].x =0.0;
            target[i].y =0.0;
        }
    } 
    if(start==1 && count_D%4==0) buf[count_D/4].xhigh =  data;
    if(start==1 && count_D%4==1) buf[count_D/4].xlow  =  data;
    if(start==1 && count_D%4==2) buf[count_D/4].yhigh =  data;
    if(start==1 && count_D%4==3) buf[count_D/4].ylow  =  data;
    if(data==255){
        //pc.printf("count_D:%d\n",count_D);
        start = 0;
        if(count_D%4==0){
            for(i=0;i<count_D/4;i++){
                target[i].x = ((buf[i].xhigh << 6 | buf[i].xlow) & 0b111111111111);
                target[i].y = ((buf[i].yhigh << 6 | buf[i].ylow) & 0b111111111111);
                if(buf[i].xhigh>=0b10000000)target[i].x*=-1;
                if(buf[i].yhigh>=0b10000000)target[i].y*=-1;
                pc.printf("x:%f y:%f\n",target[i].x,target[i].y);
                goal = count_D/4-1;
            }
            go=1;
        }
    }
     //pc.printf("count_D:%d\n",count_D);
    if(start==1)count_D++;
}

void send(void){
    int Mx_s=Mx;
    int My_s=My;
    int range;
     
    if(ts>=0&&ts<=90)ts+=ts_;
    if(ts<0){ ts_ =1;ts=1;}
    if(ts>90){ ts_=-1;ts=89;}
    servo.pulsewidth_us(600+ts*20);
    unsigned char x_h,x_l,y_h,y_l,theta_h,theta_l,rtheta,range_h,range_l,xpm,ypm;
        if (Mx< 0) {Mx_s *= -1; xpm = 0b10000000;}
        else xpm = 0b00000000;
        if (My< 0) {My_s *= -1; ypm = 0b10000000;}
        else ypm = 0b00000000;
        x_h = ((Mx_s >> 6) & 0b00111111) + xpm;
        x_l =   Mx_s       & 0b00111111 + 0b01000000;
        y_h = ((My_s >> 6) & 0b00111111) + ypm;
        y_l =   My_s       & 0b00111111 + 0b01000000;
        y_h = ((My_s >> 6) & 0b00111111) + ypm;
        y_l =   My_s       & 0b00111111 + 0b01000000;
        theta_h =((int)theta_m >> 6) & 0b00111111;
        theta_l = (int)theta_m & 0b00111111;
        rtheta=ts;
        range = (int)(100.0/sensor1);
        range_h=(unsigned char)((range>>6)&0b00111111);
        range_l=(unsigned char)(range&0b00111111);
        bt.putc(0xAA);
        bt.putc(x_h);
        bt.putc(x_l);
        bt.putc(y_h);
        bt.putc(y_l);
        bt.putc(theta_h);
        bt.putc(theta_l);
        bt.putc(rtheta);
        bt.putc(range_h);
        bt.putc(range_l);
}


void move(int m,double s){
    switch(m){
        case 0://停止
            motorA1=0.0;
            motorA2=0.0;
            motorB1=0.0;
            motorB2=0.0;
            break;
        case 1://前進
            motorA1=1.0*s;
            motorA2=0.0;
            motorB1=1.0*s;
            motorB2=0.0;
            break;
        case 2://後退
            motorA1=0.0;
            motorA2=1.0*s;
            motorB1=0.0;
            motorB2=1.0*s;
            break;
        case 3://右
            motorA1=0.2*s;
            motorA2=0.0;
            motorB1=1.0*s;
            motorB2=0.0;
            break;
        case 4://右旋回
            motorA1=0.0;
            motorA2=0.0;
            motorB1=1.0*s;
            motorB2=0.0;
            break;
        case 5://右急旋回  
            motorA1=0.0;
            motorA2=1.0*s;
            motorB1=1.0*s;
            motorB2=0.0;
            break; 
        case 6://左
            motorA1=1.0*s;
            motorA2=0.0;
            motorB1=0.2*s;
            motorB2=0.0;
            break;
        case 7://左旋回
            motorA1=1.0;
            motorA2=0.0;
            motorB1=0.0;
            motorB2=0.0;
            break;
        case 8://左急旋回   
            motorA1=1.0;
            motorA2=0.0;
            motorB1=0.0;
            motorB2=1.0;
            break; 
        case 9://急停止
            motorA1=1.0;
            motorA2=1.0;
            motorB1=1.0;
            motorB2=1.0;
            break;
        default:
            break;
    }    
}