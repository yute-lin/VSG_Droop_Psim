#include<stdio.h>
#include<math.h>
//#include"structure_declare.h"
      
#define PFC_PRD 111 
#define PI_2_VALUE 2*3.1415926
typedef float  float32;
#define V_out_ref 400.0 //07.18
 

int flaga;
int flagb;
int flagc;
int deadtimea;
int deadtimeb;
int deadtimec; 

typedef struct
{
    float32 f32phA;   // Phase A Voltage or Current etc.
    float32 f32phB;   // Phase B Voltage or Current etc.
    float32 f32phC;   // Phase C Voltage or Current etc.
    float32 f32Beta;  // Phase Q Stationary Voltage or Current etc.
    float32 f32Alpha;  // Phase D Stationary Voltage or Current etc.
    float32 f32Q;  // Phase Q Rotating Voltage or Current etc.
    float32 f32D;  // Phase D Rotating Voltage or Current etc.
}TABC_DQ_STRUCT;

typedef struct
{
    float32 f32Beta;  // Phase Q Stationary
    float32 f32Alpha;  // Phase D Stationary

    //internal
    float32 f32un_Beta[3];//[0]=un, [1]=u[n-1], [2]=u[n-1] for integration use
    float32 f32un_Alpha[3];
    float32 f32un_qAlpha[3];
    float32 f32un_qBeta[3];

    //qv∩ is always 90- degrees lag respect to v∩.  quadrature-signals generator(QSG)
    //v'(s)/v(s) = k*w*s/s^2+k*w*s+w^2 qv'(s)/v(s) = k*w/s^2+k*w*s+w^2
    /* where 肋＊ and k set resonance frequency and damping factor
        of the SOGI-QSG respectively. Bode plots from transfer
        functions of (6) are shown in Fig. 2(b) and 2(c) for several
        values of k. These plots show how the lower value of k the
        more selective filtering response, but the longer stabilization
        time as well. A critically-damped response is achieved when
        k = sqrt(2) . This value of gain results an interesting selection
        in terms of stabilization time and overshot limitation.
     */
    float32 f32QSG_Alpha; //v∩_Alpha
    float32 f32QSG_qAlpha; //qv∩_Alpha
    float32 f32QSG_Beta; //v∩_Beta
    float32 f32QSG_qBeta; //qv∩_Beta

    //positive-sequence component
    float32 f32Alpha_Pos;  // positive-sequence alpha component:qv∩_Alpha + v∩_Beta
    float32 f32Beta_Pos;  // positive-sequence beta component: v∩_Alpha - qv∩_Beta

    float32 f32D_Pos; // positive-sequence D component: estimated amplitude for the positive-sequence component
    float32 f32Q_Pos; // positive-sequence Q component:PLL error input

    float32  f32k_damp;             //
    float32  f32Kp;             //
    float32  f32Ki;             //
    float32  f32Out_P;          //
    float32  f32Out_I;          //
    float32  f32Freq_Est;       //Estimate Freq
    float32  f32Freq_Filt;      //Freq Filtered, 蚚衾坶眈遠袨怓瓚隅
    float32  f32Freq_CONST;     // Center Grid frequency: Hz
    float32  f32W_Est;             // Input: Feedback input
    float32  f32W_CONST;        // Center Grid frequency: rad/s
    float32  f32Theta;          // Output: PID output
    float32  f32Sin_Theta;      // sin value for ePLL
    float32  f32Cos_Theta;      // cos value for ePLL
    float32  f32Tcal ;          // PLL calculation frequency

    float32  f32Theta_Active;       // Theta for Sin
    float32  f32Theta_Act_pu;       //

    // Uint16  u16IsLocked; //1:Locked  0:Unlock
    // Uint16  u16LockDelayCnt;
}DSOGI_PLL;

DSOGI_PLL   DsogiPLL;
TABC_DQ_STRUCT  VarAC_Volt;
TABC_DQ_STRUCT  VarAC_Curr;
TABC_DQ_STRUCT  SVRef;
TABC_DQ_STRUCT  VSG_Volt;

struct IsrVarsStruct
{
	float32 f32Tcal;
	float32 ia;
	float32 ib;
	float32 ic;
	float32 vab;
	float32 vbc;
	float32 vca;
	float32 vdc;
	float32 theta_PLL;
	float32 theta_power;
	float32 vsg_f32Sin_Theta;
	float32 vsg_f32Cos_Theta;
	float32 vcont_a;
	float32 vcont_b;
	float32 vcont_c;
	
	float32 kp; //use for Trapezoidal rule
	float32 kr;
	float32 wa;
	float32 wo; 
	float32 T;
	float32 kp_v;
	float32 kr_v;
	float32 kp_i;
	float32 ki_i;
	float32 valphacmd; //use for Trapezoidal rule
	float32 vbetacmd;
	float32 err_valpha[3];
	float32 err_vbeta[3];
	float32 err_ialpha;
	float32 err_ibeta;
	float32 acc_ialpha;
	float32 acc_ibeta;
	float32 ialphacmd[3];
	float32 ibetacmd[3];
	float32 a[3];
	float32 b[3];	
	float32 c[4];
	float32 malpha;
	float32 mbeta;
	
	float32 P0;
	float32 Qcmd;
	float32 Pins[2];
	float32 Qins;
	float32 wc;
	float32 omega[2];
	float32 delta_E[2];
	float32 delta_Q[2];
	float32 vsg_theta[2];
	float32 E;
	float32 J;
	float32 D;
	float32 m;
	float32 kq;
	float32 PCCvalphacmd;
	float32 PCCvbetacmd;	
	float32 Pave[2];
	float32 Qave[2];
	
}IsrVars;

int interrupt_cnt = 0;

void IsrVarsinitial(){	
	DsogiPLL.f32Freq_CONST = 55.0f;
   	DsogiPLL.f32Tcal = 0.000022222222f;
   	DsogiPLL.f32k_damp = 1.414214f;
   	DsogiPLL.f32Kp = 314.16f*PI_2_VALUE/320.0f;
   	DsogiPLL.f32Ki = 9763.0f*PI_2_VALUE/320.0f;
   	DsogiPLL.f32Theta = 0.0f;
   	DsogiPLL.f32Sin_Theta = 0.0f;
   	DsogiPLL.f32Cos_Theta = 1.0f;
	
	IsrVars.f32Tcal=0.000022222222f;
	IsrVars.ia=0,IsrVars.ib=0,IsrVars.ic = 0;
	IsrVars.vab=0,IsrVars.vbc=0,IsrVars.vca = 0;
	IsrVars.vdc = 0;

	IsrVars.kp_v= 0.05,IsrVars.kr_v= 25;
	IsrVars.kp_i=8;
	IsrVars.wa=3.77,IsrVars.wo=377,IsrVars.T=0.000022222222f;
	IsrVars.a[0]=4+4*IsrVars.wa*IsrVars.T+IsrVars.T*IsrVars.T*IsrVars.wo*IsrVars.wo;
	IsrVars.a[1]=(8-2*IsrVars.T*IsrVars.T*IsrVars.wo*IsrVars.wo)/IsrVars.a[0];
	IsrVars.a[2]=(-4+4*IsrVars.wa*IsrVars.T-IsrVars.T*IsrVars.T*IsrVars.wo*IsrVars.wo)/IsrVars.a[0];
	IsrVars.b[0]=IsrVars.kp_v+(4*IsrVars.kr_v*IsrVars.wa*IsrVars.T)/IsrVars.a[0];
	IsrVars.b[1]=IsrVars.kp_v*(-8+2*IsrVars.T*IsrVars.T*IsrVars.wo*IsrVars.wo)/IsrVars.a[0];
	IsrVars.b[2]=(IsrVars.kp_v*(4-4*IsrVars.wa*IsrVars.T+IsrVars.T*IsrVars.T*IsrVars.wo*IsrVars.wo)-4*IsrVars.kr_v*IsrVars.wa*IsrVars.T)/IsrVars.a[0];	
	
	IsrVars.err_valpha[0]=0,IsrVars.err_valpha[1]=0,IsrVars.err_valpha[2]=0;
	IsrVars.err_vbeta[0]=0,IsrVars.err_vbeta[1]=0,IsrVars.err_vbeta[2]=0;
	IsrVars.ialphacmd[0]=0,IsrVars.ialphacmd[1]=0,IsrVars.ialphacmd[2]=0;
	IsrVars.ibetacmd[0]=0,IsrVars.ibetacmd[1]=0,IsrVars.ibetacmd[2]=0;
	
	IsrVars.Pins[0]=0,IsrVars.Pins[1]=0,IsrVars.Qins=0;
//	IsrVars.Pave[0]=0,IsrVars.Pave[1]=0;
	IsrVars.wc=75,IsrVars.E=30;
//	IsrVars.J=50,IsrVars.D=-90;
	IsrVars.J=0.05,IsrVars.D=3,IsrVars.m=3;
	IsrVars.omega[0]=377,IsrVars.omega[1]=377,IsrVars.vsg_theta[0]=0,IsrVars.vsg_theta[1]=0;
	IsrVars.delta_Q[1]=0,IsrVars.delta_E[1]=0;
	IsrVars.P0=1100,IsrVars.Qcmd=150,IsrVars.kq=0.00001;
	IsrVars.c[3]=1/(2*IsrVars.J+IsrVars.m*IsrVars.T/377+IsrVars.D*IsrVars.T);
	IsrVars.c[0]=IsrVars.T*(2*IsrVars.P0/377+2*IsrVars.m+2*IsrVars.D*377);
	IsrVars.c[1]=IsrVars.T/377;
	IsrVars.c[2]=(2*IsrVars.J-IsrVars.m*IsrVars.T/377-IsrVars.D*IsrVars.T);
	
}

void Clark_ABC2AlphaBeta(TABC_DQ_STRUCT *ABC_DQ_Data);//Clark
void iclark_AlphaBeta2abc(TABC_DQ_STRUCT *ABC_DQ_Data);
void Park_AlphaBeta2DQ(TABC_DQ_STRUCT *ABC_DQ_Data, float32 SINx, float32 COSx);

float32 __cos(float32 theta);
float32 __sin(float32 theta);



double __fsat(double a,double h,double l);
void mOffPfcPwm(double *out);
#define stop_pwm_time 1

int cal(double t,double delt,double *in,double *out){
	double max,min,zss;	
	double pwm = in[7];
	if(interrupt_cnt >= PFC_PRD*2)
	//if(interrupt_cnt*delt >= 1./45000)
	{
		IsrVars.ia = -1*in[3];
		IsrVars.ib = -1*in[4];
		IsrVars.ic = -1*in[5];
		IsrVars.vab = in[0];
		IsrVars.vbc = in[1];
		IsrVars.vca = in[2];
		IsrVars.vdc = in[6];					
		
		// if((Max_VAB- 1294) > 50)
		VarAC_Volt.f32phA = (IsrVars.vab-IsrVars.vca)/3;
		VarAC_Volt.f32phB = (IsrVars.vbc-IsrVars.vab)/3;
		VarAC_Volt.f32phC = (IsrVars.vca-IsrVars.vbc)/3;

		Clark_ABC2AlphaBeta(&VarAC_Volt);
		Park_AlphaBeta2DQ(&VarAC_Volt, DsogiPLL.f32Sin_Theta, DsogiPLL.f32Cos_Theta);//DsogiPLL

		VarAC_Curr.f32phA=IsrVars.ia;//VarAC_Curr.f32phA*0.0390625;
	    VarAC_Curr.f32phB=IsrVars.ib;//VarAC_Curr.f32phB*0.0390625;
	    VarAC_Curr.f32phC=IsrVars.ic;//VarAC_Curr.f32phC*0.0390625;
//	    	max=VarAC_Volt.f32phA;
//			min=VarAC_Volt.f32phA;
//			if(max<VarAC_Volt.f32phB) max=VarAC_Volt.f32phB;
//			else min=VarAC_Volt.f32phB;
//			if(max<VarAC_Volt.f32phC) max=VarAC_Volt.f32phC;
//			if(min>VarAC_Volt.f32phC) min=VarAC_Volt.f32phC;
//			zss=-(max+min)/2/400; //zss=0;
	    Clark_ABC2AlphaBeta(&VarAC_Curr);
		Park_AlphaBeta2DQ(&VarAC_Curr, DsogiPLL.f32Sin_Theta, DsogiPLL.f32Cos_Theta);
		
		{
			DsogiPLL.f32Alpha = VarAC_Volt.f32Alpha/81.0f;//81.0f;//320; //(IsrVars.van*2.0-IsrVars.vbn-IsrVars.vcn)/3.0/320.0f;//VarAC_Volt.f32Alpha * 0.7097901f / 320.0f;
			DsogiPLL.f32Beta = VarAC_Volt.f32Beta/81.0f;//(IsrVars.vbn-IsrVars.vcn)/1.7320508f/320.0f;//VarAC_Volt.f32Beta * 0.7097901f / 320.0f;

			DsogiPLL.f32un_Alpha[0] = DsogiPLL.f32k_damp*(DsogiPLL.f32Alpha - DsogiPLL.f32QSG_Alpha) - DsogiPLL.f32QSG_qAlpha;
			DsogiPLL.f32un_Beta[0] = DsogiPLL.f32k_damp*(DsogiPLL.f32Beta - DsogiPLL.f32QSG_Beta) - DsogiPLL.f32QSG_qBeta;

			DsogiPLL.f32QSG_Alpha += DsogiPLL.f32W_Est * DsogiPLL.f32Tcal/12.0f * \
					(23.0f*DsogiPLL.f32un_Alpha[0] - 16.0f*DsogiPLL.f32un_Alpha[1] + 5.0f*DsogiPLL.f32un_Alpha[2]);
			DsogiPLL.f32un_Alpha[2] = DsogiPLL.f32un_Alpha[1];
			DsogiPLL.f32un_Alpha[1] = DsogiPLL.f32un_Alpha[0];

			DsogiPLL.f32QSG_Beta += DsogiPLL.f32W_Est * DsogiPLL.f32Tcal/12.0f * \
					(23.0f*DsogiPLL.f32un_Beta[0] - 16.0f*DsogiPLL.f32un_Beta[1] + 5.0f*DsogiPLL.f32un_Beta[2]);
			DsogiPLL.f32un_Beta[2] = DsogiPLL.f32un_Beta[1];
			DsogiPLL.f32un_Beta[1] = DsogiPLL.f32un_Beta[0];

			DsogiPLL.f32un_qAlpha[0] = DsogiPLL.f32QSG_Alpha;
			DsogiPLL.f32un_qBeta[0] = DsogiPLL.f32QSG_Beta;
			DsogiPLL.f32QSG_qAlpha += DsogiPLL.f32W_Est * DsogiPLL.f32Tcal/12.0f * \
					(23.0f*DsogiPLL.f32un_qAlpha[0] - 16.0f*DsogiPLL.f32un_qAlpha[1] + 5.0f*DsogiPLL.f32un_qAlpha[2]);
			DsogiPLL.f32un_qAlpha[2] = DsogiPLL.f32un_qAlpha[1];
			DsogiPLL.f32un_qAlpha[1] = DsogiPLL.f32un_qAlpha[0];

			DsogiPLL.f32QSG_qBeta += DsogiPLL.f32W_Est * DsogiPLL.f32Tcal/12.0f * \
					(23.0f*DsogiPLL.f32un_qBeta[0] - 16.0f*DsogiPLL.f32un_qBeta[1] + 5.0f*DsogiPLL.f32un_qBeta[2]);
			DsogiPLL.f32un_qBeta[2] = DsogiPLL.f32un_qBeta[1];
			DsogiPLL.f32un_qBeta[1] = DsogiPLL.f32un_qBeta[0];

			DsogiPLL.f32Alpha_Pos = 0.5f*(DsogiPLL.f32QSG_Alpha - DsogiPLL.f32QSG_qBeta);
			DsogiPLL.f32Beta_Pos = 0.5f*(DsogiPLL.f32QSG_qAlpha + DsogiPLL.f32QSG_Beta);

			DsogiPLL.f32D_Pos =    DsogiPLL.f32Alpha_Pos * DsogiPLL.f32Cos_Theta + DsogiPLL.f32Beta_Pos * DsogiPLL.f32Sin_Theta;//cos sin
			DsogiPLL.f32Q_Pos =  - DsogiPLL.f32Alpha_Pos * DsogiPLL.f32Sin_Theta + DsogiPLL.f32Beta_Pos * DsogiPLL.f32Cos_Theta;//-sin cos

			DsogiPLL.f32Out_I = __fsat(DsogiPLL.f32Out_I + DsogiPLL.f32Q_Pos * DsogiPLL.f32Ki * DsogiPLL.f32Tcal,
									15.0f, -15.0f); //fixed CAL! not fixed ratio to grid freq.
			DsogiPLL.f32Out_P = DsogiPLL.f32Q_Pos * DsogiPLL.f32Kp;
			DsogiPLL.f32Freq_Est = __fsat(DsogiPLL.f32Out_P + DsogiPLL.f32Out_I + DsogiPLL.f32Freq_CONST, 70.0f, 40.0f);

			DsogiPLL.f32W_Est = DsogiPLL.f32Freq_Est * PI_2_VALUE; //cannot be removed. Used for Integration.
			DsogiPLL.f32Theta += DsogiPLL.f32Tcal * DsogiPLL.f32Freq_Est * PI_2_VALUE;//Radius

			IsrVars.theta_PLL = DsogiPLL.f32Theta + PI_2_VALUE/4;
//			if(IsrVars.theta_PLL >= PI_2_VALUE)
//			{
//				IsrVars.theta_PLL -= PI_2_VALUE;
//			}
			
//			if(DsogiPLL.f32Theta >= PI_2_VALUE)
//			{
//				DsogiPLL.f32Theta -= PI_2_VALUE;
//			}
			DsogiPLL.f32Cos_Theta = __cos(DsogiPLL.f32Theta); //cos(DsogiPLL.f32Theta);
			DsogiPLL.f32Sin_Theta = __sin(DsogiPLL.f32Theta); //sin(DsogiPLL.f32Theta);

			DsogiPLL.f32Freq_Filt += (DsogiPLL.f32Freq_Est - DsogiPLL.f32Freq_Filt)*10.0f/45010.0f;//60010.0f;	

			
			
   	 	}

//----------------power control---------------------------------------------------		
		IsrVars.Pins[0]=1.5*(VarAC_Volt.f32Alpha*VarAC_Curr.f32Alpha+VarAC_Volt.f32Beta*VarAC_Curr.f32Beta);
		IsrVars.Qins=1.5*(VarAC_Volt.f32Beta*VarAC_Curr.f32Alpha-VarAC_Volt.f32Alpha*VarAC_Curr.f32Beta);
		
		IsrVars.Pins[0] = (IsrVars.T*IsrVars.wc*IsrVars.Pins[0]+IsrVars.Pins[1])/(1+IsrVars.T*IsrVars.wc);
		IsrVars.Pins[1] = IsrVars.Pins[0];
		
		IsrVars.Qave[0] = IsrVars.Qins;
		IsrVars.Qave[0] = (IsrVars.T*IsrVars.wc*IsrVars.Qave[0]+IsrVars.Qave[1])/(1+IsrVars.T*IsrVars.wc);
		IsrVars.Qave[1] = IsrVars.Qave[0];
							
//		IsrVars.omega[0] = 2*IsrVars.T/(2*IsrVars.J+IsrVars.D*IsrVars.T)*(IsrVars.Pcmd/377+IsrVars.D*377)-(IsrVars.T/377/(2*IsrVars.J+IsrVars.D*IsrVars.T))*(IsrVars.Pins[0]+IsrVars.Pins[1])+(2*IsrVars.J-IsrVars.D*IsrVars.T)/(2*IsrVars.J+IsrVars.D*IsrVars.T)*IsrVars.omega[1];
		IsrVars.omega[0] = IsrVars.c[3]*(IsrVars.c[0]-IsrVars.c[1]*(IsrVars.Pins[0]+IsrVars.Pins[1])+IsrVars.c[2]*IsrVars.omega[1]);
//		IsrVars.omega[0] = __fsat(IsrVars.omega[0],378,376);
		IsrVars.vsg_theta[0] = 0.5*(IsrVars.T*IsrVars.omega[0]+IsrVars.T*IsrVars.omega[1]+2*IsrVars.vsg_theta[1]);
		
//		if(IsrVars.vsg_theta[0] >= PI_2_VALUE) 
//		{
//			IsrVars.vsg_theta[0] -= PI_2_VALUE;
//		}								
		
//		if(IsrVars.E[0]>=88) IsrVars.E[0]=IsrVars.T/IsrVars.kq*IsrVars.Qcmd-IsrVars.T/2/IsrVars.kq*(IsrVars.Qins[0]+IsrVars.Qins[1])+IsrVars.E[1];
//		if(IsrVars.E>=89) IsrVars.E = IsrVars.kq*(IsrVars.Qcmd-IsrVars.Qins) + 89.8;
		if(IsrVars.E>=89) IsrVars.E = IsrVars.kq*(IsrVars.Qcmd-IsrVars.Qave[0]) + 89.8;
		else IsrVars.E += 0.02;
		IsrVars.E = __fsat(IsrVars.E,91,89);
		
		IsrVars.Pins[1] = IsrVars.Pins[0];
		IsrVars.omega[1] = IsrVars.omega[0];
		IsrVars.vsg_theta[1] = IsrVars.vsg_theta[0];
		
		VSG_Volt.f32phA = IsrVars.E*sin(IsrVars.vsg_theta[0]);
		VSG_Volt.f32phB = IsrVars.E*sin(IsrVars.vsg_theta[0]-PI_2_VALUE/3);
		VSG_Volt.f32phC = IsrVars.E*sin(IsrVars.vsg_theta[0]+PI_2_VALUE/3);

		Clark_ABC2AlphaBeta(&VSG_Volt);
		
//		IsrVars.valphacmd = (VSG_Volt.f32Alpha+0.09425*VarAC_Curr.f32Beta);
//		IsrVars.vbetacmd = (VSG_Volt.f32Beta-0.09425*VarAC_Curr.f32Alpha);
		
		IsrVars.valphacmd = VSG_Volt.f32Alpha;
		IsrVars.vbetacmd = VSG_Volt.f32Beta; // no add iL for Vo to VC
		
		IsrVars.theta_power =  IsrVars.theta_PLL - IsrVars.vsg_theta[0];
		
		if(IsrVars.theta_power >= PI_2_VALUE)
		{
			IsrVars.theta_power -= PI_2_VALUE;
		}
		else if(IsrVars.theta_power <= -PI_2_VALUE)
		{
			IsrVars.theta_power += PI_2_VALUE;
		}
		
		if(t>=stop_pwm_time)
		{
			IsrVars.theta_power = 0;
		}
		


//----------------power control---------------------------------------------------			
//----------------voltage control---------------------------------------------------		
		IsrVars.err_valpha[0] = IsrVars.valphacmd - VarAC_Volt.f32Alpha;		
		IsrVars.ialphacmd[0] = IsrVars.a[1]*IsrVars.ialphacmd[1]+IsrVars.a[2]*IsrVars.ialphacmd[2]+IsrVars.b[0]*IsrVars.err_valpha[0]+IsrVars.b[1]*IsrVars.err_valpha[1]+IsrVars.b[2]*IsrVars.err_valpha[2];
//		IsrVars.ialphacmd[0]=__fsat(IsrVars.ialphacmd[0],10.0f,-10.0f);
		
		IsrVars.err_ialpha=IsrVars.ialphacmd[0]-VarAC_Curr.f32Alpha; 
		IsrVars.malpha = (IsrVars.kp_i*IsrVars.err_ialpha+VarAC_Volt.f32Alpha)/400; 
		IsrVars.err_valpha[2] = IsrVars.err_valpha[1];	
		IsrVars.err_valpha[1] = IsrVars.err_valpha[0];
		IsrVars.ialphacmd[2] = IsrVars.ialphacmd[1];
		IsrVars.ialphacmd[1] = IsrVars.ialphacmd[0];

		IsrVars.err_vbeta[0] = IsrVars.vbetacmd - VarAC_Volt.f32Beta;		
		IsrVars.ibetacmd[0] = IsrVars.a[1]*IsrVars.ibetacmd[1]+IsrVars.a[2]*IsrVars.ibetacmd[2]+IsrVars.b[0]*IsrVars.err_vbeta[0]+IsrVars.b[1]*IsrVars.err_vbeta[1]+IsrVars.b[2]*IsrVars.err_vbeta[2];
//		IsrVars.ibetacmd[0]=__fsat(IsrVars.ibetacmd[0],10.0f,-10.0f);
		
		IsrVars.err_ibeta=IsrVars.ibetacmd[0]-VarAC_Curr.f32Beta; 
		IsrVars.mbeta = (IsrVars.kp_i*IsrVars.err_ibeta+VarAC_Volt.f32Beta)/400; 
		IsrVars.err_vbeta[2] = IsrVars.err_vbeta[1];	
		IsrVars.err_vbeta[1] = IsrVars.err_vbeta[0];
		IsrVars.ibetacmd[2] = IsrVars.ibetacmd[1];
		IsrVars.ibetacmd[1] = IsrVars.ibetacmd[0];
//----------------voltage control---------------------------------------------------				
		
		SVRef.f32Alpha=IsrVars.malpha;
		SVRef.f32Beta=IsrVars.mbeta;
		iclark_AlphaBeta2abc(&SVRef);
//			max=SVRef.f32phA;
//			min=SVRef.f32phA;
//			if(max<SVRef.f32phB) max=SVRef.f32phB;
//			else min=SVRef.f32phB;
//			if(max<SVRef.f32phC) max=SVRef.f32phC;
//			if(min>SVRef.f32phC) min=SVRef.f32phC;
//			zss=-(max+min)/2; //zss=0;
		IsrVars.vcont_a =__fsat(SVRef.f32phA,0.49,-0.49)+0.5;//__cos(IsrVars.theta_e)*IsrVars.mq + __sin(IsrVars.theta_e)*IsrVars.md;
		IsrVars.vcont_b =__fsat(SVRef.f32phB,0.49,-0.49)+0.5;//__cos(IsrVars.theta_e - 1./3*PI_2_VALUE)*IsrVars.mq + __sin(IsrVars.theta_e - 1./3*PI_2_VALUE)*IsrVars.md;
		IsrVars.vcont_c =__fsat(SVRef.f32phC,0.49,-0.49)+0.5;//__cos(IsrVars.theta_e + 1./3*PI_2_VALUE)*IsrVars.mq + __sin(IsrVars.theta_e + 1./3*PI_2_VALUE)*IsrVars.md;
				
		interrupt_cnt = 0;

	}

	interrupt_cnt+=1;

	deadtimea++;
	deadtimeb++;
	deadtimec++;

	if(IsrVars.vcont_a > pwm)
	{ 	if(deadtimea<3 && flaga==0){out[0]=0; out[1]=0;}
		else{ 
			out[0]=1; out[1]=0; flaga=1; deadtimea=0;
		}
	}
	else{ 
		if(deadtimea<3 && flaga==1){out[0]=0; out[1]=0;}
		else{ 
			out[0]=0; out[1]=1; flaga=0; deadtimea=0;
		}
	}

	if(IsrVars.vcont_b > pwm)
	{ 	if(deadtimeb<3 && flagb==0){out[2]=0; out[3]=0;}
		else{ 
			out[2]=1; out[3]=0; flagb=1; deadtimeb=0;
		}
	}
	else{ if(deadtimeb<3 && flagb==1){out[2]=0; out[3]=0;}
		else{ out[2]=0; out[3]=1; flagb=0; deadtimeb=0;}
	}

	if(IsrVars.vcont_c > pwm)
	{ if(deadtimec<3 && flagc==0){out[4]=0; out[5]=0;}
		else{ out[4]=1; out[5]=0; flagc=1; deadtimec=0;}
	}
	else{ if(deadtimec<3 && flagc==1){out[4]=0; out[5]=0;}
		else{ out[4]=0; out[5]=1; flagc=0; deadtimec=0;}
	}
	
	if(t>=stop_pwm_time)
		mOffPfcPwm(out);
 
	out[6]=IsrVars.theta_PLL;
	out[7]=IsrVars.vsg_theta[0];
	out[8]=IsrVars.theta_power;
	out[9]=VarAC_Curr.f32Beta;
	out[10]=IsrVars.valphacmd;
	out[11]=IsrVars.vbetacmd;
	out[12]=VarAC_Volt.f32Alpha;
	out[13]=VarAC_Volt.f32Beta;
	out[14]=VSG_Volt.f32Alpha;
	out[15]=VSG_Volt.f32Beta;
	out[16]=IsrVars.omega[0];
	out[17]=IsrVars.Pins[0];
	out[18]=IsrVars.Qins;
	out[19]=IsrVars.E;
	out[20]=IsrVars.vsg_theta[0];
}

void Clark_ABC2AlphaBeta(TABC_DQ_STRUCT *ABC_DQ_Data)//Clark
{
    ABC_DQ_Data->f32Alpha = (ABC_DQ_Data->f32phA * 2.0f - ABC_DQ_Data->f32phB - ABC_DQ_Data->f32phC) / 3.0f;
    ABC_DQ_Data->f32Beta = (ABC_DQ_Data->f32phB - ABC_DQ_Data->f32phC) / 1.7320508f;
} 

void iclark_AlphaBeta2abc(TABC_DQ_STRUCT *ABC_DQ_Data)
{	
	  ABC_DQ_Data->f32phA = ABC_DQ_Data->f32Alpha;
	  ABC_DQ_Data->f32phB = (-ABC_DQ_Data->f32Alpha+1.7320508f*ABC_DQ_Data->f32Beta)/2.0;
	  ABC_DQ_Data->f32phC = (-ABC_DQ_Data->f32Alpha-1.7320508f*ABC_DQ_Data->f32Beta)/2.0;
}

void Park_AlphaBeta2DQ(TABC_DQ_STRUCT *ABC_DQ_Data, float32 SINx, float32 COSx)//Park
{
    //ABC_DQ_Data->f32D = ABC_DQ_Data->f32Alpha * COSx + ABC_DQ_Data->f32Beta * SINx; //2022.07.08 by billy
    //ABC_DQ_Data->f32Q = ABC_DQ_Data->f32Beta * COSx - ABC_DQ_Data->f32Alpha * SINx;
    ABC_DQ_Data->f32Q = ABC_DQ_Data->f32Alpha * COSx + ABC_DQ_Data->f32Beta * SINx; //2022.07.08 by billy
    ABC_DQ_Data->f32D = - ABC_DQ_Data->f32Beta * COSx + ABC_DQ_Data->f32Alpha * SINx;
} 

float32 __cos(float32 theta){
	return cos(theta);
}

float32 __sin(float32 theta){
	return sin(theta);
}

double __fsat(double a,double h,double l){
	if(a>=h) return h;
	else if(a<=l) return l;
	else return a;
}

void mOffPfcPwm(double *out)   { 
	int i=0; 
	for(i=0;i<6;i++) 
		out[i]=0;
}

//if(IsrVars.vcont_a > pwm)
//	{ 	if(deadtimea<3 && flaga==0){out[0]=0; out[1]=0;}
//		else{ 
//			out[0]=0; out[1]=1; flaga=1; deadtimea=0;
//		}
//	}