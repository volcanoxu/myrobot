#include "Classic6dofKine.h"
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <string.h>
#include <math.h>
#include <stdio.h>

#define sqrtf(X) (float)sqrt(X)
#define sinf(X) (float)sin(X)
#define cosf(X) (float)cos(X)
#define asinf(X) (float)asin(X)
#define acosf(X) (float)acos(X)
#define atanf(X) (float)atan(X)
#define atan2f(Y, X) (float)atan2(Y, X)


#define CLASSIC6DOF_L_BS 430.00f    //机器人尺寸
#define CLASSIC6DOF_D_BS 150.00f
#define CLASSIC6DOF_L_SE 447.00f
#define CLASSIC6DOF_L_EW 392.00f
#define CLASSIC6DOF_D_EW 130.00f
#define CLASSIC6DOF_L_WT 100.00f

#define LIMIT_MAX_1 160		//机器人每个轴的限位
#define LIMIT_MIN_1 -160
#define LIMIT_MAX_2 150
#define LIMIT_MIN_2 -65
#define LIMIT_MAX_3 75
#define LIMIT_MIN_3 -90
#define LIMIT_MAX_4 180
#define LIMIT_MIN_4 -180
#define LIMIT_MAX_5 160
#define LIMIT_MIN_5 -160
#define LIMIT_MAX_6 180
#define LIMIT_MIN_6 -180



static float classic6dof_DH[6][4] = {
	//	|home Rz 2		|d Tz 1				|a Tx 3						|alpha Rx 4
	{	0.0f,			CLASSIC6DOF_L_BS,	CLASSIC6DOF_D_BS,			-(float)M_PI_2	},
	{	-(float)M_PI_2,	0.0f,				CLASSIC6DOF_L_SE,			0.0f			},
	{	(float)M_PI_2,	0.0f,				-(float)CLASSIC6DOF_D_EW,	(float)M_PI_2	},
	{	0.0f,			CLASSIC6DOF_L_EW,	0.0f,						-(float)M_PI_2	},
	{	0.0f,			0.0f,				0.0f,				 		(float)M_PI_2	},
	{	0.0f,			CLASSIC6DOF_L_WT,	0.0f,						0.0f			}
}; // home, d, a, alpha

static float L1_bs[3] =	{	 CLASSIC6DOF_D_BS,	-CLASSIC6DOF_L_BS,	0.0f				};
static float L2_se[3] = {	 CLASSIC6DOF_L_SE,	 0.0f,				0.0f				};
static float L3_ew[3] = { -CLASSIC6DOF_D_EW,	 0.0f,				CLASSIC6DOF_L_EW	};
static float L6_wt[3] = {	 0.0f,				 0.0f,				CLASSIC6DOF_L_WT	};


static void matMultiply(float* M1, float* M2, float* M, int m, int l, int n)
{
	float tmp;
	int i, j, k;
	for (i = 0; i < m; i++) {
		for (j = 0; j < n; j++) {
			tmp = 0.0f;
			for (k = 0; k < l; k++) {
				tmp += M1[l*i + k] * M2[n*k+j];
			}
			M[n*i+j] = tmp;
		}
	}
}

static void matRotMatToFixedAngle(float* R, float* fa)
{
	//规定旋转角度的范围是(-M_PI,M_PI],范围不应该超过这个值，这个仅仅代表
	float A, B, C, cb;
	if (fabs(R[6]) >= 1.0 - 0.0001) {   //这种情况cos(fa[1])=1 or -1
		if (R[6] < 0) {
			A = 0.0f;
			B =  (float)M_PI_2;
			C = atan2f(R[1], R[4]);
		} else {
			A = 0.0f;
			B = -(float)M_PI_2;
			C = -atan2f(R[1], R[4]);
		}
	} else {
		B = atan2f(-R[6], sqrtf(R[0] * R[0] + R[3] * R[3]));
		cb = cosf(B);
		A = atan2f(R[3] / cb, R[0] / cb);
		C = atan2f(R[7] / cb, R[8] / cb);
	}
	fa[0] = C;	//rotate Axis X
	fa[1] = B;	//rotate Axis Y
	fa[2] = A;	//rotate Axis Z
}
static void matFixedAngleToRotMat(float* fa, float* R)
{
	float ca, cb, cc, sa, sb, sc; //使用的是RPY的方法，先绕x roll旋转fa[2] gamma,然后是绕Y pitch 上下俯仰角f[1] beta，
	cc = cosf(fa[0]);			  //最后是绕Z yaw偏转角f[0],gamma
	cb = cosf(fa[1]);
	ca = cosf(fa[2]);
	sc = sinf(fa[0]);
	sb = sinf(fa[1]);
	sa = sinf(fa[2]);

	R[0] = ca*cb; R[1] = ca*sb*sc - sa*cc; R[2] = ca*sb*cc + sa*sc;
	R[3] = sa*cb; R[4] = sa*sb*sc + ca*cc; R[5] = sa*sb*cc - ca*sc;
	R[6] = -sb;   R[7] = cb*sc;            R[8] = cb*cc;
}

void classic6dofForKine(float* q_, Kine6d* pose_)
{
	float q[6];
	float cosq, sinq;
	float cosa, sina;
	float d, a;
	float P06[6];
	float R06[16];
	float R[6][16];
	float R02[16];
	float R03[16];
	float R04[16];
	float R05[16];
	float R_tem[9];
	float L0_bs[3];
	float L0_se[3];
	float L0_ew[3];
	float L0_wt[3];
	
	int i;

	for (i = 0; i < 6; i++) {
		q[i] = q_[i] + classic6dof_DH[i][0];
		cosq = cosf(q[i]);
		sinq = sinf(q[i]);
		cosa = cosf(classic6dof_DH[i][3]);
		sina = sinf(classic6dof_DH[i][3]);
		d = classic6dof_DH[i][1];
		a = classic6dof_DH[i][2];

		R[i][0] = cosq; R[i][1] = -cosa * sinq; R[i][2] =  sina * sinq; R[i][3]=a*cosq;
		R[i][4] = sinq; R[i][5] =  cosa * cosq; R[i][6] = -sina * cosq; R[i][7]=a*sinq;
		R[i][8] = 0.0f; R[i][9] =  sina;        R[i][10] =  cosa;		R[i][11]=d;
		R[i][12]=0.0f;	R[i][13]=0.0f;			R[i][14]=0.0f;			R[i][15]=1.0f;
	}

	matMultiply(R[0], R[1], R02, 4, 4, 4);
	matMultiply(R02 , R[2], R03, 4, 4, 4);
	matMultiply(R03 , R[3], R04, 4, 4, 4);
	matMultiply(R04 , R[4], R05, 4, 4, 4);
	matMultiply(R05 , R[5], R06, 4, 4, 4);

	//matMultiply(R[0], L1_bs, L0_bs, 3, 3, 1);
	//matMultiply(R02 , L2_se, L0_se, 3, 3, 1);
	//matMultiply(R03 , L3_ew, L0_ew, 3, 3, 1);
	//matMultiply(R06 , L6_wt, L0_wt, 3, 3, 1);

	//for (i = 0; i < 3; i++) {
	//	P06[i] = L0_bs[i] + L0_se[i] + L0_ew[i] + L0_wt[i];
	//}
	R_tem[0]=R06[0];R_tem[1]=R06[1];R_tem[2]=R06[2];
	R_tem[3]=R06[4];R_tem[4]=R06[5];R_tem[5]=R06[6];
	R_tem[6]=R06[8];R_tem[7]=R06[9];R_tem[8]=R06[10];
	matRotMatToFixedAngle(R_tem, &P06[3]);

	pose_->X = R06[3];
	pose_->Y = R06[7];
	pose_->Z = R06[11];
	pose_->A = P06[3];
	pose_->B = P06[4];
	pose_->C = P06[5];
	memcpy(pose_->R, R06, 16*sizeof(float));
}

void classic6dofInvKine(Kine6d* pose_, float* q_last_, Kine6dSol* q_) //q_last是上一次的位置信息，根据上次的位置信息和现在要求的位置pose
{																		//来求取八组解
	static float l_se_2 = CLASSIC6DOF_L_SE * CLASSIC6DOF_L_SE;
	static float l_se = CLASSIC6DOF_L_SE;
	static float l_ew_2 = CLASSIC6DOF_L_EW * CLASSIC6DOF_L_EW + CLASSIC6DOF_D_EW * CLASSIC6DOF_D_EW;
	static float l_ew = 0;
	static float atan_e = 0;

	float qs[2];
	float qa[2][2];
	float qw[2][3];
	
	float cosqs, sinqs;
	float cosqa[2], sinqa[2];
	float cosqw, sinqw;
	float P06[6];   //记录末端位姿
	float R06[9];	//记录旋转矩阵
	float P0_w[3];	//记录去除Lwt段后，剩余部分的XYZ坐标值
	float P1_w[3];	//记录在坐标系{1}下某段的坐标系
	float L0_wt[3];  //Lwt段在基座标下的坐标值
	float L1_sw[3];
	float R10[9];
	float R31[9];
	float R30[9];
	float R36[9];
	float l_sw_2, l_sw, atan_a, acos_a, acos_e;

	int ind_arm, ind_elbow, ind_wrist; //和arm elbow wrist相关的索引变量，和三个转动关节相关
	int i;

	if (0 == l_ew) {
		l_ew = sqrtf(l_ew_2);
		atan_e = atanf(CLASSIC6DOF_D_EW / CLASSIC6DOF_L_EW);
	}

	P06[0] = pose_->X;
	P06[1] = pose_->Y;
	P06[2] = pose_->Z;
	if (0 == pose_->fgR) {    //fgR是标志位，用来表示pose_->R是否求出
		P06[3] = pose_->A;
		P06[4] = pose_->B;
		P06[5] = pose_->C;
		matFixedAngleToRotMat(&P06[3], R06);
	} else {
		memcpy(R06, pose_->R, 9*sizeof(float));
	}
	for (i = 0; i < 2; i++) {  //记录了两组解
		qs[i] = q_last_[0]; //第一个关节角
		qa[i][0] = q_last_[1]; qa[i][1] = q_last_[2]; //第二、三个角度值
		qw[i][0] = q_last_[3]; qw[i][1] = q_last_[4]; qw[i][2] = q_last_[5]; //第四、五、六关节的关节角
	}
	// q1 solution pair ///////////
	matMultiply(R06, L6_wt, L0_wt, 3, 3, 1);  //将{6}和Lwt的原点与起始点都平移到基座标{0}的原点，L6_wt是Lwt在坐标系{6}下的投影，通过R06求出Lwt在基座标系{0}下的投影坐标L0_wt。
	for (i = 0; i < 3; i++) {
		P0_w[i] = P06[i] - L0_wt[i];   //都要以基座标作为参考系，每段在基座标系下的投影之和就是最后TCP点的坐标
	}
	if (sqrt(P0_w[0]*P0_w[0] + P0_w[1]*P0_w[1]) <= 0.000001) { //除去Lwt段之后剩余部分的坐标X、Y=0，仅剩Z不为0
		qs[0] = q_last_[0]; // right arm   出现这种情况时，1轴不动，所以等于q_last_[0]
		qs[1] = q_last_[0]; // left arm
		for (i = 0; i < 4; i++) {
			q_->sol_flag[0 + i][0] = -1;   //标志第一个轴的这种情况
			q_->sol_flag[4 + i][0] = -1;
		}
	} else {
		qs[0] = atan2f( P0_w[1],  P0_w[0]); // right arm   用来标识方向角，两个对称的方向角
		qs[1] = atan2f(-P0_w[1], -P0_w[0]); // left arm    Axis1转动的两个对称的方位角，相距180度。
		for (i = 0; i < 4; i++) {
			q_->sol_flag[0 + i][0] =  1;
			q_->sol_flag[4 + i][0] =  1;
		}
	}
	// two arm config. ////////////
	for (ind_arm = 0; ind_arm < 2; ind_arm++) {     //对应上面两种情况，Axis1两种对称的位置。
		// q2, q3 solution pair ///                 //三个循环的嵌套，从最外层的arm->elbow->wrist，相当于穷举，每个转动轴都有两个方向
		cosqs = cosf(qs[ind_arm] + classic6dof_DH[0][0]); 
		sinqs = sinf(qs[ind_arm] + classic6dof_DH[0][0]);  //下面这个变化仅仅和轴1的角度有关
		printf("qs[%d]=%f\n",ind_arm,qs[ind_arm]);
		R10[0] =  cosqs; R10[1] = sinqs; R10[2] =  0.0f;   //这是一个变基函数，将P0_w->P1_w，即将基地坐标系有零（基座标系）
		R10[3] =   0.0f; R10[4] =  0.0f; R10[5] = -1.0f;	//变到{1}joint1。这样就能做坐标系{1}下进行加减运算 》旋转矩阵为单位正交矩阵，求转置即为逆解《
		R10[6] = -sinqs; R10[7] = cosqs; R10[8] =  0.0f;	//

		matMultiply(R10, P0_w, P1_w, 3, 3, 1);   
		
		for (i = 0; i < 3; i++) {
			L1_sw[i] = P1_w[i] - L1_bs[i];   //把Lbs和dbs减去  L1_bs就是在坐标系{1}，原点和基座标系重合是，LBS和dbs组成的系统末端在{1}下的坐标
		}
		l_sw_2 = L1_sw[0]*L1_sw[0] + L1_sw[1]*L1_sw[1];
		l_sw = sqrtf(l_sw_2);
		printf("l_se=%f,l_ew=%f,l_sw=%f\n",l_se,l_ew,l_sw);
		if(fabs(l_se + l_ew - l_sw) <= 0.001) {   //在一条直线上   PS: l_ew和Lew是不同的，是直角三角形里面的一个直角边和斜边
			printf("在一条直线上，#1\n");
			qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);   //第二个关节角度
			qa[1][0] = qa[0][0];                     //第二个关节的角度
			
			qa[0][1] = 0.0f;						 
			qa[1][1] = 0.0f;						 
			if (l_sw > l_se + l_ew) {
				for (i = 0; i < 2; i++) {
					q_->sol_flag[4*ind_arm + 0 + i][1] = 0;  //根据ind_arm分成两组，每组有四个解，一共是八个解
					q_->sol_flag[4*ind_arm + 2 + i][1] = 0;		//用于标识是否能组成三角形，即两边之和和第三遍的关系，若能组成三角形为1，若不符合三角形为0
				}
			} else {
				for (i = 0; i < 2; i++) {
					q_->sol_flag[4*ind_arm + 0 + i][1] = 1;
					q_->sol_flag[4*ind_arm + 2 + i][1] = 1;
				}
			}
		} else if(fabs(l_sw - fabs(l_se - l_ew)) <= 0.001) {   //这是另一种在一条直线上的情况 位
			printf("在一条直线上，#2\n");
			qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
			qa[1][0] = qa[0][0];
			if (0 == ind_arm) { // right arm   要根据一轴的情况来确定3轴的角度。
				qa[0][1] =  (float)M_PI; // above elbow
				qa[1][1] = -(float)M_PI; // below elbow
			} else { // /////// // left arm
				qa[0][1] = -(float)M_PI; // above elbow
				qa[1][1] =  (float)M_PI; // below elbow
			}
			if	(l_sw < fabs(l_se - l_ew)) {
				for (i = 0; i < 2; i++) {
					q_->sol_flag[4*ind_arm + 0 + i][1] = 0;//求axis2 3两个轴的，对第二个flag进行标记
					q_->sol_flag[4*ind_arm + 2 + i][1] = 0;
				}
			} else {
				for (i = 0; i < 2; i++) {
					q_->sol_flag[4*ind_arm + 0 + i][1] = 1;
					q_->sol_flag[4*ind_arm + 2 + i][1] = 1;
				}
			}
		} else {     //正常的不在一条直线上的情况，这种情况显然可以分成两种情况
			printf("不在一条直线上\n");
			atan_a = atan2f(L1_sw[1], L1_sw[0]); //注意和atan_e做出区分，这个值是l_sw与{1}的x轴的夹角，很可能是负值，搞清楚atan2f的使用方法
			acos_a = 0.5f*(l_se_2 + l_sw_2 - l_ew_2) / (l_se*l_sw);  //这个是余弦定理   l_ew对应的角度
			if	(acos_a >=  1.0f) acos_a = 0.0f;  //余弦的两种边界情况
			else if	(acos_a <= -1.0f) acos_a = (float)M_PI;
			else	acos_a = acosf(acos_a);       //arcos,求出角度
			acos_e = 0.5f*(l_se_2 + l_ew_2 - l_sw_2) / (l_se*l_ew);  //求出l_sw对应的角度余弦
			if	(acos_e >=  1.0f) acos_e = 0.0f;
			else if	(acos_e <= -1.0f) acos_e = (float)M_PI;
			else	acos_e = acosf(acos_e);
			if (0 == ind_arm) { // right arm  代表在坐标系{1}中的右半部分，，，，
				// above elbow		肘关节向上,三关节在l_sw的左侧
				qa[0][0] = atan_a - acos_a + (float)M_PI_2;  //axis 3    
				qa[0][1] = atan_e - acos_e + (float)M_PI;    //axis 4
				// below elbow    //肘部在下方
				qa[1][0] = atan_a + acos_a + (float)M_PI_2;   //axis3    应该是肘关节向下
				qa[1][1] = atan_e + acos_e - (float)M_PI;	  //axis4

			} else { // /////// // left arm   这种情况时，图形关于Y1轴对称
				// above elbow
				qa[0][0] = atan_a + acos_a + (float)M_PI_2;   //肘部在上方
				qa[0][1] = atan_e + acos_e - (float)M_PI;
				// below elbow     这种情况是肘部在下方
				qa[1][0] = atan_a - acos_a + (float)M_PI_2;
				qa[1][1] = atan_e - acos_e + (float)M_PI;
			}
			for (i = 0; i < 2; i++) {
				q_->sol_flag[4*ind_arm + 0 + i][1] = 1;  //这种情况肯定能组成三角形
				q_->sol_flag[4*ind_arm + 2 + i][1] = 1;
			}
		}
		// two elbow config. ////////    肘部也是两种情况，肘部朝上或者肘部朝下
		for (ind_elbow = 0; ind_elbow < 2; ind_elbow++) {
			// q3,q4,q5 solution pair
			cosqa[0] = cosf(qa[ind_elbow][0] + classic6dof_DH[1][0]); sinqa[0] = sinf(qa[ind_elbow][0] + classic6dof_DH[1][0]);
			cosqa[1] = cosf(qa[ind_elbow][1] + classic6dof_DH[2][0]); sinqa[1] = sinf(qa[ind_elbow][1] + classic6dof_DH[2][0]);

			R31[0] = cosqa[0]*cosqa[1] - sinqa[0]*sinqa[1]; R31[1] =   cosqa[0]*sinqa[1] + sinqa[0]*cosqa[1]; R31[2] = 0.0f;
			R31[3] = 0.0f; R31[4] = 0.0f; R31[5] = 1.0f;
			R31[6] = cosqa[0]*sinqa[1] + sinqa[0]*cosqa[1]; R31[7] = - cosqa[0]*cosqa[1] + sinqa[0]*sinqa[1]; R31[8] = 0.0f;

			matMultiply(R31, R10, R30, 3, 3, 3);
			matMultiply(R30, R06, R36, 3, 3, 3);

			if			(R36[8] >= 1.0 - 0.000001) {
				cosqw =  1.0f;
				qw[0][1] = 0.0f;
				qw[1][1] = 0.0f;
			} else if	(R36[8] <= -1.0 + 0.000001) {
				cosqw = -1.0f;
				if (0 == ind_arm) { // right arm
					qw[0][1] =  (float)M_PI;
					qw[1][1] = -(float)M_PI;
				} else { // /////// // left arm
					qw[0][1] = -(float)M_PI;
					qw[1][1] =  (float)M_PI;
				}
			} else {
				cosqw = R36[8];
				if (0 == ind_arm) { // right arm
					qw[0][1] =  acosf(cosqw); // up wrist
					qw[1][1] = -acosf(cosqw); // down wrist
				} else { // /////// // left arm
					qw[0][1] = -acosf(cosqw); // up wrist
					qw[1][1] =  acosf(cosqw); // down wrist
				}
			}
			if (1.0f == cosqw || -1.0f == cosqw) {
				if (0 == ind_arm) { // right arm
					// q4 = q_last
					qw[0][0] = q_last_[3];
					cosqw = cosf(q_last_[3] + classic6dof_DH[3][0]); sinqw = sinf(q_last_[3] + classic6dof_DH[3][0]);
					qw[0][2] = atan2f(cosqw*R36[3] - sinqw*R36[0], cosqw*R36[0] + sinqw*R36[3]);
					// q6 = q_last
					qw[1][2] = q_last_[5];
					cosqw = cosf(q_last_[5] + classic6dof_DH[5][0]); sinqw = sinf(q_last_[5] + classic6dof_DH[5][0]);
					qw[1][0] = atan2f(cosqw*R36[3] - sinqw*R36[0], cosqw*R36[0] + sinqw*R36[3]);
				} else { // /////// // left arm
					// q6 = q_last
					qw[0][2] = q_last_[5];
					cosqw = cosf(q_last_[5] + classic6dof_DH[5][0]); sinqw = sinf(q_last_[5] + classic6dof_DH[5][0]);
					qw[0][0] = atan2f(cosqw*R36[3] - sinqw*R36[0], cosqw*R36[0] + sinqw*R36[3]);
					// q4 = q_last
					qw[1][0] = q_last_[3];
					cosqw = cosf(q_last_[3] + classic6dof_DH[3][0]); sinqw = sinf(q_last_[3] + classic6dof_DH[3][0]);
					qw[1][2] = atan2f(cosqw*R36[3] - sinqw*R36[0], cosqw*R36[0] + sinqw*R36[3]);
				}
				q_->sol_flag[4*ind_arm+2*ind_elbow+0][2] = -1;
				q_->sol_flag[4*ind_arm+2*ind_elbow+1][2] = -1;
			} else {
				if (0 == ind_arm) { // right arm
					// q4
					qw[0][0] = atan2f( R36[5],  R36[2]); // up wrist
					qw[1][0] = atan2f(-R36[5], -R36[2]); // down wrist
					// q6
					qw[0][2] = atan2f( R36[7], -R36[6]); // up wrist
					qw[1][2] = atan2f(-R36[7],  R36[6]); // down wrist
				} else { // /////// // left arm
					// q4
					qw[0][0] = atan2f(-R36[5], -R36[2]); // up wrist
					qw[1][0] = atan2f( R36[5],  R36[2]); // down wrist
					// q6
					qw[0][2] = atan2f(-R36[7],  R36[6]); // up wrist
					qw[1][2] = atan2f( R36[7], -R36[6]); // down wrist
				}
				q_->sol_flag[4*ind_arm+2*ind_elbow+0][2] =  1;
				q_->sol_flag[4*ind_arm+2*ind_elbow+1][2] =  1;
			}
			// two wrist config. ////
			for (ind_wrist = 0; ind_wrist < 2; ind_wrist++) {
				if		(qs[ind_arm] >  (float)M_PI)
					q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][0] = qs[ind_arm] - (float)M_PI;
				else if	(qs[ind_arm] < -(float)M_PI)
					q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][0] = qs[ind_arm] + (float)M_PI;
				else
					q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][0] = qs[ind_arm];
				for (i = 0; i < 2; i++) {
					if		(qa[ind_elbow][i] >  (float)M_PI)
						q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][1 + i] = qa[ind_elbow][i] - (float)M_PI;
					else if	(qa[ind_elbow][i] < -(float)M_PI)
						q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][1 + i] = qa[ind_elbow][i] + (float)M_PI;
					else
						q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][1 + i] = qa[ind_elbow][i];
				}
				for (i = 0; i < 3; i++) {
					if		(qw[ind_wrist][i] >  (float)M_PI) 
						q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][3 + i] = qw[ind_wrist][i] - (float)M_PI;
					else if	(qw[ind_wrist][i] < -(float)M_PI)
						q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][3 + i] = qw[ind_wrist][i] + (float)M_PI;
					else
						q_->sol[4*ind_arm+2*ind_elbow+ind_wrist][3 + i] = qw[ind_wrist][i];
				}
			} // for ind_wrist
		} // for ind_elbow
	} // for ind_arm
	
}
