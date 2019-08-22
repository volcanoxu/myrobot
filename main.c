#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Classic6dofKine.h"

int main()
{
	float q0[6] = { 0,0,0,0,0,0 };
	float q[6] = { 0,0,1.570796f,0,1.570796f,0 };
	//****************选取几组数进行验证
	float joint[5][6]={
		{0,0,0,0,0,0},
		{20,45,60,90,0,0},
		{-30,-45,90,-90,90,0},
		{-90,120,150,0,0,90},
		{20,30,40,50,60,70}
		
	};
	int ii,jj;
	//*****************************
	float q_last[6] = { 0,0,0,0,0,0 };
	Kine6d pose;
	Kine6dSol q_sol;
	char ch = '\0';
	int i;

	pose.fgR = 0; //用来表示旋转矩阵是否求出

	//classic6dofForKine(q0, &pose);
	//printf("FK(q0)  :=< %.6f, %.6f, %.6f, %.6f, %.6f, %.6f >\n", pose.X, pose.Y, pose.Z, pose.A, pose.B, pose.C);
	//选取五组位置，尽量让位置具有代表性
	for(int ii=0;ii<5;ii++){
		
		for(jj=0;jj<6;jj++){
			joint[ii][jj]*=(3.14159265f / 180.0f);
		}
		printf("group%d    :=< %11.6f, %11.6f, %11.6f, %11.6f, %11.6f, %11.6f >\n",ii+1,joint[ii][0],joint[ii][1],joint[ii][2],joint[ii][3],joint[ii][4],joint[ii][5]);	
		classic6dofForKine(joint[ii],&pose);
		printf("FK(group%d):=< %11.6f, %11.6f, %11.6f, %11.6f, %11.6f, %11.6f >\n", ii+1, pose.X, pose.Y, pose.Z, pose.A, pose.B, pose.C);
	}
	
	
	while (1) {
		printf("quit 'q' or continue 'c' ?");
		//fflush(stdin);
		scanf(" %c", &ch);
		printf("%c\n", ch);
		if ('q' == ch) break;
		while ((ch = getchar()) != '\n' && ch != EOF);

		printf("input q:=");
		scanf("%f %f %f %f %f %f", &q[0], &q[1], &q[2], &q[3], &q[4], &q[5]);
		while ((ch = getchar()) != '\n' && ch != EOF);
		
		for (i = 0; i < 6; i++) {
			q[i] *= (3.14159265f / 180.0f); //将角度转化成弧度
		}
		printf("\nq       :=< %.6f, %.6f, %.6f, %.6f, %.6f, %.6f >\n", q[0], q[1], q[2], q[3], q[4], q[5]);
		
		classic6dofForKine(q, &pose);
		printf("FK      :=< %.6f, %.6f, %.6f, %.6f, %.6f, %.6f >\n", pose.X, pose.Y, pose.Z, pose.A, pose.B, pose.C);
		
		classic6dofInvKine(&pose, q_last, &q_sol);

		for (i = 0; i < 8; i++) {
			printf("q[%d]    :=< %.6f, %.6f, %.6f, %.6f, %.6f, %.6f >\n", i, q_sol.sol[i][0], q_sol.sol[i][1], q_sol.sol[i][2], q_sol.sol[i][3], q_sol.sol[i][4], q_sol.sol[i][5]);
			classic6dofForKine(q_sol.sol[i], &pose);   //可以通过求正解来验证逆解的正确性，把不正确的逆解去除掉，留下正确的逆解。
			printf("FK(q[%d]):=< %.6f, %.6f, %.6f, %.6f, %.6f, %.6f >\n", i, pose.X, pose.Y, pose.Z, pose.A, pose.B, pose.C);
		}

		memcpy(q_last, q, 6*sizeof(float)); //q_last用来记录上次计算的角度值q    void * memcpy ( void * destination, const void * source, size_t num );   上次计算的逆解对这次计算逆解有影响
	}    
	//system("pause");
	return 0;
}
