#include "SpeedPlaning.h"
#include <stdlib.h>
#include "MotionCard.h"
#include "ringBuffer.h"
#include "math.h"
#include "calculate.h"
float GetAccMax(void)
{
	return 700;
}

float GetVelMax(void)
{
	return 2000;
}


//通过配置的轮子最大加速度进行降速
//适当比例的降速，算完后记得把最新速度数据放在ringbuffer里
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree)
{
	float time = 0.0f;
	int n = GetCount();
	float tempAcc = 0.0f;

	//每次加速度降低至上次的百分值
	float percent = 0.9;

	//先正向削减速度
	for (int i = 2; i < n + 1; i++)
	{

		//粗略计算每两示教点之间的运动的时间
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		//轮1
		//只处理速度同向的情况
		if (wheelOne[i - 1] * wheelOne[i - 2] > 0)
		{
			tempAcc = (fabs(wheelOne[i - 1]) - fabs(wheelOne[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax())
		{
			//每次削减0.05的加速度
			wheelOne[i - 1] = wheelOne[i - 1] > 0 ? wheelOne[i - 2] + tempAcc*percent * time : wheelOne[i - 2] - tempAcc*percent * time;
		}
		//轮2
		//只处理速度同向的情况
		if (wheelTwo[i - 1] * wheelTwo[i - 2] > 0)
		{
			tempAcc = (fabs(wheelTwo[i - 1]) - fabs(wheelTwo[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax())
		{
			//每次削减0.05的加速度
			wheelTwo[i - 1] = wheelTwo[i - 1] > 0 ? wheelTwo[i - 2] + tempAcc*percent * time : wheelTwo[i - 2] - tempAcc*percent * time;
		}
		//轮3
		//只处理速度同向的情况
		if (wheelThree[i - 1] * wheelThree[i - 2] > 0)
		{
			tempAcc = (fabs(wheelThree[i - 1]) - fabs(wheelThree[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax())
		{
			//每次削减0.05的加速度
			wheelThree[i - 1] = wheelThree[i - 1] > 0 ? wheelThree[i - 2] + tempAcc*percent * time : wheelThree[i - 2] - tempAcc*percent * time;
		}
	}

	//反向削减速度
	for (int i = n; i > 0; i--)
	{

		//粗略计算每两示教点之间的运动的时间
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		//轮1
		//只处理速度同向的情况
		if (wheelOne[i - 1] * wheelOne[i - 2] > 0)
		{
			tempAcc = (fabs(wheelOne[i - 1]) - fabs(wheelOne[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//每次削减0.05的加速度
			wheelOne[i - 2] = wheelOne[i - 2] > 0 ? wheelOne[i - 1] - tempAcc*percent * time : wheelOne[i - 1] + tempAcc*percent * time;
		}


		//轮2
		//只处理速度同向的情况
		if (wheelTwo[i - 1] * wheelTwo[i - 2] > 0)
		{
			tempAcc = (fabs(wheelTwo[i - 1]) - fabs(wheelTwo[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//每次削减0.05的加速度
			wheelTwo[i - 2] = wheelTwo[i - 2] > 0 ? wheelTwo[i - 1] - tempAcc*percent * time : wheelTwo[i - 1] + tempAcc*percent * time;
		}


		//轮3
		//只处理速度同向的情况
		if (wheelThree[i - 1] * wheelThree[i - 2] > 0)
		{
			tempAcc = (fabs(wheelThree[i - 1]) - fabs(wheelThree[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//每次削减0.05的加速度
			wheelThree[i - 2] = wheelThree[i - 2] > 0 ? wheelThree[i - 1] - tempAcc*percent * time : wheelThree[i - 1] + tempAcc*percent * time;
		}
	}



	for (int i = 0; i < n - 1; i++)
	{
		TriWheelVel_t tempTrueVell;
		TriWheelVel2_t tempVel2;
		tempTrueVell.v1 = wheelOne[i];
		tempTrueVell.v2 = wheelTwo[i];
		tempTrueVell.v3 = wheelThree[i];

		//tempVel2 = GetTrueVell(tempTrueVell, GetRingBufferPointPoseAngle(i + 1));
		//SetRingBufferPointVell(i + 1, tempVel2.speed);
	}

}










//通过ringBuffer里的数据计算每一点处三个轮子的速度
//目的更新wheelOne wheelTwo wheelThree这三个数组里的三轮速度，便于下一次的速度削减
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void CalculateThreeWheelVell(float* wheelOne, float* wheelTwo, float* wheelThree)
{
	//分解到三个轮对全局速度进行规划
	TriWheelVel_t threeVell;
	float n = GetCount();


	for (int i = 2; i < n + 1; i++)
	{
		float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;

		float time = 0.0f;

		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

		float rotationVell = angErr / time;


		//threeVell = CaculateThreeWheelVel(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell, GetRingBufferPointPoseAngle(i));


		//wheelOne[i - 1] = threeVell.v1;

		//wheelTwo[i - 1] = threeVell.v2;

		//wheelThree[i - 1] = threeVell.v3;

	}
	wheelOne[0] = wheelOne[1];
	wheelTwo[0] = wheelTwo[1];
	wheelThree[0] = wheelThree[1];

}



//
//
////通过降低合速度保证某轮的速度要求
////vellCar 降速前的前进合速度 单位 mm/s
////orientation 速度朝向 单位 度
////rotationalVell 旋转速度 单位 度每秒
////wheelNum  所降速的轮号 
//// targetWheelVell   所降速的目标
//// 返回所降低后的合速度
//float DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell, float zAngle, int wheelNum, float targetWheelVell)
//{
//	TriWheelVel_t vell;
//	int i;
//	switch (wheelNum)
//	{
//	case 1:
//		//每次合速度乘0.9,直到满足一号轮速度降低至目标速度。对于一些不能满足的，循环10次后自动退出
//		for (i = 0; i < 10; i++)
//		{
//
//			vellCar *= 0.9f;
//			vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
//			if (fabs(vell.v1) < fabs(targetWheelVell))
//			{
//				break;
//			}
//		}
//		break;
//
//	case 2:
//		for (i = 0; i < 10; i++)
//		{
//
//			vellCar *= 0.9f;
//			vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
//			if (fabs(vell.v2) < fabs(targetWheelVell))
//			{
//				break;
//			}
//		}
//		break;
//
//	case 3:
//		for (i = 0; i < 10; i++)
//		{
//
//			vellCar *= 0.9f;
//			vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
//			if (fabs(vell.v3) < fabs(targetWheelVell))
//			{
//				break;
//			}
//		}
//		break;
//	}
//	return vellCar;
//}

#define MIN_VELL 70
//速度规划函数
void SpeedPlaning()
{
	float* vell = NULL;
	float* curvature = NULL;



	int n = GetCount();

	vell = (float *)malloc(n * sizeof(float));
	curvature = (float *)malloc(n * sizeof(float));



	for (int i = 0; i < n; i++)
	{
		curvature[i] = GetRingBufferAverCurvature(i + 1);
	}
	curvature[n - 1] = curvature[n - 2];
	//	curvature[0] = curvature[1];


	//电机最大速度能够满足的最小曲率半径
	float curvatureMaxVell = GetVelMax() * GetVelMax() / (2 * GetAccMax());


	//将过大的曲率半径全设为能最大速度能满足的最小曲率半径
	for (int i = 0; i < n; i++)
	{
		if (curvature[i] > curvatureMaxVell)
		{
			curvature[i] = curvatureMaxVell;
		}
	}
	curvature[0] = curvature[1];
	curvature[n - 1] = curvature[n - 2];


	//通过曲率半径计算该段能满足的最大速度                                         
	for (int i = 0; i < n; i++)
	{
		vell[i] = sqrt((2 * GetAccMax()) * curvature[i]);
	}


	vell[0] = 100;
	vell[n - 1] = 100;

	float tempVell = 0.0f;
	//通过v2^2 - v1^2 = 2*a*s对速度再次规划
	for (int i = 0; i < n - 1; i++)
	{
		if (vell[i + 1] > vell[i])
		{
			tempVell = sqrt(2 * (1 * GetAccMax()) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			if (tempVell < vell[i + 1])
			{
				vell[i + 1] = tempVell;
			}
		}
	}

	for (int i = n - 1; i > 0; i--)
	{
		if (vell[i - 1] > vell[i])
		{
			tempVell = sqrt(2 * (1 * GetAccMax()) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			if (tempVell < vell[i - 1])
			{
				vell[i - 1] = tempVell;
			}
		}
	}

	//将暂时规划的速度放入环形数组里
	for (int i = 0; i < n; i++)
	{
		SetRingBufferPointVell(i + 1, vell[i]);
	}

}






