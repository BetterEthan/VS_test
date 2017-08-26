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


//ͨ�����õ����������ٶȽ��н���
//�ʵ������Ľ��٣������ǵð������ٶ����ݷ���ringbuffer��
//wheelOne һ�����ٶ������׵�ַ
//wheelTwo 	�������ٶ������׵�ַ
//wheelThree �������ٶ������׵�ַ
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree)
{
	float time = 0.0f;
	int n = GetCount();
	float tempAcc = 0.0f;

	//ÿ�μ��ٶȽ������ϴεİٷ�ֵ
	float percent = 0.9;

	//�����������ٶ�
	for (int i = 2; i < n + 1; i++)
	{

		//���Լ���ÿ��ʾ�̵�֮����˶���ʱ��
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		//��1
		//ֻ�����ٶ�ͬ������
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
			//ÿ������0.05�ļ��ٶ�
			wheelOne[i - 1] = wheelOne[i - 1] > 0 ? wheelOne[i - 2] + tempAcc*percent * time : wheelOne[i - 2] - tempAcc*percent * time;
		}
		//��2
		//ֻ�����ٶ�ͬ������
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
			//ÿ������0.05�ļ��ٶ�
			wheelTwo[i - 1] = wheelTwo[i - 1] > 0 ? wheelTwo[i - 2] + tempAcc*percent * time : wheelTwo[i - 2] - tempAcc*percent * time;
		}
		//��3
		//ֻ�����ٶ�ͬ������
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
			//ÿ������0.05�ļ��ٶ�
			wheelThree[i - 1] = wheelThree[i - 1] > 0 ? wheelThree[i - 2] + tempAcc*percent * time : wheelThree[i - 2] - tempAcc*percent * time;
		}
	}

	//���������ٶ�
	for (int i = n; i > 0; i--)
	{

		//���Լ���ÿ��ʾ�̵�֮����˶���ʱ��
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		//��1
		//ֻ�����ٶ�ͬ������
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
			//ÿ������0.05�ļ��ٶ�
			wheelOne[i - 2] = wheelOne[i - 2] > 0 ? wheelOne[i - 1] - tempAcc*percent * time : wheelOne[i - 1] + tempAcc*percent * time;
		}


		//��2
		//ֻ�����ٶ�ͬ������
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
			//ÿ������0.05�ļ��ٶ�
			wheelTwo[i - 2] = wheelTwo[i - 2] > 0 ? wheelTwo[i - 1] - tempAcc*percent * time : wheelTwo[i - 1] + tempAcc*percent * time;
		}


		//��3
		//ֻ�����ٶ�ͬ������
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
			//ÿ������0.05�ļ��ٶ�
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










//ͨ��ringBuffer������ݼ���ÿһ�㴦�������ӵ��ٶ�
//Ŀ�ĸ���wheelOne wheelTwo wheelThree������������������ٶȣ�������һ�ε��ٶ�����
//wheelOne һ�����ٶ������׵�ַ
//wheelTwo 	�������ٶ������׵�ַ
//wheelThree �������ٶ������׵�ַ
void CalculateThreeWheelVell(float* wheelOne, float* wheelTwo, float* wheelThree)
{
	//�ֽ⵽�����ֶ�ȫ���ٶȽ��й滮
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
////ͨ�����ͺ��ٶȱ�֤ĳ�ֵ��ٶ�Ҫ��
////vellCar ����ǰ��ǰ�����ٶ� ��λ mm/s
////orientation �ٶȳ��� ��λ ��
////rotationalVell ��ת�ٶ� ��λ ��ÿ��
////wheelNum  �����ٵ��ֺ� 
//// targetWheelVell   �����ٵ�Ŀ��
//// ���������ͺ�ĺ��ٶ�
//float DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell, float zAngle, int wheelNum, float targetWheelVell)
//{
//	TriWheelVel_t vell;
//	int i;
//	switch (wheelNum)
//	{
//	case 1:
//		//ÿ�κ��ٶȳ�0.9,ֱ������һ�����ٶȽ�����Ŀ���ٶȡ�����һЩ��������ģ�ѭ��10�κ��Զ��˳�
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
//�ٶȹ滮����
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


	//�������ٶ��ܹ��������С���ʰ뾶
	float curvatureMaxVell = GetVelMax() * GetVelMax() / (2 * GetAccMax());


	//����������ʰ뾶ȫ��Ϊ������ٶ����������С���ʰ뾶
	for (int i = 0; i < n; i++)
	{
		if (curvature[i] > curvatureMaxVell)
		{
			curvature[i] = curvatureMaxVell;
		}
	}
	curvature[0] = curvature[1];
	curvature[n - 1] = curvature[n - 2];


	//ͨ�����ʰ뾶����ö������������ٶ�                                         
	for (int i = 0; i < n; i++)
	{
		vell[i] = sqrt((2 * GetAccMax()) * curvature[i]);
	}


	vell[0] = 100;
	vell[n - 1] = 100;

	float tempVell = 0.0f;
	//ͨ��v2^2 - v1^2 = 2*a*s���ٶ��ٴι滮
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

	//����ʱ�滮���ٶȷ��뻷��������
	for (int i = 0; i < n; i++)
	{
		SetRingBufferPointVell(i + 1, vell[i]);
	}

}






