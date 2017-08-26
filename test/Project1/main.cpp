#include "stdio.h"
#include "calculate.h"
#include "MotionCard.h"
#include "ringbuffer.h"
#include <iostream>
using namespace std;


#define NUM 3
Pose_t aaa[NUM] = {
	{0,0,0},
	{500,500,0},
	{500,0,0}

};



void main()
{
	BufferZizeInit(500);
	InputPoints2RingBuffer(aaa,NUM);
	for (int i = 1; i <= GetCount(); i++)
	{
		cout << GetRingBufferPointVell(i) << endl;
	}


	printf("aaaa");
	while (true);
}