#pragma once
#include"ElementBase.h"
class ElementLine2D:public ElementBase
{
//ƽ��2�ڵ��ߵ�Ԫ���� 
public:
	ElementLine2D();
	//���㳤�ȣ����ط�������(c,s)
	double Get_Les(double& c, double& s);
};

