#pragma once
#include"ElementBase.h"
class ElementLine2D:public ElementBase
{
//平面2节点线单元基类 
public:
	ElementLine2D();
	//计算长度，返回方向余弦(c,s)
	double Get_Les(double& c, double& s);
};

