#pragma once
#include"EntityBase.h"
class Node:public  EntityBase
{//节点类
public:
	double m_x, m_y;
	std::vector<int> m_DOF;	//自由度编号
	Node();
};

