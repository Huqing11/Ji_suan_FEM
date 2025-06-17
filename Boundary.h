#pragma once
#include "EntityBase.h"
class Node;
class Boundary:public EntityBase
{//边界条件类
public:
	std::weak_ptr<Node> m_pNode;//约束的节点
	int m_Direction;//0->X,1->Y
	double m_Value;//位移量
};

