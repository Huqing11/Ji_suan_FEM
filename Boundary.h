#pragma once
#include "EntityBase.h"
class Node;
class Boundary:public EntityBase
{//�߽�������
public:
	std::weak_ptr<Node> m_pNode;//Լ���Ľڵ�
	int m_Direction;//0->X,1->Y
	double m_Value;//λ����
};

