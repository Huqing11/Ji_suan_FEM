#pragma once
#include"EntityBase.h"
class Node:public  EntityBase
{//�ڵ���
public:
	double m_x, m_y;
	std::vector<int> m_DOF;	//���ɶȱ��
	Node();
};

