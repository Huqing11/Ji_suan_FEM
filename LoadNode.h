#pragma once
#include"LoadBase.h"
class Node;

class LoadNode :public LoadBase
{
	//�ڵ����
public:
	std::weak_ptr<Node>m_pNode;//���õĽڵ�
	double m_P[3];//��������ش�С
	virtual void Assemble(VectorXd& F1, VectorXd& F2) ;
};

