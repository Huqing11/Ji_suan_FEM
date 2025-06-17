#pragma once
#include"LoadBase.h"
class Node;

class LoadNode :public LoadBase
{
	//节点荷载
public:
	std::weak_ptr<Node>m_pNode;//作用的节点
	double m_P[3];//各方向荷载大小
	virtual void Assemble(VectorXd& F1, VectorXd& F2) ;
};

