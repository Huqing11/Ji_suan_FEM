#pragma once
#include"EntityBase.h"
class Node;
class SectionBase;
#include<vector>
class ElementBase:public EntityBase
{//单元基类
public:
	std::vector<std::weak_ptr<Node>>m_pNode;//多个节点
	std::weak_ptr<SectionBase>m_pSection;//截面基类指针

	virtual int GetNodeDof() = 0;//每个节点的自由度个数
	void GetDOFs(std::vector<int>& DOFs);//取得单元的各个节点的自由度编号
	virtual void Get_ke(MatrixXd& ke)=0;
};

