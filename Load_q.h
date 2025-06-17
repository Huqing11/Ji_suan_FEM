#pragma once
#include"LoadBase.h"
#include<vector>

class ElementBase;

class Load_q:public LoadBase
{
	//均部荷载
public:
	bool m_bLocalCoord = true;//缺省定义在局部单元系下
	double m_qx, n_qm;
	std::vector < std::weak_ptr<ElementBase>>m_Elements;

	virtual void Assemble(VectorXd& F1, VectorXd& F2) ;
	//计算在一个单元上的等效节点载荷（在整体坐标系下）
	void GetFeq(std::shared_ptr<ElementBase> pElement,VectorXd & Feq);
};

