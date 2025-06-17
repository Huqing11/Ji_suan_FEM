#pragma once

#include"ElementLine2D.h"

class ElementTruss2D:public ElementLine2D
{
	//平面2节点桁架单元
public:


	void Get_ke(MatrixXd& ke);

	virtual int GetNodeDof() { return 2; };//每个节点的自由度个数
	
};

