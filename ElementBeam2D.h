#pragma once
#include"ElementLine2D.h"
class ElementBeam2D:public ElementLine2D
{//平面2节点梁单元
public:
	virtual int GetNodeDof() { return 3; }
	virtual void Get_ke(MatrixXd& ke) ;

	void Get_keLocal(MatrixXd& keLocal);//计算单元坐标系下的单元刚度矩阵
};

