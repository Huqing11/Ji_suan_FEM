#pragma once

#include"ElementLine2D.h"

class ElementTruss2D:public ElementLine2D
{
	//ƽ��2�ڵ���ܵ�Ԫ
public:


	void Get_ke(MatrixXd& ke);

	virtual int GetNodeDof() { return 2; };//ÿ���ڵ�����ɶȸ���
	
};

