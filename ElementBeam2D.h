#pragma once
#include"ElementLine2D.h"
class ElementBeam2D:public ElementLine2D
{//ƽ��2�ڵ�����Ԫ
public:
	virtual int GetNodeDof() { return 3; }
	virtual void Get_ke(MatrixXd& ke) ;

	void Get_keLocal(MatrixXd& keLocal);//���㵥Ԫ����ϵ�µĵ�Ԫ�նȾ���
};

