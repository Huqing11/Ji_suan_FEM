#pragma once
#include"EntityBase.h"
class LoadBase:public EntityBase
{//���ػ���
public:
	virtual void Assemble(VectorXd & F1,VectorXd& F2)=0;
};

