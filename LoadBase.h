#pragma once
#include"EntityBase.h"
class LoadBase:public EntityBase
{//ºÉÔØ»ùÀà
public:
	virtual void Assemble(VectorXd & F1,VectorXd& F2)=0;
};

