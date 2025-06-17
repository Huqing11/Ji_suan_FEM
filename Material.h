#pragma once
#include "EntityBase.h"
class Material:public EntityBase
{//材料类
public:
	double m_E;//弹性模量
	double m_v;//泊松比
};

