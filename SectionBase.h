#pragma once
#include"EntityBase.h"
class Material;

class SectionBase:public EntityBase
{//����
public :
	std::weak_ptr<Material>m_pMaterial;
};

