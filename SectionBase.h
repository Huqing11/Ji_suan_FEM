#pragma once
#include"EntityBase.h"
class Material;

class SectionBase:public EntityBase
{//Ωÿ√Ê
public :
	std::weak_ptr<Material>m_pMaterial;
};

