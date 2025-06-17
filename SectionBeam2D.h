#pragma once
#include"SectionTruss.h"
class SectionBeam2D :public SectionTruss
{
	//平面梁截面
public:
	double m_Iz;//对z洲的惯性矩
};

