#pragma once
#include <memory>
#include"fstream"
#include"iostream"
#include"Eigen/Dense"
#include "Eigen/Sparse"
using namespace Eigen;

class EntityBase
{//有编号对象的基类
public:
	int m_id;

	virtual ~EntityBase(){}
	


};

