#pragma once

#include "EntityBase.h"
class Node;
class Dependent : public EntityBase
{
	//������Ϣ
public:
	std::weak_ptr<Node> m_pNodeSlave;//�ӽڵ�
	std::weak_ptr<Node> m_pNodeMaster[3];

	void AssignDOF(int& iStart);
};

