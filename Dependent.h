#pragma once

#include "EntityBase.h"
class Node;
class Dependent : public EntityBase
{
	//主从信息
public:
	std::weak_ptr<Node> m_pNodeSlave;//从节点
	std::weak_ptr<Node> m_pNodeMaster[3];

	void AssignDOF(int& iStart);
};

