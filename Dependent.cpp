#include "Dependent.h"
#include"Node.h"
void Dependent::AssignDOF(int& iStart)
{
	//分配自由度
	auto pNodeSlave = m_pNodeSlave.lock();
	int k = 0;
	for (auto& a : pNodeSlave->m_DOF)
	{
		if (a == -1)
		{//没有编过号
			auto pNodeMaster = m_pNodeMaster[k].lock();//主节点
			if (pNodeMaster != nullptr)
			{
				//设置了主节点
				auto& it = pNodeMaster->m_DOF[k];//主节点的自由度编号
				if (it == -1) it = iStart++;//给主节点编号
				a = it;//从节点的自由度编号=主节点的
			}
			++k;


		}
	}
}