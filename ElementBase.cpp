#include "ElementBase.h"
#include"Node.h"
void ElementBase::GetDOFs(std::vector<int>& DOFs)
{
	//取得单元自由度编号
	auto nNode = m_pNode.size();

	auto nDofNode = GetNodeDof();//一个节点的自由度个数

	DOFs.resize(nNode * nDofNode);
	int k = 0;
	for (auto& a : m_pNode)
	{
		auto pNode = a.lock();
		for (auto& b : pNode->m_DOF)
		{
			DOFs[k++] = b;
		}
	}
}
