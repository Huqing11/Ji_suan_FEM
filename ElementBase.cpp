#include "ElementBase.h"
#include"Node.h"
void ElementBase::GetDOFs(std::vector<int>& DOFs)
{
	//ȡ�õ�Ԫ���ɶȱ��
	auto nNode = m_pNode.size();

	auto nDofNode = GetNodeDof();//һ���ڵ�����ɶȸ���

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
