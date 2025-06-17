#include "LoadNode.h"
#include"Node.h"
void LoadNode::Assemble(VectorXd& F1, VectorXd& F2)
{
	auto nFixed = F1.size();
	auto pNode = m_pNode.lock();
	int k = 0;
	for (auto& it : pNode->m_DOF)
	{

		if (it < nFixed)
		{//���ɶ���Լ�����ɶ�
			F1[it] +=m_P[k] ;
		}
		else
		{
			F2[it - nFixed] +=m_P[k];
		}
		++k;
	}
}
