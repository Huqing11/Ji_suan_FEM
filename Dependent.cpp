#include "Dependent.h"
#include"Node.h"
void Dependent::AssignDOF(int& iStart)
{
	//�������ɶ�
	auto pNodeSlave = m_pNodeSlave.lock();
	int k = 0;
	for (auto& a : pNodeSlave->m_DOF)
	{
		if (a == -1)
		{//û�б����
			auto pNodeMaster = m_pNodeMaster[k].lock();//���ڵ�
			if (pNodeMaster != nullptr)
			{
				//���������ڵ�
				auto& it = pNodeMaster->m_DOF[k];//���ڵ�����ɶȱ��
				if (it == -1) it = iStart++;//�����ڵ���
				a = it;//�ӽڵ�����ɶȱ��=���ڵ��
			}
			++k;


		}
	}
}