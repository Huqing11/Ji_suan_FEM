#include "Load_q.h"
#include"ElementBase.h"
void Load_q::Assemble(VectorXd& F1, VectorXd& F2)
{

	auto nFixed = F1.size();//Լ�����ɶȸ���


	VectorXd Feq;
	std::vector<int> DOFs;
	for (auto& a : m_Elements)
	{
		auto pElement = a.lock();
		GetFeq(pElement, Feq);
		pElement->GetDOFs();//ȡ�õ�Ԫ�����ɶ�
		auto nDOF = DOFs.size();

		assert(nDOF == Feq.size());
		for (int i = 0; i < nDOF; ++i)
		{
			auto it = DOFs[i];
			if (it < nFixed)
			{//���ɶ���Լ�����ɶ�
				F1[it] += Feq[i];
			}
			else
			{
				F2[it - nFixed] += Feq[i];
			}
		}
	}
}

void Load_q::GetFeq(std::shared_ptr<ElementBase> pElement, VectorXd& Feq)
{

}
