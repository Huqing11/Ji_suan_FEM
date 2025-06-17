#include "Load_q.h"
#include"ElementBase.h"
void Load_q::Assemble(VectorXd& F1, VectorXd& F2)
{

	auto nFixed = F1.size();//约束自由度个数


	VectorXd Feq;
	std::vector<int> DOFs;
	for (auto& a : m_Elements)
	{
		auto pElement = a.lock();
		GetFeq(pElement, Feq);
		pElement->GetDOFs();//取得单元的自由度
		auto nDOF = DOFs.size();

		assert(nDOF == Feq.size());
		for (int i = 0; i < nDOF; ++i)
		{
			auto it = DOFs[i];
			if (it < nFixed)
			{//自由度是约束自由度
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
