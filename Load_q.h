#pragma once
#include"LoadBase.h"
#include<vector>

class ElementBase;

class Load_q:public LoadBase
{
	//��������
public:
	bool m_bLocalCoord = true;//ȱʡ�����ھֲ���Ԫϵ��
	double m_qx, n_qm;
	std::vector < std::weak_ptr<ElementBase>>m_Elements;

	virtual void Assemble(VectorXd& F1, VectorXd& F2) ;
	//������һ����Ԫ�ϵĵ�Ч�ڵ��غɣ�����������ϵ�£�
	void GetFeq(std::shared_ptr<ElementBase> pElement,VectorXd & Feq);
};

