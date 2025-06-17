#pragma once
#include"EntityBase.h"
class Node;
class SectionBase;
#include<vector>
class ElementBase:public EntityBase
{//��Ԫ����
public:
	std::vector<std::weak_ptr<Node>>m_pNode;//����ڵ�
	std::weak_ptr<SectionBase>m_pSection;//�������ָ��

	virtual int GetNodeDof() = 0;//ÿ���ڵ�����ɶȸ���
	void GetDOFs(std::vector<int>& DOFs);//ȡ�õ�Ԫ�ĸ����ڵ�����ɶȱ��
	virtual void Get_ke(MatrixXd& ke)=0;
};

