#include "ElementLine2D.h"
#include"Node.h"
ElementLine2D::ElementLine2D()
{
	m_pNode.resize(2);
}

double ElementLine2D::Get_Les(double& c, double& s)
{//计算长度，返回方向余弦(c,s)
	auto pNode0 = m_pNode[0].lock();
	auto pNode1 = m_pNode[1].lock();
	double dx = pNode1->m_x - pNode0->m_x;
	double dy = pNode1->m_y - pNode0->m_y;
	double L = sqrt(dx * dx + dy * dy);
	c= dx / L;//cos
	s= dy / L;// sin
	return L;
}
