#include "ElementTruss2D.h"
#include"SectionTruss.h"
#include"Node.h"
#include "Material.h"
void ElementTruss2D::Get_ke(MatrixXd& ke)
{
	auto pSection = m_pSection.lock();
	auto pSectionTruss = std::dynamic_pointer_cast<SectionTruss>(pSection);
	auto pMaterial = pSection->m_pMaterial.lock();
	
	double EA = pMaterial->m_E * pSectionTruss->m_A;

	double lx, ly;//lx: cos ,ly:sin

	double L = Get_Les(lx, ly);//计算长度和方向
	Matrix2d k;
	k << lx * lx, lx* ly,
		lx* ly, ly* ly;
	k *= EA / L;

	ke.resize(4, 4);
	ke.block<2, 2>(0, 0) = k;  ke.block<2, 2>(0, 2) = -k;
	ke.block<2, 2>(2, 0) = -k; ke.block<2, 2>(2, 2) = k;
	std::cout << "单元编号："  << m_id << "\n";
	std::cout << ke << "\n";
	std::cout << "\n";




}


