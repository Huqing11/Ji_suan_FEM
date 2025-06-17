#include "ElementBeam2D.h"
#include"SectionBeam2D.h"
#include"Material.h"

void ElementBeam2D::Get_ke(MatrixXd& ke)
{
	MatrixXd  keLocal;
	Get_keLocal(keLocal);
	double c, s;
	const double L = Get_Les(c, s);
	Matrix3d R;
	R << c, s, 0,
	         -s, c, 0,
		      0, 0, 1;
	MatrixXd RR(6, 6);
	RR.setZero();
	RR.block<3, 3>(0, 0) = R;
	RR.block<3, 3>(3, 3) = R;

	ke = RR.transpose() * keLocal*RR;


}

void ElementBeam2D::Get_keLocal(MatrixXd& keLocal)
{
	//计算单元坐标系下的单元刚度矩阵
	auto pSection = m_pSection.lock();
	auto pSectionBeam2D = std::dynamic_pointer_cast<SectionBeam2D>(pSection);
	assert(pSectionBeam2D != nullptr);

	auto pMaterial = pSectionBeam2D->m_pMaterial.lock();
	const double E = pMaterial->m_E;
	const double A = pSectionBeam2D->m_A;
	const double Iz = pSectionBeam2D->m_Iz;
	double lx, ly;
	const double L = Get_Les(lx, ly);
	

	const double L2 = L * L;
	const double L3 = L2 * L;
	const double EA_L = E* A/L;
	const double EIz = E * Iz;
	const double EIz_L = EIz / L;
	const double EIz_L2 = EIz / L2;
	const double EIz_L3 = EIz / L3;
	keLocal.resize(6, 6);


	keLocal(0, 0) = EA_L;
	keLocal(0, 1) = 0.0;
	keLocal(0, 2) = 0.0;
	keLocal(0, 3) = -EA_L;
	keLocal(0, 4) = 0.0;
	keLocal(0, 5) = 0.0;

	const double a = 12.0 * EIz_L3;
	const double b = 6.0 * EIz_L2;
	keLocal(1, 0) = 0.0;
	keLocal(1, 1) = a;
	keLocal(1,2 ) = b;
	keLocal(1, 3) = 0.0;
	keLocal(1, 4) = -a;
	keLocal(1, 5) = b;

	const double c = 4.0 * EIz_L;
	const double d = 2.0 * EIz_L;
	keLocal(2, 0) = 0.0;
	keLocal(2, 1) = b;
	keLocal(2, 2) = c;
	keLocal(2, 3) = 0.0;
	keLocal(2, 4) = -b;
	keLocal(2, 5) = d;

	keLocal(3, 0) = -EA_L;
	keLocal(3, 1) = 0.0;
	keLocal(3, 2) = 0.0;
	keLocal(3, 3) = EA_L;
	keLocal(3, 4) = 0.0;
	keLocal(3, 5) = 0.0;

	keLocal(4, 0) = 0.0;
	keLocal(4, 1) = -a;
	keLocal(4, 2) = -b;
	keLocal(4, 3) = 0.0;
	keLocal(4, 4) = a;
	keLocal(4, 5) = -b;

	keLocal(5, 0) = 0.0;
	keLocal(5, 1) = b;
	keLocal(5, 2) = d;
	keLocal(5, 3) = 0.0;
	keLocal(5, 4) = -b;
	keLocal(5, 5) = c;




	std::cout << "\n keLocal:\n" << keLocal << "\n";



	


}

