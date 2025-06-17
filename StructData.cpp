#include "StructData.h"
#include <fstream>
#include"Material.h"
#include"SectionTruss.h"
#include"Node.h"
#include"ElementTruss2D.h"
#include"Boundary.h"
#include"LoadNode.h"
#include<list>
#include"ElementBeam2D.h"
#include"SectionBeam2D.h"
#include"Dependent.h"

typedef Eigen::Triplet<double> Trid;//重说明三元组(i,j,ki,j)
typedef std::list<Trid> listTrid;//重说明三元组链表

void StructData::InputFile(const char* FileName)
{
	std::ifstream fin(FileName);
	if (!fin.is_open())
	{
		std::cout<<"文件打开失败！\n";
		exit(1);
	}

	InputMaterial(fin);
	InputSection(fin);
	InputNode(fin);
	InputDependent(fin);
	InputElement(fin);
	InputBoundary(fin);
	InputLoad(fin);
	
}

std::shared_ptr<Material> StructData::FindMaterial(int idEntity)
{
	auto iFind = m_Materials.find(idEntity);
	if (iFind != m_Materials.end())
	{
		return iFind->second;
	}
	return nullptr;
}

std::shared_ptr<SectionBase> StructData::FindSection(int idEntity)
{
	auto iFind = m_Sections.find(idEntity);
	if (iFind != m_Sections.end())
	{
		return iFind->second;
	}
	return nullptr;
}

std::shared_ptr<Node> StructData::FindNode(int idEntity)
{
	auto iFind = m_Nodes.find(idEntity);
	if (iFind != m_Nodes.end())
	{
		return iFind->second;
	}
	return nullptr;
}

void StructData::AnalyseStatic()
{
	//静力分析
	AssignNodeDof();//分配节点自由度
	AssembleStiff();//组装总刚矩阵

	VectorXd F1(m_nFixed), F2(m_nfree);
	//F1向量是被约束的节点所受到的力，比如说固定铰支座x，y方向被固定，x，y方向会收到力，储存在在F1向量中
	//F2向量是自由端的节点（没有受到铰支座约束）所受到的力，比如说一个桁架除了铰支座节点外，其余节点收到的力，注意：移动铰支座x方向未被受到约束，属于F2范畴
	AssembleLoad(F1,F2);//组装荷载向量
	std::cout << "\nF1=\n" << F1 << "\n";
	std::cout << "\nF2=\n" << F2 << "\n";

	VectorXd x1(m_nFixed);
	for (auto& a : m_Boundarys)
	{
		auto pBoundary = a.second;
		auto pNode = pBoundary->m_pNode.lock();
		auto it = pNode->m_DOF[pBoundary->m_Direction];//约束自由度编号
		x1[it] = pBoundary->m_Value;
	}
	std::cout << "\nx1=\n" << x1 << "\n";

	SimplicialLDLT<SparseMatrix<double>>solver;
	solver.analyzePattern(K22);//分析非零元素结构
	solver.factorize(K22);//数值分解
	VectorXd x2 = solver.solve(F2 - K21*x1);
	std::cout << "\nx2=\n" << x2 << "\n";

	VectorXd R1 = K11 * x1 + K21.transpose() * x2 - F1;
	std::cout << "\nR1=\n" << R1 << "\n";
	
	std::cout << "\n节点位移：\n" << "\n";
	for (auto& a : m_Nodes)
	{
		auto pNode = a.second;
		std::cout << pNode->m_id<<"  ";


		for (auto &it :pNode->m_DOF)
		{

			if (it < m_nFixed)
			{
				std::cout << x1[it] << " ";
			}
			else
			{
				std::cout << x2[it - m_nFixed] << " ";
			}
		}
		std::cout << "\n";
	}
	std::cout << "约束反力" << "\n";
	for (auto& a : m_Boundarys)
	{
		auto pBoundary = a.second;
		auto pNode = pBoundary->m_pNode.lock();
		auto it = pNode->m_DOF[pBoundary->m_Direction];//约束自由度编号
		std::cout << pBoundary->m_id << " " << pNode->m_id << " ";
		std::cout << pBoundary->m_Direction << " ";
		std::cout << R1[it]<<"\n";
	}
}

void StructData::InputMaterial(std::ifstream& fin)
{
	//读取材料
	int nMaterial = 0;
	fin >> nMaterial;
	std::cout << "\nnMaterial= " << nMaterial << "\n";
	for (int i = 0; i < nMaterial; ++i)
	{
		auto pMaterial = std::make_shared<Material>();
		fin >> pMaterial->m_id;
		fin >> pMaterial->m_E >> pMaterial->m_v;
		m_Materials.insert({ pMaterial->m_id,pMaterial });
	}
	for (auto& a : m_Materials)
	{
		auto pMaterial = a.second;
		std::cout << pMaterial->m_id << " " << pMaterial->m_E;
		std::cout << " " << pMaterial->m_v << " \n";

	}
	
	 
}

void StructData::InputSection(std::ifstream& fin)
{
	//读取截面
	int nSectionTruss = 0;
	fin >> nSectionTruss;
	std::cout << "\nnSectionTruss= " << nSectionTruss << "\n";
	for (int i = 0; i < nSectionTruss; ++i)
	{
		auto pSection = std::make_shared<SectionTruss>();
		fin >> pSection->m_id;
		int idMaterial = 0;
		fin >> idMaterial;

		auto pMaterial = FindMaterial(idMaterial);
		if (pMaterial == nullptr)
		{
			std::cout << "截面材料号不正确，截面：" << pSection->m_id << "\n";
			exit(1);
		}
		pSection->m_pMaterial = pMaterial;
		fin >> pSection->m_A;
		m_Sections.insert({ pSection->m_id,pSection });
	}

	int nSectionBeam2D = 0;
	fin >> nSectionBeam2D;
	std::cout << "\nnSectionBeam2D= " << nSectionBeam2D << "\n";
	for (int i = 0; i < nSectionBeam2D; ++i)
	{
		auto pSection = std::make_shared<SectionBeam2D>();
		fin >> pSection->m_id;
		int idMaterial = 0;
		fin >> idMaterial;

		auto pMaterial = FindMaterial(idMaterial);
		if (pMaterial == nullptr)
		{
			std::cout << "截面材料号不正确，截面：" << pSection->m_id << "\n";
			exit(1);
		}
		pSection->m_pMaterial = pMaterial;

		fin >> pSection->m_A;
		fin >> pSection->m_Iz;
		m_Sections.insert({ pSection->m_id,pSection });
	}

	for (auto& a : m_Sections)
	{
		auto pSection = a.second;//取得截面指针
		std::cout << pSection->m_id << " ";
		auto pMaterial = pSection->m_pMaterial.lock();//取得材料指针
		if (pMaterial == nullptr)
		{
			std::cout << "截面材料未找到! \n";
			exit(1);
		}
		std::cout << pMaterial->m_id << "";
		if (auto pSectionBeam2D = std::dynamic_pointer_cast<SectionBeam2D>(pSection))
		{
			std::cout << pSectionBeam2D->m_Iz;
		}
		std::cout << "\n ";


	}
}

void StructData::InputNode(std::ifstream& fin)
{
	//读取节点
	int nNode = 0;
	fin >> nNode;
	std::cout << "\nnNode= " << nNode << "\n";
	for (int i = 0; i < nNode; ++i)
	{
		auto pNode = std::make_shared< Node>();
		fin >> pNode->m_id;
		fin >> pNode->m_x >> pNode->m_y;
		m_Nodes.insert({ pNode->m_id,pNode});
	}
	for (auto& a : m_Nodes)
	{
		auto pNode = a.second;//取得截面指针
		std::cout << pNode->m_id << " ";
		std::cout << pNode->m_x<<" "<< pNode->m_y << "\n";
	}
}

void StructData::InputElement(std::ifstream& fin)
{
	//读取单元
	int nTruss = 0;
	fin >> nTruss;
	std::cout << "\nnElement= " << nTruss << "\n";
	for (int i = 0; i < nTruss; ++i)
	{//读取桁架单元
		auto pElement = std::make_shared< ElementTruss2D>();
		fin >> pElement->m_id;
		int idNode[2], idSection;
		fin >> idNode[0] >> idNode[1] >> idSection;
		auto pNode0 = FindNode(idNode[0]);
		auto pNode1 = FindNode(idNode[1]);
		if (pNode0 == nullptr || pNode1 == nullptr)
		{
			std::cout << "单元的节点数据不正确! 单元： \n"<< pElement->m_id<<"\n";
			exit(1);

		}
		auto pSection = FindSection(idSection);

		auto pSectionTruss = std::dynamic_pointer_cast<SectionTruss>(pSection);
		auto pMaterial = pSection->m_pMaterial.lock();
		if (pSectionTruss == nullptr)
		{
			std::cout << "单元截面错误单元：" << pElement->m_id << "  ,截面:" << idSection<< "\n";
			exit(1);
		}


		pElement->m_pNode[0] = pNode0;
		pElement->m_pNode[1] = pNode1;
		pElement->m_pSection =pSection;

		m_Elements.insert({pElement->m_id,pElement });
	}

	//读取单元
	int nBeam2D = 0;
	fin >> nBeam2D;
	std::cout << "\nnBeam2D= " << nBeam2D << "\n";
	for (int i = 0; i < nBeam2D; ++i)
	{//读取梁单元

		auto pElement = std::make_shared< ElementBeam2D>();
		fin >> pElement->m_id;
		int idNode[2], idSection;
		fin >> idNode[0] >> idNode[1] >> idSection;
		auto pNode0 = FindNode(idNode[0]);
		auto pNode1 = FindNode(idNode[1]);
		if (pNode0 == nullptr || pNode1 == nullptr)
		{
			std::cout << "单元的节点数据不正确! 单元： \n" << pElement->m_id << "\n";
			exit(1);

		}
		auto pSection = FindSection(idSection);
		auto pSectionBeam2D = std::dynamic_pointer_cast<SectionBeam2D>(pSection);
		if (pSectionBeam2D == nullptr)
		{
			std::cout << "单元截面错误单元：" << pElement->m_id << "  ,截面:" << idSection << "\n";
			exit(1);
		}


		pElement->m_pNode[0] = pNode0;
		pElement->m_pNode[1] = pNode1;
		pElement->m_pSection = pSection;

		m_Elements.insert({ pElement->m_id,pElement });
	}

	for (auto& a : m_Elements)
	{
		auto pElement = a.second;//取得截面指针
		std::cout << pElement->m_id << " ";
		for (auto& b : pElement->m_pNode)
		{
			std::cout << b.lock()->m_id << " ";
		}
	
		std::cout << pElement->m_pSection.lock()->m_id << "\n";
	}
}

void StructData::InputBoundary(std::ifstream& fin)
{
	//读取边界条件
	int nBoundary = 0;
	fin >> nBoundary;
	std::cout << "\nnBoundary= " << nBoundary << "\n";
	for (int i = 0; i < nBoundary; ++i)
	{
		auto pBoundary = std::make_shared< Boundary>();
		fin >> pBoundary->m_id;
		int idNode;
		fin >> idNode;
		auto pNode= FindNode(idNode);
		if (pNode == nullptr )
		{
			std::cout << "Boundary data不正确!\n";
			exit(1);

		}
		pBoundary->m_pNode = pNode;
		fin >> pBoundary->m_Direction >> pBoundary->m_Value;
	
		m_Boundarys.insert({ pBoundary->m_id,pBoundary });
	}
	for (auto& a : m_Boundarys)
	{
		auto pBoundary = a.second;//取得截面指针
		std::cout << pBoundary->m_id << " ";
		std::cout << pBoundary->m_pNode.lock()->m_id << " ";
		std::cout << pBoundary->m_Direction<< " ";
		std::cout << pBoundary->m_Value<< "\n";
	}
}

void StructData::InputLoad(std::ifstream& fin)
{
	//读取load
	int nLoad= 0;
	fin >> nLoad;
	std::cout << "\nnLoad= " << nLoad << "\n";
	for (int i = 0; i < nLoad; ++i)
	{
		auto pLoad = std::make_shared<LoadNode>();
		fin >>pLoad->m_id;
		int idNode;
		fin >> idNode;
		auto pNode = FindNode(idNode);
		if (pNode == nullptr)
		{
			std::cout << "Load data不正确!\n";
			exit(1);

		}
		pLoad->m_pNode = pNode;
		fin >> pLoad->m_P[0] >> pLoad->m_P[1] >> pLoad->m_P[2];
		m_Loads.insert({ pLoad->m_id,pLoad });
	}
	for (auto& a : m_Loads)
	{
		auto pLoad = a.second;//取得截面指针
		std::cout << pLoad->m_id << " ";
		std::cout << pLoad->m_pNode.lock()->m_id << " ";
		std::cout << pLoad->m_P[0] << " ";
		std::cout << pLoad->m_P[1] << "  ";
		std::cout << pLoad->m_P[2]<< "\n";
	}
}

void StructData::InputDependent(std::ifstream& fin)
{
	int nDependent = 0;
	fin >> nDependent;
	std::cout << "\nnDependent= " << nDependent << "\n";
	for (int i = 0; i < nDependent; ++i)
	{
		auto pDependent = std::make_shared<Dependent>();
		fin >> pDependent->m_id;//主从信息编号
		int idNodeSlave, idNodeMaster[3];
		fin >> idNodeSlave;//从节点号
		auto pNodeSlave = FindNode(idNodeSlave);

		if (pNodeSlave == nullptr)
		{
			std::cout << "从节点号不正确 ! ,从节点号：" << pNodeSlave << " ";
			exit(1);

		}
		pDependent->m_pNodeSlave = pNodeSlave;

		fin >> idNodeMaster[0] >> idNodeMaster[1] >> idNodeMaster[2];

		for (int j = 0; j < 3; ++j)
		{
			if (idNodeMaster[j] > 0)
			{
				//设置了主节点
				auto pNodeMaster = FindNode(idNodeMaster[j]);
				if (pNodeMaster == nullptr)
				{
					std::cout << "从节点的主节点不正确 ! ,主节点号：";
					std::cout << idNodeMaster[j] << "\n";
					exit(1);
				}
				pDependent->m_pNodeMaster[j] = pNodeMaster;
			}
	}
		m_Dependents.insert({ pDependent->m_id,pDependent });
	}
	for (auto& a : m_Dependents)
	{
		auto pDependent = a.second;//取得截面指针
		std::cout << pDependent->m_id << " ";
		std::cout << pDependent->m_pNodeSlave.lock()->m_id<<" ";
		for (int j = 0; j < 3; ++j)
		{
			auto pNodeMaster = pDependent->m_pNodeMaster[j].lock();
			if (pNodeMaster != nullptr)
			{//设置了主节点
				std::cout << pNodeMaster->m_id << "";
			}
			else
			{
				std::cout << 0<< "";
			}

		}

		std::cout  << "\n";
	}
}

void StructData::AssignNodeDof()
{
	//分配节点自由度
	//设置每个节点的自由度个数
	for (auto& a : m_Elements)
	{
		auto pElement = a.second;
		auto nDOF = pElement->GetNodeDof();
		for(auto & b:pElement->m_pNode)
		{
			auto pNode = b.lock();
			if (pNode->m_DOF.size() < nDOF)
			{//根据节点所在的单元的需求，设置节点的自由度个数
				pNode->m_DOF.resize(nDOF);
			}

		}
	}
	//每个节点的自由度编号初始化为-1
	for (auto& a : m_Nodes)
	{
		auto pNode = a.second;
		for (auto& b : pNode->m_DOF) b = -1;
	}

	int iStart = 0;
	for (auto& a : m_Boundarys)
	{
		auto pBoudary = a.second;
		auto pNode = pBoudary->m_pNode.lock();
		pNode->m_DOF[pBoudary->m_Direction] = iStart++;
	}

	m_nFixed = iStart;//约束自由度个数
	//利用主从信息分配自由度编号

	for (auto& a : m_Dependents)
	{
		auto pDependent = a.second;
		pDependent->AssignDOF(iStart);
	}
	//对所有的没有编过号的自由度依次编号
	for (auto& a : m_Nodes)
	{
		auto pNode = a.second;
		for (auto& b : pNode->m_DOF)
		{
			if (b == -1)b = iStart++;
		}
		if (pNode->m_DOF[0] == -1) pNode->m_DOF[0] = iStart++;
		if (pNode->m_DOF[1] == -1) pNode->m_DOF[1] = iStart++;

	}
	m_nfree = iStart - m_nFixed;
	std::cout << "\nm_Fixed= " << m_nFixed << "\n";
	std::cout << "\nm_free= " << m_nfree << "\n";

	std::cout << "\n节点自由度编号: \n";
	for (auto& a : m_Nodes)
	{
		auto pNode = a.second;
		std::cout << pNode->m_id << " ";
		for (auto& b : pNode->m_DOF)
		{
			std::cout << b << " ";
		}
		/*std::cout << pNode->m_DOF[0] << " " << pNode->m_DOF[1] << "\n";*/
		std::cout << "\n";

	}
}

void StructData::AssembleStiff()
{
	//组装刚度矩阵
	K11.resize(m_nFixed, m_nFixed);
	K21.resize(m_nfree, m_nFixed);
	K22.resize(m_nfree, m_nfree);

	MatrixXd ke;
	std::vector<int>DOFs;
	listTrid L11, L21, L22;

	std::cout << "\n单元刚度矩阵:\n";

	for (auto& a : m_Elements)
	{
		auto pElement = a.second;
		pElement->Get_ke(ke);
		pElement->GetDOFs(DOFs);
		auto nDOF = DOFs.size();
		for (int i = 0; i < nDOF; ++i)
		{
			//对ke行循环
			int ii = DOFs[i];//航自由度的整体自由度编号
			for (int j = 0; j < nDOF; ++j)
			{
				int jj = DOFs[j];//列自由度的整体自由度编号
				double& kij = ke(i, j);
				if (ii < m_nFixed && jj < m_nFixed)
				{//kij应组装到K11
					L11.push_back(Trid(ii, jj, kij));
				}
				else if (ii >= m_nFixed && jj < m_nFixed)
				{//kij应组装到K21
					L21.push_back(Trid(ii-m_nFixed, jj, kij));
				}
				else if (ii >= m_nFixed && jj >= m_nFixed)
				{//kij应组装到K22
					L22.push_back(Trid(ii - m_nFixed, jj-m_nFixed, kij));
				}
			}
		}


	}
	K11.setFromTriplets(L11.begin(), L11.end());
	K21.setFromTriplets(L21.begin(), L21.end());
	K22.setFromTriplets(L22.begin(), L22.end());

	std::cout << "\nK11:\n" << MatrixXd(K11) << "\n";
	std::cout << "\nK21:\n" << MatrixXd(K21) << "\n";
	std::cout << "\nK22:\n" << MatrixXd(K22) << "\n";


}

void StructData::AssembleLoad(VectorXd& F1, VectorXd& F2)
{

	F1.setZero();
	F2.setZero();

	for (auto& a : m_Loads)
	{
		auto pLoad = a.second;
		pLoad->Assemble(F1, F2);
		
	}
}
