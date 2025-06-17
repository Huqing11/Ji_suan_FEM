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

typedef Eigen::Triplet<double> Trid;//��˵����Ԫ��(i,j,ki,j)
typedef std::list<Trid> listTrid;//��˵����Ԫ������

void StructData::InputFile(const char* FileName)
{
	std::ifstream fin(FileName);
	if (!fin.is_open())
	{
		std::cout<<"�ļ���ʧ�ܣ�\n";
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
	//��������
	AssignNodeDof();//����ڵ����ɶ�
	AssembleStiff();//��װ�ܸվ���

	VectorXd F1(m_nFixed), F2(m_nfree);
	//F1�����Ǳ�Լ���Ľڵ����ܵ�����������˵�̶���֧��x��y���򱻹̶���x��y������յ�������������F1������
	//F2���������ɶ˵Ľڵ㣨û���ܵ���֧��Լ�������ܵ�����������˵һ����ܳ��˽�֧���ڵ��⣬����ڵ��յ�������ע�⣺�ƶ���֧��x����δ���ܵ�Լ��������F2����
	AssembleLoad(F1,F2);//��װ��������
	std::cout << "\nF1=\n" << F1 << "\n";
	std::cout << "\nF2=\n" << F2 << "\n";

	VectorXd x1(m_nFixed);
	for (auto& a : m_Boundarys)
	{
		auto pBoundary = a.second;
		auto pNode = pBoundary->m_pNode.lock();
		auto it = pNode->m_DOF[pBoundary->m_Direction];//Լ�����ɶȱ��
		x1[it] = pBoundary->m_Value;
	}
	std::cout << "\nx1=\n" << x1 << "\n";

	SimplicialLDLT<SparseMatrix<double>>solver;
	solver.analyzePattern(K22);//��������Ԫ�ؽṹ
	solver.factorize(K22);//��ֵ�ֽ�
	VectorXd x2 = solver.solve(F2 - K21*x1);
	std::cout << "\nx2=\n" << x2 << "\n";

	VectorXd R1 = K11 * x1 + K21.transpose() * x2 - F1;
	std::cout << "\nR1=\n" << R1 << "\n";
	
	std::cout << "\n�ڵ�λ�ƣ�\n" << "\n";
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
	std::cout << "Լ������" << "\n";
	for (auto& a : m_Boundarys)
	{
		auto pBoundary = a.second;
		auto pNode = pBoundary->m_pNode.lock();
		auto it = pNode->m_DOF[pBoundary->m_Direction];//Լ�����ɶȱ��
		std::cout << pBoundary->m_id << " " << pNode->m_id << " ";
		std::cout << pBoundary->m_Direction << " ";
		std::cout << R1[it]<<"\n";
	}
}

void StructData::InputMaterial(std::ifstream& fin)
{
	//��ȡ����
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
	//��ȡ����
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
			std::cout << "������ϺŲ���ȷ�����棺" << pSection->m_id << "\n";
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
			std::cout << "������ϺŲ���ȷ�����棺" << pSection->m_id << "\n";
			exit(1);
		}
		pSection->m_pMaterial = pMaterial;

		fin >> pSection->m_A;
		fin >> pSection->m_Iz;
		m_Sections.insert({ pSection->m_id,pSection });
	}

	for (auto& a : m_Sections)
	{
		auto pSection = a.second;//ȡ�ý���ָ��
		std::cout << pSection->m_id << " ";
		auto pMaterial = pSection->m_pMaterial.lock();//ȡ�ò���ָ��
		if (pMaterial == nullptr)
		{
			std::cout << "�������δ�ҵ�! \n";
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
	//��ȡ�ڵ�
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
		auto pNode = a.second;//ȡ�ý���ָ��
		std::cout << pNode->m_id << " ";
		std::cout << pNode->m_x<<" "<< pNode->m_y << "\n";
	}
}

void StructData::InputElement(std::ifstream& fin)
{
	//��ȡ��Ԫ
	int nTruss = 0;
	fin >> nTruss;
	std::cout << "\nnElement= " << nTruss << "\n";
	for (int i = 0; i < nTruss; ++i)
	{//��ȡ��ܵ�Ԫ
		auto pElement = std::make_shared< ElementTruss2D>();
		fin >> pElement->m_id;
		int idNode[2], idSection;
		fin >> idNode[0] >> idNode[1] >> idSection;
		auto pNode0 = FindNode(idNode[0]);
		auto pNode1 = FindNode(idNode[1]);
		if (pNode0 == nullptr || pNode1 == nullptr)
		{
			std::cout << "��Ԫ�Ľڵ����ݲ���ȷ! ��Ԫ�� \n"<< pElement->m_id<<"\n";
			exit(1);

		}
		auto pSection = FindSection(idSection);

		auto pSectionTruss = std::dynamic_pointer_cast<SectionTruss>(pSection);
		auto pMaterial = pSection->m_pMaterial.lock();
		if (pSectionTruss == nullptr)
		{
			std::cout << "��Ԫ�������Ԫ��" << pElement->m_id << "  ,����:" << idSection<< "\n";
			exit(1);
		}


		pElement->m_pNode[0] = pNode0;
		pElement->m_pNode[1] = pNode1;
		pElement->m_pSection =pSection;

		m_Elements.insert({pElement->m_id,pElement });
	}

	//��ȡ��Ԫ
	int nBeam2D = 0;
	fin >> nBeam2D;
	std::cout << "\nnBeam2D= " << nBeam2D << "\n";
	for (int i = 0; i < nBeam2D; ++i)
	{//��ȡ����Ԫ

		auto pElement = std::make_shared< ElementBeam2D>();
		fin >> pElement->m_id;
		int idNode[2], idSection;
		fin >> idNode[0] >> idNode[1] >> idSection;
		auto pNode0 = FindNode(idNode[0]);
		auto pNode1 = FindNode(idNode[1]);
		if (pNode0 == nullptr || pNode1 == nullptr)
		{
			std::cout << "��Ԫ�Ľڵ����ݲ���ȷ! ��Ԫ�� \n" << pElement->m_id << "\n";
			exit(1);

		}
		auto pSection = FindSection(idSection);
		auto pSectionBeam2D = std::dynamic_pointer_cast<SectionBeam2D>(pSection);
		if (pSectionBeam2D == nullptr)
		{
			std::cout << "��Ԫ�������Ԫ��" << pElement->m_id << "  ,����:" << idSection << "\n";
			exit(1);
		}


		pElement->m_pNode[0] = pNode0;
		pElement->m_pNode[1] = pNode1;
		pElement->m_pSection = pSection;

		m_Elements.insert({ pElement->m_id,pElement });
	}

	for (auto& a : m_Elements)
	{
		auto pElement = a.second;//ȡ�ý���ָ��
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
	//��ȡ�߽�����
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
			std::cout << "Boundary data����ȷ!\n";
			exit(1);

		}
		pBoundary->m_pNode = pNode;
		fin >> pBoundary->m_Direction >> pBoundary->m_Value;
	
		m_Boundarys.insert({ pBoundary->m_id,pBoundary });
	}
	for (auto& a : m_Boundarys)
	{
		auto pBoundary = a.second;//ȡ�ý���ָ��
		std::cout << pBoundary->m_id << " ";
		std::cout << pBoundary->m_pNode.lock()->m_id << " ";
		std::cout << pBoundary->m_Direction<< " ";
		std::cout << pBoundary->m_Value<< "\n";
	}
}

void StructData::InputLoad(std::ifstream& fin)
{
	//��ȡload
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
			std::cout << "Load data����ȷ!\n";
			exit(1);

		}
		pLoad->m_pNode = pNode;
		fin >> pLoad->m_P[0] >> pLoad->m_P[1] >> pLoad->m_P[2];
		m_Loads.insert({ pLoad->m_id,pLoad });
	}
	for (auto& a : m_Loads)
	{
		auto pLoad = a.second;//ȡ�ý���ָ��
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
		fin >> pDependent->m_id;//������Ϣ���
		int idNodeSlave, idNodeMaster[3];
		fin >> idNodeSlave;//�ӽڵ��
		auto pNodeSlave = FindNode(idNodeSlave);

		if (pNodeSlave == nullptr)
		{
			std::cout << "�ӽڵ�Ų���ȷ ! ,�ӽڵ�ţ�" << pNodeSlave << " ";
			exit(1);

		}
		pDependent->m_pNodeSlave = pNodeSlave;

		fin >> idNodeMaster[0] >> idNodeMaster[1] >> idNodeMaster[2];

		for (int j = 0; j < 3; ++j)
		{
			if (idNodeMaster[j] > 0)
			{
				//���������ڵ�
				auto pNodeMaster = FindNode(idNodeMaster[j]);
				if (pNodeMaster == nullptr)
				{
					std::cout << "�ӽڵ�����ڵ㲻��ȷ ! ,���ڵ�ţ�";
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
		auto pDependent = a.second;//ȡ�ý���ָ��
		std::cout << pDependent->m_id << " ";
		std::cout << pDependent->m_pNodeSlave.lock()->m_id<<" ";
		for (int j = 0; j < 3; ++j)
		{
			auto pNodeMaster = pDependent->m_pNodeMaster[j].lock();
			if (pNodeMaster != nullptr)
			{//���������ڵ�
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
	//����ڵ����ɶ�
	//����ÿ���ڵ�����ɶȸ���
	for (auto& a : m_Elements)
	{
		auto pElement = a.second;
		auto nDOF = pElement->GetNodeDof();
		for(auto & b:pElement->m_pNode)
		{
			auto pNode = b.lock();
			if (pNode->m_DOF.size() < nDOF)
			{//���ݽڵ����ڵĵ�Ԫ���������ýڵ�����ɶȸ���
				pNode->m_DOF.resize(nDOF);
			}

		}
	}
	//ÿ���ڵ�����ɶȱ�ų�ʼ��Ϊ-1
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

	m_nFixed = iStart;//Լ�����ɶȸ���
	//����������Ϣ�������ɶȱ��

	for (auto& a : m_Dependents)
	{
		auto pDependent = a.second;
		pDependent->AssignDOF(iStart);
	}
	//�����е�û�б���ŵ����ɶ����α��
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

	std::cout << "\n�ڵ����ɶȱ��: \n";
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
	//��װ�նȾ���
	K11.resize(m_nFixed, m_nFixed);
	K21.resize(m_nfree, m_nFixed);
	K22.resize(m_nfree, m_nfree);

	MatrixXd ke;
	std::vector<int>DOFs;
	listTrid L11, L21, L22;

	std::cout << "\n��Ԫ�նȾ���:\n";

	for (auto& a : m_Elements)
	{
		auto pElement = a.second;
		pElement->Get_ke(ke);
		pElement->GetDOFs(DOFs);
		auto nDOF = DOFs.size();
		for (int i = 0; i < nDOF; ++i)
		{
			//��ke��ѭ��
			int ii = DOFs[i];//�����ɶȵ��������ɶȱ��
			for (int j = 0; j < nDOF; ++j)
			{
				int jj = DOFs[j];//�����ɶȵ��������ɶȱ��
				double& kij = ke(i, j);
				if (ii < m_nFixed && jj < m_nFixed)
				{//kijӦ��װ��K11
					L11.push_back(Trid(ii, jj, kij));
				}
				else if (ii >= m_nFixed && jj < m_nFixed)
				{//kijӦ��װ��K21
					L21.push_back(Trid(ii-m_nFixed, jj, kij));
				}
				else if (ii >= m_nFixed && jj >= m_nFixed)
				{//kijӦ��װ��K22
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
