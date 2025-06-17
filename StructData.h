#pragma once
#include <map>
#include<memory>
#include "Boundary.h"
#include"fstream"
#include"Dependent.h"
class Node;
class Material;
class SectionBase;
class LoadBase;
class Boudary;
class ElementBase;
class Dependent;
class StructData
{
	//�ṹ����

private:
	void InputMaterial(std::ifstream& fin);
	void InputSection(std::ifstream& fin);//��ȡ����
	void InputNode(std::ifstream& fin);//��ȡ�ڵ�
	void InputElement(std::ifstream& fin);
	void InputBoundary(std::ifstream& fin);
	void InputLoad(std::ifstream& fin);
	void InputDependent(std::ifstream& fin);//��ȡ������Ϣ

	void AssignNodeDof();//����ڵ����ɶ�
	void AssembleStiff();//��װ�ܸվ���
	Eigen::SparseMatrix<double > K11, K21, K22;//�ֿ��ܸ�

	void AssembleLoad(VectorXd &F1,VectorXd &F2);//��װ��������


public:
	std::map<int, std::shared_ptr<Material>>m_Materials;//����
	std::map<int, std::shared_ptr< SectionBase>>m_Sections;//����
	std::map<int, std::shared_ptr<Node>>m_Nodes;//�ڵ�
	std::map<int, std::shared_ptr<ElementBase>>m_Elements;//��Ԫ
	std::map<int, std::shared_ptr<LoadBase>>m_Loads;//����
	std::map<int, std::shared_ptr<Boundary>>m_Boundarys;//�߽�����
	std::map<int, std::shared_ptr<Dependent>>m_Dependents;//���ӹ�ϵ

	void InputFile(const  char* FileName);//��ȡ�ļ�

	std::shared_ptr<Material>FindMaterial(int idEntity);
	std::shared_ptr<SectionBase>FindSection(int idEntity);
	std::shared_ptr<Node>FindNode(int idEntity);
	void  AnalyseStatic();//��������
	int m_nFixed;//Լ�����ɶȸ���
	int m_nfree;//����Լ�����ɶȸ���
	
};

