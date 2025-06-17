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
	//结构数据

private:
	void InputMaterial(std::ifstream& fin);
	void InputSection(std::ifstream& fin);//读取截面
	void InputNode(std::ifstream& fin);//读取节点
	void InputElement(std::ifstream& fin);
	void InputBoundary(std::ifstream& fin);
	void InputLoad(std::ifstream& fin);
	void InputDependent(std::ifstream& fin);//读取主从信息

	void AssignNodeDof();//分配节点自由度
	void AssembleStiff();//组装总刚矩阵
	Eigen::SparseMatrix<double > K11, K21, K22;//分块总刚

	void AssembleLoad(VectorXd &F1,VectorXd &F2);//组装荷载向量


public:
	std::map<int, std::shared_ptr<Material>>m_Materials;//材料
	std::map<int, std::shared_ptr< SectionBase>>m_Sections;//截面
	std::map<int, std::shared_ptr<Node>>m_Nodes;//节点
	std::map<int, std::shared_ptr<ElementBase>>m_Elements;//单元
	std::map<int, std::shared_ptr<LoadBase>>m_Loads;//荷载
	std::map<int, std::shared_ptr<Boundary>>m_Boundarys;//边界条件
	std::map<int, std::shared_ptr<Dependent>>m_Dependents;//主从关系

	void InputFile(const  char* FileName);//读取文件

	std::shared_ptr<Material>FindMaterial(int idEntity);
	std::shared_ptr<SectionBase>FindSection(int idEntity);
	std::shared_ptr<Node>FindNode(int idEntity);
	void  AnalyseStatic();//静力分析
	int m_nFixed;//约束自由度个数
	int m_nfree;//不受约束自由度个数
	
};

