#include "csv.h"


void showInCsv(Eigen::MatrixXd _matrix ,const char* _name)
{
    // 写文件  
    std::ofstream outFile;  
    outFile.open(_name, std::ios::out); // 打开模式可省略  
    for(int i = 0; i < _matrix.rows(); i++)
    {
        for(int j = 0; j < _matrix.cols(); j++)
        {
            outFile << _matrix(i,j)<< ',';
        }
        outFile << std::endl;
    }
    outFile.close();  
}

void showInCsv(std::vector<double> _vector ,const char* _name)
{
    // 写文件  
    std::ofstream outFile;  
    outFile.open(_name, std::ios::out); // 打开模式可省略  
    for(int i = 0; i < _vector.size(); i++)
    {
        outFile << _vector[i]<< ',';
        outFile << std::endl;
    }
    outFile.close();  
}

void showInCsv(std::vector<int> _vector ,const char* _name)
{
    // 写文件  
    std::ofstream outFile;  
    outFile.open(_name, std::ios::out); // 打开模式可省略  
    for(int i = 0; i < _vector.size(); i++)
    {
        outFile << _vector[i]<< ',';
        outFile << std::endl;
    }
    outFile.close();   
}
