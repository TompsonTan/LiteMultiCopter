#ifndef MATRIX33_H
#define MATRIX33_H

#include"Vector.h"

class Matrix33//3x3矩阵类
{
public:
    double v[3][3];
public:
    Matrix33();
    Matrix33(double i00,double i01,double i02,
             double i10,double i11,double i12,
             double i20,double i21,double i22);
    Matrix33(const Matrix33& mb);
public:
    double det();//其行列式的值
    Matrix33  inverse();//求逆阵
    Matrix33& operator=(const Matrix33&b);//矩阵间赋值
    Vector    multrans(const Vector&b);//行向量左乘矩阵
    Vector   operator*(const Vector&b);//矩阵右乘列向量
    Matrix33 operator*(const Matrix33& b);//相乘
    Matrix33 operator-(const Matrix33& b);//相减
    Matrix33 trans();//转置
};

#endif // MATRIX33_H
