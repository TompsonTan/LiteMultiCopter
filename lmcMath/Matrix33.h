#ifndef MATRIX33_H
#define MATRIX33_H

#include"Vector.h"

class Matrix33//3x3������
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
    double det();//������ʽ��ֵ
    Matrix33  inverse();//������
    Matrix33& operator=(const Matrix33&b);//����丳ֵ
    Vector    multrans(const Vector&b);//��������˾���
    Vector   operator*(const Vector&b);//�����ҳ�������
    Matrix33 operator*(const Matrix33& b);//���
    Matrix33 operator-(const Matrix33& b);//���
    Matrix33 trans();//ת��
};

#endif // MATRIX33_H
