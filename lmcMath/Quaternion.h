#ifndef QUATERNION_H
#define QUATERNION_H

#include"Vector.h"
#include"Matrix33.h"

class Quaternion//��Ԫ����
{
public:
    Quaternion();
    Quaternion(double const &t, double const &x, double const &y, double const &z);
    Quaternion(double const &Angle, Vector const &R);
public:
    Vector ToBody(Vector local);//ȫ��ת��������
    Vector ToLocal(Vector body);//����ת����ȫ��
    void init(Vector eulerAngle);//��ŷ���ǳ�ʼ����Ԫ��
    void updateEuler();//����ŷ����
    void step(double dt,Vector omega);//����

    void Settxx();
    void Conjugate(Vector const &Vin, Vector &Vout);
    void Conjugate(Vector &V);
    void Conjugate(double &x, double &y, double &z);
    void Set(double const &real, double const &x, double const &y, double const &z);
    void Set(double const &Angle, Vector const &R);
public:
    double a, qx, qy,qz;
    void QuattoMat(double m[][4]);
    void Normalize();
    void operator*=(Quaternion Q);
    void operator ~();
    void operator =(Quaternion Q);
    Quaternion operator *(Quaternion Q);

    Vector euler;//ŷ���� ����̬��phi, theta, psi��
    Matrix33 mat;//ȫ�ֵ������ת������
private:
    double theta;
    double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t15, t19, t20, t24;
    Vector R;

    void update_mat();//���¾���
    double length();//��ģ
    Intergration x,y,z,w;//��Ԫ������
};

#endif // QUATERNION_H
