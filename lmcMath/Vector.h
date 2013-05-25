#ifndef VECTOR_H
#define VECTOR_H

#include<math.h>

//����
double const Pi = 3.141592653;
double const g  = 9.81;
double const rho=1.29;

inline double DegToRad(double deg);//�Ƕȡ�����ת��
inline double RadToDeg(double Rad);


class Intergration//����������
{
public:
    void init(double StartValue,double StartDot);//��ʼ��
    void step(double dt,double dot);//����
    double val;//valueֵ�Ĵ�С
private:
    double lastdot;//Ҫ�ۻ�������������
};


class Vector//������
{
public:
    Vector();
    Vector(double const &xi, double const &yi, double const &zi);

    double Magnitude();//��ģ
    void Reverse();   //��ת����

    bool operator ==(Vector const &V);
    void operator=(Vector const &T);//�����丳ֵ����
    void operator+=(Vector const &T);
    void operator-=(Vector const &T);
    void operator*=(double const &d);

    Vector operator-();//����
    Vector operator*(double const &d);
    Vector operator*(Vector const &T);
    Vector operator/(double const &d);
    Vector operator+(Vector const &V);
    Vector operator-(Vector const &V);

    void Copy(Vector const &V);//��������
    void Set(double const &x0, double const &y0, double const &z0);//������ʵ����ʼ������
    void Set(Vector const &V);//��������ʼ������
    void Normalize();
    double VAbs();
    double dot(Vector const &V);
    bool IsSame(Vector const &V);
    void Translate(Vector const &T);
public:
    double x;
    double y;
    double z;
};

Vector operator^(Vector u,Vector v);//���
Vector operator*(double s    ,Vector u);//������ʵ�������
Vector operator/(Vector u,double s);//��������ʵ��#endif // VECTOR_H

#endif
