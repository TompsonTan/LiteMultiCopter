#ifndef VECTOR_H
#define VECTOR_H

#include<math.h>

//常量
double const Pi = 3.141592653;
double const g  = 9.81;
double const rho=1.29;

inline double DegToRad(double deg);//角度、弧度转换
inline double RadToDeg(double Rad);


class Intergration//浮点数积分
{
public:
    void init(double StartValue,double StartDot);//初始化
    void step(double dt,double dot);//积分
    double val;//value值的大小
private:
    double lastdot;//要累积的量（导数）
};


class Vector//向量类
{
public:
    Vector();
    Vector(double const &xi, double const &yi, double const &zi);

    double Magnitude();//求模
    void Reverse();   //反转向量

    bool operator ==(Vector const &V);
    void operator=(Vector const &T);//向量间赋值运算
    void operator+=(Vector const &T);
    void operator-=(Vector const &T);
    void operator*=(double const &d);

    Vector operator-();//共轭
    Vector operator*(double const &d);
    Vector operator*(Vector const &T);
    Vector operator/(double const &d);
    Vector operator+(Vector const &V);
    Vector operator-(Vector const &V);

    void Copy(Vector const &V);//向量复制
    void Set(double const &x0, double const &y0, double const &z0);//由三个实数初始化向量
    void Set(Vector const &V);//由向量初始化向量
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

Vector operator^(Vector u,Vector v);//叉积
Vector operator*(double s    ,Vector u);//向量、实数间相乘
Vector operator/(Vector u,double s);//向量除以实数#endif // VECTOR_H

#endif
