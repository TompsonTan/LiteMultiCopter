#ifndef QUATERNION_H
#define QUATERNION_H

#include"Vector.h"
#include"Matrix33.h"

class Quaternion//四元数类
{
public:
    Quaternion();
    Quaternion(double const &t, double const &x, double const &y, double const &z);
    Quaternion(double const &Angle, Vector const &R);
public:
    Vector ToBody(Vector local);//全局转换到机体
    Vector ToLocal(Vector body);//机体转换到全局
    void init(Vector eulerAngle);//由欧拉角初始化四元数
    void updateEuler();//更新欧拉角
    void step(double dt,Vector omega);//积分

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

    Vector euler;//欧拉角 （姿态角phi, theta, psi）
    Matrix33 mat;//全局到机体的转换矩阵
private:
    double theta;
    double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t15, t19, t20, t24;
    Vector R;

    void update_mat();//更新矩阵
    double length();//求模
    Intergration x,y,z,w;//四元数分量
};

#endif // QUATERNION_H
