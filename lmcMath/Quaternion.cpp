#include"Quaternion.h"

//���캯��
Quaternion::Quaternion()
{
    a=0.0; qx= 0.0; qy=0.0; qz = 0.0;
    theta = 0.0;
    Settxx();
}

Quaternion::Quaternion(double const &t, double const &x, double const &y, double const &z)
{
    a=t; qx= x; qy=y; qz = z;
    theta = 2.0*acos(t);
    Settxx();
}

//����ǶԹ�����Ԫ��
Quaternion::Quaternion(double const &Angle, Vector const &R)
{
    Vector N;
    N = R;
    N.Normalize();
    theta = Angle*3.1415926/180.0;

    a = cos(theta/2.0);
    double sina = sin(theta/2.0);

    qx = N.x*sina;
    qy = N.y*sina;
    qz = N.z*sina;
    Settxx();
}

//ת��Ϊ����
void Quaternion::QuattoMat(double m[][4])
{
    if(fabs(a)<=1.0)
        theta = 2.0 * acos(a);
    else
        theta = 0.0;

    t1 =  cos(theta);
    t2 =  1.0 - t1;
    t3 =  qx*qx;
    t6 =  t2*qx;
    t7 =  t6*qy;
    t8 =  sin(theta);
    t9 =  t8*qz;
    t11 = t6*qz;
    t12 = t8*qy;
    t15 = qy*qy;
    t19 = t2*qy*qz;
    t20 = t8*qx;
    t24 = qz*qz;
    m[0][0] = t1 + t2*t3;
    m[0][1] = t7 - t9;
    m[0][2] = t11 + t12;
    m[1][0] = t7 + t9;
    m[1][1] = t1 + t2*t15;
    m[1][2] = t19 - t20;
    m[2][0] = t11 - t12;
    m[2][1] = t19 + t20;
    m[2][2] = t1 + t2*t24;
}

//��λ��
void Quaternion::Normalize()
{
    double norm = sqrt(a*a + qx*qx + qy*qy + qz*qz);
    if (norm < 1.0e-10)
    {
        a = 1.0;
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
    }
    else
    {
        a *= 1/norm;

        qx *= 1/norm;
        qy *= 1/norm;
        qz *= 1/norm;
    }
    Settxx();
}

void Quaternion::operator *=(Quaternion Q)
{
    double t1,t2,t3,t4;

    t1 = a*Q.a  - qx*Q.qx - qy*Q.qy - qz*Q.qz;
    t2 = a*Q.qx + qx*Q.a  + qy*Q.qz - qz*Q.qy ;
    t3 = a*Q.qy + qy*Q.a  + qz*Q.qx - qx*Q.qz ;
    t4 = a*Q.qz + qz*Q.a  + qx*Q.qy - qy*Q.qx ;

    a  = t1;
    qx = t2;
    qy = t3;
    qz = t4;
    Settxx();
}

//���������
Quaternion Quaternion::operator *(Quaternion Q)
{
    Quaternion prod;

    prod.a = a*Q.a  - qx*Q.qx - qy*Q.qy - qz*Q.qz;

    prod.qx = a*Q.qx + qx*Q.a  + qy*Q.qz - qz*Q.qy ;
    prod.qy = a*Q.qy + qy*Q.a  + qz*Q.qx - qx*Q.qz ;
    prod.qz = a*Q.qz + qz*Q.a  + qx*Q.qy - qy*Q.qx ;
    prod.Settxx();

    return prod;
}


void Quaternion::operator =(Quaternion Q)
{
    a  = Q.a;
    qx = Q.qx;
    qy = Q.qy;
    qz = Q.qz;
    Settxx();
}


void Quaternion::operator ~()
{
    qx = -qx;
    qy = -qy;
    qz = -qz;
    Settxx();
}

Vector Quaternion:: ToBody(Vector local)
{
    return(mat*local);
}

Vector Quaternion::ToLocal(Vector body)
{
    return(mat.multrans(body));
}

void Quaternion::init(Vector eulerAngle)//��ŷ��������������Ϊ0ʱ����ʼ���õ�����Ԫ��Ϊ1,0,0,0,����ת����Ϊ��λ����
{
    eulerAngle.x=eulerAngle.y=eulerAngle.z=0;

    double sh = sin(eulerAngle.x/2);//3D��ѧP171���㷨���Աȵ���Ϊ��ȷ
    double ch = cos(eulerAngle.x/2);
    double sp = sin(eulerAngle.y/2);
    double cp = cos(eulerAngle.y/2);
    double sb = sin(eulerAngle.z/2);
    double cb = cos(eulerAngle.z/2);
    //�ӹ��Ե��������Ԫ��
    w.val =  ch*cp*cb+sh*sp*sb;//��
    x.val =  ch*cp*sb-sh*sp*cb;//��
    y.val =  ch*sp*cb+sh*cp*sb;
    z.val = -ch*sp*sb+sh*cp*cb;

    w.init(w.val, 0);
    x.init(x.val, 0);
    y.init(y.val, 0);
    z.init(z.val, 0);

    update_mat();

    updateEuler();
}

void Quaternion::updateEuler()//3D��ѧP164���ӹ��ԡ���������ת������ȡŷ����
{
    euler.y = asin(-1*mat.v[0][2]);            // theta

    if (mat.v[0][0] == 0)
        euler.x = 0;
    else
        euler.x = atan2(mat.v[0][1], mat.v[0][0]); // phi  ƫ��

    if (mat.v[2][2] == 0)
        euler.z = 0;
    else
        euler.z = atan2(mat.v[1][2], mat.v[2][2]); // psi  ��ת

    //ʹPsi��0-360��֮��
    if (euler.x < 0 ) euler.x = euler.x + 2*3.141592653;

}

void Quaternion::step(double dt, Vector omega)//�пط�����ѧ���������� P55
{
    double epw = 0.5 * (-omega.x*x.val -omega.y*y.val -omega.z*z.val);
    double epx = 0.5 * ( omega.x*w.val -omega.y*z.val +omega.z*y.val);
    double epy = 0.5 * ( omega.x*z.val +omega.y*w.val -omega.z*x.val);
    double epz = 0.5 * (-omega.x*y.val +omega.y*x.val +omega.z*w.val);//�˴���ep�������ȶ��㷨��ȷ

    w.step(dt, epw);
    x.step(dt, epx);
    y.step(dt, epy);
    z.step(dt, epz);

    w.val /= length();//��λ��
    x.val /= length();
    y.val /= length();
    z.val /= length();

    update_mat();//����mat
}


void  Quaternion::update_mat()//3D��ѧ P167����Ԫ��ת��������
{
    mat.v[0][0] =  1 - 2*(y.val*y.val + z.val*z.val);//��һ��
    mat.v[0][1] =  2*(x.val*y.val + w.val*z.val);
    mat.v[0][2] =  2*(x.val*z.val - w.val*y.val);

    mat.v[1][0] =  2*(x.val*y.val - w.val*z.val);//�ڶ���
    mat.v[1][1] =  1 - 2*(x.val*x.val + z.val*z.val);
    mat.v[1][2] =  2*(y.val*z.val + w.val*x.val);

    mat.v[2][0] =  2*(x.val*z.val + w.val*y.val);//������
    mat.v[2][1] =  2*(y.val*z.val - w.val*x.val);
    mat.v[2][2] =  1 - 2*(x.val*x.val + y.val*y.val);
}

double Quaternion::length()
{
    //return(sqrt(w.val*w.val + x.val*x.val + y.val*y.val + z.val*z.val));
    double pingfangHe = w.val*w.val + x.val*x.val + y.val*y.val + z.val*z.val;
    return sqrt(pingfangHe);
}

void Quaternion::Settxx()
{
    t2 =   a*qx;
    t3 =   a*qy;
    t4 =   a*qz;
    t5 =  -qx*qx;
    t6 =   qx*qy;
    t7 =   qx*qz;
    t8 =  -qy*qy;
    t9 =   qy*qz;
    t10 = -qz*qz;
}

//inline functions
void Quaternion::Conjugate(Vector const &Vin, Vector &Vout)
{
    Vout.x = 2.0*( (t8 + t10)*Vin.x + (t6 -  t4)*Vin.y + (t3 + t7)*Vin.z ) + Vin.x;
    Vout.y = 2.0*( (t4 +  t6)*Vin.x + (t5 + t10)*Vin.y + (t9 - t2)*Vin.z ) + Vin.y;
    Vout.z = 2.0*( (t7 -  t3)*Vin.x + (t2 +  t9)*Vin.y + (t5 + t8)*Vin.z ) + Vin.z;
}

void Quaternion::Conjugate(Vector &V)
{
    R.x = V.x;
    R.y = V.y;
    R.z = V.z;

    V.x = 2.0*( (t8 + t10)*R.x + (t6 -  t4)*R.y + (t3 + t7)*R.z ) + R.x;
    V.y = 2.0*( (t4 +  t6)*R.x + (t5 + t10)*R.y + (t9 - t2)*R.z ) + R.y;
    V.z = 2.0*( (t7 -  t3)*R.x + (t2 +  t9)*R.y + (t5 + t8)*R.z ) + R.z;
}

void Quaternion::Conjugate(double &x, double &y, double &z)
{
    R.x = x;
    R.y = y;
    R.z = z;

    x = 2.0*( (t8 + t10)*R.x + (t6 -  t4)*R.y + (t3 + t7)*R.z ) + R.x;
    y = 2.0*( (t4 +  t6)*R.x + (t5 + t10)*R.y + (t9 - t2)*R.z ) + R.y;
    z = 2.0*( (t7 -  t3)*R.x + (t2 +  t9)*R.y + (t5 + t8)*R.z ) + R.z;
}

void Quaternion::Set(double const &real, double const &x, double const &y, double const &z)
{
    a = real;
    qx = x;
    qy = y;
    qz = z;
    Settxx();
}

void Quaternion::Set(double const &Angle, Vector const &R)
{
    Vector N;
    N = R;
    N.Normalize();
    theta = Angle*3.1415926/180.0;

    a = cos(theta/2.0);
    double sina = sin(theta/2.0);

    qx = N.x*sina;
    qy = N.y*sina;
    qz = N.z*sina;
    Settxx();
}
