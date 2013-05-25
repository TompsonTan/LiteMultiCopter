#include "Vector.h"

void Intergration::init(double StartValue, double StartDot)
{
    lastdot = StartDot;
    val = StartValue;
}

void Intergration::step(double dt, double dot)
{
    val = val+(dot+ lastdot)*0.5*dt;
    lastdot=dot;
}

Vector::Vector()
{
    x = 0;
    y = 0;
    z = 0;
}

Vector::Vector(double const &xi, double const &yi, double const &zi)
{
    x = xi;
    y = yi;
    z = zi;
}

double Vector::Magnitude()
{
    return (double)sqrt(x*x+y*y+z*z);
}

void Vector::Reverse()
{
    x = -x;
    y = -y;
    z = -z;
}

bool Vector::operator ==(Vector const &V)
{
    return (V.x-x)*(V.x-x) + (V.y-y)*(V.y-y) + (V.z-z)*(V.z-z)<0.000000001;
}

void Vector::operator=(Vector const &T)//向量间赋值
{
    x=T.x;
    y=T.y;
    z=T.z;
}

void Vector::operator+=(Vector const &T)
{
    x += T.x;
    y += T.y;
    z += T.z;
}

void Vector::operator-=(Vector const &T)
{
    x -= T.x;
    y -= T.y;
    z -= T.z;
}

void Vector::operator*=(double const &d)
{
    x *= d;
    y *= d;
    z *= d;
}


Vector Vector::operator -()
{
    return Vector(-x,-y,-z);
}

Vector Vector::operator *(double const &d)
{
    Vector T(x*d, y*d, z*d);
    return T;
}

//点积
Vector Vector::operator *(Vector const &T)
{
    Vector C;
    C.x =  y*T.z - z*T.y;
    C.y = -x*T.z + z*T.x;
    C.z =  x*T.y - y*T.x;
    return C;
}

Vector Vector::operator /(double const &d)
{
    Vector T(x/d, y/d, z/d);
    return T;
}

Vector Vector::operator +(Vector const &V)
{
    Vector T(x+V.x, y+V.y, z+V.z);
    return T;
}

Vector Vector::operator -(Vector const &V)
{
    Vector T(x-V.x, y-V.y, z-V.z);
    return T;
}

void Vector::Copy(Vector const &V)
{
    x = V.x;
    y = V.y;
    z = V.z;
}

void Vector::Set(double const &x0, double const &y0, double const &z0)
{
    x = x0;
    y = y0;
    z = z0;
}

void Vector::Set(Vector const &V)
{
    x = V.x;
    y = V.y;
    z = V.z;
}

void Vector::Normalize()
{
    double abs = VAbs();
    if(abs< 1.e-10) return;
    x/=abs;
    y/=abs;
    z/=abs;
}

double Vector::VAbs()
{
    return sqrt(x*x+y*y+z*z);
}

double Vector::dot(Vector const &V)
{
    return x*V.x + y*V.y + z*V.z;
}

bool Vector::IsSame(Vector const &V)
{
    //used only to compare point positions
    return (V.x-x)*(V.x-x) + (V.y-y)*(V.y-y) + (V.z-z)*(V.z-z)<0.000000001;
}

void Vector::Translate(Vector const &T)
{
    x += T.x;
    y += T.y;
    z += T.z;
}

//运算符重载
Vector operator^(Vector u,Vector v)//叉积
{
    return Vector(u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x);
}

Vector operator*(double s    ,Vector u)//向量、实数间相乘
{
    return Vector(u.x*s, u.y*s, u.z*s);
}

Vector operator/(Vector u, double s)//除以实数
{
    return Vector(u.x/s,u.y/s,u.z/s);
}
