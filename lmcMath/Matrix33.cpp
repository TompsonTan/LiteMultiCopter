#include"Matrix33.h"

Matrix33::Matrix33()
{
    for(int m=0;m<3;m++)
        for(int n=0;n<3;n++)
            v[m][n]=0;
}

Matrix33::Matrix33(double i00, double i01, double i02,
                   double i10, double i11, double i12,
                   double i20, double i21, double i22)
{
    v[0][0] = i00;
    v[0][1] = i01;
    v[0][2] = i02;
    v[1][0] = i10;
    v[1][1] = i11;
    v[1][2] = i12;
    v[2][0] = i20;
    v[2][1] = i21;
    v[2][2] = i22;
}

Matrix33::Matrix33(const Matrix33 &mb)
{
    for(int m=0;m<3;m++)
        for(int n=0;n<3;n++)
            v[m][n]=mb.v[m][n];
}

double Matrix33::det()
{
    return(
                v[0][0]*v[1][1]*v[2][2] + v[0][1]*v[1][2]*v[2][0]
                + v[0][2]*v[1][0]*v[2][1] - v[0][2]*v[1][1]*v[2][0]
                - v[0][1]*v[1][0]*v[2][2] - v[1][2]*v[2][1]*v[0][0]
                );
}

Matrix33  Matrix33::inverse()
{
    double rdet = 1.0/det();

    double i00 = rdet*(v[1][1]*v[2][2]-v[1][2]*v[2][1]);
    double i10 = rdet*(v[1][2]*v[2][0]-v[1][0]*v[2][2]);
    double i20 = rdet*(v[1][0]*v[2][1]-v[1][1]*v[2][0]);
    double i01 = rdet*(v[0][2]*v[2][1]-v[0][1]*v[2][2]);
    double i11 = rdet*(v[0][0]*v[2][2]-v[0][2]*v[2][0]);
    double i21 = rdet*(v[0][1]*v[2][0]-v[0][0]*v[2][1]);
    double i02 = rdet*(v[0][1]*v[1][2]-v[0][2]*v[1][1]);
    double i12 = rdet*(v[0][2]*v[1][0]-v[0][0]*v[1][2]);
    double i22 = rdet*(v[0][0]*v[1][1]-v[0][1]*v[1][0]);

    return Matrix33(
                i00, i01, i02,
                i10, i11, i12,
                i20, i21, i22
                );
}

Matrix33& Matrix33::operator =(const Matrix33&b)
{
    for (int m=0; m<3; m++)
        for (int n=0; n<3; n++)
            v[m][n] = b.v[m][n];
    return *this;
}

Vector    Matrix33::multrans(const Vector &b)
{
    Vector tmp;

    tmp.x=b.x*v[0][0]+b.y*v[1][0]+b.z*v[2][0];
    tmp.y=b.x*v[0][1]+b.y*v[1][1]+b.z*v[2][1];
    tmp.z=b.x*v[0][2]+b.y*v[1][2]+b.z*v[2][2];

    return tmp;
}

Vector    Matrix33::operator *(const Vector &b)
{
    Vector tmp;

    tmp.x=v[0][0]*b.x+v[0][1]*b.y+v[0][2]*b.z;
    tmp.y=v[1][0]*b.x+v[1][1]*b.y+v[1][2]*b.z;
    tmp.z=v[2][0]*b.x+v[2][1]*b.y+v[2][2]*b.z;
    return tmp;
}

Matrix33  Matrix33::operator *(const Matrix33 &b)
{
    Matrix33 tmp;

    for (int m=0; m<3; m++)
    {
        for (int n=0; n<3; n++)
        {
            tmp.v[m][n] = 0;
            for (int i=0; i<3; i++)
                tmp.v[m][n] += v[m][i]*b.v[i][n];
        }
    }

    return tmp;
}

Matrix33  Matrix33::operator -(const Matrix33 &b)
{
    Matrix33 tmp;

    for (int m=0; m<3; m++)
    {
        for (int n=0; n<3; n++)
        {
            tmp.v[m][n] = v[m][n]-b.v[m][n];
        }
    }

    return tmp;
}

Matrix33  Matrix33::trans()
{
    return(Matrix33(
               v[0][0], v[1][0], v[2][0],
               v[0][1], v[1][1], v[2][1],
               v[0][2], v[1][2], v[2][2]
               )
           );
}
