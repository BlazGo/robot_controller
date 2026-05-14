#ifndef UTILS_H
#define UTILS_H

struct Matrix6x6 {
  float m[6][6];
};

struct Matrix4x4 {
  float m[4][4];
};

struct Matrix3x3 {
  float m[3][3];
};

struct Vect3f {
  float v[3];
};

struct Vect6f {
  float v[6];
};

Vect3f cross(const Vect3f& a, const Vect3f& b);
Vect3f subtractVec(const Vect3f& A, const Vect3f& B);
Vect3f sumVec(const Vect3f& A, const Vect3f& B);
Matrix3x3 multiplyMatrices(const Matrix3x3& A, const Matrix3x3& B);
Matrix4x4 multiplyMatrices(const Matrix4x4& A, const Matrix4x4& B);
Matrix6x6 multiplyMatrices(const Matrix6x6& A, const Matrix6x6& B);
Matrix6x6 invertMatrix(const Matrix6x6& A);
Matrix3x3 transposeMat(Matrix3x3 mat);
Matrix6x6 transposeMat(Matrix6x6 mat);
Matrix4x4 getIdentityMatrix(void);

Matrix3x3 getRotationMatrixFromPoseMatrix(const Matrix4x4& M);
Matrix3x3 eulerAnglesToRotationMatrix(float euler[3]);
Vect3f rotationMatrixToEulerAngles(Matrix3x3& M);
Vect3f eulRadToDeg(Vect3f& eulAngles);

Matrix4x4 getAMatrix(float d, float theta, float a, float alpha, float q);


void printArr6x1(float arr[6]);
void printVect6x1(Vect6f vect);
void printMatrix4x4(const Matrix4x4& M);
void printMatrix6x6(const Matrix6x6& M);

float moveTowards(float current, float target, float max_delta);
float clampAbsFloat(float value, float abs_limit);


#endif // UTILS_H