#include "Arduino.h"
#include "utils.h"
#include "MatrixMath.h"

Vect3f cross(const Vect3f& a, const Vect3f& b){
  Vect3f res;
  res.v[0] = a.v[1]*b.v[2] - a.v[2]*b.v[1];
  res.v[1] = a.v[2]*b.v[0] - a.v[0]*b.v[2];
  res.v[2] = a.v[0]*b.v[1] - a.v[1]*b.v[0];
  return res;
}

Vect3f subtractVec(const Vect3f& A, const Vect3f& B){
  Vect3f res;
  for (int i=0; i<3; i++){
    res.v[i] = A.v[i] - B.v[i];
  }
  return res;
}

Vect3f sumVec(const Vect3f& A, const Vect3f& B){
  Vect3f res;
  for (int i=0; i<3; i++){
    res.v[i] = A.v[i] + B.v[i];
  }
  return res;
}

Matrix4x4 multiplyMatrices(const Matrix4x4& A, const Matrix4x4& B){
  Matrix4x4 AB{};

  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      for (int k=0; k<4; k++){
        AB.m[i][j] += A.m[i][k] * B.m[k][j];
      }
    }
  }
  return AB;
}

Matrix3x3 multiplyMatrices(const Matrix3x3& A, const Matrix3x3& B){
  Matrix3x3 AB{};

  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      for (int k=0; k<3; k++){
        AB.m[i][j] += A.m[i][k] * B.m[k][j];
      }
    }
  }
  return AB;
}

Matrix6x6 multiplyMatrices(const Matrix6x6& A, const Matrix6x6& B){
  Matrix6x6 AB{};

  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      for (int k=0; k<6; k++){
        AB.m[i][j] += A.m[i][k] * B.m[k][j];
      }
    }
  }
  return AB;
}

Matrix6x6 invertMatrix(const Matrix6x6& A){
  Matrix6x6 A_inv;

  mtx_type A_temp[6][6];
  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      A_temp[i][j] = A.m[i][j];
    }
  }

  Matrix.Invert((mtx_type*)A_temp, 6);
  
  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      A_inv.m[i][j] = A_temp[i][j];
    }
  }
  
  return A_inv;
}

Matrix4x4 getIdentityMatrix(void){
  Matrix4x4 I = {
    {{1.0f, 0.0f, 0.0f, 0.0f},
     {0.0f, 1.0f, 0.0f, 0.0f},
     {0.0f, 0.0f, 1.0f, 0.0f},
     {0.0f, 0.0f, 0.0f, 1.0f}}
  };
  return I;
}

void printArr6x1(float arr[6]){
  for (int i=0; i<6; i++){
    Serial.print(arr[i], 2);
    Serial.print("\t");
  }
  Serial.println("");
}

void printVect6x1(Vect6f vect){
  uint8_t precision = 2;
  for (int i=0; i<6; i++){
    Serial.print(vect.v[i], precision);
    Serial.print("\t");
  }
  Serial.println(" ");
}

void printMatrix4x4(const Matrix4x4& M){
  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      Serial.print(M.m[i][j], 2);
      Serial.print("\t");
    }
    Serial.println(" ");
  }
  Serial.println(" ");
}

void printMatrix6x6(const Matrix6x6& M){
  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      Serial.print(M.m[i][j], 2);
      Serial.print("\t");
    }
    Serial.println(" ");
  }
  Serial.println(" ");
}

Matrix4x4 getAMatrix(float d, float theta, float a, float alpha, float q){
  float th = theta + q;

  Matrix4x4 A = {{
    {cos(th), -sin(th)*cos(alpha),  sin(th)*sin(alpha), a*cos(th)},
    {sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th)},
    {0.0f,     sin(alpha),          cos(alpha),         d},
    {0.0f,     0.0f,                0.0f,               1.0f}
  }};
  return A;
}

Matrix3x3 getRotationMatrixFromPoseMatrix(const Matrix4x4& M){
  Matrix3x3 rotM;

  rotM.m[0][0] = M.m[0][0];
  rotM.m[0][1] = M.m[0][1];
  rotM.m[0][2] = M.m[0][2];

  rotM.m[1][0] = M.m[1][0];
  rotM.m[1][1] = M.m[1][1];
  rotM.m[1][2] = M.m[1][2];

  rotM.m[2][0] = M.m[2][0];
  rotM.m[2][1] = M.m[2][1];
  rotM.m[2][2] = M.m[2][2];
  return rotM;
}

Matrix3x3 eulerAnglesToRotationMatrix(float euler[3]){
  Matrix3x3 rotM;

  float psi   = euler[2]; // Z yaw
  float theta = euler[1]; // Y pitch
  float fi    = euler[0]; // x roll

  rotM.m[0][0] = cos(psi) * cos(theta);
  rotM.m[0][1] = cos(psi) * sin(theta) * sin(fi) - sin(psi) * cos(fi);
  rotM.m[0][2] = cos(psi) * sin(theta) * cos(fi) + sin(psi) * sin(fi);
  rotM.m[1][0] = sin(psi) * cos(theta);
  rotM.m[1][1] = sin(psi) * sin(theta) * sin(fi) + cos(psi) * cos(fi);
  rotM.m[1][2] = sin(psi) * sin(theta) * cos(fi) - cos(psi) * sin(fi);
  rotM.m[2][0] = -sin(theta);
  rotM.m[2][1] = cos(theta) * sin(fi);
  rotM.m[2][2] = cos(theta) * cos(fi);

  return rotM;
}

Vect3f rotationMatrixToEulerAngles(Matrix3x3& M){
  Vect3f eulAngles;
  eulAngles.v[1] = -asin(M.m[2][0]);
  
  if (abs(M.m[2][0]) < 0.9999999) {
    eulAngles.v[0] = atan2(M.m[2][1], M.m[2][2]);
    eulAngles.v[2] =  atan2(M.m[1][0], M.m[0][0]);
  }
  else{
    // Gimbal lock case
    eulAngles.v[0] = atan2(-M.m[1][2], M.m[1][1]);
    eulAngles.v[2] = 0;
  }

  return eulAngles;
}

Vect3f eulRadToDeg(Vect3f& eulAngles){
  Vect3f res;
  res.v[0] = eulAngles.v[0] * (180.0 / PI);
  res.v[1] = eulAngles.v[1] * (180.0 / PI);
  res.v[2] = eulAngles.v[2] * (180.0 / PI);
  return res;
}

Matrix6x6 transposeMat(Matrix6x6 mat){
  Matrix6x6 mat_T;
  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      mat_T.m[j][i] = mat.m[i][j];
    }    
  }
  return mat_T;
}

Matrix3x3 transposeMat(Matrix3x3 mat){
  Matrix3x3 mat_T;
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      mat_T.m[j][i] = mat.m[i][j];
    }    
  }
  return mat_T;
}

float moveTowards(float current, float target, float max_delta){
  float delta = target - current;
  
  // if target is outside of boudaries clamp it
  if (delta > max_delta){
    return current + max_delta;
  }
  if (delta < -max_delta){
    return current - max_delta;
  }
  // if the target is within boundaries return the already valid target
  return target;
}

float clampAbsFloat(float value, float abs_limit){
  // check if it's negative
  if (abs_limit < 0.0f){
    abs_limit = -abs_limit;
  }

  if (value > abs_limit){
    return abs_limit;
  }
  if (value < -abs_limit){
    return -abs_limit;
  }
  return value;
}

