#include <BasicLinearAlgebra.h>
using namespace BLA;

Matrix<4> Normalize(Matrix<4> v) {
  float mag = sqrt(v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
  v(0) /= mag;
  v(1) /= mag;
  v(2) /= mag;

  return v;
}

float Dot(Matrix<4> v1, Matrix<4> v2) {
  float result = v1(0) * v2(0) + v1(1) * v2(1) + v1(2) * v2(2);
  return result;
}

Matrix<4> Cross(Matrix<4> v1, Matrix<4> v2) {
  Matrix<4> result = {
    v1(1) * v2(2) - v1(2) * v2(1),
    v1(2) * v2(0) - v1(0) * v2(2),
    v1(0) * v2(1) - v1(1) * v2(0),
    v1(3)
  };
  return result;
}

Matrix<4> PlaneEquation(Matrix<4> p1, Matrix<4> p2, Matrix<4> p3) {

  float a = (p2(1) - p1(1)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(1) - p1(1));
  float b = (p2(2) - p1(2)) * (p3(0) - p1(0)) - (p2(0) - p1(0)) * (p3(2) - p1(2));
  float c = (p2(0) - p1(0)) * (p3(1) - p1(1)) - (p2(1) - p1(1)) * (p3(0) - p1(0));
  float d = -a * p1(0) - b * p1(1) - c * p1(2);

  Matrix<4> equation = { a, b, c, d };
  return equation;
}

Matrix<4> GetIntersection(Matrix<4> plane, Matrix<4> p1, Matrix<4> p2) {
  float a = plane(0);
  float b = plane(1);
  float c = plane(2);
  float d = plane(3);
  float x4 = p1(0);
  float y4 = p1(1);
  float z4 = p1(2);
  float x5 = p2(0);
  float y5 = p2(1);
  float z5 = p2(2);

  float t = -(a * x4 + b * y4 + c * z4 + d) / (a * (x5 - x4) + b * (y5 - y4) + c * (z5 - z4));

  Matrix<4> intersection = {
    x4 + (x5 - x4) * t,
    y4 + (y5 - y4) * t,
    z4 + (z5 - z4) * t,
    1
  };

  return intersection;
}