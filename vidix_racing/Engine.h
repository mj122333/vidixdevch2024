#include <Adafruit_ILI9341.h>
#include <Adafruit_GFX.h>
#include "Staza.h"

#define TFT_WIDTH 320
#define TFT_HEIGHT 240

class Object {
public:
  int numOfVerts;
  Matrix<3> *verteces;
  int numOfLines;
  Matrix<2, 1, int> *lines;

  Matrix<3> position;
  Matrix<3> rotation;
  Matrix<3> scale;

  Object(Object *other) {
    this->numOfVerts = other->numOfVerts;
    this->verteces = (Matrix<3> *)malloc(this->numOfVerts * sizeof(Matrix<3>));
    for (int i = 0; i < this->numOfVerts; i++) {
      this->verteces[i] = other->verteces[i];
    }
    this->numOfLines = other->numOfLines;
    this->lines = (Matrix<2, 1, int> *)malloc(this->numOfLines * sizeof(Matrix<2, 1, int>));
    for (int i = 0; i < this->numOfLines; i++) {
      this->lines[i] = other->lines[i];
    }
    this->position = other->position;
    this->rotation = other->rotation;
    this->scale = other->scale;
  }

  Object(int numOfVerts, Matrix<3> verteces[], int numOfLines, Matrix<2, 1, int> lines[]) {
    this->numOfVerts = numOfVerts;
    this->verteces = verteces;
    this->numOfLines = numOfLines;
    this->lines = lines;

    position.Fill(0);
    rotation.Fill(0);
    scale.Fill(1);
  }

  Object(int numOfVerts, Matrix<3> verteces[], int numOfLines, Matrix<2, 1, int> lines[], Matrix<3> position, Matrix<3> rotation, Matrix<3> scale) {
    this->numOfVerts = numOfVerts;
    this->verteces = verteces;
    this->numOfLines = numOfLines;
    this->lines = lines;

    this->position = position;
    this->rotation = rotation;
    this->scale = scale;
  }

  Matrix<4, 4> getObjectToWorldMatrix() {
    Matrix<4, 4> translationMat = {
      1, 0, 0, position(0),
      0, 1, 0, position(1),
      0, 0, 1, position(2),
      0, 0, 0, 1
    };
    Matrix<4, 4> scaleMat = {
      scale(0), 0, 0, 0,
      0, scale(1), 0, 0,
      0, 0, scale(2), 0,
      0, 0, 0, 1
    };
    Matrix<4, 4> rotateMatX = {
      1, 0, 0, 0,
      0, cos(rotation(0)), -sin(rotation(0)), 0,
      0, sin(rotation(0)), cos(rotation(0)), 0,
      0, 0, 0, 1
    };
    Matrix<4, 4> rotateMatY = {
      cos(rotation(1)), 0, sin(rotation(1)), 0,
      0, 1, 0, 0,
      -sin(rotation(1)), 0, cos(rotation(1)), 0,
      0, 0, 0, 1
    };
    Matrix<4, 4> rotateMatZ = {
      cos(rotation(2)), -sin(rotation(2)), 0, 0,
      sin(rotation(2)), cos(rotation(2)), 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
    };

    return translationMat * rotateMatX * rotateMatY * rotateMatZ * scaleMat;
  }
};

class Camera {
private:
  Matrix<4> farTopLeft, farTopRight, farBottomLeft, farBottomRight;
  Matrix<4> nearTopLeft, nearTopRight, nearBottomLeft, nearBottomRight;

  Matrix<4> leftPlaneNormal, topPlaneNormal, rightPlaneNormal, bottomPlaneNormal;

  Matrix<4> nearPlane, farPlane, leftPlane, topPlane, rightPlane, bottomPlane;

  void calculateFrustum() {
    Matrix<4> right = { 1, 0, 0, 0 };
    Matrix<4> up = { 0, 1, 0, 0 };
    Matrix<4> forward = { 0, 0, 1, 0 };
    Matrix<4> nearCenter = forward * znear;
    Matrix<4> farCenter = forward * zfar;
    float nearHeightHalf = tan(fov / 2) * znear;
    float farHeightHalf = tan(fov / 2) * zfar;
    float nearWidthHalf = nearHeightHalf * aspect;
    float farWidthHalf = farHeightHalf * aspect;

    farTopLeft = farCenter + up * farHeightHalf - right * farWidthHalf;
    farTopRight = farCenter + up * farHeightHalf + right * farWidthHalf;
    farBottomLeft = farCenter - up * farHeightHalf - right * farWidthHalf;
    farBottomRight = farCenter - up * farHeightHalf + right * farWidthHalf;

    nearTopLeft = nearCenter + up * nearHeightHalf - right * nearWidthHalf;
    nearTopRight = nearCenter + up * nearHeightHalf + right * nearWidthHalf;
    nearBottomLeft = nearCenter - up * nearHeightHalf - right * nearWidthHalf;
    nearBottomRight = nearCenter - up * nearHeightHalf + right * nearWidthHalf;

    Matrix<4> p0, p1, p2;

    p0 = nearBottomLeft;
    p1 = farBottomLeft;
    p2 = farTopLeft;
    leftPlane = PlaneEquation(p0, p1, p2);
    leftPlaneNormal = Normalize(Cross(Normalize(p1 - p0), Normalize(p2 - p1)));

    p0 = nearTopLeft;
    p1 = farTopLeft;
    p2 = farTopRight;
    topPlane = PlaneEquation(p0, p1, p2);
    topPlaneNormal = Normalize(Cross(Normalize(p1 - p0), Normalize(p2 - p1)));

    p0 = nearTopRight;
    p1 = farTopRight;
    p2 = farBottomRight;
    rightPlane = PlaneEquation(p0, p1, p2);
    rightPlaneNormal = Normalize(Cross(Normalize(p1 - p0), Normalize(p2 - p1)));

    p0 = nearBottomRight;
    p1 = farBottomRight;
    p2 = farBottomLeft;
    bottomPlane = PlaneEquation(p0, p1, p2);
    bottomPlaneNormal = Normalize(Cross(Normalize(p1 - p0), Normalize(p2 - p1)));

    p0 = nearBottomRight;
    p1 = nearTopRight;
    p2 = nearTopLeft;
    nearPlane = PlaneEquation(p0, p1, p2);

    p0 = farBottomRight;
    p1 = farTopRight;
    p2 = farTopLeft;
    farPlane = PlaneEquation(p0, p1, p2);
  }

  int insideFrustum(Matrix<4> &point) {
    //if (-point(2) < znear) return 1;
    //if (-point(2) > zfar) return 2;
    float left = max((float)0, Dot(Normalize(point), leftPlaneNormal));
    float top = max((float)0, Dot(Normalize(point), topPlaneNormal));
    float right = max((float)0, Dot(Normalize(point), rightPlaneNormal));
    float bottom = max((float)0, Dot(Normalize(point), bottomPlaneNormal));
    if (left <= 0.001 && top <= 0.001 && right <= 0.001 && bottom <= 0.001) return 0;

    if (left >= top && left >= right && left >= bottom) return 3;
    if (top >= left && top >= right && top >= bottom) return 4;
    if (right >= top && right >= left && right >= bottom) return 5;
    return 6;
  }

  void drawLine(Matrix<4> &p1, Matrix<4> &p2, int out1, int out2, Matrix<4, 4> &perspectiveMat, Adafruit_ILI9341 &tft, int color){
    if (out1 && out2) return;
      else if (out1) {
        switch (out1) {
          case 1:
            p1 = GetIntersection(nearPlane, p1, p2);
            break;
          case 2:
            p1 = GetIntersection(farPlane, p1, p2);
            break;
          case 3:
            p1 = GetIntersection(leftPlane, p1, p2);
            break;
          case 4:
            p1 = GetIntersection(topPlane, p1, p2);
            break;
          case 5:
            p1 = GetIntersection(rightPlane, p1, p2);
            break;
          case 6:
            p1 = GetIntersection(bottomPlane, p1, p2);
            break;
        }
      } else if (out2) {
        switch (out2) {
          case 1:
            p2 = GetIntersection(nearPlane, p1, p2);
            break;
          case 2:
            p2 = GetIntersection(farPlane, p1, p2);
            break;
          case 3:
            p2 = GetIntersection(leftPlane, p1, p2);
            break;
          case 4:
            p2 = GetIntersection(topPlane, p1, p2);
            break;
          case 5:
            p2 = GetIntersection(rightPlane, p1, p2);
            break;
          case 6:
            p2 = GetIntersection(bottomPlane, p1, p2);
            break;
        }
      }

      p1 = perspectiveMat * p1;
      p2 = perspectiveMat * p2;

      if (p1(3) != 0) {
        p1(0) /= (p1(3));
        p1(1) /= (p1(3));
      }
      p1(0) = (p1(0) + 1) / 2;
      p1(0) = p1(0) * TFT_WIDTH;
      p1(1) = (p1(1) + 1) / 2;
      p1(1) = p1(1) * TFT_HEIGHT;
      if (p2(3) != 0) {
        p2(0) /= (p2(3));
        p2(1) /= (p2(3));
      }
      p2(0) = (p2(0) + 1) / 2;
      p2(0) = p2(0) * TFT_WIDTH;
      p2(1) = (p2(1) + 1) / 2;
      p2(1) = p2(1) * TFT_HEIGHT;

      tft.drawLine(p1(0), p1(1), p2(0), p2(1), color);
  }

public:
  Matrix<3> position;
  Matrix<3> rotation;
  float znear, zfar;
  float fov;
  const float aspect = TFT_WIDTH / TFT_HEIGHT;

  Camera() {
    position.Fill(0);
    rotation.Fill(0);
    znear = 0.1;
    zfar = 1000;
    fov = PI / 2;

    calculateFrustum();
  }

  Camera(Matrix<3> position, Matrix<3> rotation) {
    this->position = position;
    this->rotation = rotation;
    znear = 0.1;
    zfar = 1000;
    fov = PI / 2;

    calculateFrustum();
  }

  Camera(Matrix<3> position, Matrix<3> rotation, float znear, float zfar, float fov) {
    this->position = position;
    this->rotation = rotation;
    this->znear = znear;
    this->zfar = zfar;
    this->fov = fov;

    calculateFrustum();
  }

  Matrix<4, 4> getWorldToViewMat() {
    Matrix<4, 4> translationMat = {
      1, 0, 0, -position(0),
      0, 1, 0, -position(1),
      0, 0, 1, -position(2),
      0, 0, 0, 1
    };
    Matrix<4, 4> rotateMatX = {
      1, 0, 0, 0,
      0, cos(-rotation(0)), -sin(-rotation(0)), 0,
      0, sin(-rotation(0)), cos(-rotation(0)), 0,
      0, 0, 0, 1
    };
    Matrix<4, 4> rotateMatY = {
      cos(-rotation(1)), 0, sin(-rotation(1)), 0,
      0, 1, 0, 0,
      -sin(-rotation(1)), 0, cos(-rotation(1)), 0,
      0, 0, 0, 1
    };
    Matrix<4, 4> rotateMatZ = {
      cos(-rotation(2)), -sin(-rotation(2)), 0, 0,
      sin(-rotation(2)), cos(-rotation(2)), 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
    };

    return rotateMatX * rotateMatY * rotateMatZ * translationMat;
  }

  Matrix<4, 4> getOrthoMat(float width, float height) {
    Matrix<4, 4> ortho = {
      1 / width, 0, 0, 0,
      0, 1 / height, 0, 0,
      0, 0, -(2 / (zfar - znear)), -((zfar + znear) / (zfar - znear)),
      0, 0, 0, 1
    };

    return ortho;
  }

  Matrix<4, 4> getPerspectiveMat() {
    float c = 1 / tan(fov / 2);

    Matrix<4, 4> perspective = {
      c / aspect, 0, 0, 0,
      0, c, 0, 0,
      0, 0, -(zfar + znear) / (zfar - znear), -2 * (zfar * znear) / (zfar - znear),
      0, 0, -1, 0
    };

    return perspective;
  }

  void drawObject(Object &object, Adafruit_ILI9341 &tft, int color) {
    Matrix<4, 4> objectToWorldMat = object.getObjectToWorldMatrix();
    Matrix<4, 4> worldToViewMat = getWorldToViewMat();
    Matrix<4, 4> perspectiveMat = getPerspectiveMat();
    Matrix<4, 4> objectToViewMat = worldToViewMat * objectToWorldMat;

    Matrix<4> verteces[object.numOfVerts];
    int outside[object.numOfLines];

    for (int i = 0; i < object.numOfVerts; i++) {
      verteces[i] = { object.verteces[i](0), object.verteces[i](1), object.verteces[i](2), 1 };
      verteces[i] = objectToViewMat * verteces[i];
      outside[i] = insideFrustum(verteces[i]);
    }

    for (int i = 0; i < object.numOfLines; i++) {
      Matrix<4> p1 = verteces[object.lines[i](0)];
      Matrix<4> p2 = verteces[object.lines[i](1)];
      int out1 = outside[object.lines[i](0)];
      int out2 = outside[object.lines[i](1)];

      drawLine(p1, p2, out1, out2, perspectiveMat, tft, color);
    }
  }

  void drawPolygon(Polygon &polygon, Adafruit_ILI9341 &tft, int color) {
    Matrix<4, 4> worldToViewMat = getWorldToViewMat();
    Matrix<4, 4> perspectiveMat = getPerspectiveMat();

    Matrix<4> points[polygon.numOfPoints];
    int outside[polygon.numOfPoints];

    for (int i = 0; i < polygon.numOfPoints; i++) {
      points[i] = { polygon.points[i](0), 0, polygon.points[i](1), 1 };
      points[i] = worldToViewMat * points[i];
      outside[i] = insideFrustum(points[i]);
    }

    for (int i = 0; i < polygon.numOfPoints; i++) {
      Matrix<4> p1 = { points[i](0), points[i](1), points[i](2), points[i](3) };
      Matrix<4> p2 = { points[(i + 1) % polygon.numOfPoints](0), points[(i + 1) % polygon.numOfPoints](1), points[(i + 1) % polygon.numOfPoints](2), points[(i + 1) % polygon.numOfPoints](3) };

      drawLine(p1, p2, outside[i], outside[(i + 1) % polygon.numOfPoints], perspectiveMat, tft, color);
    }
  }
};