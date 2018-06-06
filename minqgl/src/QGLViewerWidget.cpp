/*===========================================================================*\
 *
 * CG2 Sandbox - TU-Berlin Computer Graphics - SS13
 * Author : Olivier Rouiller
 *
\*===========================================================================*/

//== INCLUDES =================================================================

#ifdef _MSC_VER
#pragma warning(disable : 4267 4311 4305)
#endif

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <queue>
#include <sstream>

#include <cstdlib>

#include <qapplication.h>
#include <qcursor.h>
#include <qdatetime.h>
#include <qmenu.h>
#include <qmessagebox.h>
#include <qnamespace.h>
#include <QElapsedTimer>
#include <QMouseEvent>
// --------------------

#include "QGLViewerWidget.hpp"
#define GLM_ENABLE_EXPERIMENTAL 1

#if !defined(M_PI)
#define M_PI 3.1415926535897932
#endif

const double TRACKBALL_RADIUS = 0.6;

using namespace Qt;
using namespace glm;

//== IMPLEMENTATION ==========================================================

//----------------------------------------------------------------------------

QGLViewerWidget::QGLViewerWidget(QWidget *_parent) : QGLWidget(_parent) {
  assert(glGetError() == GL_NO_ERROR);
  init();
}

//----------------------------------------------------------------------------

QGLViewerWidget::QGLViewerWidget(QGLFormat &_fmt, QWidget *_parent)
    : QGLWidget(_fmt, _parent) {
  assert(glGetError() == GL_NO_ERROR);
  init();
}

//----------------------------------------------------------------------------

void QGLViewerWidget::init() {
  // qt stuff
  setAttribute(Qt::WA_NoSystemBackground, true);
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setCursor(PointingHandCursor);
  assert(glGetError() == GL_NO_ERROR);
  threads.push_back(std::thread(&QGLViewerWidget::animateLight, this));
}

//----------------------------------------------------------------------------

QGLViewerWidget::~QGLViewerWidget() {}

bool QGLViewerWidget::loadPointSet(const char *filename) {
  Parser p;
  if (!p.open(filename)) {
    std::cout << "Unable to open file " << std::endl;
    return false;
  }

  kdtree = std::make_shared<KDTree>(p.getPoints(), p.outerBox);
  pointList = kdtree->getPoints();

  // causes linker error - no idear why
  surfaces = std::make_shared<Surfaces>(kdtree, gridM, gridN, gridR);

  selectedPointIndex = 0;
  selectedPointList.clear();

  updateGL();

  return true;
}

void QGLViewerWidget::animateLight() {
  unsigned frameCounter = 300;
  while(true) {
    if (flag_animate) {
      lightPos[0] = sin(double(frameCounter) * M_PI / 100) + 0.5f;
      lightPos[1] = cos((double(frameCounter) * M_PI / 100) - M_PI) + 0.5f;
      frameCounter = (frameCounter + 1) % 200;
      update();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool QGLViewerWidget::drawPointSet() {
  if (pointList == nullptr || pointList->size() == 0) {
    return false;
  }

  glDisable(GL_LIGHTING);

  glEnable(GL_POINT_SMOOTH);
  glPointSize(3.0f);
  glBegin(GL_POINTS);
  glColor3f(1.0, 1.0, 1.0);
  for (unsigned int i = 0; i < pointList->size(); i++) {
    Point p = (*pointList)[i];
    glVertex3f(p.x, p.y, p.z);
  }
  glEnd();

  return true;
}

bool QGLViewerWidget::drawSelectedPointSet() {
  if (selectedPointList.size() == 0) {
    if (selectedPointIndex >= 0 && drawMode != 0) {
      // At least draw the selected point, otherwise
      // the selected point won't show in linear search mode
      glEnable(GL_POINT_SMOOTH);
      glPointSize(12.0f);
      glBegin(GL_POINTS);
      glColor3f(255, 0, 255);
      auto p = (*pointList)[selectedPointIndex];
      glVertex3f(p.x, p.y, p.z);
      glEnd();
    }
    return false;
  }

  glDisable(GL_LIGHTING);

  glEnable(GL_POINT_SMOOTH);
  glPointSize(10.0f);
  glBegin(GL_POINTS);
  glColor3f(0, 255, 0);
  auto selectedPoint = (*pointList)[selectedPointIndex];
  for (unsigned int i = 0; i < selectedPointList.size(); i++) {
    std::shared_ptr<Point> p = selectedPointList[i];
    if ((selectedPoint.x != p->x) || (selectedPoint.y != p->y) ||
        (selectedPoint.z != p->z)) {
      glVertex3f(p->x, p->y, p->z);
    }
  }
  glEnd();

  glEnable(GL_POINT_SMOOTH);
  glPointSize(12.0f);
  glBegin(GL_POINTS);
  glColor3f(255, 0, 255);
  // The selected point should always be at the end of the list
  auto p = (*pointList)[selectedPointIndex];
  glVertex3f(p.x, p.y, p.z);
  glEnd();

  return true;
}

//----------------------------------------------------------------------------

void QGLViewerWidget::setDefaultMaterial(void) {
  GLfloat mat_a[] = {0.1, 0.1, 0.1, 1.0};
  GLfloat mat_d[] = {0.7, 0.7, 0.5, 1.0};
  GLfloat mat_s[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat shine[] = {20.0};

  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_a);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_d);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_s);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shine);
}

//----------------------------------------------------------------------------

void QGLViewerWidget::setDefaultLight(void) {
  GLfloat pos1[] = {0.1, 0.1, -0.02, 0.0};
  GLfloat pos2[] = {-0.1, 0.1, -0.02, 0.0};
  GLfloat pos3[] = {0.0, 0.0, 0.1, 0.0};
  GLfloat col1[] = {0.7, 0.7, 0.8, 1.0};
  GLfloat col2[] = {0.8, 0.7, 0.7, 1.0};
  GLfloat col3[] = {1.0, 1.0, 1.0, 1.0};

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_POSITION, pos1);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, col1);
  glLightfv(GL_LIGHT0, GL_SPECULAR, col1);

  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_POSITION, pos2);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, col2);
  glLightfv(GL_LIGHT1, GL_SPECULAR, col2);

  glEnable(GL_LIGHT2);
  glLightfv(GL_LIGHT2, GL_POSITION, pos3);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, col3);
  glLightfv(GL_LIGHT2, GL_SPECULAR, col3);
}

//----------------------------------------------------------------------------
void QGLViewerWidget::initializeGL() {
  // OpenGL state
  glClearColor(0.0, 0.0, 0.0, 0.0);
  assert(glGetError() == GL_NO_ERROR);
  glDisable(GL_DITHER);
  assert(glGetError() == GL_NO_ERROR);
  glEnable(GL_DEPTH_TEST);
  assert(glGetError() == GL_NO_ERROR);

  // Material
  setDefaultMaterial();
  assert(glGetError() == GL_NO_ERROR);

  // Lighting
  glLoadIdentity();
  setDefaultLight();

  // Fog
  GLfloat fogColor[4] = {0.3, 0.3, 0.4, 1.0};
  glFogi(GL_FOG_MODE, GL_LINEAR);
  glFogfv(GL_FOG_COLOR, fogColor);
  glFogf(GL_FOG_DENSITY, 0.35);
  glHint(GL_FOG_HINT, GL_DONT_CARE);
  glFogf(GL_FOG_START, 5.0f);
  glFogf(GL_FOG_END, 25.0f);

  // scene pos and size
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);
  setScenePos(vec3(0.0, 0.0, 0.0), 1.0);
}

//----------------------------------------------------------------------------

void QGLViewerWidget::resizeGL(int w, int h) {
  updateProjectionMatrix();
  glViewport(0, 0, w, h);
  updateGL();
}

//----------------------------------------------------------------------------

void QGLViewerWidget::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixd(projectionMatrix);
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixd(modelviewMatrix);

  drawScene();
}

//----------------------------------------------------------------------------

void QGLViewerWidget::drawRegularGrid() {
  if (kdtree == nullptr) {
    return;
  }

  Borders borders = kdtree->getRootnode()->borders;
  float xMin = borders.xMin;
  float xMax = borders.xMax;
  float yMin = borders.yMin;
  float yMax = borders.yMax;
  float mDelta = double(yMax - yMin) / gridM;
  float nDelta = double(xMax - xMin) / gridN;

  // draw grid lines
  glBegin(GL_LINES);
  glColor3f(0, 1, 0);
  for (int m = 0; m <= gridM; ++m) {
    glVertex3f(xMin, yMin + (m * mDelta), 0);
    glVertex3f(xMax, yMin + (m * mDelta), 0);
  }
  for (int n = 0; n <= gridN; ++n) {
    glVertex3f(xMin + (n * nDelta), yMin, 0);
    glVertex3f(xMin + (n * nDelta), yMax, 0);
  }
  glEnd();
}

void drawBox(struct Borders borders) {
  double x_0 = borders.xMin;
  double x_1 = borders.xMax;
  double y_0 = borders.yMin;
  double y_1 = borders.yMax;
  double z_0 = borders.zMin;
  double z_1 = borders.zMax;

  glBegin(GL_LINE_LOOP);
  glVertex3f(x_0, y_0, z_0);
  glVertex3f(x_0, y_0, z_1);
  glVertex3f(x_0, y_1, z_1);
  glVertex3f(x_0, y_1, z_0);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(x_1, y_0, z_0);
  glVertex3f(x_1, y_0, z_1);
  glVertex3f(x_1, y_1, z_1);
  glVertex3f(x_1, y_1, z_0);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(x_0, y_0, z_0);
  glVertex3f(x_1, y_0, z_0);

  glVertex3f(x_0, y_0, z_1);
  glVertex3f(x_1, y_0, z_1);

  glVertex3f(x_0, y_1, z_1);
  glVertex3f(x_1, y_1, z_1);

  glVertex3f(x_0, y_1, z_0);
  glVertex3f(x_1, y_1, z_0);
  glEnd();
}

void recursiveDrawKDTree(std::shared_ptr<Node> node, unsigned remainingLevels) {
  if (remainingLevels <= 0) {
    return;
  }

  // recursively traverse the tree
  if (node->nlist.size() > 0) {
    double x_0 = node->borders.xMin;
    double x_1 = node->borders.xMax;
    double y_0 = node->borders.yMin;
    double y_1 = node->borders.yMax;
    double z_0 = node->borders.zMin;
    double z_1 = node->borders.zMax;
    unsigned axis = node->split.axis;
    double splitVal = node->split.value;

    if (axis == 0) {  // split along x
      double x_new = splitVal;
      glColor3f(1.0f, 0, 0);
      glVertex3f(x_new, y_0, z_0);
      glVertex3f(x_new, y_0, z_1);
      glColor3f(0.25f, 0, 0);
      glVertex3f(x_new, y_1, z_1);
      glVertex3f(x_new, y_1, z_0);
    } else if (axis == 1) {  // split along y
      double y_new = splitVal;
      glColor3f(0, 1.0f, 0);
      glVertex3f(x_0, y_new, z_0);
      glVertex3f(x_0, y_new, z_1);
      glColor3f(0, 0.25f, 0);
      glVertex3f(x_1, y_new, z_1);
      glVertex3f(x_1, y_new, z_0);
    } else if (axis == 2) {  // split along z
      double z_new = splitVal;
      glColor3f(0, 0, 1.0f);
      glVertex3f(x_0, y_0, z_new);
      glVertex3f(x_0, y_1, z_new);
      glColor3f(0, 0, 0.25f);
      glVertex3f(x_1, y_1, z_new);
      glVertex3f(x_1, y_0, z_new);
    } else {
      std::cout << "recursiveDrawKDTree() error" << std::endl;
    }

    if (node->nlist[0]->nlist.size() > 0)  // avoid leaf nodes
      recursiveDrawKDTree(node->nlist[0], remainingLevels - 1);
    if (node->nlist[1]->nlist.size() > 0)  // avoid leaf nodes
      recursiveDrawKDTree(node->nlist[1], remainingLevels - 1);
  }
}

void QGLViewerWidget::drawKDTree() {
  if (kdtree == nullptr) return;

  drawBox(kdtree->getRootnode()->borders);

  glBegin(GL_QUADS);
  recursiveDrawKDTree(kdtree->getRootnode(), drawLevelsOfTree);
  glEnd();
}

glm::vec3 QGLViewerWidget::triangleNormal(const Point &v1, const Point &v2, const Point &v3) {
  // Get the cross product of u - v
  glm::vec3 u(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
  glm::vec3 v(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z);
  auto normalX = (u.y * v.z) - (u.z * v.y);
  auto normalY = (u.z * v.x) - (u.x * v.z);
  auto normalZ = (u.x * v.y) - (u.y * v.x);

  // Normalize the cross product
  float d =
      sqrt(normalX * normalX + normalY * normalY + normalZ * normalZ);

  return glm::vec3(normalX / d, normalY / d, normalZ / d);
}

glm::vec3 QGLViewerWidget::gourad(const Point &v1, const glm::vec3 &normal) {
  glm::vec3 vertPos(v1.x, v1.y, v1.z);
  glm::vec3 ambientColor(0.1f, 0.0f, 0.0f);
  glm::vec3 diffuseColor(1.0f, 0.0f, 0.0f);
  glm::vec3 specularColor(1.0f, 1.0f, 1.0f);

  // camara positions
  glm::vec3 camPos = computeCamPos();

  glm::vec3 vertToCam = glm::normalize(camPos - vertPos);
  glm::vec3 vertToLight = lightPos - vertPos;
  float len = sqrt(vertToLight[0] * vertToLight[0] + vertToLight[1] * vertToLight[1] + vertToLight[2] * vertToLight[2]);
  float attenuation = 1.0f / (1.0f + 0.1f * len + 0.01f * len * len);

  glm::vec3 N(glm::normalize(normal));
  glm::vec3 L(glm::normalize(vertToLight));
  glm::vec3 E(glm::normalize(vertToCam));
  glm::vec3 R(0.0f, 0.0f, 0.0f);
  float NL = glm::dot(N, L);
  if (NL >= 0.0f) {
    R = glm::normalize((2.0f * (N * NL)) - L);
  }

  glm::vec3 diffuse = attenuation * diffuseColor * glm::max(glm::dot(normal, L), 0.0f);

  glm::vec3 specular = attenuation * specularColor * glm::pow(glm::max(glm::dot(R, E), 0.0f), 50.0f);

  return ambientColor + diffuse + specular;
}

bool intersectRayTriangle(glm::vec3 rayPos, glm::vec3 rayDir, glm::vec3 p0, glm::vec3 p1, glm::vec3 p2) {
  glm::vec3 v01(p1 - p0);
  glm::vec3 v02(p2 - p0);
  glm::vec3 h(glm::cross(rayDir, v02));
  double a = glm::dot(h, v01);
  if (a > -0.00001 && a < 0.00001) {
    return false;
  }

  double a_inv = 1/a;
  glm::vec3 s(rayPos - p0);
  double u = a_inv * glm::dot(h, s);
  if (u <= 0.0 || u >= 1.0) {
    return false;
  }

  glm::vec3 q(glm::cross(s, v01));
  double v = a_inv * glm::dot(q, rayDir);
  if (v <= 0.0 || u+v >= 1.0) {
    return false;
  }

  double t = a_inv * glm::dot(q, v02);
  if (t > 0.00001) {
    return true;
  } else {
    return false;;
  }
}


glm::vec3 QGLViewerWidget::gourad(const Point &v1,
                                  const glm::vec3 &normal,
                                  std::vector<quadPrimitiv> surface) {
  // shadow test
  glm::vec3 rayPos(v1.x, v1.y, v1.z);
  glm::vec3 rayDir(glm::normalize(lightPos - rayPos));
  for (int i = 0; i < surface.size(); ++i) {
    struct trianglePrimitiv t0 = surface[i].t0;
    glm::vec3 p00 = t0.p0->toVec3();
    glm::vec3 p01 = t0.p1->toVec3();
    glm::vec3 p02 = t0.p2->toVec3();
    struct trianglePrimitiv t1 = surface[i].t1;
    glm::vec3 p10 = t1.p0->toVec3();
    glm::vec3 p11 = t1.p1->toVec3();
    glm::vec3 p12 = t1.p2->toVec3();
    if (intersectRayTriangle(rayPos, rayDir, p00, p01, p02) ||
        intersectRayTriangle(rayPos, rayDir, p10, p11, p12)) {
      return glm::vec3(0.1f, 0.0f, 0.0f);
    }
  }

  return gourad(v1, normal);
}

void QGLViewerWidget::drawSurface(std::vector<quadPrimitiv> surfaceFaces) {
  glBegin(GL_TRIANGLES);
  for (uint i = 0; i < surfaceFaces.size(); i++) {
    struct trianglePrimitiv t0 = surfaceFaces[i].t0;
    struct trianglePrimitiv t1 = surfaceFaces[i].t1;

    assert(t0.p0 == t1.p1);
    assert(t0.p1 == t1.p0);

    auto p0 = t1.p1;
    auto p1 = t1.p2;
    auto p2 = t0.p2;
    auto p3 = t1.p0;

    glm::vec3 normal_t0_p0 = (glm::length(p0->normal) == 0)? t0.norm : p0->normal;
    glm::vec3 normal_t0_p2 = (glm::length(p2->normal) == 0)? t0.norm : p2->normal;
    glm::vec3 normal_t0_p3 = (glm::length(p3->normal) == 0)? t0.norm : p3->normal;

    auto col4 = gourad(*p0, normal_t0_p0);
    glColor3f(col4[0], col4[1], col4[2]);
    glVertex3f(p0->x, p0->y, p0->z);

    auto col5 = gourad(*p3, normal_t0_p3);
    glColor3f(col5[0], col5[1], col5[2]);
    glVertex3f(p3->x, p3->y, p3->z);

    auto col6 = gourad(*p2, normal_t0_p2);
    glColor3f(col6[0], col6[1], col6[2]);
    glVertex3f(p2->x, p2->y, p2->z);

    glm::vec3 normal_t1_p0 = (glm::length(p0->normal) == 0)? t1.norm : p0->normal;
    glm::vec3 normal_t1_p1 = (glm::length(p1->normal) == 0)? t1.norm : p1->normal;
    glm::vec3 normal_t1_p3 = (glm::length(p3->normal) == 0)? t1.norm : p3->normal;

    // Second triangle
    auto col1 = gourad(*p0, normal_t1_p0);
    glColor3f(col1[0], col1[1], col1[2]);
    glVertex3f(p0->x, p0->y, p0->z);

    auto col2 = gourad(*p1, normal_t1_p1);
    glColor3f(col2[0], col2[1], col2[2]);
    glVertex3f(p1->x, p1->y, p1->z);

    auto col3 = gourad(*p3, normal_t1_p3);
    glColor3f(col3[0], col3[1], col3[2]);
    glVertex3f(p3->x, p3->y, p3->z);
  }
  glEnd();

  // Draw light source for debugging
  glBegin(GL_POINTS);
  glEnable(GL_POINT_SMOOTH);
  glPointSize(15.0f);
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex3f(lightPos[0], lightPos[1], lightPos[2]);
  glEnd();
}


void QGLViewerWidget::drawControlMesh() {
  if (surfaces == nullptr) {
    return;
  }

  // Grab our vertices to shade
  auto surfaceFaces = surfaces->getControlFaces();

  drawSurface(surfaceFaces);
}

void QGLViewerWidget::drawSurfaceBTPS() {
  if (surfaces == nullptr) {
    return;
  }

  // Grab our vertices to shade
  auto surfaceFaces = surfaces->getSurfaceFacesBTPS();

  drawSurface(surfaceFaces);
}

void QGLViewerWidget::drawSurfaceMLS() {
  if (surfaces == nullptr) {
    return;
  }

  // Grab our vertices to shade
  auto surfaceFaces = surfaces->getSurfaceFacesMLS();

  drawSurface(surfaceFaces);
}

void QGLViewerWidget::drawScene() {
  glDisable(GL_LIGHTING);

  if (flag_drawControlMesh) {
    drawControlMesh();
  }
  if (drawPoints) {
    drawPointSet();
  }
  if (drawGrid) {
    drawRegularGrid();
  }
  if (flag_drawSurfaceBTPS) {
    drawSurfaceBTPS();
  }
  if (flag_drawSurfaceMLS) {
    drawSurfaceMLS();
  }
  if (flag_drawTree) {
    drawKDTree();
  }

  // Draw a coordinate system
  if (!drawGrid && kdtree == nullptr) {
    glBegin(GL_LINES);
    // x-axis
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0.25, 0, 0);
    // y-axis
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0.25, 0);
    // z-axis
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 0.25);
    glEnd();
  }
}

//----------------------------------------------------------------------------
void QGLViewerWidget::mousePressEvent(QMouseEvent *event) {
  // popup menu
  if (event->button() == RightButton && event->buttons() == RightButton) {
    selectOnRelease = false;
  } else {
    lastPointOk = mapToSphere(lastPoint2D = event->pos(), lastPoint3D);
    selectOnRelease = true;
  }
}

//----------------------------------------------------------------------------
void QGLViewerWidget::mouseMoveEvent(QMouseEvent *event) {
  selectOnRelease = false;
  QPoint newPoint2D = event->pos();

  // Left button: rotate around center
  // Middle button: translate object
  // Left & middle button: zoom in/out

  vec3 newPoint3D;
  bool newPoint_hitSphere = mapToSphere(newPoint2D, newPoint3D);

  float dx = newPoint2D.x() - lastPoint2D.x();
  float dy = newPoint2D.y() - lastPoint2D.y();

  float w = width();
  float h = height();

  // enable GL context
  makeCurrent();

  // move in z direction
  if ((event->buttons() == (LeftButton + MidButton)) ||
      (event->buttons() == LeftButton &&
       event->modifiers() == ControlModifier)) {
    float value_y = radius * dy * 3.0 / h;
    translate(vec3(0.0, 0.0, value_y));
  } else if ((event->buttons() == MidButton) ||
             (event->buttons() == LeftButton &&
              event->modifiers() == AltModifier)) {
    // move in x,y direction
    float z =
        -(modelviewMatrix[2] * center[0] + modelviewMatrix[6] * center[1] +
          modelviewMatrix[10] * center[2] + modelviewMatrix[14]) /
        (modelviewMatrix[3] * center[0] + modelviewMatrix[7] * center[1] +
         modelviewMatrix[11] * center[2] + modelviewMatrix[15]);

    float aspect = w / h;
    float near_plane = 0.01 * radius;
    float top = tan(fovy() / 2.0f * M_PI / 180.0f) * near_plane;
    float right = aspect * top;

    translate(vec3(2.0 * dx / w * right / near_plane * z,
                   -2.0 * dy / h * top / near_plane * z, 0.0f));
  } else if (event->buttons() == LeftButton) {
    // rotate
    if (lastPointOk) {
      if ((newPoint_hitSphere = mapToSphere(newPoint2D, newPoint3D))) {
        vec3 axis = cross(lastPoint3D, newPoint3D);

        if (dot(axis, axis) < 1e-7) {
          axis = vec3(1, 0, 0);
        } else {
          axis = normalize(axis);
        }
        // find the amount of rotation
        vec3 d = lastPoint3D - newPoint3D;
        float t = 0.5 * glm::length(d) / TRACKBALL_RADIUS;
        if (t < -1.0)
          t = -1.0;
        else if (t > 1.0)
          t = 1.0;
        float phi = 2.0 * asin(t);
        float angle = phi * 180.0 / M_PI;
        rotate(axis, angle);
      }
    }
  }

  // remember this point
  lastPoint2D = newPoint2D;
  lastPoint3D = newPoint3D;
  lastPointOk = newPoint_hitSphere;

  // trigger redraw
  updateGL();
}

//----------------------------------------------------------------------------

bool intersectRayPoint(glm::vec3 rayPos, glm::vec3 rayDir, glm::vec3 pointPos,
                       float *dist) {
  glm::vec3 vecRS = pointPos - rayPos;
  float lenRS = glm::length(vecRS);

  float r = 0.005 * lenRS;  // relativ to distance between camera and point

  float t_ca = glm::dot(vecRS, rayDir);

  float t_ca_Sqrt = std::pow(t_ca, 2.0f);
  float lenRS_Sqrt = std::pow(lenRS, 2.0f);
  float r_Sqrt = std::pow(r, 2.0f);

  float t_hc_Sqrt = r_Sqrt - lenRS_Sqrt + t_ca_Sqrt;
  if (t_hc_Sqrt < 0.0f) {  // no intersection
    return false;
  }

  float t1 = t_ca + std::sqrt(t_hc_Sqrt);
  float t2 = t_ca - std::sqrt(t_hc_Sqrt);

  *dist = (t1 < t2) ? t1 : t2;

  return true;
}

glm::mat4 QGLViewerWidget::computeModelViewInv() {
  glm::mat4 modelView_inv;
  modelView_inv[0][0] = modelviewMatrix[0];
  modelView_inv[0][1] = modelviewMatrix[1];
  modelView_inv[0][2] = modelviewMatrix[2];
  modelView_inv[0][3] = modelviewMatrix[3];
  modelView_inv[1][0] = modelviewMatrix[4];
  modelView_inv[1][1] = modelviewMatrix[5];
  modelView_inv[1][2] = modelviewMatrix[6];
  modelView_inv[1][3] = modelviewMatrix[7];
  modelView_inv[2][0] = modelviewMatrix[8];
  modelView_inv[2][1] = modelviewMatrix[9];
  modelView_inv[2][2] = modelviewMatrix[10];
  modelView_inv[2][3] = modelviewMatrix[11];
  modelView_inv[3][0] = modelviewMatrix[12];
  modelView_inv[3][1] = modelviewMatrix[13];
  modelView_inv[3][2] = modelviewMatrix[14];
  modelView_inv[3][3] = modelviewMatrix[15];
  modelView_inv = glm::inverse(modelView_inv);

  return modelView_inv;
}

glm::vec3 QGLViewerWidget::computeCamPos() {
  glm::mat4 modelView_inv = computeModelViewInv();

  glm::vec4 tmp = (modelView_inv * vec4(0, 0, 0, 1));
  glm::vec3 camPos;  // camera position
  camPos[0] = tmp[0] / tmp[3];
  camPos[1] = tmp[1] / tmp[3];
  camPos[2] = tmp[2] / tmp[3];

  return camPos;
}

int QGLViewerWidget::selectByMouse(std::shared_ptr<PointList> points,
                                   GLdouble mouseX, GLdouble mouseY) {
  if (points == nullptr) {
    return -1;
  }

  glm::mat4 modelView_inv = computeModelViewInv();

  // collect information
  glm::vec4 tmp = (modelView_inv * vec4(0, 0, 0, 1));
  glm::vec3 camPos;  // camera position
  camPos[0] = tmp[0] / tmp[3];
  camPos[1] = tmp[1] / tmp[3];
  camPos[2] = tmp[2] / tmp[3];
  glm::vec3 camUp;  // camera up direction
  camUp[0] = modelviewMatrix[1];
  camUp[1] = modelviewMatrix[5];
  camUp[2] = modelviewMatrix[9];
  float zNear = zNearFactor * radius;

  // viewing direction
  glm::vec3 camView;
  camView[0] = -modelviewMatrix[2];
  camView[1] = -modelviewMatrix[6];
  camView[2] = -modelviewMatrix[10];
  camView = glm::normalize(camView);

  // calc vector span for view plane
  glm::vec3 h = glm::cross(camView, camUp);
  h = glm::normalize(h);
  glm::vec3 v = glm::cross(h, camView);
  v = glm::normalize(v);
  double rad = fovy() * M_PI / 180.0f;
  float vLength = std::tan(rad / 2.0f) * zNear;
  float hLength = vLength * ((double)width() / height());
  v = v * vLength;
  h = h * hLength;

  // ray position and direction
  mouseX -= width() / 2.0f;
  mouseY = -1.0f * (mouseY - height() / 2.0f);
  float viewX = mouseX / (width() / 2.0f);
  float viewY = mouseY / (height() / 2.0f);
  glm::vec3 rayPos = camPos + (camView * zNear) + (h * viewX) + (v * viewY);
  glm::vec3 rayDir = glm::normalize(rayPos - camPos);

  // ray picking starts hear
  int selected_index = -1;
  float dist_to_selected = zFarFactor * radius;  // initiate at zFar
  for (unsigned int i = 0; i < pointList->size(); ++i) {
    Point p = (*points)[i];

    // cast ray and check for intersection w/ p
    glm::vec3 pointPos = vec3(p.x, p.y, p.z);
    float curr_dist = 0.0f;
    if (intersectRayPoint(rayPos, rayDir, pointPos, &curr_dist)) {
      if (curr_dist < dist_to_selected) {
        dist_to_selected = curr_dist;
        selected_index = i;
      }
    }
  }

  return selected_index;
}

void QGLViewerWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (selectOnRelease == true) {
    if (pointList == nullptr || pointList->size() == 0) {
      std::cout << "No OFF file loaded. Won't draw. Please load a file before "
                   "changing values"
                << std::endl;
      return;
    }

    auto tempIndex =
        selectByMouse(pointList, event->pos().x(), event->pos().y());

    if (tempIndex >= 0) {
      selectedPointIndex = tempIndex;
      updateTreeState(currentSliderValue);
    }
  }

  // finish up
  lastPointOk = false;
  selectOnRelease = false;
}

//-----------------------------------------------------------------------------

void QGLViewerWidget::wheelEvent(QWheelEvent *_event) {
  // Use the mouse wheel to zoom in/out

  float d = -(float)_event->delta() / 120.0 * 0.2 * radius;
  translate(vec3(0.0, 0.0, d));
  updateGL();
  _event->accept();
}

//----------------------------------------------------------------------------

void QGLViewerWidget::keyPressEvent(QKeyEvent *_event) {
  switch (_event->key()) {
    case Key_Print:
      slotSnapshot();
      break;

    case Key_A:
      flag_animate = (flag_animate) ? false : true;
      break;

    case Key_J:
      if (drawLevelsOfTree > 0) drawLevelsOfTree--;
      break;

    case Key_K:
      if (drawLevelsOfTree < 8) drawLevelsOfTree++;
      break;

    case Key_T:
      flag_drawTree = flag_drawTree ? false : true;
      break;

    case Key_H:
      std::cout << "Keys:\n";
      std::cout << "  Print\tMake snapshot\n";
      break;

    case Key_Q:
    case Key_Escape:
      qApp->quit();
  }
  _event->ignore();

  updateGL();
}

//----------------------------------------------------------------------------

void QGLViewerWidget::translate(const vec3 &_trans) {
  // Translate the object by _trans
  // Update modelviewMatrix
  makeCurrent();
  glLoadIdentity();
  glTranslated(_trans[0], _trans[1], _trans[2]);
  glMultMatrixd(modelviewMatrix);
  glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);
}

//----------------------------------------------------------------------------

void QGLViewerWidget::rotate(const vec3 &_axis, float _angle) {
  // Rotate around center center, axis _axis, by angle _angle
  // Update modelviewMatrix

  vec3 t(modelviewMatrix[0] * center[0] + modelviewMatrix[4] * center[1] +
             modelviewMatrix[8] * center[2] + modelviewMatrix[12],
         modelviewMatrix[1] * center[0] + modelviewMatrix[5] * center[1] +
             modelviewMatrix[9] * center[2] + modelviewMatrix[13],
         modelviewMatrix[2] * center[0] + modelviewMatrix[6] * center[1] +
             modelviewMatrix[10] * center[2] + modelviewMatrix[14]);

  makeCurrent();
  glLoadIdentity();
  glTranslatef(t[0], t[1], t[2]);
  glRotated(_angle, _axis[0], _axis[1], _axis[2]);
  glTranslatef(-t[0], -t[1], -t[2]);
  glMultMatrixd(modelviewMatrix);
  glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);
}

//----------------------------------------------------------------------------

bool QGLViewerWidget::mapToSphere(const QPoint &_v2D, vec3 &_v3D) {
  // This is actually doing the Sphere/Hyperbolic sheet hybrid thing,
  // based on Ken Shoemake's ArcBall in Graphics Gems IV, 1993.
  double x = (2.0 * _v2D.x() - width()) / width();
  double y = -(2.0 * _v2D.y() - height()) / height();
  double xval = x;
  double yval = y;
  double x2y2 = xval * xval + yval * yval;

  const double rsqr = TRACKBALL_RADIUS * TRACKBALL_RADIUS;
  _v3D[0] = xval;
  _v3D[1] = yval;
  if (x2y2 < 0.5 * rsqr) {
    _v3D[2] = std::sqrt(rsqr - x2y2);
  } else {
    _v3D[2] = 0.5 * rsqr / std::sqrt(x2y2);
  }

  return true;
}

//----------------------------------------------------------------------------

void QGLViewerWidget::updateProjectionMatrix() {
  makeCurrent();
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fovy(), (GLfloat)width() / (GLfloat)height(),
                 zNearFactor * radius, zFarFactor * radius);
  glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
  glMatrixMode(GL_MODELVIEW);
}

//----------------------------------------------------------------------------

void QGLViewerWidget::viewAll() {
  translate(vec3(
      -(modelviewMatrix[0] * center[0] + modelviewMatrix[4] * center[1] +
        modelviewMatrix[8] * center[2] + modelviewMatrix[12]),
      -(modelviewMatrix[1] * center[0] + modelviewMatrix[5] * center[1] +
        modelviewMatrix[9] * center[2] + modelviewMatrix[13]),
      -(modelviewMatrix[2] * center[0] + modelviewMatrix[6] * center[1] +
        modelviewMatrix[10] * center[2] + modelviewMatrix[14] + 3.0 * radius)));
}

//----------------------------------------------------------------------------

void QGLViewerWidget::setScenePos(const vec3 &_cog, float _radius) {
  center = _cog;
  radius = _radius;

  updateProjectionMatrix();
  viewAll();
}

//----------------------------------------------------------------------------

void QGLViewerWidget::addAction(QAction *act, const char *name) {
  namesToActions[name] = act;
  Super::addAction(act);
}

void QGLViewerWidget::removeAction(QAction *act) {
  ActionMap::iterator it = namesToActions.begin(), e = namesToActions.end();
  ActionMap::iterator found = e;
  for (; it != e; ++it) {
    if (it->second == act) {
      found = it;
      break;
    }
  }
  if (found != e) {
    namesToActions.erase(found);
  }
  popupMenu->removeAction(act);
  drawModesGroup->removeAction(act);
  Super::removeAction(act);
}

void QGLViewerWidget::removeAction(const char *name) {
  QString namestr = QString(name);
  ActionMap::iterator e = namesToActions.end();

  ActionMap::iterator found = namesToActions.find(namestr);
  if (found != e) {
    removeAction(found->second);
  }
}

QAction *QGLViewerWidget::findAction(const char *name) {
  QString namestr = QString(name);
  ActionMap::iterator e = namesToActions.end();

  ActionMap::iterator found = namesToActions.find(namestr);
  if (found != e) {
    return found->second;
  }
  return 0;
}

//----------------------------------------------------------------------------

//----------------------------------------------------------------------------

void QGLViewerWidget::setDrawMode(int value) { drawMode = value; }

void QGLViewerWidget::setPerformLinearSearch(bool value) {
  performLinearSearch = value;
}

void QGLViewerWidget::updateTreeState(int value) {
  if (pointList == nullptr || pointList->size() == 0) {
    std::cout << "No OFF file loaded. Won't draw. Please load a file before "
                 "changing values"
              << std::endl;
    return;
  }

  currentSliderValue = value;
  if (drawMode == 0) {
    drawLevelsOfTree = value;
  } else {
    std::cout << (performLinearSearch ? "[Linear]" : "[KDTree]");
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    if (drawMode == 1) {
      float v = static_cast<float>(currentSliderValue / 10.0f);
      std::cout << " Collect in radius=" << v << " ";
      if (performLinearSearch) {
        start = std::chrono::high_resolution_clock::now();
        selectedPointList =
            kdtree->collectInRadiusSimple((*pointList)[selectedPointIndex], v);
        end = std::chrono::high_resolution_clock::now();
      } else {
        start = std::chrono::high_resolution_clock::now();
        selectedPointList =
            kdtree->collectInRadius((*pointList)[selectedPointIndex], v);
        end = std::chrono::high_resolution_clock::now();
      }
    } else {
      std::cout << " KNearestNeighbors k=" << currentSliderValue << " ";
      if (performLinearSearch) {
        start = std::chrono::high_resolution_clock::now();
        selectedPointList = kdtree->collectKNearestSimple(
            (*pointList)[selectedPointIndex], currentSliderValue);
        end = std::chrono::high_resolution_clock::now();
      } else {
        start = std::chrono::high_resolution_clock::now();
        selectedPointList = kdtree->collectKNearest(
            (*pointList)[selectedPointIndex], currentSliderValue);
        end = std::chrono::high_resolution_clock::now();
      }
    }
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();
    std::cout << "-> execution duration: " << duration << " micro seconds"
              << std::endl;
  }

  // Need to redraw after changing settings
  updateGL();
}

void QGLViewerWidget::setDrawPoints(bool value) {
  /* std::cout << "Changing drawPoints value to " << value << std::endl; */
  drawPoints = value;
  paintGL();
  updateGL();
}

void QGLViewerWidget::setDrawRegularGrid(bool value) {
  /* std::cout << "Changing drawGrid value to " << value << std::endl; */
  drawGrid = value;
  paintGL();
  updateGL();
}

void QGLViewerWidget::setDrawControlMeshPoints(bool value) {
  /* std::cout << "Changing drawControlMesh value to " << value << std::endl; */
  flag_drawControlMesh = value;
  if (surfaces != nullptr && value != 0) {
    surfaces->updateControlFaces();
  }
  paintGL();
  updateGL();
}

void QGLViewerWidget::setGridXDim(int value) {
  /* std::cout << "Changing grid X dimension value to " << value << std::endl; */
  gridN = value;
  if (surfaces != nullptr) {
    this->surfaces->setGrid(gridM, gridN);

    // update surface if draw flag ist true
    if (flag_drawControlMesh) {
      surfaces->updateControlFaces();
    }
    if (flag_drawSurfaceBTPS) {
      surfaces->updateSurfacesFacesBTPS(kBTPS);
    }
    if (flag_drawSurfaceMLS) {
      surfaces->updateSurfacesFacesMLS(kMLS);
    }
  }
  paintGL();
  updateGL();
}

void QGLViewerWidget::setGridYDim(int value) {
  /* std::cout << "Changing grid Y dimension value to " << value << std::endl; */
  gridM = value;
  if (surfaces != nullptr) {
    this->surfaces->setGrid(gridM, gridN);

    // update surface if draw flag ist true
    if (flag_drawControlMesh) {
      surfaces->updateControlFaces();
    }
    if (flag_drawSurfaceBTPS) {
      surfaces->updateSurfacesFacesBTPS(kBTPS);
    }
    if (flag_drawSurfaceMLS) {
      surfaces->updateSurfacesFacesMLS(kMLS);
    }
  }
  paintGL();
  updateGL();
}

void QGLViewerWidget::setRadius(double r) {
  /* std::cout << "Changing radius to " << radius << std::endl; */
  this->gridR = r;
  if (surfaces != nullptr) {
    this->surfaces->setRadius(r);

    // update surface if draw flag ist true
    if (flag_drawControlMesh) {
      surfaces->updateControlFaces();
    }
    if (flag_drawSurfaceBTPS) {
      surfaces->updateSurfacesFacesBTPS(kBTPS);
    }
    if (flag_drawSurfaceMLS) {
      surfaces->updateSurfacesFacesMLS(kMLS);
    }
  }
  paintGL();
  updateGL();
}

void QGLViewerWidget::setDrawBezier(bool value) {
  /* std::cout << "Changing drawSurfaceBTPS value to " << value << std::endl; */
  flag_drawSurfaceBTPS = value;
  if (surfaces != nullptr && value != 0) {
    surfaces->updateSurfacesFacesBTPS(kBTPS);
  }
  paintGL();
  updateGL();
}

void QGLViewerWidget::setBezierSubdivisions(int k) {
  /* std::cout << "Changing kBTPS value to " << k << std::endl; */
  kBTPS = k;
  if (surfaces != nullptr && flag_drawSurfaceBTPS != 0) {
    surfaces->updateSurfacesFacesBTPS(kBTPS);
  }
  paintGL();
  updateGL();
}

void QGLViewerWidget::setDrawMls(bool value) {
  /* std::cout << "Changing drawSurfaceMLS value to " << value << std::endl; */
  flag_drawSurfaceMLS = value;
  if (surfaces != nullptr && value != 0) {
    surfaces->updateSurfacesFacesMLS(kMLS);
  }
  paintGL();
  updateGL();
}

void QGLViewerWidget::setMlsSubdivisions(int k) {
  /* std::cout << "Changing kMLS value to " << k << std::endl; */
  kMLS = k;
  if (surfaces != nullptr && flag_drawSurfaceMLS != 0) {
    surfaces->updateSurfacesFacesMLS(kMLS);
  }
  paintGL();
  updateGL();
}

void QGLViewerWidget::slotSnapshot(void) {
  // empty....
}

//=============================================================================
