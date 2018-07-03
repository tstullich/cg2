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
  // surfaces = std::make_shared<Surfaces>(kdtree, gridM, gridN, gridR);

  implicitSurface = std::make_shared<ImplicitSurface>(
      kdtree, this->gridSubdivision, this->implicitRadius);

  Borders b = this->kdtree->getRootnode()->borders;
  this->cloudSize = glm::vec3(b.xMax-b.xMin, b.yMax-b.yMin, b.zMax-b.zMin);
  float maxSize = std::max(std::max(cloudSize[0], cloudSize[1]), cloudSize[2]);
  defaultRadius = maxSize*0.5;
  setScenePos(kdtree->getCenter(), defaultRadius);
  /* setScenePos(kdtree->getCenterOfGravity(), 1.0); */
  /* setScenePos(glm::vec3(0.5f, 0.5f, 0.35f), 1.0f); */

  selectedPointIndex = 0;
  selectedPointList.clear();
  clearRayCasting();

  updateGL();

  return true;
}

void QGLViewerWidget::animateLight() {
  unsigned frameCounter = 25;
  while (true) {
    if (flag_animate) {
      lightPos[0] = cloudSize[0] * sin(double(frameCounter) * M_PI / 100) + center[0];
      lightPos[1] = cloudSize[1] * cos(double(frameCounter) * M_PI / 100) + center[1];
      lightPos[2] = center[2] + 0.75 * cloudSize[2];
      frameCounter = (frameCounter + 1) % 200;
      update();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool QGLViewerWidget::drawPointSetNormals() {
  if (pointList == nullptr || pointList->size() == 0) {
    return false;
  }

  Borders outerBox = this->kdtree->getRootnode()->borders;
  Point p1(outerBox.xMin, outerBox.yMin, outerBox.zMin);
  Point p2(outerBox.xMax, outerBox.yMax, outerBox.zMax);
  double lengthScalar = 0.03 * p1.distPoint(p2);

  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glColor3f(0.75f, 0.75f, 0.75f);
  for (unsigned int i = 0; i < pointList->size(); i++) {
    Point p = (*pointList)[i];
    if (p.type == originalPoint) {
      if (glm::length(p.normal) == 0) {
      }
      glVertex3f(p.x, p.y, p.z);
      glVertex3f(p.x + lengthScalar * p.normal[0],
                 p.y + lengthScalar * p.normal[1],
                 p.z + lengthScalar * p.normal[2]);
    }
  }
  glEnd();

  return true;
}

bool QGLViewerWidget::drawPointSet() {
  if (pointList == nullptr || pointList->size() == 0) {
    return false;
  }

  glDisable(GL_LIGHTING);

  glEnable(GL_POINT_SMOOTH);
  glPointSize(3.0f);
  glBegin(GL_POINTS);

  glColor3f(1.0f, 0.0f, 0.0f);
  for (std::shared_ptr<Point> p : this->implicitSurface->getOriginalPoints()) {
    glVertex3f(p->x, p->y, p->z);
  }

  glColor3f(0.0f, 1.0f, 0.0f);
  for (std::shared_ptr<Point> p : this->implicitSurface->getPositivePoints()) {
    glVertex3f(p->x, p->y, p->z);
  }

  glColor3f(0.0f, 0.0f, 1.0f);
  for (std::shared_ptr<Point> p : this->implicitSurface->getNegativePoints()) {
    glVertex3f(p->x, p->y, p->z);
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

void QGLViewerWidget::drawImplicitGridPoints() {
  if (this->implicitSurface == nullptr) {
    return;
  }

  std::vector<std::vector<std::vector<std::shared_ptr<Point>>>>
      implicitGridPoints = this->implicitSurface->getImplicitGridPoints();

  if (implicitGridPoints.size() == 0) {
    return;
  }

  glDisable(GL_LIGHTING);
  glEnable(GL_POINT_SMOOTH);
  glPointSize(5.0f);
  glBegin(GL_POINTS);

  for (unsigned int i = 0; i < implicitGridPoints.size(); i++) {
    for (unsigned int j = 0; j < implicitGridPoints[0].size(); j++) {
      for (unsigned int k = 0; k < implicitGridPoints[0][0].size(); k++) {
        std::shared_ptr<Point> p = implicitGridPoints[i][j][k];
        if (p->f == std::numeric_limits<float>::max()) {
          continue;
          glColor3f(0.1, 0.1, 0.1);
          glVertex3f(p->x, p->y, p->z);
        } else {
          if (p->f > 0.0 && drawPositiveSamples) {
            glColor3f(0.0, 0.0, 1.0);
            glVertex3f(p->x, p->y, p->z);
          } else if (p->f == 0.0) {
            glColor3f(1.0, 0.0, 0.0);
            glVertex3f(p->x, p->y, p->z);
          } else if (p->f < 0.0 && drawNegativeSamples) {
            glColor3f(1.0, 1.0, 0.0);
            glVertex3f(p->x, p->y, p->z);
          }
        }
      }
    }
  }

  glEnd();
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
  const unsigned char *version = glGetString(GL_VERSION);
  const unsigned char *vendor = glGetString(GL_VENDOR);
  const unsigned char *renderer = glGetString(GL_RENDERER);
  const unsigned char *glsl = glGetString(GL_SHADING_LANGUAGE_VERSION);
  std::cout << "using OpenGL " << version << " | "
                               << vendor << " | "
                               << renderer << " | "
                               << "GLSL " << glsl << std::endl;
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
  glColor3f(0.0, 1.0, 0.0);
  glVertex3f(x_0, y_0, z_0);
  glVertex3f(x_0, y_0, z_1);
  glVertex3f(x_0, y_1, z_1);
  glVertex3f(x_0, y_1, z_0);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glColor3f(0.0, 1.0, 0.0);
  glVertex3f(x_1, y_0, z_0);
  glVertex3f(x_1, y_0, z_1);
  glVertex3f(x_1, y_1, z_1);
  glVertex3f(x_1, y_1, z_0);
  glEnd();

  glBegin(GL_LINES);
  glColor3f(0.0, 1.0, 0.0);
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

glm::vec3 QGLViewerWidget::triangleNormal(const Point &v1, const Point &v2,
                                          const Point &v3) {
  // Get the cross product of u - v
  glm::vec3 u(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
  glm::vec3 v(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z);
  auto normalX = (u.y * v.z) - (u.z * v.y);
  auto normalY = (u.z * v.x) - (u.x * v.z);
  auto normalZ = (u.x * v.y) - (u.y * v.x);

  // Normalize the cross product
  float d = sqrt(normalX * normalX + normalY * normalY + normalZ * normalZ);

  return glm::vec3(normalX / d, normalY / d, normalZ / d);
}

glm::vec3 QGLViewerWidget::gourad(const Point &v1, const glm::vec3 &normal) {
  glm::vec3 vertPos(v1.x, v1.y, v1.z);
  glm::vec3 ambientColor(0.1f, 0.0f, 0.0f);
  glm::vec3 diffuseColor(1.0f, 0.0f, 0.0f);
  glm::vec3 specularColor(1.0f, 1.0f, 1.0f);

  // camera positions
  glm::vec3 camPos = computeCamPos();

  glm::vec3 vertToCam = glm::normalize(camPos - vertPos);
  glm::vec3 vertToLight = glm::normalize(lightPos - vertPos);

  glm::vec3 N(glm::normalize(normal));
  glm::vec3 L(glm::normalize(vertToLight));
  glm::vec3 E(glm::normalize(vertToCam));
  glm::vec3 R(0.0f, 0.0f, 0.0f);
  float NL = glm::dot(N, L);
  if (NL >= 0.0f) {
    R = glm::normalize((2.0f * (N * NL)) - L);
  }

  glm::vec3 diffuse = diffuseColor * glm::max(glm::dot(normal, L), 0.0f);

  glm::vec3 specular =
      specularColor * glm::pow(glm::max(glm::dot(R, E), 0.0f), 50.0f);

  return ambientColor + diffuse + specular;
}

bool intersectRayTriangle(glm::vec3 rayPos, glm::vec3 rayDir, glm::vec3 p0,
                          glm::vec3 p1, glm::vec3 p2) {
  glm::vec3 v01(p1 - p0);
  glm::vec3 v02(p2 - p0);
  glm::vec3 h(glm::cross(rayDir, v02));
  double a = glm::dot(h, v01);
  if (a > -0.00001 && a < 0.00001) {
    return false;
  }

  double a_inv = 1 / a;
  glm::vec3 s(rayPos - p0);
  double u = a_inv * glm::dot(h, s);
  if (u <= 0.0 || u >= 1.0) {
    return false;
  }

  glm::vec3 q(glm::cross(s, v01));
  double v = a_inv * glm::dot(q, rayDir);
  if (v <= 0.0 || u + v >= 1.0) {
    return false;
  }

  double t = a_inv * glm::dot(q, v02);
  if (t > 0.00001) {
    return true;
  } else {
    return false;
    ;
  }
}

bool QGLViewerWidget::rayMarching(glm::vec3 rayPos, glm::vec3 rayDir, float *dist) {
  if (this->implicitSurface == nullptr) {
    return false;
  }

  rayDir = glm::normalize(rayDir);
  assert(glm::length(rayDir) != 0.0);

  // evaluation at ray origin
  Point p(rayPos[0], rayPos[1], rayPos[2]);
  float lastVal = this->implicitSurface->computeMLS(p);
  float maxDist = *dist;
  float stepSize = 0.01*kdtree->getDiagonal();
  for (*dist = stepSize; (*dist)+stepSize < maxDist; (*dist)+=stepSize) {
    float x = rayPos[0] + (*dist)*rayDir[0];
    float y = rayPos[1] + (*dist)*rayDir[1];
    float z = rayPos[2] + (*dist)*rayDir[2];
    Point p(x, y, z);
    float currVal = this->implicitSurface->computeMLS(p);
    // check for sign change
    if ((lastVal > 0.0 && currVal <= 0.0) || (lastVal < 0.0 && currVal >= 0.0)) {
      return true;
    }
    lastVal = currVal;
  }

  return false;
}

bool QGLViewerWidget::sphereTracing(glm::vec3 rayPos, glm::vec3 rayDir, float *dist) {
  if (this->implicitSurface == nullptr) {
    return false;
  }

  float threshold = 0.001;

  rayDir = glm::normalize(rayDir);
  assert(glm::length(rayDir) != 0.0);

  // evaluation at ray origin
  Point p(rayPos[0], rayPos[1], rayPos[2]);
  float lastDist = std::numeric_limits<float>::max();
  float currDist = this->kdtree->distToSurface(p, rayPos, rayDir);
  float totalDist = 0.0;
  while(currDist < lastDist) {
    lastDist = currDist - (0.1 * threshold);
    // make sphere raius step
    p.x += lastDist*rayDir[0];
    p.y += lastDist*rayDir[1];
    p.z += lastDist*rayDir[2];
    totalDist += lastDist;
    // calculate new sphere radius
    currDist = this->kdtree->distToSurface(p, rayPos, rayDir);
    // if it converges, we hit the surface
    if (currDist < threshold) {
      *dist = totalDist;
      return true;
    }
  }

  return false;
}

void QGLViewerWidget::clearRayCasting() {
  intersections.clear();
  frustum.near0 = glm::vec3(0.0, 0.0, 0.0);
  frustum.near1 = glm::vec3(0.0, 0.0, 0.0);
  frustum.near2 = glm::vec3(0.0, 0.0, 0.0);
  frustum.near3 = glm::vec3(0.0, 0.0, 0.0);
  frustum.far0 = glm::vec3(0.0, 0.0, 0.0);
  frustum.far1 = glm::vec3(0.0, 0.0, 0.0);
  frustum.far2 = glm::vec3(0.0, 0.0, 0.0);
  frustum.far3 = glm::vec3(0.0, 0.0, 0.0);
  frustum.ray0 = glm::vec3(0.0, 0.0, 0.0);
  frustum.ray1 = glm::vec3(0.0, 0.0, 0.0);
}

void QGLViewerWidget::raycasting() {
  std::cout << "raycasting() start!" << std::endl;
  flag_shootRays = false;
  flag_animate = true;

  clearRayCasting();

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
  float zNear = zNearFactor * 1.0;
  /* float zFar = (rayMode == MARCHING)? 1.5 : zFarFactor * 1.0; */
  float zFar = 1.5*kdtree->getDiagonal();

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

  unsigned sphereDrawOff = 2;
  for (int m = 0; m <= height(); m+=1) {
    if (rayMode == SPHERE) {
      if (!(m+sphereDrawOff < height()/2.0 || m-sphereDrawOff > height()/2.0)) {
        std::cout << "line" << m << std::endl;
      }
    }
    for (int n = 0; n <= width(); n+=1) {
      // ray position and direction
      double N = n - (width() / 2.0f);
      double M = -1.0f * (m - height() / 2.0f);
      float viewN = N / (width() / 2.0f);
      float viewM = M / (height() / 2.0f);
      glm::vec3 rayPos = camPos + (camView * zNear) + (h * viewN) + (v * viewM);
      glm::vec3 rayDir = glm::normalize(rayPos - camPos);

      // fill in frustum information
      frustum.ray0 = rayPos;
      frustum.ray1 = rayPos + zFar * rayDir;
      if (m == 0 && n == 0) {
        frustum.near0 = rayPos;
        frustum.far0 = rayPos + zFar * rayDir;
      } else if (m == 0 && n == width()) {
        frustum.near1 = rayPos;
        frustum.far1 = rayPos + zFar * rayDir;
      } else if (n == 0) {
        frustum.near3 = rayPos;
        frustum.far3 = rayPos + zFar * rayDir;
      } else if (n == width()) {
        frustum.near2 = rayPos;
        frustum.far2 = rayPos + zFar * rayDir;
      }

      if (rayMode == SPHERE) {
        if (m+sphereDrawOff < height()/2.0 || m-sphereDrawOff > height()/2.0) {
          continue;
        }
      }

      // ray casting starts hear
      float dist = zFar;
      bool hit = false;
      switch(rayMode) {
        case MARCHING:
          hit = rayMarching(rayPos, rayDir, &dist);
          break;
        case SPHERE:
          hit = sphereTracing(rayPos, rayDir, &dist);
          break;
        default:
          return;
      }
      if (hit) {
        struct Intersection newIntersection;
        // test screen
        newIntersection.point = glm::vec3(rayPos[0]+ dist*rayDir[0],
                                          rayPos[1]+ dist*rayDir[1],
                                          rayPos[2]+ dist*rayDir[2]);
        newIntersection.color = glm::vec3(1.0, 1.0, 1.0);
        intersections.push_back(newIntersection);
      }
    }
  }

  std::cout << "raycasting() finish!" << std::endl;
  flag_animate = false;

  return;
}

void drawVec3(glm::vec3 v) {
  glVertex3f(v[0], v[1], v[2]);
}

void QGLViewerWidget::drawIntersections() {
  glDisable(GL_LIGHTING);
  glBegin(GL_LINE_LOOP);
  // draw frustum
  glColor3f(0.0, 1.0, 0.0);
  // near plane
  drawVec3(frustum.near0);
  drawVec3(frustum.near1);
  drawVec3(frustum.near2);
  drawVec3(frustum.near3);
  drawVec3(frustum.near0);
  // far plane
  drawVec3(frustum.far0);
  drawVec3(frustum.far1);
  drawVec3(frustum.far2);
  drawVec3(frustum.far3);
  drawVec3(frustum.far0);
  glEnd();
  glBegin(GL_LINES);
  glColor3f(0.0, 1.0, 0.0);
  // lines between near and far
  drawVec3(frustum.near0);
  drawVec3(frustum.far0);
  drawVec3(frustum.near1);
  drawVec3(frustum.far1);
  drawVec3(frustum.near2);
  drawVec3(frustum.far2);
  drawVec3(frustum.near3);
  drawVec3(frustum.far3);

  // draw ray
  drawVec3(frustum.ray0);
  drawVec3(frustum.ray1);
  glEnd();

  if (this->intersections.size() == 0) {
    return;
  }

  glDisable(GL_LIGHTING);
  glEnable(GL_POINT_SMOOTH);
  glPointSize(1.0f);
  glBegin(GL_POINTS);
  for (struct Intersection i : this->intersections) {
    glColor3f(i.color[0], i.color[1], i.color[2]);
    glVertex3f(i.point[0], i.point[1], i.point[2]);
  }
  glEnd();
}

void QGLViewerWidget::drawTriangleMesh(std::vector<Triangle> mesh) {
  // No mesh to draw. Just return
  if (mesh.empty()) {
    return;
  }

  glBegin(GL_TRIANGLES);
  glPointSize(10.0f);
  for (uint i = 0; i < mesh.size(); i++) {
    auto p0 = mesh[i][0];
    auto p1 = mesh[i][1];
    auto p2 = mesh[i][2];

    auto normal = triangleNormal(p0, p2, p1);

    auto col0 = gourad(p0, normal);
    glColor3f(col0[0], col0[1], col0[2]);
    glVertex3f(p0.x, p0.y, p0.z);

    auto col1 = gourad(p1, normal);
    glColor3f(col1[0], col1[1], col1[2]);
    glVertex3f(p1.x, p1.y, p1.z);

    auto col2 = gourad(p2, normal);
    glColor3f(col2[0], col2[1], col2[2]);
    glVertex3f(p2.x, p2.y, p2.z);
  }
  glEnd();

  float distToLight = glm::length(lightPos - computeCamPos());
  distToLight *= distToLight * 0.5f;
  glBegin(GL_POINTS);
  glPointSize(32.0f / distToLight);
  glEnable(GL_POINT_SMOOTH);
  glColor3f(1.0f, 1.0f, 0.0f);
  glVertex3f(lightPos[0], lightPos[1], lightPos[2]);
  glEnd();
}

void QGLViewerWidget::drawMarchingCubesMesh() {
  if (implicitSurface == nullptr) {
    return;
  }

  drawTriangleMesh(implicitSurface->getMarchingCubesMesh());
}

void QGLViewerWidget::drawScene() {
  glDisable(GL_LIGHTING);

  if (drawPoints) {
    drawPointSet();
  }
  if (flag_drawPointSetNormals) {
    drawPointSetNormals();
  }
  if (drawGrid) {
    drawRegularGrid();
  }
  if (drawPositiveSamples || drawNegativeSamples) {
    drawImplicitGridPoints();
  }
  if (flag_drawTree) {
    drawKDTree();
  }
  if (flag_drawMarchingCubes) {
    drawMarchingCubesMesh();
  }
  if (flag_shootRays) {
    if (rayMode == MARCHING || true) {
      threads.push_back(std::thread(&QGLViewerWidget::raycasting, this));
    } else {
      raycasting();
    }
  }
  if (flag_drawIntersections) {
    drawIntersections();
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

    case Key_C:
      setScenePos(center, defaultRadius);
      break;

    case Key_J:
      if (drawLevelsOfTree > 0) drawLevelsOfTree--;
      break;

    case Key_K:
      if (drawLevelsOfTree < 8) drawLevelsOfTree++;
      break;

    case Key_M:
      if (rayMode == MARCHING) {
        rayMode = SPHERE;
        std::cout << "ray casting mode: sphere tracing" << std::endl;
      } else {
        rayMode = MARCHING;
        std::cout << "ray casting mode: ray marching" << std::endl;
      }
      break;

    case Key_R:
      flag_drawIntersections = (flag_drawIntersections)? false : true;
      std::cout << "flag_drawIntersections = " << flag_drawIntersections << std::endl;
      break;

    case Key_S:
      flag_drawIntersections = true;
      flag_shootRays = (flag_shootRays)? false : true;
      std::cout << "flag_shootRays = " << flag_shootRays << std::endl;
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
  drawPoints = value;
  paintGL();
  updateGL();
}

void QGLViewerWidget::setDrawNormals(bool value) {
  std::cout << "Changing drawNormals value to " << value << std::endl;
  flag_drawPointSetNormals = value;
  paintGL();
  updateGL();
}

void QGLViewerWidget::flipNormals() {
  std::cout << "Calling flipNormals()" << std::endl;
}

void QGLViewerWidget::setDrawPositiveSamples(bool value) {
  std::cout << "Changing drawSamples value to " << value << std::endl;
  this->drawPositiveSamples = value;
  updateGL();
}

void QGLViewerWidget::setDrawNegativeSamples(bool value) {
  std::cout << "Changing drawSamples value to " << value << std::endl;
  this->drawNegativeSamples = value;
  updateGL();
}

void QGLViewerWidget::setDrawConstraints(bool value) {
  std::cout << "Changing drawConstraints value to " << value << std::endl;
}

void QGLViewerWidget::setGridSubdivision(int value) {
  std::cout << "Changing grid dimension value to " << value << std::endl;
  this->gridSubdivision = value;
  if (implicitSurface != nullptr) {
    this->implicitSurface->setGrid(value);
  }
  updateGL();
}

void QGLViewerWidget::setBoundingBoxFactor(double value) {
  std::cout << "Changing boundingBoxFactor to " << value << std::endl;
}

void QGLViewerWidget::setEpsilon(double value) {
  std::cout << "Changing epsilon to " << value << std::endl;
}

void QGLViewerWidget::setRadius(double value) {
  std::cout << "Changing radius to " << value << std::endl;
  this->implicitRadius = value;
  if (implicitSurface != nullptr) {
    this->implicitSurface->setRadius(this->implicitRadius);
  }
  updateGL();
}

void QGLViewerWidget::computeSamples() {
  std::cout << "Calling computeSamples" << std::endl;
  if (implicitSurface != nullptr) {
    this->implicitSurface->computeImplicitGridPoints();
  }
  updateGL();
}

void QGLViewerWidget::setDrawMCMesh(bool value) {
  std::cout << "Changing setDrawMCMesh value to " << value << std::endl;
  this->flag_drawMarchingCubes = value;
  updateGL();
}

void QGLViewerWidget::computeMC() {
  // TODO This does not properly stop if samples haven't been calculated yet
  // Find another way to check
  if (implicitSurface != nullptr) {
    this->implicitSurface->computeMarchingCubes();
  }
}

void QGLViewerWidget::computeEMC() {
  std::cout << "Calling computeEMC()" << std::endl;
}

void QGLViewerWidget::slotSnapshot(void) {
  // empty....
}

//=============================================================================
