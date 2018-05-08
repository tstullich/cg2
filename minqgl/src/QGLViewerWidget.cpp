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

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <queue>
#include <sstream>

#include <cstdlib>

// --------------------
#ifndef __APPLE__
#include <GL/glut.h>
#else
#include <glut.h>
#endif
// --------------------
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
#include "glm/gtx/intersect.hpp"

#if !defined(M_PI)
#define M_PI 3.1415926535897932
#endif

const double TRACKBALL_RADIUS = 0.6;

using namespace Qt;
using namespace glm;

//== IMPLEMENTATION ==========================================================

//----------------------------------------------------------------------------

QGLViewerWidget::QGLViewerWidget(QWidget *_parent) : QGLWidget(_parent) {
  init();
}

//----------------------------------------------------------------------------

QGLViewerWidget::QGLViewerWidget(QGLFormat &_fmt, QWidget *_parent)
    : QGLWidget(_fmt, _parent) {
  init();
}

//----------------------------------------------------------------------------

void QGLViewerWidget::init() {
  // qt stuff
  setAttribute(Qt::WA_NoSystemBackground, true);
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setCursor(PointingHandCursor);
}

//----------------------------------------------------------------------------

QGLViewerWidget::~QGLViewerWidget() {}

bool QGLViewerWidget::loadPointSet(const char *filename) {
  Parser p;
  if (!p.open(filename)) {
    std::cout << "Unable to open file " << std::endl;
    return false;
  }

  std::shared_ptr<KDTree> newTree(
      new KDTree(p.getPoints(), std::make_unique<EuclDist>(), p.outerBox));
  kdtree = newTree;
  pointList = kdtree->getPoints();

  // Notify Sidebar of the size of K-Nearest max
  kNearestChanged(pointList->size());
  // clear selected point index for drawing
  selectedPointIndex = 0;

  updateGL();

  return true;
}

bool QGLViewerWidget::drawPointSet() {
  if (pointList != nullptr && pointList->size() > 0) {
    glDisable(GL_LIGHTING);

    glEnable(GL_POINT_SMOOTH);
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    glColor3f(0, 0, 0);
    for (unsigned int i = 0; i < pointList->size(); i++) {
      Point p = (*pointList)[i];
      glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
    return true;
  }
  return false;
}

bool QGLViewerWidget::drawSelectedPointSet() {
  if (selectedPointList.size() > 0) {
    glDisable(GL_LIGHTING);

    glEnable(GL_POINT_SMOOTH);
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(0, 255, 0);
    for (unsigned int i = 0; i < selectedPointList.size(); i++) {
      std::shared_ptr<Point> p = selectedPointList[i];
      glVertex3f(p->x, p->y, p->z);
    }
    glEnd();
    return true;
  }
  return false;
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
  glClearColor(1.0, 1.0, 1.0, 0.0);
  glDisable(GL_DITHER);
  glEnable(GL_DEPTH_TEST);

  // Material
  setDefaultMaterial();

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

void QGLViewerWidget::drawScene() {
  glDisable(GL_LIGHTING);

  if (flag_drawPoints) drawPointSet();
  if (flag_drawSelectedPoints) drawSelectedPointSet();
  if (flag_drawTree) drawKDTree();

  // Draw a coordinate system
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 1);
  glEnd();
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

  float radius = 0.005 * lenRS;  // relativ to distance between camera and point

  float t_ca = glm::dot(vecRS, rayDir);

  float t_ca_Sqrt = std::pow(t_ca, 2.0f);
  float lenRS_Sqrt = std::pow(lenRS, 2.0f);
  float radius_Sqrt = std::pow(radius, 2.0f);

  float t_hc_Sqrt = radius_Sqrt - lenRS_Sqrt + t_ca_Sqrt;
  if (t_hc_Sqrt < 0.0f) {  // no intersection
    return false;
  }

  float t1 = t_ca + std::sqrt(t_hc_Sqrt);
  float t2 = t_ca - std::sqrt(t_hc_Sqrt);

  *dist = (t1 < t2) ? t1 : t2;

  return true;
}

int QGLViewerWidget::selectByMouse(std::shared_ptr<PointList> points,
                                   GLdouble mouseX, GLdouble mouseY) {
  if (points == nullptr) {
    return -1;
  }

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
    selectedPointIndex =
        selectByMouse(pointList, event->pos().x(), event->pos().y());
    if (selectedPointIndex >= 0 && drawMode == 2) {
      selectedPointList =
          kdtree->collectKNearest((*pointList)[selectedPointIndex], 100);
    } else if (selectedPointIndex >= 0 && drawMode == 1) {
      selectedPointList =
          kdtree->collectInRadius((*pointList)[selectedPointIndex], 0.01);
    }
    updateGL();
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

    case Key_T:
      flag_drawTree = flag_drawTree ? false : true;
      break;

    case Key_J:
      if (drawLevelsOfTree > 0) drawLevelsOfTree--;
      break;

    case Key_K:
      if (drawLevelsOfTree < 8) drawLevelsOfTree++;
      break;

    case Key_H:
      std::cout << "Keys:\n";
      std::cout << "  Print\tMake snapshot\n";
      std::cout << "  C\tenable/disable back face culling\n";
      std::cout << "  F\tenable/disable fog\n";
      std::cout << "  I\tDisplay information\n";
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

void QGLViewerWidget::sliderValueChanged(int value) {
  if (pointList == nullptr || pointList->size() == 0) {
    std::cout << "No OFF file loaded. Won't draw. Please load a file before "
                 "changing values"
              << std::endl;
    return;
  }

  if (drawMode == 0) {
    drawLevelsOfTree = value;
  } else if (drawMode == 1) {
    selectedPointList =
        kdtree->collectInRadius((*pointList)[selectedPointIndex], value * 10);
  } else {
    selectedPointList =
        kdtree->collectKNearest((*pointList)[selectedPointIndex], value);
    drawSelectedPointSet();
  }

  // Need to redraw after changing settings
  updateGL();
}

void QGLViewerWidget::setPerformLinearSearch(bool value) {
  performLinearSearch = value;
}

void QGLViewerWidget::slotSnapshot(void) {
  // empty....
}

//=============================================================================
