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

QGLViewerWidget::QGLViewerWidget(QWidget *_parent) : QGLWidget(_parent) {
  assert(glGetError() == GL_NO_ERROR);
  init();
}

QGLViewerWidget::QGLViewerWidget(QGLFormat &_fmt, QWidget *_parent)
    : QGLWidget(_fmt, _parent) {
  assert(glGetError() == GL_NO_ERROR);
  init();
}

void QGLViewerWidget::init() {
  // qt stuff
  setAttribute(Qt::WA_NoSystemBackground, true);
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setCursor(PointingHandCursor);
  assert(glGetError() == GL_NO_ERROR);
  // threads.push_back(std::thread(&QGLViewerWidget::animateLight, this));
}

QGLViewerWidget::~QGLViewerWidget() {}

bool QGLViewerWidget::loadPointSet(const char *filename) {
  Parser p;
  if (!p.open(filename)) {
    std::cout << "Unable to open file " << std::endl;
    return false;
  }

  // The new logic for parsing our OBJ data files
  p.parse();
  auto vertices = p.getVertices();
  std::cout << "Vertices: " << vertices.size() << std::endl;
  auto faces = p.getFaces();
  std::cout << "Faces: " << faces.size() << std::endl;

  this->mesh = std::make_shared<Mesh>(vertices, faces);

  this->mesh->computeUnweightedNormals();
  this->mesh->computeWeightedNormals();
  unsigned int numEigenVectors = 5;
  this->mesh->computeUniformLaplacian(numEigenVectors);

  // set mesh dependent parameters
  this->defaultRadius = 1.5 * this->mesh->getBoundingRadius();
  setScenePos(mesh->getCenter(), defaultRadius);
  this->lightPos = glm::vec3(defaultRadius, defaultRadius, defaultRadius);

  updateGL();

  return true;
}

// void QGLViewerWidget::animateLight() {
//  unsigned frameCounter = 25;
//  while (true) {
//    if (flags.animate) {
//      lightPos[0] =
//          cloudSize[0] * sin(double(frameCounter) * M_PI / 100) + center[0];
//      lightPos[1] =
//          cloudSize[1] * cos(double(frameCounter) * M_PI / 100) + center[1];
//      lightPos[2] = center[2] + 0.75 * cloudSize[2];
//      frameCounter = (frameCounter + 1) % 200;
//      update();
//    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(10));
//  }
//}

// bool QGLViewerWidget::drawPointSetNormals() {
// glDisable(GL_LIGHTING);
// glBegin(GL_LINES);
// glColor3f(0.75f, 0.75f, 0.75f);
// for (unsigned int i = 0; i < pointList->size(); i++) {
//  Point p = (*pointList)[i];
//  if (p.type == originalPoint) {
//    if (glm::length(p.normal) == 0) {
//    }
//    glVertex3f(p.x, p.y, p.z);
//    glVertex3f(p.x + lengthScalar * p.normal[0],
//               p.y + lengthScalar * p.normal[1],
//               p.z + lengthScalar * p.normal[2]);
//  }
//}
// glEnd();

//  return true;
//}

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
  std::cout << "using OpenGL " << version << " | " << vendor << " | "
            << renderer << " | "
            << "GLSL " << glsl << std::endl;
  // OpenGL state
  glClearColor(1.0, 1.0, 1.0, 1.0);
  assert(glGetError() == GL_NO_ERROR);
  glDisable(GL_DITHER);
  assert(glGetError() == GL_NO_ERROR);
  glEnable(GL_DEPTH_TEST);
  assert(glGetError() == GL_NO_ERROR);
  glEnable(GL_CULL_FACE);
  assert(glGetError() == GL_NO_ERROR);
  glFrontFace(GL_CCW);

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

glm::vec3 QGLViewerWidget::triangleNormal(const Point &v1, const Point &v2,
                                          const Point &v3) {
  glm::vec3 N =
      glm::cross((v2.toVec3() - v1.toVec3()), (v3.toVec3() - v1.toVec3()));
  return glm::normalize(N);
}

glm::vec3 QGLViewerWidget::gourad(const Point &v1, const glm::vec3 &normal) {
  glm::vec3 vertPos(v1.x, v1.y, v1.z);
  glm::vec3 ambientColor(0.2f, 0.2f, 0.2f);
  glm::vec3 diffuseColor(0.5f, 0.5f, 0.5f);
  glm::vec3 specularColor(1.0f, 1.0f, 1.0f);

  // camera positions
  glm::vec3 camPos = computeCamPos();

  glm::vec3 vertToCam = glm::normalize(camPos - vertPos);
  glm::vec3 vertToLight = glm::normalize(lightPos - vertPos);

  glm::vec3 N(glm::normalize(normal));
  if (flags.frontFaceCCW == false) {
    N = -N;
  }
  glm::vec3 L(glm::normalize(vertToLight));
  glm::vec3 E(glm::normalize(vertToCam));
  glm::vec3 R(0.0f, 0.0f, 0.0f);
  float NL = glm::dot(N, L);
  if (NL >= 0.0f) {
    R = glm::normalize((2.0f * (N * NL)) - L);
  }

  glm::vec3 diffuse = diffuseColor * glm::max(glm::dot(N, L), 0.0f);

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

void drawVec3(glm::vec3 v) { glVertex3f(v[0], v[1], v[2]); }

void QGLViewerWidget::drawMesh() {
  // No mesh to draw. Just return
  if (this->mesh == nullptr) {
    return;
  }

  std::vector<Point> vertices = mesh->getVertices();
  std::vector<Face> faces = mesh->getFaces();

  for (Face f : faces) {
    assert(f.numVertices() == 3);
    Point p0 = vertices[f[0]];
    Point p1 = vertices[f[1]];
    Point p2 = vertices[f[2]];

    // draw wire mesh
    glBegin(GL_LINE_LOOP);
    glColor3f(1.0, 1.0, 1.0);
    glVertex3f(p0.x, p0.y, p0.z);
    glVertex3f(p1.x, p1.y, p1.z);
    glVertex3f(p2.x, p2.y, p2.z);
    glVertex3f(p0.x, p0.y, p0.z);
    glEnd();

    // draw Face
    glBegin(GL_TRIANGLES);
    // calc normal
    auto normal = triangleNormal(p0, p1, p2);
    // present first point
    auto col0 = gourad(p0, normal);
    glColor3f(col0[0], col0[1], col0[2]);
    glVertex3f(p0.x, p0.y, p0.z);
    // present second point
    auto col1 = gourad(p1, normal);
    glColor3f(col1[0], col1[1], col1[2]);
    glVertex3f(p1.x, p1.y, p1.z);
    // present third point
    auto col2 = gourad(p2, normal);
    glColor3f(col2[0], col2[1], col2[2]);
    glVertex3f(p2.x, p2.y, p2.z);
    glEnd();
  }

  /* // calc distance for perspectiv point size */
  /* float distToLight = glm::length(lightPos - computeCamPos()); */
  /* distToLight *= distToLight * 0.5f; */
  /* glPointSize(32.0f / distToLight); */
  glPointSize(10.0f);
  glEnable(GL_POINT_SMOOTH);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex3f(lightPos[0], lightPos[1], lightPos[2]);
  glEnd();
}

void QGLViewerWidget::drawUnweightedVertexNormals() {
  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glColor3f(0.0f, 0.0f, 1.0f);
  float lengthScalar = 0.1 * mesh->getBoundingRadius();
  std::vector<Point> vertices = mesh->getVertices();
  for (Point v : vertices) {
    glVertex3f(v.x, v.y, v.z);
    glVertex3f(v.x + lengthScalar * v.normal[0],
               v.y + lengthScalar * v.normal[1],
               v.z + lengthScalar * v.normal[2]);
  }
  glEnd();
}

void QGLViewerWidget::drawWeightedVertexNormals() {
  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glColor3f(1.0f, 0.0f, 0.0f);
  float lengthScalar = 0.1 * mesh->getBoundingRadius();
  std::vector<Point> vertices = mesh->getVertices();
  for (Point v : vertices) {
    glVertex3f(v.x, v.y, v.z);
    glVertex3f(v.x + lengthScalar * v.weightedNormal[0],
               v.y + lengthScalar * v.weightedNormal[1],
               v.z + lengthScalar * v.weightedNormal[2]);
  }
  glEnd();
}

void QGLViewerWidget::drawLaplacian() {
  // No mesh to draw. Just return
  if (this->mesh == nullptr ||
      this->mesh->verticesUniformLaplacian.size() == 0) {
    return;
  }

  std::vector<Point> vertices = mesh->verticesUniformLaplacian;
  std::vector<Face> faces = mesh->getFaces();

  glBegin(GL_TRIANGLES);
  for (Face f : faces) {
    assert(f.numVertices() == 3);
    Point p0 = vertices[f[0]];
    Point p1 = vertices[f[1]];
    Point p2 = vertices[f[2]];

    auto normal = triangleNormal(p0, p1, p2);

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

  /* // calc distance for perspectiv point size */
  /* float distToLight = glm::length(lightPos - computeCamPos()); */
  /* distToLight *= distToLight * 0.5f; */
  /* glPointSize(32.0f / distToLight); */
  glPointSize(10.0f);
  glEnable(GL_POINT_SMOOTH);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex3f(lightPos[0], lightPos[1], lightPos[2]);
  glEnd();
}

// This function assumes that only one of Implicit or Explicit integration is running at a time
// Will see breakage here if we performed implicit and explicit integration at the same time
void QGLViewerWidget::drawCotanLaplace(const std::vector<Point> &vertices) {
  // Check which mesh to use
  //std::vector<Point> vertices = !mesh->verticesExplicitLaplace.empty() ? mesh->verticesExplicitLaplace : mesh->verticesImplicitLaplace;
  std::vector<Face> faces = mesh->getFaces();

  glBegin(GL_TRIANGLES);
  for (Face f : faces) {
    assert(f.numVertices() == 3);
    Point p0 = vertices[f[0]];
    Point p1 = vertices[f[1]];
    Point p2 = vertices[f[2]];

    auto normal = triangleNormal(p0, p1, p2);

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

  glPointSize(10.0f);
  glEnable(GL_POINT_SMOOTH);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex3f(lightPos[0], lightPos[1], lightPos[2]);
  glEnd();
}

void QGLViewerWidget::drawScene() {
  glDisable(GL_LIGHTING);

  if (flags.drawMesh) {
    drawMesh();
  }
  if (flags.drawUnweightedNormals) {
    drawUnweightedVertexNormals();
  }
  if (flags.drawWeightedNormals) {
    drawWeightedVertexNormals();
  }
  if (flags.drawCotanLaplace) {
    std::vector<Point> verts;
    if (this->mesh != nullptr && !this->mesh->verticesExplicitLaplace.empty()) {
      verts = this->mesh->verticesExplicitLaplace;
      drawCotanLaplace(verts);
    } else if (this->mesh != nullptr && !this->mesh->verticesImplicitLaplace.empty()) {
      verts = this->mesh->verticesImplicitLaplace;
      drawCotanLaplace(verts);
    }
  }
  if (flags.drawColorCube && this->mesh == nullptr) {
    drawColorCube();
  }

  drawLaplacian();

  if (!flags.drawMesh) {
    // Draw a coordinate system
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
  } else {
    lastPointOk = mapToSphere(lastPoint2D = event->pos(), lastPoint3D);
  }
}

//----------------------------------------------------------------------------
void QGLViewerWidget::mouseMoveEvent(QMouseEvent *event) {
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

void QGLViewerWidget::mouseReleaseEvent(QMouseEvent *event) {
  // finish up
  lastPointOk = false;
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
      flags.animate = (flags.animate) ? false : true;
      break;

    case Key_C:
      setScenePos(center, defaultRadius);
      break;

    case Key_F:
      if (flags.frontFaceCCW) {
        glCullFace(GL_FRONT);
        flags.frontFaceCCW = false;
      } else {
        flags.frontFaceCCW = true;
        glCullFace(GL_BACK);
      }
      break;

    case Key_H:
      std::cout << "Keys:\n";
      std::cout << "  Print\tMake snapshot\n";
      break;

    case Key_P:
      flags.drawColorCube = (flags.drawColorCube)? false : true;
      break;

    case Key_Q:
    case Key_Escape:
      qApp->quit();
  }
  _event->ignore();

  updateGL();
}

void QGLViewerWidget::drawColorCube() {
  glBegin(GL_QUADS);
  // x = 0
  glColor3f( 0.0, 0.0, 0.0);
  glVertex3f(-0.5, -0.5, -0.5);
  glColor3f( 0.0, 0.0, 1.0);
  glVertex3f(-0.5, -0.5, 0.5);
  glColor3f( 0.0, 1.0, 1.0);
  glVertex3f(-0.5, 0.5, 0.5);
  glColor3f( 0.0, 1.0, 0.0);
  glVertex3f(-0.5, 0.5, -0.5);
  // x = 1
  glColor3f( 1.0, 0.0, 0.0);
  glVertex3f(0.5, -0.5, -0.5);
  glColor3f( 1.0, 1.0, 0.0);
  glVertex3f(0.5, 0.5, -0.5);
  glColor3f( 1.0, 1.0, 1.0);
  glVertex3f(0.5, 0.5, 0.5);
  glColor3f( 1.0, 0.0, 1.0);
  glVertex3f(0.5, -0.5, 0.5);
  // y = 0
  glColor3f( 0.0, 0.0, 0.0);
  glVertex3f(-0.5, -0.5, -0.5);
  glColor3f( 1.0, 0.0, 0.0);
  glVertex3f(0.5, -0.5, -0.5);
  glColor3f( 1.0, 0.0, 1.0);
  glVertex3f(0.5, -0.5, 0.5);
  glColor3f( 0.0, 0.0, 1.0);
  glVertex3f(-0.5, -0.5, 0.5);
  // y = 1
  glColor3f( 0.0, 1.0, 0.0);
  glVertex3f(-0.5, 0.5, -0.5);
  glColor3f( 0.0, 1.0, 1.0);
  glVertex3f(-0.5, 0.5, 0.5);
  glColor3f( 1.0, 1.0, 1.0);
  glVertex3f(0.5, 0.5, 0.5);
  glColor3f( 1.0, 1.0, 0.0);
  glVertex3f(0.5, 0.5, -0.5);
  // z=0 face
  glColor3f(0.0, 0.0, 0.0);
  glVertex3f(-0.5, -0.5, -0.5);
  glColor3f(0.0, 1.0, 0.0);
  glVertex3f(-0.5, 0.5, -0.5);
  glColor3f(1.0, 1.0, 0.0);
  glVertex3f(0.5, 0.5, -0.5);
  glColor3f(1.0, 0.0, 0.0);
  glVertex3f(0.5, -0.5, -0.5);
  // z=1 face
  glColor3f(0.0, 0.0, 1.0);
  glVertex3f(-0.5, -0.5, 0.5);
  glColor3f(1.0, 0.0, 1.0);
  glVertex3f(0.5, -0.5, 0.5);
  glColor3f(1.0, 1.0, 1.0);
  glVertex3f(0.5, 0.5, 0.5);
  glColor3f(0.0, 1.0, 1.0);
  glVertex3f(-0.5, 0.5, 0.5);
  glEnd();

  return;
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

void QGLViewerWidget::setDrawMesh(bool value) {
  std::cout << "setting flags.drawMesh to " << value << std::endl;
  flags.drawMesh = value;
  updateGL();
}

void QGLViewerWidget::setMeshAlpha(double value) {
  std::cout << "Setting mesh alpha " << value << std::endl;
}

void QGLViewerWidget::setDrawUnweightedNormals(bool value) {
  std::cout << "setting drawUnweightedNormals to " << value << std::endl;
  flags.drawUnweightedNormals = value;
  updateGL();
}

void QGLViewerWidget::setDrawWeightedNormals(bool value) {
  std::cout << "setting drawWeightedNormals to " << value << std::endl;
  flags.drawWeightedNormals = value;
  updateGL();
}

void QGLViewerWidget::setDrawGraphLaplace(bool value) {
  std::cout << "Setting draw graph laplace " << value << std::endl;
}

void QGLViewerWidget::setStepSize(double value) {
  std::cout << "Setting step size " << value << std::endl;
}

void QGLViewerWidget::graphLaplaceMove() {
  std::cout << "Calling graph laplace move" << std::endl;
}

void QGLViewerWidget::graphLaplaceReset() {
  std::cout << "Calling graph laplace reset" << std::endl;
}

void QGLViewerWidget::setDrawCotanLaplace(bool value) {
  flags.drawCotanLaplace = true;
  updateGL();
}

void QGLViewerWidget::setExplicitStep(double value) {
  explicitStepSize = value;
}

void QGLViewerWidget::cotanLaplaceExplicitStep() {
  mesh->computeExplicitCotan(explicitStepSize, basisFunctions);
  updateGL();
}

void QGLViewerWidget::setImplicitStep(double value) {
  implicitStepSize = value;
}

void QGLViewerWidget::cotanLaplaceImplicitStep() {
  mesh->computeImplicitCotan(implicitStepSize, basisFunctions);
  updateGL();
}

void QGLViewerWidget::setBasisFunctions(int value) {
  basisFunctions = (unsigned int)value;
}

void QGLViewerWidget::setManifoldHarmonics(bool value) {
  std::cout << "Setting manifold harmonics " << value << std::endl;
}

void QGLViewerWidget::cotanLaplaceReset() {
  mesh->resetCotanLaplace();
  updateGL();
}

void QGLViewerWidget::slotSnapshot(void) {
  // empty....
}
