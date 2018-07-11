/*===========================================================================*\
 *
 * CG2 Sandbox - TU-Berlin Computer Graphics - SS13
 * Author : Olivier Rouiller
 *
\*===========================================================================*/

#ifndef QGLVIEWERWIDGET_H
#define QGLVIEWERWIDGET_H

//== INCLUDES =================================================================

#include <iostream>
#include <map>
#include <memory>
#include <algorithm>
#include <string>
#include <vector>
#include <chrono>
#include <assert.h>
#include <thread>

#include <QElapsedTimer>
#include <QFileDialog>
#include <QMainWindow>
#include <QDesktopWidget>

#ifndef __APPLE__
#include <GL/glut.h>
#else
#include <glut.h>
#endif

#include <QtOpenGL/qgl.h>
#include <glm/glm.hpp>

// --------------------
#include "kdtree.hpp"
#include "parser.hpp"
#include "point.hpp"
#include "mesh.hpp"
// --------------------

//== FORWARD DECLARATIONS =====================================================

class QMenu;
class QActionGroup;
class QAction;

//== CLASS DEFINITION =========================================================

class QGLViewerWidget : public QGLWidget {
  Q_OBJECT

public:
  /* Sets the center and size of the whole scene.
  The _center is used as fixpoint for rotations and for adjusting
  the camera/viewer (see view_all()). */
  void setScenePos(const glm::vec3 &center, float radius);

  /* view the whole scene: the eye point is moved far enough from the
  center so that the whole scene is visible. */
  void viewAll();

  float getRadius() const { return radius; }
  const glm::vec3 &getCenter() const { return center; }

  const GLdouble *getModelviewMatrix() const { return modelviewMatrix; }
  const GLdouble *getProjectionMatrix() const { return projectionMatrix; }

  float fovy() const { return 45.0f; }

  QAction *findAction(const char *name);
  void addAction(QAction *action, const char *name);
  void removeAction(const char *name);
  void removeAction(QAction *action);

  typedef QGLWidget Super;

  // Default constructor.
  QGLViewerWidget(QWidget *_parent = 0);

  //
  QGLViewerWidget(QGLFormat &_fmt, QWidget *_parent = 0);

  // Destructor.
  virtual ~QGLViewerWidget();

  // Specific to algorithms
protected:
  // loads data points from OFF file using the parser
  bool loadPointSet(const char *filename);

  void drawRegularGrid();
  void drawKDTree();

  void drawIntersections();

  // draw the scene: will be called by the paintGL() method.
  virtual void drawScene();

  void setDefaultMaterial(void);
  void setDefaultLight(void);

  // Qt mouse events
  virtual void mousePressEvent(QMouseEvent *);
  virtual void mouseReleaseEvent(QMouseEvent *);
  virtual void mouseMoveEvent(QMouseEvent *);
  virtual void wheelEvent(QWheelEvent *);
  virtual void keyPressEvent(QKeyEvent *);

private slots:
  // popup menu clicked
  void slotSnapshot(void);

  // open point set
  void queryOpenPointSetFile() {
    QString fileName =
        QFileDialog::getOpenFileName(this, tr("Open point set file"), tr(""),
                                     tr("OBJ Files (*.obj);;"
                                        "All Files (*)"));
    if (!fileName.isEmpty()) {
      loadPointSet(fileName.toLocal8Bit());
    }
  }

  void drawMesh();
  void drawUnweightedVertexNormals();
  void drawWeightedVertexNormals();
  void drawLaplacian();

  void setDrawMesh(bool value);
  void setMeshAlpha(double value);

  void setDrawUnweightedNormals(bool value);
  void setDrawWeightedNormals(bool value);

  void setDrawGraphLaplace(bool value);
  void setStepSize(double value);
  void graphLaplaceMove();
  void graphLaplaceReset();

  void setDrawCotanLaplace(bool value);
  void setExplicitStep(double value);
  void cotanLaplaceExplicitStep();
  void setImplicitStep(double value);
  void cotanLaplaceImplicitStep();
  void setBasisFunctions(int value);
  void setManifoldHarmonics(bool value);
  void cotanLaplaceReset();

private:
  void init();

  //void animateLight();

  // initialize OpenGL states (triggered by Qt)
  void initializeGL();

  // draw the scene (triggered by Qt)
  void paintGL();

  // handle resize events (triggered by Qt)
  void resizeGL(int w, int h);

  // updates projection matrix
  void updateProjectionMatrix();

  // translate the scene and update modelview matrix
  void translate(const glm::vec3 &trans);

  // rotate the scene (around its center) and update modelview matrix
  void rotate(const glm::vec3 &axis, float angle);

  glm::vec3 triangleNormal(const Point &v1, const Point &v2, const Point &v3);

  glm::vec3 gourad(const Point &v1, const glm::vec3 &normal);

  glm::mat4 computeModelViewInv();

  glm::vec3 computeCamPos();

  // Hardcoding a position here since we are just demoing and need
  // only one light source
  // 1.2071067812f to start in sync w/ point animation
  glm::vec3 lightPos = glm::vec3(1.2071067812f, 1.2071067812f, 1.0f);

  GLdouble projectionMatrix[16], modelviewMatrix[16];

  // popup menu for draw mode selection
  QMenu *popupMenu;
  QActionGroup *drawModesGroup;
  QPoint lastPoint2D;

  typedef std::map<QString, QAction *> ActionMap;
  ActionMap namesToActions;

  float radius;
  float defaultRadius = 1.0;
  glm::vec3 center = glm::vec3(0.0, 0.0, 0.0);
  // virtual trackball: map 2D screen point to unit sphere
  bool mapToSphere(const QPoint &point, glm::vec3 &result);

  glm::vec3 lastPoint3D;

  // Holds the current drawing mode set by our dropdown
  bool lastPointOk;

  // flag struct
  struct Flags {
    bool drawMesh = true;
    bool animate = false;
    bool drawUnweightedNormals = false;
    bool drawWeightedNormals = false;
    bool frontFaceCCW = true;
  }flags;

  std::vector<std::thread> threads;

  GLdouble zNearFactor = 0.01;
  GLdouble zFarFactor = 10000.0;

  std::shared_ptr<Mesh> mesh;
};

//=============================================================================
#endif // OPENMESHAPPS_QGLVIEWERWIDGET_HH
//=============================================================================
