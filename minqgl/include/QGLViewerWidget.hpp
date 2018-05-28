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
#include <string>
#include <vector>
#include <chrono>

#include <QElapsedTimer>
#include <QFileDialog>
#include <QMainWindow>

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
#include "surfaces.hpp"
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
  void drawControlMesh();
  void drawSurfaceBTPS();
  void drawSurfaceMLS();

  // draw the scene: will be called by the paintGL() method.
  virtual void drawScene();

  // draws the given point list
  bool drawPointSet();

  // draws the selected point list
  bool drawSelectedPointSet();

  // list of data points
  std::shared_ptr<PointList> pointList;

  // list of selected data points
  PointPointerList selectedPointList;

  void setDefaultMaterial(void);
  void setDefaultLight(void);

  // Qt mouse events
  virtual void mousePressEvent(QMouseEvent *);
  int selectByMouse(std::shared_ptr<PointList> points, GLdouble mouseX,
                    GLdouble mouseY);
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
                                     tr("OFF Files (*.off);;"
                                        "All Files (*)"));
    if (!fileName.isEmpty()) {
      loadPointSet(fileName.toLocal8Bit());
    }
  }

  void setDrawMode(int value);
  void setPerformLinearSearch(bool value);
  void updateTreeState(int value);

  void setDrawPoints(bool value);
  void setDrawRegularGrid(bool value);
  void setDrawControlMeshPoints(bool value);

  void setGridXDim(int value);
  void setGridYDim(int value);

  void setRadius(double radius);

  void setDrawBezier(bool value);
  void setBezierSubdivisions(int k);

  void setDrawMls(bool value);
  void setMlsSubdivisions(int k);

private:
  void init();

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

  void calculateNormal(const Point &v1, const Point &v2);

  glm::vec3 center;
  float radius;

  GLdouble projectionMatrix[16], modelviewMatrix[16];

  // popup menu for draw mode selection
  QMenu *popupMenu;
  QActionGroup *drawModesGroup;
  QPoint lastPoint2D;

  typedef std::map<QString, QAction *> ActionMap;
  ActionMap namesToActions;

  // virtual trackball: map 2D screen point to unit sphere
  bool mapToSphere(const QPoint &point, glm::vec3 &result);

  glm::vec3 lastPoint3D;

  // Holds the current drawing mode set by our dropdown
  int drawMode = 0;
  int currentSliderValue = 0;

  bool drawPoints = true;
  bool drawGrid = true;
  int gridM = 10;
  int gridN = 10;
  bool flag_drawControlMesh = false;
  bool flag_drawSurfaceMLS = false;
  bool flag_drawSurfaceBTPS = false;
  bool flag_drawSelectedPoints = true;
  bool flag_drawTree = false;
  bool lastPointOk;
  bool performLinearSearch = false;
  bool selectOnRelease = false;

  GLdouble zNearFactor = 0.01;
  GLdouble zFarFactor = 500.0;

  unsigned drawLevelsOfTree = 0;
  int64_t selectedPointIndex = -1;

  std::shared_ptr<KDTree> kdtree;
  std::shared_ptr<Surfaces> surfaces;
};

//=============================================================================
#endif // OPENMESHAPPS_QGLVIEWERWIDGET_HH
//=============================================================================
