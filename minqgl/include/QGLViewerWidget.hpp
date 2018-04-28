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
#include <memory>
#include <map>
#include <string>
#include <vector>

#include <QtOpenGL/qgl.h>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QMainWindow>

#include <glm/glm.hpp>

// --------------------
#include "cg2_framework.hpp"
#include "parser.hpp"
// --------------------

//== FORWARD DECLARATIONS =====================================================

class QMenu;
class QActionGroup;
class QAction;

//== CLASS DEFINITION =========================================================

class QGLViewerWidget : public QGLWidget {
  Q_OBJECT

 public:
  typedef QGLWidget Super;

  // Default constructor.
  QGLViewerWidget(QWidget* _parent = 0);

  //
  QGLViewerWidget(QGLFormat& _fmt, QWidget* _parent = 0);

  // Destructor.
  virtual ~QGLViewerWidget();

 private:
  std::shared_ptr<KDTree> kdtree;
  bool flag_drawTree = false; // used in key event handler
  unsigned drawLevelsOfTree = 8;

  void init(void);

  // Specific to algorithms
 protected:
  // loads data points from OFF file using the parser
  bool loadPointSet(const char* filename);

  void drawKDTree();

  // draw the scene: will be called by the paintGL() method.
  virtual void drawScene();

  // draws the given point list
  bool drawPointSet();

  // list of data points
  std::shared_ptr<PointList> pointList;

 protected:
  void setDefaultMaterial(void);
  void setDefaultLight(void);

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

  /********************************************************************/
  //   STANDARD OPENGL QT STUFS
  /********************************************************************/

 public:
  /* Sets the center and size of the whole scene.
  The _center is used as fixpoint for rotations and for adjusting
  the camera/viewer (see view_all()). */
  void setScenePos(const glm::vec3& center, float radius);

  /* view the whole scene: the eye point is moved far enough from the
  center so that the whole scene is visible. */
  void viewAll();

  float getRadius() const { return radius; }
  const glm::vec3& getCenter() const { return center; }

  const GLdouble* getModelviewMatrix() const { return modelviewMatrix; }
  const GLdouble* getProjectionMatrix() const { return projectionMatrix; }

  float fovy() const { return 45.0f; }

  QAction* findAction(const char* name);
  void addAction(QAction* action, const char* name);
  void removeAction(const char* name);
  void removeAction(QAction* action);

 private:  // inherited
  // initialize OpenGL states (triggered by Qt)
  void initializeGL();

  // draw the scene (triggered by Qt)
  void paintGL();

  // handle resize events (triggered by Qt)
  void resizeGL(int w, int h);

 protected:
  // Qt mouse events
  virtual void mousePressEvent(QMouseEvent*);
  virtual void mouseReleaseEvent(QMouseEvent*);
  virtual void mouseMoveEvent(QMouseEvent*);
  virtual void wheelEvent(QWheelEvent*);
  virtual void keyPressEvent(QKeyEvent*);

 private:
  // updates projection matrix
  void updateProjectionMatrix();

  // translate the scene and update modelview matrix
  void translate(const glm::vec3& trans);

  // rotate the scene (around its center) and update modelview matrix
  void rotate(const glm::vec3& axis, float angle);

  glm::vec3 center;
  float radius;

  GLdouble projectionMatrix[16], modelviewMatrix[16];

  // popup menu for draw mode selection
  QMenu* popupMenu;
  QActionGroup* drawModesGroup;
  typedef std::map<QString, QAction*> ActionMap;
  ActionMap namesToActions;

  // virtual trackball: map 2D screen point to unit sphere
  bool mapToSphere(const QPoint& point, glm::vec3& result);

  QPoint lastPoint2D;
  glm::vec3 lastPoint3D;
  bool lastPointOk;
};

//=============================================================================
#endif  // OPENMESHAPPS_QGLVIEWERWIDGET_HH
//=============================================================================