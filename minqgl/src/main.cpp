/*===========================================================================*\
 *
 * CG2 Sandbox - TU-Berlin Computer Graphics - SS13
 * Author : Olivier Rouiller
 *
\*===========================================================================*/

// main window settings (relativ size to screen resolution)
#define WINDOW_SCALE 0.8
#define WINDOW_PLACEMENT (1 - WINDOW_SCALE) / 2

#include <qdockwidget.h>
#include <QApplication>
#include <QComboBox>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QMainWindow>
#include <QMenuBar>
#include <QMessageBox>
#include <QSlider>
#include <QStringList>
#include <fstream>
#include <iostream>

#include <GL/glew.h>

#ifndef __APPLE__
#include <GL/glut.h>
#else
#include <glut.h>
#endif

#include "QGLViewerWidget.hpp"
#include "SidebarWidget.hpp"

void createMenu(QMainWindow *w);
void createSidebar(QMainWindow *w);

int main(int argc, char **argv) {
  // OpenGL check
  QApplication::setColorSpec(QApplication::CustomColor);
  QApplication app(argc, argv);

  if (!QGLFormat::hasOpenGL()) {
    QString msg = "System has no OpenGL support!";
    QMessageBox::critical(0, QString("OpenGL"), msg + QString(argv[1]));
    return -1;
  }
  std::cout << QGLFormat::openGLVersionFlags() << std::endl;

#if !defined(__APPLE__)
  glutInit(&argc, argv);
#endif

  // create widget
  QMainWindow *mainWin = new QMainWindow();
  QGLFormat format;
  QGLFormat::setDefaultFormat(format);
  format.setProfile(QGLFormat::CompatibilityProfile);
  format.setVersion(3, 3);
  QGLViewerWidget *w = new QGLViewerWidget(format, mainWin);
  mainWin->setCentralWidget(w);

  createMenu(mainWin);
  createSidebar(mainWin);

  QDesktopWidget widget;
  QRect mainScreenSize = widget.availableGeometry(widget.primaryScreen());
  int screenHeight = mainScreenSize.height();
  int screenWidth = mainScreenSize.width();

  mainWin->resize(WINDOW_SCALE * screenWidth, WINDOW_SCALE * screenHeight);
  mainWin->move(WINDOW_PLACEMENT * screenWidth,
                WINDOW_PLACEMENT * screenHeight);
  mainWin->show();

  return app.exec();
}

void createMenu(QMainWindow *w) {
  // Create the file menu
  using namespace Qt;
  QMenu *fileMenu = w->menuBar()->addMenu(w->tr("&File"));

  // Create an action to open a point set
  QAction *openAct = new QAction(w->tr("&load point set..."), w);
  openAct->setShortcut(w->tr("Ctrl+O"));
  openAct->setStatusTip(w->tr("Open a point set file"));
  QObject::connect(openAct, SIGNAL(triggered()), w->centralWidget(),
                   SLOT(queryOpenPointSetFile()));
  fileMenu->addAction(openAct);
}

void createSidebar(QMainWindow *w) {
  auto sidebar = new SidebarWidget(w->centralWidget());
  w->addDockWidget(Qt::RightDockWidgetArea, sidebar);
}
