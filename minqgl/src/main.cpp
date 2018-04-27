/*===========================================================================*\
 *
 * CG2 Sandbox - TU-Berlin Computer Graphics - SS13
 * Author : Olivier Rouiller
 *
\*===========================================================================*/

// main window settings (relativ size to screen resolution)
#define WINDOW_SCALE     0.8
#define WINDOW_PLACEMENT (1-WINDOW_SCALE)/2

#include <qdockwidget.h>
#include <QApplication>
#include <QFileDialog>
#include <QMainWindow>
#include <QMenuBar>
#include <QMessageBox>
#include <fstream>
#include <iostream>
#include <QDesktopWidget>

#ifndef __APPLE__
#include <GL/glut.h>
#else
#include <glut.h>
#endif

#include <QGLContext>

#include "QGLViewerWidget.hpp"

void createMenu(QMainWindow* w);

int main(int argc, char** argv) {
  // OpenGL check
  QApplication::setColorSpec(QApplication::CustomColor);
  QApplication app(argc, argv);

  if (!QGLFormat::hasOpenGL()) {
    QString msg = "System has no OpenGL support!";
    QMessageBox::critical(0, QString("OpenGL"), msg + QString(argv[1]));
    return -1;
  }

  // create widget
  QMainWindow* mainWin = new QMainWindow();
  QGLViewerWidget* w = new QGLViewerWidget(mainWin);
  mainWin->setCentralWidget(w);

#if !defined(__APPLE__)
  glutInit(&argc, argv);
#endif

  createMenu(mainWin);

  QDesktopWidget widget;
  QRect mainScreenSize = widget.availableGeometry(widget.primaryScreen());
  int screenHeight = mainScreenSize.height();
  int screenWidth = mainScreenSize.width();

  mainWin->resize(WINDOW_SCALE*screenWidth, WINDOW_SCALE*screenHeight);
  mainWin->move(WINDOW_PLACEMENT*screenWidth, WINDOW_PLACEMENT*screenHeight);
  mainWin->show();

  return app.exec();
}

void createMenu(QMainWindow* w) {
  // Create the file menu
  using namespace Qt;
  QMenu* fileMenu = w->menuBar()->addMenu(w->tr("&File"));

  // Create an action to open a point set
  QAction* openAct = new QAction(w->tr("&load point set..."), w);
  openAct->setShortcut(w->tr("Ctrl+O"));
  openAct->setStatusTip(w->tr("Open a point set file"));
  QObject::connect(openAct, SIGNAL(triggered()), w->centralWidget(),
                   SLOT(queryOpenPointSetFile()));
  fileMenu->addAction(openAct);
}
