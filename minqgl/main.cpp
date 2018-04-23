/*===========================================================================*\
 *
 * CG2 Sandbox - TU-Berlin Computer Graphics - SS13
 * Author : Olivier Rouiller
 *	
\*===========================================================================*/


#include <iostream>
#include <fstream>
#include <QApplication>
#include <QMessageBox>
#include <qdockwidget.h>
#include <QMainWindow>
#include <QMenuBar>
#include <QFileDialog>

#ifndef __APPLE__
#include <GL/glut.h>
#else
#include <glut.h>
#endif

#include <QGLContext>


#include "QGLViewerWidget.h"

void createMenu(QMainWindow* w);

int main(int argc, char **argv)
{
	// OpenGL check
	QApplication::setColorSpec( QApplication::CustomColor );
	QApplication app(argc,argv);

	if ( !QGLFormat::hasOpenGL() ) 
	{
		QString msg = "System has no OpenGL support!";
		QMessageBox::critical( 0, QString("OpenGL"), msg + QString(argv[1]) );
		return -1;
	}

	// create widget
	QMainWindow* mainWin = new QMainWindow();
	QGLViewerWidget* w = new QGLViewerWidget(mainWin);
	mainWin->setCentralWidget(w);

	#if !defined(__APPLE__)
	glutInit(&argc,argv);
	#endif

	createMenu(mainWin);

	mainWin->resize(1024, 768);
	mainWin->show();

	return app.exec();
}

void createMenu(QMainWindow* w)
{
	// Create the file menu
	using namespace Qt;
	QMenu *fileMenu = w->menuBar()->addMenu(w->tr("&File"));

	// Create an action to open a point set
	QAction* openAct = new QAction(w->tr("&load point set..."), w);
	openAct->setShortcut(w->tr("Ctrl+O"));
	openAct->setStatusTip(w->tr("Open a point set file"));
	QObject::connect(openAct, SIGNAL(triggered()), w->centralWidget(), SLOT(queryOpenPointSetFile()));
	fileMenu->addAction(openAct);
}
