#include "MainWindow.h"
#include <MeshIO.h>
#include <QtGui/QFileDialog>

#pragma warning (disable : 4996)


MyWindow::MyWindow()
{
	ui.setupUi(this);
	m_viewer = ui.DisplayWidget;
	connectSlot();
//	setCentralWidget(m_viewer);
}

MyWindow::~MyWindow()
{
//	delete m_viewer;
}

void MyWindow::setModelView(ModelView * modelView)
{
	m_viewer->setModelView(modelView);
}

void MyWindow::connectSlot()
{
//	QObject::connect( ui.pushButton,  SIGNAL(clicked()), this, SLOT(measure()) );
//	QObject::connect( ui.pushButton2,  SIGNAL(clicked()), this, SLOT(saveMeasurement()) );
	QObject::connect( ui.actionOpen,  SIGNAL(triggered()), this, SLOT(load()) );
//	QObject::connect( ui.actionLoad_Marker,  SIGNAL(triggered()), this, SLOT(loadMarker()) );
//	QObject::connect( ui.actionSave_Marker,  SIGNAL(triggered()), this, SLOT(saveMarker()) );
	QObject::connect( ui.actionWireframe,  SIGNAL(toggled(bool)), this, SLOT(showWireFrame(bool)) );

//	QKeySequence MKey(tr("M"));
//	ui.pushButton->setShortcut(MKey);

}

void MyWindow::load()
{
	QString filename = QFileDialog::getOpenFileName(this, tr("Load File"), getenv( "HOME" ),"*.m;*.obj;*.ply");
	if(filename.isNull())
		return;
	QString suffix = filename.section('.', -1);
	SimMesh * mesh = new SimMesh;
	ModelView * modelView = new ModelView;
	modelView->theMesh = mesh;
	if (suffix == "m" && !MeshIO::ReadM(filename.toLocal8Bit().data(), *mesh))
	{
		std::cout<<"Fail to read the File!"<<std::endl;
		return ;
	}
	if (suffix == "obj" && !MeshIO::ReadOBJ(filename.toLocal8Bit().data(), *modelView))
	{
		std::cout<<"Fail to read the File!"<<std::endl;
		return ;
	}
	if (suffix == "ply" && !MeshIO::ReadPLY(filename.toLocal8Bit().data(), *modelView))
	{
		std::cout<<"Fail to read the File!"<<std::endl;
		return ;
	}
	modelView->LoadMesh(mesh);
	setModelView(modelView);
}

//void MyWindow::loadMarker()
//{
//	QString filename = QFileDialog::getOpenFileName(this, tr("Load File"), getenv( "HOME" ),"*.cm");
//	if(filename.isNull())
//		return;
//	m_viewer->loadSelectedVerticesByPosition(filename.toLocal8Bit().data());
//}
//
//void MyWindow::saveMarker()
//{
//	QString filename = QFileDialog::getSaveFileName(this, tr("Load File"), getenv( "HOME" ),"*.txt");
//	if(filename.isNull())
//		return;
//	m_viewer->saveSelectedVertices(filename.toLocal8Bit().data());
//}

void MyWindow::showWireFrame(bool show)
{
	m_viewer->ShowWireFrame(show);
}

//void MyWindow::measure()
//{
//	m_viewer->measureBetweenSelectedVertices();
//}
//
//void MyWindow::saveMeasurement()
//{
//	QString filename = QFileDialog::getSaveFileName(this, tr("Load File"), getenv( "HOME" ),"*.txt");
//	if(filename.isNull())
//		return;
//	m_viewer->saveMeasurement(filename.toLocal8Bit().data());
//}