#include "MainWindow.h"
#include "../../MeshLib_Extension/MeshIO.h"
#include <QtGui/QFileDialog>
#include <QtGui/QInputDialog>
#include <QtGui/QLabel>
#include <QtGui/QMessageBox>

#pragma warning (disable : 4996)


MyWindow::MyWindow()
{
	ui.setupUi(this);
	m_viewer = ui.DisplayWidget;
	signalMapper = new QSignalMapper(this);
	connectSlot();
//	setCentralWidget(m_viewer);
}

MyWindow::~MyWindow()
{
	delete m_viewer;
}

void MyWindow::connectSlot()
{
	QObject::connect( ui.actionOpen,  SIGNAL(triggered()), this, SLOT(load()) );
	QObject::connect( ui.actionImport,  SIGNAL(triggered()), this, SLOT(import()) );
	QObject::connect( ui.actionSave,  SIGNAL(triggered()), this, SLOT(save()) );
	QObject::connect( signalMapper,  SIGNAL(mapped(int)), this, SLOT(showMeshChanged(int)) );
	QObject::connect( ui.checkbox,  SIGNAL(clicked()), this, SLOT(showMeshChanged()) );
	QObject::connect( ui.actionWireframe,  SIGNAL(toggled(bool)), this, SLOT(showWireFrame(bool)) );

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
	if (suffix == "m" && !MeshIO::ReadM(filename.toLocal8Bit().data(), *modelView))
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
	modelView->model_name = filename.section('/', -1).toLocal8Bit().data();
	ui.tableWidget->setRowCount(0);
	ui.tableWidget->insertRow(0);
	ui.tableWidget->setCellWidget(0, 0, new QCheckBox);
	QCheckBox * checkBox = (QCheckBox *)ui.tableWidget->cellWidget(0, 0);
	checkBox->setChecked(true);
	QObject::connect(checkBox, SIGNAL(clicked()), signalMapper, SLOT(map()));
	signalMapper->setMapping(checkBox, 0);
	ui.tableWidget->setCellWidget(0, 1, new QLabel(modelView->model_name.c_str()));
	QLabel * label_name = (QLabel *)ui.tableWidget->cellWidget(0, 1);
	label_name->setToolTip(tr(modelView->model_name.c_str()));
	ui.tableWidget->setCellWidget(0, 2, new QLabel("  " + QString::number(mesh->numVertices())));
	ui.tableWidget->setCellWidget(0, 3, new QLabel("  " + QString::number(mesh->numFaces())));
	m_viewer->setModelView(modelView);
	ui.checkbox->setChecked(true);
}

void MyWindow::import()
{
	QFileDialog dialog(this, tr("Load File"));
	dialog.setFileMode(QFileDialog::ExistingFiles);
	dialog.setNameFilter(trUtf8("Model Files (*.m *.ply *.obj)"));
	QStringList filenames;
	if (dialog.exec())
		filenames = dialog.selectedFiles();
	for (int i = 0; i < filenames.size(); i++) {
		QString filename = filenames[i];
		if(filename.isNull())
			return;
		QString suffix = filename.section('.', -1);
		SimMesh * mesh = new SimMesh;
		ModelView * modelView = new ModelView;
		modelView->theMesh = mesh;
		if (suffix == "m" && !MeshIO::ReadM(filename.toLocal8Bit().data(), *modelView))
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
		modelView->model_name = filename.section('/', -1).toLocal8Bit().data();
		ui.tableWidget->insertRow(m_viewer->getModelViewSize());
		ui.tableWidget->setCellWidget(m_viewer->getModelViewSize(), 0, new QCheckBox);
		QCheckBox * checkBox = (QCheckBox *)ui.tableWidget->cellWidget(m_viewer->getModelViewSize(), 0);
		checkBox->setChecked(true);
		QObject::connect(checkBox, SIGNAL(clicked()), signalMapper, SLOT(map()));
		signalMapper->setMapping(checkBox, m_viewer->getModelViewSize());
		ui.tableWidget->setCellWidget(m_viewer->getModelViewSize(), 1, new QLabel(tr(modelView->model_name.c_str())));
		QLabel * label_name = (QLabel *)ui.tableWidget->cellWidget(m_viewer->getModelViewSize(), 1);
		label_name->setToolTip(tr(modelView->model_name.c_str()));
		ui.tableWidget->setCellWidget(m_viewer->getModelViewSize(), 2, new QLabel("  " + QString::number(mesh->numVertices())));
		ui.tableWidget->setCellWidget(m_viewer->getModelViewSize(), 3, new QLabel("  " + QString::number(mesh->numFaces())));
		if (m_viewer->getModelViewSize() == 0)
			m_viewer->setModelView(modelView);
		else
			m_viewer->addModelView(modelView);
	}
	ui.checkbox->setChecked(true);
}

void MyWindow::save()
{
	if (!m_viewer || !m_viewer->getModelViewSize())
	{
		QMessageBox msgBox;
		msgBox.setText("No mesh exist!");
		msgBox.exec();
		return;
	}
	bool ok = true;
	int idx = 1;
	if (m_viewer->getModelViewSize() >= 2)
		QInputDialog::getInt(this, tr("Enter mesh ID"), tr("ID:"), 0, 1, m_viewer->getModelViewSize(), 1, &ok);
	if (ok)
	{
		ModelView *theModelView = m_viewer->getModelView(idx - 1);
		QString filename = QFileDialog::getSaveFileName(this, tr("Save File"),
			getenv("HOME"),
			tr("Model Files (*.m *.obj *.ply)"));
		QString suffix = filename.section('.', -1);
		if (suffix == "m" && !MeshIO::WriteM(filename.toLocal8Bit().data(), *theModelView))
		{
			std::cout<<"Fail to read the File!"<<std::endl;
			return ;
		}
		if (suffix == "obj" && !MeshIO::WriteOBJ(filename.toLocal8Bit().data(), *theModelView->theMesh))
		{
			std::cout<<"Fail to read the File!"<<std::endl;
			return ;
		}
		if (suffix == "ply" && !MeshIO::ReadPLY(filename.toLocal8Bit().data(), *theModelView))
		{
			std::cout<<"Fail to read the File!"<<std::endl;
			return ;
		}
	}
}

void MyWindow::showWireFrame(bool show)
{
	m_viewer->ShowWireFrame(show);
}

void MyWindow::showMeshChanged(int row)
{
	QCheckBox * checkBox = (QCheckBox *)ui.tableWidget->cellWidget(row, 0);
	m_viewer->setShowMesh(row, checkBox->isChecked());
}

void MyWindow::showMeshChanged() 
{
	for (int i = 0; i < ui.tableWidget->rowCount(); i++)
	{
		QCheckBox * checkBox = (QCheckBox *)ui.tableWidget->cellWidget(i, 0);
		checkBox->setChecked(ui.checkbox->isChecked());
		m_viewer->setShowMesh(i, checkBox->isChecked());
	}
}