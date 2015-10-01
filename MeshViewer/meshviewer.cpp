#include "meshviewer.h"
#include "MeshLib_Extension/MeshIO.h"
#include <QFileDialog>
#include <QLabel>
#include <QInputDialog>
#include <QMessageBox>

MeshViewer::MeshViewer(QWidget *parent)
	: QMainWindow(parent), signalMapper(this)
{
	ui.setupUi(this);
	connectSlot();
	ui.tableWidget->setColumnWidth(0, 20);
	ui.tableWidget->setColumnWidth(1, 100);
	ui.tableWidget->setColumnWidth(2, 60);
	ui.tableWidget->setColumnWidth(3, 60);
}

MeshViewer::~MeshViewer()
{

}

void MeshViewer::connectSlot() 
{
	QObject::connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(load()));
	QObject::connect( ui.actionImport,  SIGNAL(triggered()), this, SLOT(import()) );
	QObject::connect( ui.actionSave,  SIGNAL(triggered()), this, SLOT(save()) );
	QObject::connect( &signalMapper,  SIGNAL(mapped(int)), this, SLOT(showMeshChanged(int)) );
	QObject::connect( ui.checkBox,  SIGNAL(clicked()), this, SLOT(showMeshChanged()) );
	QObject::connect( ui.actionWireFrame,  SIGNAL(toggled(bool)), this, SLOT(showWireFrame(bool)) );
}

void MeshViewer::load() 
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
	QCheckBox * checkbox_table = (QCheckBox *)ui.tableWidget->cellWidget(0, 0);
	checkbox_table->setChecked(true);
	QObject::connect(checkbox_table, SIGNAL(clicked()), &signalMapper, SLOT(map()));
	signalMapper.setMapping(checkbox_table, 0);
	ui.tableWidget->setCellWidget(0, 1, new QLabel(modelView->model_name.c_str()));
	QLabel * label_name = (QLabel *)ui.tableWidget->cellWidget(0, 1);
	label_name->setToolTip(tr(modelView->model_name.c_str()));
	ui.tableWidget->setCellWidget(0, 2, new QLabel("  " + QString::number(mesh->numVertices())));
	ui.tableWidget->setCellWidget(0, 3, new QLabel("  " + QString::number(mesh->numFaces())));
	ui.widget->setModelView(modelView);
	ui.checkBox->setChecked(true);
}
void MeshViewer::import()
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
		ui.tableWidget->insertRow(ui.widget->getModelViewSize());
		ui.tableWidget->setCellWidget(ui.widget->getModelViewSize(), 0, new QCheckBox);
		QCheckBox * checkBox = (QCheckBox *)ui.tableWidget->cellWidget(ui.widget->getModelViewSize(), 0);
		checkBox->setChecked(true);
		QObject::connect(checkBox, SIGNAL(clicked()), &signalMapper, SLOT(map()));
		signalMapper.setMapping(checkBox, ui.widget->getModelViewSize());
		ui.tableWidget->setCellWidget(ui.widget->getModelViewSize(), 1, new QLabel(tr(modelView->model_name.c_str())));
		QLabel * label_name = (QLabel *)ui.tableWidget->cellWidget(ui.widget->getModelViewSize(), 1);
		label_name->setToolTip(tr(modelView->model_name.c_str()));
		ui.tableWidget->setCellWidget(ui.widget->getModelViewSize(), 2, new QLabel("  " + QString::number(mesh->numVertices())));
		ui.tableWidget->setCellWidget(ui.widget->getModelViewSize(), 3, new QLabel("  " + QString::number(mesh->numFaces())));
		if (ui.widget->getModelViewSize() == 0)
			ui.widget->setModelView(modelView);
		else
			ui.widget->addModelView(modelView);
	}
	ui.checkBox->setChecked(true);
}

void MeshViewer::save()
{
	if (!ui.widget || !ui.widget->getModelViewSize())
	{
		QMessageBox msgBox;
		msgBox.setText("No mesh exist!");
		msgBox.exec();
		return;
	}
	bool ok = true;
	int idx = 1;
	if (ui.widget->getModelViewSize() >= 2)
		QInputDialog::getInt(this, tr("Enter mesh ID"), tr("ID:"), 0, 1, ui.widget->getModelViewSize(), 1, &ok);
	if (ok)
	{
		ModelView *theModelView = ui.widget->getModelView(idx - 1);
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

void MeshViewer::showWireFrame(bool show)
{
	ui.widget->ShowWireFrame(show);
}

void MeshViewer::showMeshChanged(int row)
{
	QCheckBox * checkBox = (QCheckBox *)ui.tableWidget->cellWidget(row, 0);
	ui.widget->setShowMesh(row, checkBox->isChecked());
}

void MeshViewer::showMeshChanged() 
{
	for (int i = 0; i < ui.tableWidget->rowCount(); i++)
	{
		QCheckBox * checkBox = (QCheckBox *)ui.tableWidget->cellWidget(i, 0);
		checkBox->setChecked(ui.checkBox->isChecked());
		ui.widget->setShowMesh(i, checkBox->isChecked());
	}
}