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
	ui.tableWidget->setColumnCount(4);
	ui.tableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem("s"));
	ui.tableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem("name"));
	ui.tableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("vn"));
	ui.tableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem("fn"));
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
	QObject::connect(ui.widget, SIGNAL(picked()), this, SLOT(setChosenElement()));
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

void MeshViewer::setChosenElement()
{
	const SelectedObject& selectedObject = ui.widget->getSelectedObjecct();
	ui.tableWidget_2->setRowCount(0);
	if (selectedObject.mode == SelectedObject::Vertex_Mode)
	{
		ui.tabWidget->setTabText(ui.tabWidget->indexOf(ui.tab_2), "Chosen Vertices");
		ui.tableWidget_2->setColumnCount(4);
		ui.tableWidget_2->setHorizontalHeaderItem(0, new QTableWidgetItem("vid"));
		ui.tableWidget_2->setHorizontalHeaderItem(1, new QTableWidgetItem("coord_x"));
		ui.tableWidget_2->setHorizontalHeaderItem(2, new QTableWidgetItem("coord_y"));
		ui.tableWidget_2->setHorizontalHeaderItem(3, new QTableWidgetItem("coord_z"));
		ui.tableWidget_2->setColumnWidth(0, 60);
		ui.tableWidget_2->setColumnWidth(1, 80);
		ui.tableWidget_2->setColumnWidth(2, 80);
		ui.tableWidget_2->setColumnWidth(3, 80);
		std::vector<SimVertex *> vertices;
		selectedObject.orderedSeletedVertex(vertices);
		for (size_t i = 0; i < vertices.size(); i++)
		{
			ui.tableWidget_2->insertRow(i);
			ui.tableWidget_2->setCellWidget(i, 0, new QLabel(" " + QString::number(vertices[i]->idx)));
			ui.tableWidget_2->setCellWidget(i, 1, new QLabel(" " + QString::number(vertices[i]->p[0])));
			ui.tableWidget_2->setCellWidget(i, 2, new QLabel(" " + QString::number(vertices[i]->p[1])));
			ui.tableWidget_2->setCellWidget(i, 3, new QLabel(" " + QString::number(vertices[i]->p[2])));
		}
	}
	else if (selectedObject.mode == SelectedObject::Edge_Mode)
	{
		ui.tabWidget->setTabText(ui.tabWidget->indexOf(ui.tab_2), "Chosen Edges");
		ui.tableWidget_2->setColumnCount(3);
		ui.tableWidget_2->setHorizontalHeaderItem(0, new QTableWidgetItem("eid"));
		ui.tableWidget_2->setHorizontalHeaderItem(1, new QTableWidgetItem("v0 id"));
		ui.tableWidget_2->setHorizontalHeaderItem(2, new QTableWidgetItem("v1 id"));
		ui.tableWidget_2->setColumnWidth(0, 80);
		ui.tableWidget_2->setColumnWidth(1, 80);
		ui.tableWidget_2->setColumnWidth(2, 80);
		std::vector<SimEdge *> edges;
		selectedObject.orderedSeletedEdge(edges);
		for (size_t i = 0; i < edges.size(); i++)
		{
			ui.tableWidget_2->insertRow(i);
			ui.tableWidget_2->setCellWidget(i, 0, new QLabel(" " + QString::number(edges[i]->idx)));
			ui.tableWidget_2->setCellWidget(i, 1, new QLabel(" " + QString::number(edges[i]->v0->idx)));
			ui.tableWidget_2->setCellWidget(i, 2, new QLabel(" " + QString::number(edges[i]->v1->idx)));
		}
	}
	else if (selectedObject.mode == SelectedObject::Face_Mode)
	{
		ui.tabWidget->setTabText(ui.tabWidget->indexOf(ui.tab_2), "Chosen Faces");
		ui.tableWidget_2->setColumnCount(4);
		ui.tableWidget_2->setHorizontalHeaderItem(0, new QTableWidgetItem("fid"));
		ui.tableWidget_2->setHorizontalHeaderItem(1, new QTableWidgetItem("v0 id"));
		ui.tableWidget_2->setHorizontalHeaderItem(2, new QTableWidgetItem("v1 id"));
		ui.tableWidget_2->setHorizontalHeaderItem(3, new QTableWidgetItem("v2 id"));
		ui.tableWidget_2->setColumnWidth(0, 60);
		ui.tableWidget_2->setColumnWidth(1, 60);
		ui.tableWidget_2->setColumnWidth(2, 60);
		ui.tableWidget_2->setColumnWidth(3, 60);
		std::vector<SimFace *> faces;
		selectedObject.orderedSeletedFace(faces);
		for (size_t i = 0; i < faces.size(); i++)
		{
			ui.tableWidget_2->insertRow(i);
			ui.tableWidget_2->setCellWidget(i, 0, new QLabel(" " + QString::number(faces[i]->idx)));
			ui.tableWidget_2->setCellWidget(i, 1, new QLabel(" " + QString::number(faces[i]->ver[0]->idx)));
			ui.tableWidget_2->setCellWidget(i, 2, new QLabel(" " + QString::number(faces[i]->ver[1]->idx)));
			ui.tableWidget_2->setCellWidget(i, 3, new QLabel(" " + QString::number(faces[i]->ver[2]->idx)));
		}
	}
	else
	{
		ui.tabWidget->setTabText(ui.tabWidget->indexOf(ui.tab_2), "Chosen Elements");
		ui.tableWidget_2->setColumnCount(0);
	}

}