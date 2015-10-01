#ifndef MESHVIEWER_H
#define MESHVIEWER_H

#include <QtWidgets/QMainWindow>
#include <QSignalMapper>
#include "ui_meshviewer.h"

class MeshViewer : public QMainWindow
{
	Q_OBJECT

public:
	MeshViewer(QWidget *parent = 0);
	~MeshViewer();

private:
	Ui::MeshViewerClass ui;
	QSignalMapper signalMapper;

	void connectSlot();

	private slots:
		void load();
		void import();
		void save();
		void showWireFrame(bool show);
		void showMeshChanged(int row);
		void showMeshChanged();
		void setChosenElement();
};

#endif // MESHVIEWER_H
