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
		void Export();
		void showWireFrame(bool show);
		void showMeshChanged(int row);
		void showMeshChanged();
		void showChosenID(bool show);
		void setChosenElement();
		void traceShortestCurve();
		void traceShortestLoop();
};

#endif // MESHVIEWER_H
