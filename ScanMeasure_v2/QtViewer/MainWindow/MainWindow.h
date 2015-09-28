#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include "ui_MainWindow.h"
#include <QtCore/QSignalMapper>


class MyWindow : public QMainWindow
{

	Q_OBJECT

private:
	Ui::MainWindow ui;
	PickingViewer * m_viewer;
	QSignalMapper *signalMapper;
public:
	MyWindow();
	~MyWindow();

private:

	void connectSlot();

	private slots:


		void load();
		void import();
		void showWireFrame(bool show);
		void showMeshChanged(int row);
		void showMeshChanged();

};



#endif