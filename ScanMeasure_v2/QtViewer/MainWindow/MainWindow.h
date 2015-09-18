#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include "ui_MainWindow.h"


class MyWindow : public QMainWindow
{

	Q_OBJECT

private:
	Ui::MainWindow ui;
	QtViewer * m_viewer;
public:
	MyWindow();
	~MyWindow();

	void setModelView(ModelView * modelView);

private:

	void connectSlot();

	private slots:


		void load();
//		void loadMarker();
//		void saveMarker();
		void showWireFrame(bool show);
//		void measure();
//		void saveMeasurement();

};



#endif