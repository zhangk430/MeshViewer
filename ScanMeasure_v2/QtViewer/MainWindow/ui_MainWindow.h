/********************************************************************************
** Form generated from reading UI file 'mainwindowXd9240.ui'
**
** Created: Tue Jun 11 19:13:07 2013
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MAINWINDOWXD9240_H
#define MAINWINDOWXD9240_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "../PickingViewer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
	QAction *actionOpen;
	QAction *actionSave;
	QAction *actionLoad_Marker;
	QAction *actionSave_Marker;
	QAction *actionWireframe;
	QWidget *centralwidget;
	QHBoxLayout *horizontalLayout;
	PickingViewer *DisplayWidget;
	QVBoxLayout *verticalLayout;
	QPushButton *pushButton;
	QPushButton *pushButton2;
	QMenuBar *menubar;
	QMenu *menuFile;
	QMenu *menuEdit;
	QMenu *menuShading;
	QStatusBar *statusbar;

	void setupUi(QMainWindow *MainWindow)
	{
		if (MainWindow->objectName().isEmpty())
			MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
		MainWindow->resize(800, 612);
		actionOpen = new QAction(MainWindow);
		actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
		actionSave = new QAction(MainWindow);
		actionSave->setObjectName(QString::fromUtf8("actionSave"));
		actionLoad_Marker = new QAction(MainWindow);
		actionLoad_Marker->setObjectName(QString::fromUtf8("actionLoad_Marker"));
		actionSave_Marker = new QAction(MainWindow);
		actionSave_Marker->setObjectName(QString::fromUtf8("actionSave_Marker"));
		actionWireframe = new QAction(MainWindow);
		actionWireframe->setObjectName(QString::fromUtf8("actionWireframe"));
		actionWireframe->setCheckable(true);
		centralwidget = new QWidget(MainWindow);
		centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
		horizontalLayout = new QHBoxLayout(centralwidget);
		horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
		horizontalLayout->setContentsMargins(0, 0, 0, 0);
		DisplayWidget = new PickingViewer;
		DisplayWidget->setObjectName(QString::fromUtf8("DisplayWidget"));

		horizontalLayout->addWidget(DisplayWidget);

		verticalLayout = new QVBoxLayout;
		verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
		verticalLayout->setContentsMargins(20, 50, 20, -1);
		pushButton = new QPushButton;
		pushButton->setObjectName(QString::fromUtf8("traceSPButton"));
		pushButton->setFixedSize(135, 50);
		pushButton2 = new QPushButton;
		pushButton2->setObjectName(QString::fromUtf8("saveMeasurement"));
		pushButton2->setFixedSize(135, 50);

		verticalLayout->setAlignment(Qt::AlignTop);	
		verticalLayout->setSpacing(50);
		verticalLayout->addWidget(pushButton);
		verticalLayout->addWidget(pushButton2);


		horizontalLayout->addLayout(verticalLayout);

		MainWindow->setCentralWidget(centralwidget);
		menubar = new QMenuBar(MainWindow);
		menubar->setObjectName(QString::fromUtf8("menubar"));
		menubar->setGeometry(QRect(0, 0, 800, 21));
		menuFile = new QMenu(menubar);
		menuFile->setObjectName(QString::fromUtf8("menuFile"));
		menuEdit = new QMenu(menubar);
		menuEdit->setObjectName(QString::fromUtf8("menuEdit"));
		menuShading = new QMenu(menubar);
		menuShading->setObjectName(QString::fromUtf8("menuShading"));
		MainWindow->setMenuBar(menubar);
		statusbar = new QStatusBar(MainWindow);
		statusbar->setObjectName(QString::fromUtf8("statusbar"));
		MainWindow->setStatusBar(statusbar);

		menubar->addAction(menuFile->menuAction());
		menubar->addAction(menuEdit->menuAction());
		menubar->addAction(menuShading->menuAction());
		menuFile->addAction(actionOpen);
		menuFile->addAction(actionSave);
		menuFile->addAction(actionLoad_Marker);
		menuFile->addAction(actionSave_Marker);
		menuShading->addAction(actionWireframe);

		retranslateUi(MainWindow);

		QMetaObject::connectSlotsByName(MainWindow);
	} // setupUi

	void retranslateUi(QMainWindow *MainWindow)
	{
		MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
		actionOpen->setText(QApplication::translate("MainWindow", "Open...", 0, QApplication::UnicodeUTF8));
		actionSave->setText(QApplication::translate("MainWindow", "Save...", 0, QApplication::UnicodeUTF8));
		actionLoad_Marker->setText(QApplication::translate("MainWindow", "Load Marker...", 0, QApplication::UnicodeUTF8));
		actionSave_Marker->setText(QApplication::translate("MainWindow", "Save Marker...", 0, QApplication::UnicodeUTF8));
		actionWireframe->setText(QApplication::translate("MainWindow", "Wireframe", 0, QApplication::UnicodeUTF8));
		pushButton->setText(QApplication::translate("MainWindow", "Measuring (M)", 0, QApplication::UnicodeUTF8));
		pushButton2->setText(QApplication::translate("MainWindow", "Save Measurement", 0, QApplication::UnicodeUTF8));
		menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
		menuEdit->setTitle(QApplication::translate("MainWindow", "Edit", 0, QApplication::UnicodeUTF8));
		menuShading->setTitle(QApplication::translate("MainWindow", "Shading", 0, QApplication::UnicodeUTF8));
	} // retranslateUi

};

namespace Ui {
	class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MAINWINDOWXD9240_H
