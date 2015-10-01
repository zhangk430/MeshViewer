/********************************************************************************
** Form generated from reading UI file 'meshviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MESHVIEWER_H
#define UI_MESHVIEWER_H

#include <QtCore/QVariant>
#include <QtViewer/PickingViewer.h>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MeshViewerClass
{
public:
    QAction *actionOpen;
    QAction *actionImport;
    QAction *actionSave;
    QAction *actionWireFrame;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    PickingViewer *widget;
    QVBoxLayout *verticalLayout;
    QTableWidget *tableWidget;
    QCheckBox *checkBox;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuShading;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MeshViewerClass)
    {
        if (MeshViewerClass->objectName().isEmpty())
            MeshViewerClass->setObjectName(QStringLiteral("MeshViewerClass"));
        MeshViewerClass->resize(1162, 825);
        actionOpen = new QAction(MeshViewerClass);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionImport = new QAction(MeshViewerClass);
        actionImport->setObjectName(QStringLiteral("actionImport"));
        actionSave = new QAction(MeshViewerClass);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionWireFrame = new QAction(MeshViewerClass);
        actionWireFrame->setObjectName(QStringLiteral("actionWireFrame"));
        actionWireFrame->setCheckable(true);
        centralWidget = new QWidget(MeshViewerClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        widget = new PickingViewer(centralWidget);
        widget->setObjectName(QStringLiteral("widget"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy1);

        horizontalLayout->addWidget(widget);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(20);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        verticalLayout->setContentsMargins(-1, -1, 0, -1);
        tableWidget = new QTableWidget(centralWidget);
        if (tableWidget->columnCount() < 4)
            tableWidget->setColumnCount(4);
        tableWidget->setObjectName(QStringLiteral("tableWidget"));
        tableWidget->setEnabled(true);
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(tableWidget->sizePolicy().hasHeightForWidth());
        tableWidget->setSizePolicy(sizePolicy2);
        tableWidget->setMaximumSize(QSize(240, 16777215));
        tableWidget->setColumnCount(4);
        tableWidget->horizontalHeader()->setVisible(true);
        tableWidget->horizontalHeader()->setCascadingSectionResizes(true);
        tableWidget->horizontalHeader()->setDefaultSectionSize(50);
        tableWidget->horizontalHeader()->setHighlightSections(false);
        tableWidget->horizontalHeader()->setMinimumSectionSize(20);
        tableWidget->verticalHeader()->setVisible(true);
        tableWidget->verticalHeader()->setCascadingSectionResizes(true);
        tableWidget->verticalHeader()->setDefaultSectionSize(37);
        tableWidget->verticalHeader()->setHighlightSections(false);
        tableWidget->verticalHeader()->setMinimumSectionSize(30);

        verticalLayout->addWidget(tableWidget);

        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        sizePolicy2.setHeightForWidth(checkBox->sizePolicy().hasHeightForWidth());
        checkBox->setSizePolicy(sizePolicy2);
        checkBox->setChecked(true);

        verticalLayout->addWidget(checkBox, 0, Qt::AlignHCenter|Qt::AlignTop);


        horizontalLayout->addLayout(verticalLayout);

        MeshViewerClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MeshViewerClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1162, 26));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuShading = new QMenu(menuBar);
        menuShading->setObjectName(QStringLiteral("menuShading"));
        MeshViewerClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MeshViewerClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MeshViewerClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MeshViewerClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MeshViewerClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuShading->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionImport);
        menuFile->addAction(actionSave);
        menuShading->addAction(actionWireFrame);

        retranslateUi(MeshViewerClass);

        QMetaObject::connectSlotsByName(MeshViewerClass);
    } // setupUi

    void retranslateUi(QMainWindow *MeshViewerClass)
    {
        MeshViewerClass->setWindowTitle(QApplication::translate("MeshViewerClass", "MeshViewer", 0));
        actionOpen->setText(QApplication::translate("MeshViewerClass", "Open", 0));
        actionImport->setText(QApplication::translate("MeshViewerClass", "Import", 0));
        actionSave->setText(QApplication::translate("MeshViewerClass", "Save", 0));
        actionWireFrame->setText(QApplication::translate("MeshViewerClass", "WireFrame", 0));
        checkBox->setText(QApplication::translate("MeshViewerClass", "CheckBox", 0));
        menuFile->setTitle(QApplication::translate("MeshViewerClass", "File", 0));
        menuShading->setTitle(QApplication::translate("MeshViewerClass", "Shading", 0));
    } // retranslateUi

};

namespace Ui {
    class MeshViewerClass: public Ui_MeshViewerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MESHVIEWER_H