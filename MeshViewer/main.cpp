#include "meshviewer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	QApplication a(argc, argv);
	MeshViewer w;
	w.show();


	return a.exec();
}
