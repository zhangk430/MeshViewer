#include "QtViewer/MainWindow/MainWindow.h"


int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitContextVersion(3, 2);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(
		GLUT_ACTION_ON_WINDOW_CLOSE,
		GLUT_ACTION_GLUTMAINLOOP_RETURNS
		);

	QApplication myapp(argc, argv);

	MyWindow *mw = new MyWindow;


	mw->show();
	return myapp.exec();
}