#include <SimMesh.h>
#include <MeshIO.h>
#include <MainWindow/MainWindow.h>

bool ParseMesh(SimMesh * mesh, const char filename[])
{
	QString qtfilename(filename); 
	if(qtfilename.isNull())
		return false;
	QString suffix = qtfilename.section('.', -1);
	if (suffix == "m")
		if (!MeshIO::ReadM(qtfilename.toLocal8Bit().data(), *mesh))
		{
			std::cout<<"Fail to read the File!"<<std::endl;
			return false;
		}
		else return true;
	else if (suffix == "obj")
		if (!MeshIO::ReadOBJ(qtfilename.toLocal8Bit().data(), *mesh))
		{
			std::cout<<"Fail to read the File!"<<std::endl;
			return false;
		}
		else return true;
	else if (suffix == "ply")
		if (!MeshIO::ReadPLY(qtfilename.toLocal8Bit().data(), *mesh))
		{
			std::cout<<"Fail to read the File!"<<std::endl;
			return false;
		}
		else
			return true;
	else return false;
}


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