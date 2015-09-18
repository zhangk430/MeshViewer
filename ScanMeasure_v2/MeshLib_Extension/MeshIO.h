#ifndef MESHIO_H
#define MESHIO_H

#include <ModelView.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <assert.h>


class MeshIO{
public:
	static bool ReadOBJ(const char inputFile[], SimMesh & cMesh);
	static bool ReadOBJ(const char inputFile[], ModelView & modelView);
	static bool ReadM(const char inputFile[], SimMesh & cMesh);
	static bool ReadPLY(const char inputFile[], SimMesh & cMesh);
	static bool ReadPLY(const char inputFile[], ModelView & modelView);
	static bool ReadPT(const char inputFile[], ModelView & modelView);
	static bool WriteOBJ(const char outputFile[], SimMesh & cMesh);
	static bool WritePLY(const char outputFile[], SimMesh & cMesh);
	static bool WriteM(const char outputFile[], SimMesh & cMesh);
	static bool WritePT(const char outputFile[], ModelView & modelView);
};

#endif
