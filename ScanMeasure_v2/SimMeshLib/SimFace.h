#ifndef SIMFACE_H
#define SIMFACE_H

#include <SimVertex.h>

class SimFace
{
public:
	SimVertex * ver[3];
	int idx;
};


#endif

