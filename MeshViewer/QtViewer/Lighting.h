#ifndef LIGHTING_H
#define LIGHTING_H

class Light
{

public:
	Light(){
		ambientLight[0] = ambientLight[1] = ambientLight[2] = 0.2f;
		ambientLight[3] = 1;			
		diffuseLight[0] = diffuseLight[1] = diffuseLight[2] = 0.8f;
		diffuseLight[3] = 1;
		specularLight[0] = specularLight[1] = specularLight[2] = 0.4f;
		specularLight[3] = 1;
		position[0] = position[1] = 0;
		position[2] = 10;
		position[3] = 1;
		isEnable = true;
	}

	void SetLightPosition(float x, float y, float z){
		position[0]=x; position[1]=y; position[2]=z;
	}

	//void SetLight(){
	//	// Assign created components to GL_LIGHT0
	//	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	//	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	//	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
	//	glLightfv(GL_LIGHT0, GL_POSITION, position);
	//}
	
	// Create light components
	float ambientLight[4];
	float	diffuseLight[4];
	float specularLight[4];
	float position[4];  

	bool isEnable;



public:
protected:
private:

};



#endif