#include "QtViewer.h"


void QtViewer::SetCameraFromModelView()
{
	ModelView *modelView = getModelView(0);
	if (modelView)
	{
		theCamera.position = modelView->center; 
		theCamera.position[2] += modelView->boxAxisLen * 3;
		theCamera.center = modelView->center;
		theCamera.boxMin = modelView->boxMin;
		theCamera.boxMax = modelView->boxMax;		
		theCamera.translation = Point();
	}
	theCamera.boxAxisLen=(theCamera.boxMax-theCamera.boxMin).norm();
}

mat4 QtViewer::getModelViewMatrix()
{

	//old OpenGL method
	/*glTranslatef(theCamera.center[0]-theCamera.translation[0], 
		theCamera.center[1]-theCamera.translation[1], 
		theCamera.center[2]-theCamera.translation[2]);	
	GLfloat mat[16];
	QMatrix4x4 m;
	m.rotate(m_Trackball.rotation());
	const qreal *data = m.constData();
	for (int index = 0; index < 16; ++index)
		mat[index] = data[index];
	glMultMatrixf(mat);
	glTranslatef(-theCamera.center[0], -theCamera.center[1], -theCamera.center[2]);*/

	vec4 eye(theCamera.position[0], theCamera.position[1], theCamera.position[2], 1.0f);
	vec4 at(theCamera.center[0], theCamera.center[1], theCamera.center[2], 1.0f);
	vec4 up(0.0f, 1.0f, 0.0f, 1.0f);
	mat4 trans = LookAt(eye, at, up);
	mat4 rot;
	QMatrix4x4 m;
	m.rotate(m_Trackball.rotation());
	const qreal *data = m.constData();
	for (int index = 0; index < 16; ++index)
		rot[index % 4][index / 4] = data[index];
	trans = trans * Translate(theCamera.center[0] - theCamera.translation[0], theCamera.center[1] - theCamera.translation[1], 
		theCamera.center[2] - theCamera.translation[2]) * rot *
		Translate(-theCamera.center[0], -theCamera.center[1], -theCamera.center[2]);
	return trans;
}

mat4 QtViewer::getProjectionMatrix() {
	const GLfloat a = GLfloat(windowWidth) / GLfloat(windowHeight);
	return Perspective(45.f, a, 0.1f, 1e5);
}

mat4 QtViewer::getDepthMatrix() {
	vec4 eye(light0.position[0], light0.position[1], light0.position[2] + theCamera.boxAxisLen / 2, 1.0f);
	vec4 at(theCamera.center[0], theCamera.center[1], theCamera.center[2], 1.0f);
	vec4 up(0.0f, 1.0f, 0.0f, 1.0f);
	mat4 trans = LookAt(eye, at, up);
	mat4 rot;
	QMatrix4x4 m;
	m.rotate(m_Trackball.rotation());
	const qreal *data = m.constData();
	for (int index = 0; index < 16; ++index)
		rot[index % 4][index / 4] = data[index];
	trans = trans * Translate(theCamera.center[0] - theCamera.translation[0], theCamera.center[1] - theCamera.translation[1], 
		theCamera.center[2] - theCamera.translation[2]) * rot *
		Translate(-theCamera.center[0], -theCamera.center[1], -theCamera.center[2]);
	mat4 depthProjectionMatrix = Ortho(theCamera.center[0] - 6 * theCamera.boxAxisLen, theCamera.center[0] + 6 * theCamera.boxAxisLen,
		theCamera.center[1] - 6 * theCamera.boxAxisLen, theCamera.center[1] + 6 * theCamera.boxAxisLen,
		theCamera.center[2] - 6 * theCamera.boxAxisLen, theCamera.center[2] + 6 * theCamera.boxAxisLen);
	//mat4 biasMatrix(0.5, 0.0, 0.0, 0.0,
	//	0.0, 0.5, 0.0, 0.0,
	//	0.0, 0.0, 0.5, 0.0, 
	//	0.5, 0.5, 0.5, 1.0);
	//depthProjectionMatrix = biasMatrix * depthProjectionMatrix;
	return depthProjectionMatrix * trans;
}

void QtViewer::paintGL(){	
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//Render Shadow
	if (!theModelView.size())
		return;
	mat4 biasMatrix(0.5, 0.0, 0.0, 0.0,
			0.0, 0.5, 0.0, 0.0,
			0.0, 0.0, 0.5, 0.0, 
			0.5, 0.5, 0.5, 1.0);
	glBindFramebuffer(GL_FRAMEBUFFER, frameBufferName);

	glViewport (0, 0, 1920, 1080);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	glUseProgram(shadowShaderProgram.shaderProgram);
	glUniformMatrix4fv(shadowShaderProgram.depthMatrixUniform, 1, GL_TRUE, getDepthMatrix());
	for (int i = 0; i < theModelView.size(); i++)
		Render_Mesh(i);
	drawFloor();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glUseProgram(0);
	glViewport(0, 0, windowWidth, windowHeight);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	

//	GLuint quad_programID = InitShader( "Passthrough.vertexshader", "SimpleTexture.fragmentshader", "color" );
//	GLuint texID = glGetUniformLocation(quad_programID, "texture");
//	GLuint shadowtexID = glGetUniformLocation(quad_programID, "shadowMap");
//	glUseProgram(quad_programID);
//
//	// Bind our texture in Texture Unit 0
//	glActiveTexture(GL_TEXTURE2);
//	glBindTexture(GL_TEXTURE_2D, m_bufferData[0]->Ka_texture);
//			glActiveTexture(GL_TEXTURE1);
//			glBindTexture(GL_TEXTURE_2D, m_bufferData[0]->Kd_texture);
//			glActiveTexture(GL_TEXTURE3);
//			glBindTexture(GL_TEXTURE_2D, m_bufferData[0]->Ks_texture);
//	glActiveTexture(GL_TEXTURE0);
//	glBindTexture(GL_TEXTURE_2D, depthTexture);
//
////	std::cout << depthTexture << " " << m_bufferData[0]->Ka_texture << " " << m_bufferData[0]->Kd_texture << " " << m_bufferData[0]->Ks_texture << " " << std::endl;
//
//	// Set our "renderedTexture" sampler to user Texture Unit 0
//	glUniform1i(texID, 1);
//	glUniform1i(shadowtexID, 0);
//
//	static const GLfloat g_quad_vertex_buffer_data[] = { 
//		-1.0f, -1.0f, 0.0f,
//		1.0f, -1.0f, 0.0f,
//		-1.0f,  1.0f, 0.0f,
//		-1.0f,  1.0f, 0.0f,
//		1.0f, -1.0f, 0.0f,
//		1.0f,  1.0f, 0.0f,
//	};
//
//	GLuint quad_vertexbuffer;
//	glGenBuffers(1, &quad_vertexbuffer);
//	glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
//	glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);
//	glEnableVertexAttribArray(0);
//	glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
//	glVertexAttribPointer(
//		0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
//		3,                  // size
//		GL_FLOAT,           // type
//		GL_FALSE,           // normalized?
//		0,                  // stride
//		(void*)0            // array buffer offset
//		);
//
//	glDrawArrays(GL_TRIANGLES, 0, 6);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glUniform1i(colorShaderProgram.shadowMap, 0);

	//Render Object
	vec4 lightPosition(light0.position[0], light0.position[1], light0.position[2], light0.position[3]);
	vec4 ambientLightIntensity(light0.ambientLight[0], light0.ambientLight[1], light0.ambientLight[2], light0.ambientLight[3]);
	vec4 diffuseLightIntensity(light0.diffuseLight[0], light0.diffuseLight[1], light0.diffuseLight[2], light0.diffuseLight[3]);
	vec4 specularLightIntensity(light0.specularLight[0], light0.specularLight[1], light0.specularLight[2], light0.specularLight[3]);
	vec4 cameraPosition(theCamera.position[0], theCamera.position[1], theCamera.position[2], 1.0f);
	Shader * shaderProgram = NULL;
	glPolygonOffset(1.0, 1.0);
	for (int i = 0; i < theModelView.size(); i++) {
		if (!showMesh[i])
			continue;
		if (theModelView[i]->theTexture)
			shaderProgram = &textureShadedrProgram;
		else 
			shaderProgram = &colorShaderProgram;
		glUseProgram(shaderProgram->shaderProgram);
		glUniformMatrix4fv(shaderProgram->projectionMatrixUniform, 1, GL_TRUE, getProjectionMatrix());
		glUniformMatrix4fv(shaderProgram->modelViewMatrixUniform, 1, GL_TRUE, getModelViewMatrix());
		glUniformMatrix4fv(shaderProgram->depthMatrixUniform, 1, GL_TRUE, biasMatrix * getDepthMatrix());
		glUniform4fv(shaderProgram->lightPositionUniform, 1, (const GLfloat *)&lightPosition);
		glUniform4fv(shaderProgram->ambientLight, 1, (const GLfloat *)&ambientLightIntensity);
		glUniform4fv(shaderProgram->diffuseLight, 1, (const GLfloat *)&diffuseLightIntensity);
		glUniform4fv(shaderProgram->specularLight, 1, (const GLfloat *)&specularLightIntensity);
		glUniform4fv(shaderProgram->cameraPositionUniform, 1, (const GLfloat *)&cameraPosition);
		if (theModelView[i]->theTexture) {
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, m_bufferData[i]->texture);
			glUniform1i(shaderProgram->texture, 1);
		}

		glEnable(GL_POLYGON_OFFSET_FILL);
		Render_Mesh(i);
		glDisable(GL_POLYGON_OFFSET_FILL);
		if (showWireFrame)
			Render_Mesh_Edge(i);
		glUseProgram(0);
	}
	if (theModelView.size()) {
		glUseProgram(colorShaderProgram.shaderProgram);
		glUniformMatrix4fv(colorShaderProgram.projectionMatrixUniform, 1, GL_TRUE, getProjectionMatrix());
		glUniformMatrix4fv(colorShaderProgram.modelViewMatrixUniform, 1, GL_TRUE, getModelViewMatrix());
		glUniformMatrix4fv(shaderProgram->depthMatrixUniform, 1, GL_TRUE, biasMatrix * getDepthMatrix());
		glUniform4fv(colorShaderProgram.lightPositionUniform, 1, (const GLfloat *)&lightPosition);
		glUniform4fv(colorShaderProgram.ambientLight, 1, (const GLfloat *)&ambientLightIntensity);
		glUniform4fv(colorShaderProgram.diffuseLight, 1, (const GLfloat *)&diffuseLightIntensity);
		glUniform4fv(colorShaderProgram.specularLight, 1, (const GLfloat *)&specularLightIntensity);
		glUniform4fv(colorShaderProgram.cameraPositionUniform, 1, (const GLfloat *)&cameraPosition);
		drawFloor();
		glUseProgram(0);
	}
}

void QtViewer::initializeGL(){


	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE);
//	glEnable(GL_CULL_FACE);
//	glFrontFace(GL_CCW);      
	glEnable(GL_DEPTH_TEST);
//	glDepthFunc(GL_LESS);
	glClearColor(1,1,1,0);
	glShadeModel(GL_LINE_SMOOTH);
	glEnable (GL_LINE_SMOOTH);
	glEnable(GL_MULTISAMPLE);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
}

void QtViewer::resizeGL(int w, int h){	
	glViewport (0, 0, (GLsizei) w, (GLsizei) h);
	windowWidth = w;
	windowHeight = h;
}

void QtViewer::mousePressEvent(QMouseEvent *event)
{
	Qt::MouseButtons mouseButtons = event->buttons();
	mouseX = event->x();
	mouseY = event->y();
	if (mouseButtons == Qt::LeftButton && event->modifiers() == Qt::AltModifier)
		m_Trackball.push(pixelPosToViewPos(event->pos()));
}

void QtViewer::mouseMoveEvent(QMouseEvent *event)
{
	Qt::MouseButtons mouseButtons = event->buttons();
	ModelView *modelView = getModelView(0);
	if (mouseButtons == Qt::LeftButton && event->modifiers() == Qt::AltModifier) 
	{
		////////////add your codes here////////////////
		m_Trackball.move(pixelPosToViewPos(event->pos()));
		////////////////////////////////
		updateGL();
	}

	/*xy translation */
	else if (mouseButtons == Qt::MiddleButton && event->modifiers() == Qt::AltModifier) 
	{
		GLfloat scale_x = 0.01f;
		GLfloat scale_y = 0.01f;
		if (modelView)
		{
			scale_y *= modelView->boxAxisLen;
			scale_x *= modelView->boxAxisLen;
		}
		theCamera.translation[0] += scale_x * ( mouseX - event->x());
		theCamera.translation[1] += scale_y * ( event->y() - mouseY);
		updateGL();
	}

	/* zoom in and out */
	else if (mouseButtons == Qt::RightButton && event->modifiers() == Qt::AltModifier) 
	{
		////////////add your codes here////////////////
		if (modelView)
			theCamera.translation[2] -= 0.03 * (event->y()-mouseY) * modelView->boxAxisLen;
		else
			theCamera.translation[2] -= 0.03 * (event->y()-mouseY);
		updateGL();
	}
	mouseX = event->x();
	mouseY = event->y();
}

void QtViewer::mouseReleaseEvent(QMouseEvent *event)
{
	if (event->buttons() == Qt::LeftButton && event->modifiers() == Qt::AltModifier)
		m_Trackball.release(pixelPosToViewPos(event->pos()));
}

void QtViewer::setModelView(ModelView * modelView)
{
	clear();
	theModelView.push_back(modelView);
	showMesh.push_back(true);
	SetCameraFromModelView();
	modelView->computeTangentSpace();
	OpenglBufferData *bufferData = new OpenglBufferData(modelView);
	bufferData->loadBufferData();
	m_bufferData.push_back(bufferData);
	Point n = theCamera.position - theModelView[0]->center;
	n /= n.norm();
	createPlane(theModelView[0]->center - n * theModelView[0]->boxAxisLen * 2, n, theModelView[0]->boxAxisLen * 3);
	generateDepthBuffer();
	updateGL();
}

void QtViewer::ShowWireFrame(bool ifshow)
{
	showWireFrame = ifshow;
	updateGL();
}

void QtViewer::clear()
{
	for (int i = 0; i < theModelView.size(); i++) {
		delete theModelView[i];
		delete m_bufferData[i];
	}
	if (floor) delete floor->theModelView;
	delete floor;
	floor = NULL;
	if (frameBufferName) glDeleteFramebuffers(1, &frameBufferName);
	if (depthTexture) glDeleteTextures(1, &depthTexture);
	theModelView.clear();
	m_bufferData.clear();
	m_Trackball.reset();
}

void QtViewer::Render_Mesh(int idx)
{
	OpenglBufferData *bufferData = m_bufferData[idx];
	if (!bufferData->vao) return;
	//Render the mesh (you may need to change them, according to your mesh data structure)
	//Non-immediate mode

	glBindVertexArray(bufferData->vao);
	glDrawElements(GL_TRIANGLES, bufferData->theModelView->theMesh->numFaces() * 3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
	glBindVertexArray(0);
}

void QtViewer::Render_Mesh_Edge(int idx)
{
	OpenglBufferData *bufferData = m_bufferData[idx];
	if (!bufferData->vao_wireFrame) return;
	//Render the mesh (you may need to change them, according to your mesh data structure)
	//Non-immediate mode

	glBindVertexArray(bufferData->vao_wireFrame);
	glDrawElements(GL_LINES, bufferData->theModelView->theMesh->numEdges() * 2, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
	glBindVertexArray(0);
}

void QtViewer::drawFloor()
{
	if (!floor || !floor->vao) return;

	glBindVertexArray(floor->vao);
	glDrawElements(GL_TRIANGLES, floor->theModelView->theMesh->numFaces() * 3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
	glBindVertexArray(0);
}

void QtViewer::createPlane(Point p, Point norm, double length)
{
	double tempY;
	double tempZ;
	Point c0, c1, c2, c3;
	if (norm[1] && norm[2])
	{
		tempY = -(norm[0] * norm[0] + norm[0] * norm[1] + norm[2] * norm[2]) / (norm[0] * norm[1] + norm[1] * norm[1] + norm[2] * norm[2]);
		tempZ = norm[2] / (norm[0] + norm[1])*(1.0 + tempY);

		Point norm1(1.0, 1.0, -(norm[0] + norm[1]) / norm[2]);
		Point norm2(-1.0, -1.0, (norm[0] + norm[1]) / norm[2]);
		Point norm3(1.0, tempY, tempZ);
		Point norm4 = norm3 * -1.0;

		c0 = p + norm1 / norm1.norm() * length;
		c1 = p + norm2 / norm2.norm() * length;
		c2 = p + norm3 / norm3.norm() * length;
		c3 = p + norm4 / norm4.norm() * length;
	}
	else 
	{
		Point norm1(1.0, 1.0, 0.0);
		Point norm2(1.0, -1.0, 0.0);
		Point norm3(-1.0, -1.0, 0.0);
		Point norm4(-1.0, 1.0, 0.0);
		c0 = p + norm1 / norm1.norm() * length;
		c1 = p + norm2 / norm2.norm() * length;
		c2 = p + norm3 / norm3.norm() * length;
		c3 = p + norm4 / norm4.norm() * length;
	}
	SimMesh *theMesh = new SimMesh;
	SimVertex *v0 = theMesh->createVertex();
	v0->p = c0;
	SimVertex *v1 = theMesh->createVertex();
	v1->p = c1;
	SimVertex *v2 = theMesh->createVertex();
	v2->p = c2;
	SimVertex *v3 = theMesh->createVertex();
	v3->p = c3;
	theMesh->init();
	int ver1[3] = {1, 0, 2}, ver2[3] = {0, 3, 2};
	theMesh->createFace(ver1);
	theMesh->createFace(ver2);
	ModelView *theModelView = new ModelView;
	theModelView->color = Point(1.0f, 1.0f, 1.0f);
	theModelView->LoadMesh(theMesh);
	floor = new OpenglBufferData(theModelView);
	floor->loadBufferData();

}

QPointF QtViewer::pixelPosToViewPos(const QPointF& p) {
	return QPointF(2.0 * float(p.x()) / width() - 1.0,
		1.0 - 2.0 * float(p.y()) / height());
}

QPointF QtViewer::viewPosToPixelPos(const QPointF& p) {
	return QPointF(((float)p.x() + 1) * width() / 2.0,
		(1.0 - (float)p.y()) * height() / 2.0);
}

QPointF QtViewer::viewPosToPixelPos(Point& p) {
	vec4 v(p[0], p[1], p[2], 1);
	vec4 v_view = getProjectionMatrix() * getModelViewMatrix() * v;
	return viewPosToPixelPos(QPointF(v_view[0] / v_view[3], v_view[1] / v_view[3]));
}



bool QtViewer::generateDepthBuffer() {
	
	glGenTextures(1, &depthTexture);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, 1920, 1080, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
	glBindTexture(GL_TEXTURE_2D, 0);
	glGenFramebuffers(1, &frameBufferName);
	glBindFramebuffer(GL_FRAMEBUFFER, frameBufferName);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);
	glDrawBuffer(GL_NONE);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		return false;
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return true;
}
