#include "PickingViewer.h"
#include <algorithm>
#include <fstream>

using namespace std;


void PickingViewer::setModelView(ModelView * modelView) {
	QtViewer::setModelView(modelView);
	selectedBufferData = new OpenGLSelectionBufferData(theModelView, &m_selectObject);
	updateGL();
}

void PickingViewer::paintGL()
{
	glLineWidth(1);
	glPointSize(1);
	QtViewer::paintGL();
	glUseProgram(selectionShaderProgram.shaderProgram);
	glUniformMatrix4fv(selectionShaderProgram.modelViewMatrixUniform, 1, GL_TRUE, getModelViewMatrix());
	glUniformMatrix4fv(selectionShaderProgram.projectionMatrixUniform, 1, GL_TRUE, getProjectionMatrix());
	if (m_selectObject.mode != SelectedObject::NONE && !showWireFrame) {

		for (int i = 0; i < theModelView.size(); i++) {
			Render_Mesh_Edge(i);
		}
	}
	//Draw seleceted Points
	glPointSize(5);
	glLineWidth(3);
	drawSelectedObject();
	glUseProgram(0);
	font.f = QFont("Helvetica", 18);
	font.generateFontTexture();
	typedef stdext::hash_map<SimVertex *, int>::iterator vertexIterator; 
	for (vertexIterator vit = m_selectObject.selectedVertex.begin(); vit != m_selectObject.selectedVertex.end(); ++vit)
	{
		std::pair<SimVertex *, int> sv = *vit;
		std::string s = "v" + std::to_string(sv.second);
		QPointF pos = viewPosToPixelPos(sv.first->p);
		drawText(pos.x(), windowHeight - pos.y() - 5, s, vec3(79.0/255, 87.0/255, 1.0));
	}
	typedef stdext::hash_map<SimEdge *, int>::iterator edgeIterator; 
	for (edgeIterator eit = m_selectObject.selectedEdge.begin(); eit != m_selectObject.selectedEdge.end(); ++eit)
	{
		std::pair<SimEdge *, int> ev = *eit;
		std::string s = "e" + std::to_string(ev.second);
		QPointF pos = viewPosToPixelPos((ev.first->v0->p + ev.first->v1->p) / 2);
		drawText(pos.x(), windowHeight - pos.y() - 5, s, vec3(79.0/255, 87.0/255, 1.0));
	}
	typedef stdext::hash_map<SimFace *, int>::iterator faceIterator; 
	for (faceIterator fit = m_selectObject.selectedFace.begin(); fit != m_selectObject.selectedFace.end(); ++fit)
	{
		std::pair<SimFace *, int> fv = *fit;
		std::string s = "f" + std::to_string(fv.second);
		QPointF pos = viewPosToPixelPos((fv.first->ver[0]->p + fv.first->ver[1]->p + fv.first->ver[2]->p) / 3);
		drawText(pos.x(), windowHeight - pos.y() - 5, s, vec3(79.0/255, 87.0/255, 1.0));
	}
}

void PickingViewer::mousePressEvent(QMouseEvent *event)
{
	QtViewer::mousePressEvent(event);
	Qt::MouseButtons mouseButtons = event->buttons();
	if (mouseButtons == Qt::LeftButton && event->modifiers() != Qt::AltModifier && m_selectObject.mode != SelectedObject::NONE)
	{
		if(event->modifiers() != Qt::ControlModifier)
		{
			if (m_selectObject.mode == SelectedObject::Vertex_Mode)
				m_selectObject.selectedVertex.clear();
			if (m_selectObject.mode == SelectedObject::Edge_Mode)
				m_selectObject.selectedEdge.clear();
			if (m_selectObject.mode == SelectedObject::Face_Mode)
				m_selectObject.selectedFace.clear();
		}
		select();
	}
	else if(mouseButtons == Qt::RightButton && event->modifiers() != Qt::AltModifier)
	{
		QMenu *_popmenu = new QMenu;
		QAction *actionSelectVertex = new QAction(this);
		actionSelectVertex->setText(QApplication::translate("MainWindow", "Select Vertex..", 0, QApplication::UnicodeUTF8));
		QObject::connect(actionSelectVertex, SIGNAL(triggered()), this, SLOT(turnOnSelectionVertex()));
		actionSelectVertex->setCheckable(true);
		if (m_selectObject.mode == SelectedObject::Vertex_Mode)
			actionSelectVertex->setChecked(true);
		QAction *actionSelectEdge = new QAction(this);
		actionSelectEdge->setText(QApplication::translate("MainWindow", "Select Edge..", 0, QApplication::UnicodeUTF8));
		QObject::connect(actionSelectEdge, SIGNAL(triggered()), this, SLOT(turnOnSelectionEdge()));
		actionSelectEdge->setCheckable(true);
		if (m_selectObject.mode == SelectedObject::Edge_Mode)
			actionSelectEdge->setChecked(true);
		QAction *actionSelectFace = new QAction(this);
		actionSelectFace->setText(QApplication::translate("MainWindow", "Select Face..", 0, QApplication::UnicodeUTF8));
		QObject::connect(actionSelectFace, SIGNAL(triggered()), this, SLOT(turnOnSelectionFace()));
		actionSelectFace->setCheckable(true);
		if (m_selectObject.mode == SelectedObject::Face_Mode)
			actionSelectFace->setChecked(true);
		QAction *actionExitSelect = new QAction(this);
		actionExitSelect->setText(QApplication::translate("MainWindow", "Exit Selection", 0, QApplication::UnicodeUTF8));
		QObject::connect(actionExitSelect, SIGNAL(triggered()), this, SLOT(turnOffSelection()));
		_popmenu->addAction(actionSelectVertex);
		_popmenu->addAction(actionSelectEdge);
		_popmenu->addAction(actionSelectFace);
		_popmenu->addAction(actionExitSelect);
		_popmenu->exec(QCursor::pos());
	}
}

void PickingViewer::select()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	GLint viewport[4];

	glGetIntegerv(GL_VIEWPORT, viewport);

	glUseProgram(selectionShaderProgram.shaderProgram);
	glUniformMatrix4fv(selectionShaderProgram.modelViewMatrixUniform, 1, GL_TRUE, getModelViewMatrix());
	glUniformMatrix4fv(selectionShaderProgram.projectionMatrixUniform, 1, GL_TRUE, getProjectionMatrix());
	glPointSize(5);
	glLineWidth(5);
	glPolygonOffset(1.0, 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	for (int i = 0; i < theModelView.size() && m_selectObject.mode != SelectedObject::Face_Mode; i++) {
		Render_Mesh(i);
	}
	glDisable(GL_POLYGON_OFFSET_FILL);
	drawSelectObject();
	glFlush();
	glFinish();
	glUseProgram(0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	int coordX = mouseX;
	int coordY = viewport[3] - mouseY;
	unsigned char data[4];
	glReadPixels(coordX, coordY, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);
	int pickedID = data[0] + data[1] * 256 + data[2] * 256 * 256 - 1;
	if (pickedID >= 0 && pickedID != 0x00FFFFFF) {
		if (m_selectObject.mode == SelectedObject::Vertex_Mode) {
			int i = 0;
			while (i < theModelView.size() && pickedID > theModelView[i]->theMesh->numVertices()) {
				pickedID -= theModelView[i]->theMesh->numVertices();
				i++;
			}
			if (i < theModelView.size()) {
				SimVertex *v = theModelView[i]->theMesh->indVertex(pickedID);
				if(m_selectObject.selectedVertex.find(v) != m_selectObject.selectedVertex.end())
				{
					m_selectObject.selectedVertex.erase(m_selectObject.selectedVertex.find(v));
					m_selectObject.updateSelectionOrder();
				}
				else {
					m_selectObject.selectedVertex[v] = m_selectObject.selectedVertex.size() + 1;
					std::cout << "Vertex " << v->idx << " is chosen\n";
					std::cout << "position " << v->p[0] << " " << v->p[1] << " " << v->p[2] << std::endl;
				}
			}
			
		}
		else if (m_selectObject.mode == SelectedObject::Edge_Mode) {
			int i = 0;
			while (i < theModelView.size() && pickedID > theModelView[i]->theMesh->numEdges()) {
				pickedID -= theModelView[i]->theMesh->numEdges();
				i++;
			}
			if (i < theModelView.size()) {
				SimEdge *e = theModelView[i]->theMesh->indEdge(pickedID);
				if(m_selectObject.selectedEdge.find(e) != m_selectObject.selectedEdge.end())
				{
					m_selectObject.selectedEdge.erase(m_selectObject.selectedEdge.find(e));
					m_selectObject.updateSelectionOrder();
				}
				else {
					m_selectObject.selectedEdge[e] = m_selectObject.selectedEdge.size() + 1;
					std::cout << "Edge " << e->idx << " is chosen\n";
					std::cout << "two endpoints " << e->v0->idx << " " << e->v1->idx << std::endl;
				}
			}
		}
		else if (m_selectObject.mode == SelectedObject::Face_Mode) {
			int i = 0;
			while (i < theModelView.size() && pickedID > theModelView[i]->theMesh->numFaces()) {
				pickedID -= theModelView[i]->theMesh->numFaces();
				i++;
			}
			if (i < theModelView.size()) {
				SimFace *f = theModelView[i]->theMesh->indFace(pickedID);
				if(m_selectObject.selectedFace.find(f) != m_selectObject.selectedFace.end())
				{
					m_selectObject.selectedFace.erase(m_selectObject.selectedFace.find(f));
					m_selectObject.updateSelectionOrder();
				}
				else {
					m_selectObject.selectedFace[f] = m_selectObject.selectedFace.size() + 1;
					std::cout << "Face " << f->idx << " is chosen\n";
					std::cout << "Three corners " << f->ver[0]->idx << " " << f->ver[1]->idx << " " << f->ver[2]->idx << std::endl;
				}
			}
		}
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

	selectedBufferData->loadSelectedObjBufferData();

	updateGL();

}

void PickingViewer::drawSelectObject() {
	if (selectedBufferData && selectedBufferData->vao) {
		glBindVertexArray(selectedBufferData->vao);
		glDrawElements(selectedBufferData->mode, selectedBufferData->bufferSize, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
		glBindVertexArray(0);
	}
}

void PickingViewer::drawSelectedObject() {
	if (selectedBufferData && selectedBufferData->vao_sobj) {
		glBindVertexArray(selectedBufferData->vao_sobj);
		glDrawElements(selectedBufferData->sobj_mode, selectedBufferData->sobj_bufferSize, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
		glBindVertexArray(0);
	}
}

void PickingViewer::drawText(float x, float y, const std::string &s, vec3 color) {

	glUseProgram(fontShaderProgram);
	glDisable(GL_DEPTH_TEST);
	GLuint xpos = glGetUniformLocation(fontShaderProgram, "xpos");
	GLuint ypos = glGetUniformLocation(fontShaderProgram, "ypos");
	GLuint scaleX = glGetUniformLocation(fontShaderProgram, "scaleX");
	GLuint scaleY = glGetUniformLocation(fontShaderProgram, "scaleY");
	GLuint textColor =  glGetUniformLocation(fontShaderProgram, "textColor");
	GLuint tex = glGetUniformLocation(fontShaderProgram, "tex");
	glUniform1f(ypos, y);
	glUniform1f(scaleX, 2.0 / windowWidth);
	glUniform1f(scaleY, 2.0 / windowHeight);
	glUniform3fv(textColor, 1, (const GLfloat *)&color);
	glActiveTexture(GL_TEXTURE0);
	for (int i = 0; i < s.size(); i++)
	{
		glUniform1f(xpos, x);
		FontChar * f = font.m_characters[s[i]];
		glBindTexture(GL_TEXTURE_2D, f->textureID);
		glUniform1i(tex, 0);

		glBindVertexArray(f->vao);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
		glBindVertexArray(0);

		x += f->width;
	}

	glEnable(GL_DEPTH_TEST);
	glUseProgram(0);
}

void PickingViewer::turnOnSelectionFace() { 
	if (!theModelView.size())
		return;
	if(m_selectObject.mode != SelectedObject::Face_Mode)
	{
		m_selectObject.mode = SelectedObject::Face_Mode; 
		m_selectObject.selectedVertex.clear();
		m_selectObject.selectedEdge.clear();
		selectedBufferData->clearSelectedBufferData();
		selectedBufferData->loadObjBufferData();
		updateGL(); 
	}
}
void PickingViewer::turnOnSelectionEdge()
{
	if (!theModelView.size())
		return;
	if(m_selectObject.mode != SelectedObject::Edge_Mode)
	{
		m_selectObject.mode = SelectedObject::Edge_Mode; 
		m_selectObject.selectedVertex.clear();
		m_selectObject.selectedFace.clear();
		selectedBufferData->clearSelectedBufferData();
		selectedBufferData->loadObjBufferData();
		updateGL(); 
	}
}

void PickingViewer::turnOnSelectionVertex()
{
	if(!theModelView.size())
		return;
	if(m_selectObject.mode != SelectedObject::Vertex_Mode)
	{
		m_selectObject.mode = SelectedObject::Vertex_Mode; 
		m_selectObject.selectedEdge.clear();
		m_selectObject.selectedFace.clear();
		selectedBufferData->clearSelectedBufferData();
		selectedBufferData->loadObjBufferData();
		updateGL(); 
	}
}
void PickingViewer::turnOffSelection(){  
	if(!theModelView.size())
		return ;
	if(m_selectObject.mode != SelectedObject::NONE) 
	{
		m_selectObject.mode = SelectedObject::NONE;  
		clearSelection();
		selectedBufferData->loadSelectedObjBufferData();
		updateGL();
	}
}

void PickingViewer::clearSelection()
{
	m_selectObject.clear();
}

void PickingViewer::clear()
{
	QtViewer::clear();
	m_selectObject.clear();
	delete selectedBufferData;
}
