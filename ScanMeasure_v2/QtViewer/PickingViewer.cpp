#include "PickingViewer.h"
#include <algorithm>
#include <fstream>

using namespace std;


void PickingViewer::paintGL()
{
	QtViewer::paintGL();
	glDisable(GL_LIGHTING);
	if (m_selectObject.mode != SelectedObject::NONE && !showWireFrame)
		glCallList(meshlist);
	//Draw seleceted Points
	glPointSize(8);
	glBegin(GL_POINTS);
	typedef stdext::hash_map<SimVertex *, int>::iterator vertexIterator; 
	for (vertexIterator vit = m_selectObject.selectedVertex.begin(); vit != m_selectObject.selectedVertex.end(); ++vit)
	{
		std::pair<SimVertex *, int> sv = *vit;
		glColor3f(0.0, 1.0, 1.0);
		glVertex3dv(sv.first->p.v);
	}
	glEnd();
	for (vertexIterator vit = m_selectObject.selectedVertex.begin(); vit != m_selectObject.selectedVertex.end(); ++vit)
	{
		std::pair<SimVertex *, int> sv = *vit;
		glColor3f(0.0, 1.0, 1.0);
		QFont font;
		font.setPointSize(12);
		QString str = " v"+QString::number(sv.second);
		Point pos = sv.first->p;
		glDisable(GL_DEPTH_TEST);
		renderText(pos[0], pos[1], pos[2], str, font);
		glEnable(GL_DEPTH_TEST);
	}
	glEnable(GL_LIGHTING);
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
		actionSelectVertex->setText(QApplication::translate("MainWindow", "Select SimVertex..", 0, QApplication::UnicodeUTF8));
		QObject::connect(actionSelectVertex, SIGNAL(triggered()), this, SLOT(turnOnSelectionVertex()));
		actionSelectVertex->setCheckable(true);
		if (m_selectObject.mode == SelectedObject::Vertex_Mode)
			actionSelectVertex->setChecked(true);
		QAction *actionSelectEdge = new QAction(this);
		actionSelectEdge->setText(QApplication::translate("MainWindow", "Select SimEdge..", 0, QApplication::UnicodeUTF8));
		QObject::connect(actionSelectEdge, SIGNAL(triggered()), this, SLOT(turnOnSelectionEdge()));
		actionSelectEdge->setCheckable(true);
		if (m_selectObject.mode == SelectedObject::Edge_Mode)
			actionSelectEdge->setChecked(true);
		QAction *actionSelectFace = new QAction(this);
		actionSelectFace->setText(QApplication::translate("MainWindow", "Select SimFace..", 0, QApplication::UnicodeUTF8));
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
		updateGL();
	}
}

void PickingViewer::Render_Mesh()
{
	bool renderPerVertex = (theModelView->theColor != NULL);
	bool renderByTexture = (theModelView->theTexture!=NULL && !renderPerVertex);
	//Render the mesh (you may need to change them, according to your mesh data structure)
	if (!renderPerVertex && !renderByTexture)
		glColor3fv(theModelView->color);
	if (renderByTexture)
	{
		glEnable(GL_TEXTURE_2D);
		glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glBindTexture(GL_TEXTURE_2D, texture[0]);
	}
	if (!theModelView->theMesh->numFaces())
	{
		glPointSize(2);
		glBegin(GL_POINTS);
		for (int i = 0; i < theModelView->theMesh->numVertices(); i++)
		{
			SimVertex * v = theModelView->theMesh->indVertex(i);
			if (renderPerVertex)
				glColor3ub(theModelView->theColor->r[v->idx], 
				theModelView->theColor->g[v->idx], 
				theModelView->theColor->b[v->idx]);
			if (theModelView->theNormals)
				glNormal3dv(theModelView->VertexNormal(v).v);
			glVertex3f(v->p[0], v->p[1], v->p[2]);	
		}
		glEnd();
	}
	else
	{
		glBegin(GL_TRIANGLES);
		for (int i = 0; i < theModelView->theMesh->numFaces(); i++)
		{
			SimFace *f = theModelView->theMesh->indFace(i);
			if(m_selectObject.selectedFace.find(f) != m_selectObject.selectedFace.end())
				continue;
			SimVertex * v[3];
			v[0]=f->ver[0];
			v[1]=f->ver[1];
			v[2]=f->ver[2];
			if (renderPerVertex)
				glColor3ub(theModelView->theColor->r[v[0]->idx], 
				theModelView->theColor->g[v[0]->idx], 
				theModelView->theColor->b[v[0]->idx]);
			else if (renderByTexture)
			{
				glTexCoord2f(theModelView->theTexture->getU(v[0], f), 
					theModelView->theTexture->getV(v[0], f));
			}
			glNormal3dv(theModelView->VertexNormal(v[0]).v);
			glVertex3dv(v[0]->p.v);	
			if (renderPerVertex)
				glColor3ub(theModelView->theColor->r[v[1]->idx], 
				theModelView->theColor->g[v[1]->idx], 
				theModelView->theColor->b[v[1]->idx]);
			else if (renderByTexture)
			{
				//			std::cout<<v[1]->id()<<std::endl;
				glTexCoord2f(theModelView->theTexture->getU(v[1], f), 
					theModelView->theTexture->getV(v[1], f));
				//			std::cout<<theModelView->theTexture->getU(f, 2)<<" " << theModelView->theTexture->getV(f, 2)<<std::endl;
			}
			glNormal3dv(theModelView->VertexNormal(v[1]).v);
			glVertex3dv(v[1]->p.v);	
			if (renderPerVertex)
				glColor3ub(theModelView->theColor->r[v[2]->idx], 
				theModelView->theColor->g[v[2]->idx], 
				theModelView->theColor->b[v[2]->idx]);
			else if (renderByTexture)
			{
				//			std::cout<<v[2]->id()<<std::endl;
				glTexCoord2f(theModelView->theTexture->getU(v[2], f), 
					theModelView->theTexture->getV(v[2], f));
				//			std::cout<<theModelView->theTexture->getU(f, 0)<<" " << theModelView->theTexture->getV(f, 0)<<std::endl;
			}
			glNormal3dv(theModelView->VertexNormal(v[2]).v);
			glVertex3dv(v[2]->p.v);
			//		if (f->id() > 10)
			//			exit(0);

		}
		typedef stdext::hash_map<SimFace *, int>::iterator faceIterator; 
		glColor3f(0.0, 1.0, 1.0);
		for (faceIterator fit = m_selectObject.selectedFace.begin(); fit != m_selectObject.selectedFace.end(); ++fit)
		{
			std::pair<SimFace *, float> sf = *fit;
			SimFace *f = sf.first;
			SimVertex * v[3];
			v[0]=f->ver[0];
			v[1]=f->ver[1];
			v[2]=f->ver[2];
			glNormal3dv(theModelView->VertexNormal(v[0]).v);
			glVertex3dv(v[0]->p.v);	
			glNormal3dv(theModelView->VertexNormal(v[1]).v);
			glVertex3dv(v[1]->p.v);	
			glNormal3dv(theModelView->VertexNormal(v[2]).v);
			glVertex3dv(v[2]->p.v);	
		}
		glEnd();
	}
	if (renderByTexture)
		glDisable(GL_TEXTURE_2D);
}

void PickingViewer::Render_Mesh_Edge()
{
	glColor3f(0.0,0.0,0.0);

	std::vector<SimEdge *> sharpOrBoundary;
	//Render the mesh (you may need to change them, according to your mesh data structure)
	glLineWidth(1);
	glBegin(GL_LINES);
	for (int i = 0; i < theModelView->theMesh->numEdges(); i++)
	{
		SimEdge *e = theModelView->theMesh->indEdge(i);
		if(m_selectObject.selectedEdge.find(e) != m_selectObject.selectedEdge.end())
			continue;
		if(theModelView->theMesh->isBoundary(e))
		{
			sharpOrBoundary.push_back(e);
			continue;
		}
		glVertex3dv(e->v0->p.v);
		glVertex3dv(e->v1->p.v);
	}
	glEnd();
	glColor3f(1.0,1.0,0.0);
	glLineWidth(1.5);
	glBegin(GL_LINES);
	for (unsigned int i=0;i<sharpOrBoundary.size();i++)
	{
		SimEdge * e = sharpOrBoundary[i];
		glVertex3dv(e->v0->p.v);
		glVertex3dv(e->v1->p.v);
	}
	glEnd();
	glColor3f(0.0,1.0,1.0);
	glLineWidth(3);
	glBegin(GL_LINES);
	typedef stdext::hash_map<SimEdge *, int>::iterator edgeIterator; 
	for (edgeIterator eit = m_selectObject.selectedEdge.begin(); eit != m_selectObject.selectedEdge.end(); ++eit)
	{
		std::pair<SimEdge *, int> se = *eit;
		SimEdge * e = se.first;
		glVertex3dv(e->v0->p.v);
		glVertex3dv(e->v1->p.v);
	}
	glEnd();
}


void PickingViewer::select()
{
	GLint viewport[4];

	float whratio = (float)width()/(float)height();

	glGetIntegerv(GL_VIEWPORT, viewport);

	GLuint select_buf[SELECT_BUFSIZE]; //selection buffer
	GLuint hits = 0; //number of hits

	glSelectBuffer(SELECT_BUFSIZE, select_buf);
	glRenderMode(GL_SELECT);
	glInitNames();
	glPushName(0);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluPickMatrix((GLdouble)mouseX, (GLdouble)(viewport[3]-mouseY), 5, 5, viewport);
	gluPerspective( 45.0, /* field of view in degrees */
		whratio, /* aspect ratio */
		theCamera.boxAxisLen*0.01, /* Z near */ 
		theCamera.boxAxisLen*100 /* Z far */);
	drawSelectObject();
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

	//	glFlush(); //flush the pipeline
	hits = glRenderMode(GL_RENDER); //switch back to RENDER mode
	ProcessHits(hits, select_buf);

	if(m_selectObject.mode == SelectedObject::Face_Mode)
	{
		glNewList(modellist, GL_COMPILE);
		Render_Mesh();
		glEndList();
	}
	if(m_selectObject.mode == SelectedObject::Edge_Mode)
	{
		glNewList(meshlist, GL_COMPILE);
		Render_Mesh_Edge();
		glEndList();
	}

	updateGL();

}

void PickingViewer::drawSelectObject()
{
	if(m_selectObject.mode == SelectedObject::Face_Mode)
	{
		for (int i = 0; i < theModelView->theMesh->numFaces(); i++){

			SimFace *f = theModelView->theMesh->indFace(i);
			glLoadName(f->idx+1);
			glBegin(GL_TRIANGLES);	
			SimVertex * v[3];
			v[0]=f->ver[0];
			v[1]=f->ver[1];
			v[2]=f->ver[2];
			glNormal3dv(theModelView->VertexNormal(v[0]).v);
			glVertex3dv(v[0]->p.v);	
			glNormal3dv(theModelView->VertexNormal(v[1]).v);
			glVertex3dv(v[1]->p.v);	
			glNormal3dv(theModelView->VertexNormal(v[2]).v);
			glVertex3dv(v[2]->p.v);		
			glEnd();
		}
	}
	if(m_selectObject.mode == SelectedObject::Edge_Mode)
	{
		for(int i = 0; i < theModelView->theMesh->numEdges(); i++){

			SimEdge *e = theModelView->theMesh->indEdge(i);
			glLoadName(e->idx+1);
			glBegin(GL_LINES);	
			glVertex3dv(e->v0->p.v);
			glVertex3dv(e->v1->p.v);
			glEnd();
		}
	}
	if(m_selectObject.mode == SelectedObject::Vertex_Mode)
	{
		for(int i = 0; i < theModelView->theMesh->numVertices(); i++){

			SimVertex *v = theModelView->theMesh->indVertex(i);
			glLoadName(v->idx+1);
			glBegin(GL_POINTS);	
			glVertex3dv(v->p.v);			
			glEnd();
		}
	}
}

void PickingViewer::ProcessHits(GLint hits, GLuint buffer[])
{


#define DEBUG_PROCESSHITS 0

	GLint i;
	GLuint names, j, *ptr;
	std::vector<pair<GLfloat, GLuint> > name_array;
	std::vector<pair<GLfloat, GLuint> >::iterator niter;
	std::vector<pair<GLfloat, pair<GLuint, GLuint> > > edge_array;
	std::vector<pair<GLfloat, pair<GLuint, GLuint> > >::iterator eiter;

	pair<GLfloat, GLuint> name_pair;
	pair<GLfloat, GLuint> other_pair;
	pair<GLfloat, pair<GLuint, GLuint> > edge_pair;

	ptr = (GLuint *) buffer;

	for (i = 0; i < hits; i++)
	{
		names = *ptr;
		ptr += 3;

		for (j = 0; j < names; j++)
		{
			// In SimVertex picking mode, we will have to eliminate duplicates
			// Duplicates don't happen in SimFace/SimEdge mode
			// This is related to the way we render the model for picking
			name_pair = std::make_pair((GLfloat)*(ptr-2)/0x7fffffff, (*ptr));
			name_array.push_back(name_pair);

#if DEBUG_PROCESSHITS
			//			fprintf(stderr, "name: %u z1: %g z2: %g\n", *ptr, (float)*(ptr-2)/0x7fffffff, (float)*(ptr-1)/0x7fffffff);
#endif				
			ptr++;
		}
	}

	// A minor problem: if two points are collinear they may both be picked
	// we need to throw away the one that is farther away
	// A tricky problem: the point which we don't actually see (hidden further away)
	// may gets picked if the picking is not accurate enough and does not actually 
	// pick the point in the front which we DO see
	// Solution: sort the names according to their z1 value

	if (name_array.size() > 1)
	{
		std::sort(name_array.begin(), name_array.end());
	}

	if (name_array.empty())
	{
#if DEBUG_PROCESSHITS
		//fprintf(stderr, "Name array empty!\n");
#endif

		return;
	}

	for (niter = name_array.begin(); niter != name_array.end(); ++niter)
	{
		name_pair = *niter;
		if(m_selectObject.mode == SelectedObject::Face_Mode)
		{
			SimFace *f = theModelView->theMesh->indFace(name_pair.second-1);
			if (f) {
				if(m_selectObject.selectedFace.find(f) != m_selectObject.selectedFace.end())
				{
					m_selectObject.selectedFace.erase(m_selectObject.selectedFace.find(f));
					m_selectObject.updateSelectionOrder();
				}
				else 
					m_selectObject.selectedFace[f] = m_selectObject.selectedFace.size() + 1;
				break;
			}
		}
		else if(m_selectObject.mode == SelectedObject::Edge_Mode)
		{
			SimEdge *e = theModelView->theMesh->indEdge(name_pair.second-1);
			if(e)
			{
				if(m_selectObject.selectedEdge.find(e) != m_selectObject.selectedEdge.end())
				{
					m_selectObject.selectedEdge.erase(m_selectObject.selectedEdge.find(e));
					m_selectObject.updateSelectionOrder();
				}
				else 
					m_selectObject.selectedEdge[e] = m_selectObject.selectedEdge.size() + 1;

				break;
			}
		}
		else if(m_selectObject.mode == 1)
		{
			SimVertex *v = theModelView->theMesh->indVertex(name_pair.second-1);
			if (v)
			{
				if(m_selectObject.selectedVertex.find(v) != m_selectObject.selectedVertex.end())
				{
					m_selectObject.selectedVertex.erase(m_selectObject.selectedVertex.find(v));
					m_selectObject.updateSelectionOrder();
				}
				else 
					m_selectObject.selectedVertex[v] = m_selectObject.selectedVertex.size() + 1;
				break;
			}
		}

		//SimVertex *v = GlobalData::gTetMesh->SimVertex(name_pair.second);

		//if (v)
	}		

#undef DEBUG_PROCESSHITS

}

void PickingViewer::turnOnSelectionFace()
{ 
	if (!theModelView)
		return;
	if(m_selectObject.mode != SelectedObject::Face_Mode)
	{
		m_selectObject.mode = SelectedObject::Face_Mode; 
		m_selectObject.selectedVertex.clear();
		if(m_selectObject.selectedEdge.size())
		{
			m_selectObject.selectedEdge.clear();
			glNewList(meshlist, GL_COMPILE);
			Render_Mesh_Edge();
			glEndList();
		}
		updateGL(); 
	}
}
void PickingViewer::turnOnSelectionEdge()
{
	if (!theModelView)
		return;
	if(m_selectObject.mode != SelectedObject::Edge_Mode)
	{
		m_selectObject.mode = SelectedObject::Edge_Mode; 
		m_selectObject.selectedVertex.clear();
		if(m_selectObject.selectedFace.size())
		{
			m_selectObject.selectedFace.clear();
			glNewList(modellist, GL_COMPILE);
			Render_Mesh();
			glEndList();
		}
		updateGL(); 
	}
}

void PickingViewer::turnOnSelectionVertex()
{
	if(!theModelView)
		return;
	if(m_selectObject.mode != SelectedObject::Vertex_Mode)
	{
		m_selectObject.mode = SelectedObject::Vertex_Mode; 
		if(m_selectObject.selectedFace.size())
		{
			m_selectObject.selectedFace.clear();
			glNewList(modellist, GL_COMPILE);
			Render_Mesh();
			glEndList();
		}
		if(m_selectObject.selectedEdge.size())
		{
			m_selectObject.selectedEdge.clear();
			glNewList(meshlist, GL_COMPILE);
			Render_Mesh_Edge();
			glEndList();
		}
		updateGL(); 
	}
}
void PickingViewer::turnOffSelection(){  
	if(!theModelView)
		return ;
	if(m_selectObject.mode != SelectedObject::NONE) 
	{
		m_selectObject.mode = SelectedObject::NONE;  
		m_selectObject.selectedVertex.clear();
		if(m_selectObject.selectedFace.size())
		{
			m_selectObject.selectedFace.clear();
			glNewList(modellist, GL_COMPILE);
			Render_Mesh();
			glEndList();
		}
		if(m_selectObject.selectedEdge.size())
		{
			m_selectObject.selectedEdge.clear();
			glNewList(meshlist, GL_COMPILE);
			Render_Mesh_Edge();
			glEndList();
		}
		updateGL();
	}
}

void PickingViewer::loadSelectedVertices(const char filename[])
{
	m_selectObject.selectedVertex.clear();
	ifstream in(filename);
	int count = 1;
	while (!in.eof())
	{
		std::string name;
		int id;
		Point p;
		in >> name;
		if (!name.compare("Vertex"))
		{
			in >> id >> p[0] >> p[1] >> p[2];
			m_selectObject.selectedVertex[theModelView->theMesh->indVertex(id - 1)] = count;
			count++;
		}
	}
	in.close();
	updateGL();

}

void PickingViewer::loadSelectedVerticesByPosition(const char filename[])
{
	m_selectObject.selectedVertex.clear();
	ifstream in(filename);
	int count = 1;
	while (!in.eof())
	{
		std::string name;
		int id;
		Point p;
		in >> name;
		if (!name.compare("Vertex"))
		{
			in >> id >> p[0] >> p[1] >> p[2];
			int vertex_id = findClosestPoints(p);
			if (vertex_id >= 0)
			{
				m_selectObject.selectedVertex[theModelView->theMesh->indVertex(vertex_id)] = count;
				count++;
			}
		}
	}
	in.close();
	updateGL();
}

void PickingViewer::saveSelectedVertices(const char filename[])
{
	ofstream out(filename);
	std::vector<SimVertex *> orderedVertex;
	m_selectObject.orderedSeletedVertex(orderedVertex);
	out << orderedVertex.size() << std::endl;
	for (unsigned int i = 0; i < orderedVertex.size(); i++)
		out << orderedVertex[i]->idx + 1 << "\n";
	out.close();
}

void PickingViewer::clearSelection()
{
	m_selectObject.clear();
}

void PickingViewer::clear()
{
	QtViewer::clear();
	m_selectObject.clear();
}

int PickingViewer::findClosestPoints(Point p)
{
	float dist = 1e6;
	int max_id = -1;
	for (int i = 0 ; i < theModelView->theMesh->numVertices(); i++)
	{
		float c_dist = (p - theModelView->theMesh->indVertex(i)->p).norm();
		if (dist > c_dist)
		{
			max_id = i;
			dist = c_dist;
		}
	}
	return max_id;
}