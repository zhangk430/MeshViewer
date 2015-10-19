#include "TracingViewer.h"
#include "../Geodesics/Geodesics.h"
#include <QMessageBox>

void TracingViewer::traceShortestPath(bool isLoop /* = false */) 
{
	if (isLoop && m_selectObject.selectedVertex.size() < 3)
	{
		QMessageBox::warning(0, "Error!", "Choose at least 3 vertices!");
		return;
	}
	if (!isLoop && m_selectObject.selectedVertex.size() < 2)
	{
		QMessageBox::warning(0, "Error!", "Choose at least 2 vertices!");
		return;
	}
	std::vector<SimVertex *> srcList, selectedVertexInOrder;
	m_selectObject.orderedSeletedVertex(selectedVertexInOrder);
	int meshID = locateVertex(selectedVertexInOrder[0]);
	for (unsigned i = 1; i < selectedVertexInOrder.size(); ++i)
	{
		int cMeshID = locateVertex(selectedVertexInOrder[i]);
		if (cMeshID != meshID)
		{
			QMessageBox::warning(0, "Error!", "Only support tracing in one mesh ");
			return;
		}
	}
	SimMesh *theMesh = theModelView[meshID]->theMesh;
	geodesics tracer(theMesh);
	if (isLoop)
	{
		for (unsigned i = 0; i < selectedVertexInOrder.size(); ++i)
		{
			srcList.clear();
			srcList.push_back(selectedVertexInOrder[i]);
			tracer.setSource(srcList);
			tracer.computeGeodesics_Dijkstra();
			std::vector<geodesic::SurfacePoint> path;
			tracer.Path(selectedVertexInOrder[(i+1)%selectedVertexInOrder.size()], path);
			std::vector<SimVertex *> verList;
			for (unsigned i = 0; i < path.size(); ++i)
			{
				geodesic::SurfacePoint sp = path[i];
				if (sp.type() == geodesic::VERTEX)
				{
					verList.push_back(tracer.convert2Vertex(sp));
				}
				else
				{
					std::cerr << "Error!" << std::endl;
					continue;
				}
			}
			for (int i = verList.size() - 2; i >= 0; --i)
			{
				SimEdge *e = theMesh->idEdge(verList[i]->idx, verList[i + 1]->idx);
				if (e)
					m_selectObject.selectedEdge[e] = m_selectObject.selectedEdge.size() + 1;
				else
				{
					std::cerr << "Error!" << std::endl;
					continue;
				}
			}
		}
	}
	else
	{
		for (unsigned i = 0; i < selectedVertexInOrder.size() - 1; ++i)
		{
			srcList.clear();
			srcList.push_back(selectedVertexInOrder[i]);
			tracer.setSource(srcList);
			tracer.computeGeodesics_Dijkstra();
			std::vector<geodesic::SurfacePoint> path;
			double len = tracer.Path(selectedVertexInOrder[i + 1], path);
			std::vector<SimVertex *> verList;
			for (unsigned i = 0; i < path.size(); ++i)
			{
				geodesic::SurfacePoint sp = path[i];
				if (sp.type() == geodesic::VERTEX)
				{
					verList.push_back(tracer.convert2Vertex(sp));
				}
				else
				{
					std::cerr << "Error!" << std::endl;
					continue;
				}
			}
			for (int i = verList.size() - 2; i >= 0; --i)
			{
				SimEdge *e = theMesh->idEdge(verList[i]->idx, verList[i + 1]->idx);
				if (e)
					m_selectObject.selectedEdge[e] = m_selectObject.selectedEdge.size() + 1;
				else
				{
					std::cerr << "Error!" << std::endl;
					continue;
				}
			}
		}
	}
	m_selectObject.selectedVertex.clear();
	selectedBufferData->loadSelectedObjBufferData();
	m_selectObject.mode = SelectedObject::Edge_Mode;
	emit picked();
	update();
}