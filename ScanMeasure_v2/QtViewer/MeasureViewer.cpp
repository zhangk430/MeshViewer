#include "MeasureViewer.h"
#include <fstream>



void MeasureViewer::setModelView(ModelView * modelView)
{
	QtViewer::setModelView(modelView);
}

void MeasureViewer::keyPressEvent(QKeyEvent *key)
{
	switch (key->key())
	{
	case Qt::Key_M:
		{
			measureBetweenSelectedVertices();
			break;
		}
	}
}

void MeasureViewer::measureBetweenSelectedVertices()
{
	if (m_selectObject.selectedVertex.size()<2)
	{
		QMessageBox::warning(0, "Error!", "Choose at least 2 vertices!");
		return;
	}
	ms.clear();
	std::vector<SimVertex *> orderedVertex;
	m_selectObject.orderedSeletedVertex(orderedVertex);
	for (int i = 0; i < orderedVertex.size() - 1; i++)
	{
		for (int j = i+1; j < orderedVertex.size(); j++)
		{
			double len = (orderedVertex[j]->p - orderedVertex[i]->p).norm();
			std::cout << "The distance between marker " << i+1 << " and " << j+1 << " is " << len << "\n";
			dist dt;
			dt.fromIdx = i + 1;
			dt.toIdx = j + 1;
			dt.d = len;
			ms.push_back(dt);
		}
	}
}

void MeasureViewer::saveMeasurement(const char *filename)
{
	std::ofstream out(filename);
	for (int i = 0; i < ms.size(); i++)
	{
		dist dt = ms[i];
		out << dt.fromIdx << " " << dt.toIdx << " " << dt.d << std::endl;
	}
	out.close();
}

void MeasureViewer::clear()
{
	PickingViewer::clear();
	ms.clear();
}