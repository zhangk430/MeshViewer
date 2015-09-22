#include "RenderText.h"
#include <QtGui/QImage>
#include <QtGui/QPainter>
#include <QtOpenGL/qgl.h>

static unsigned int 
	nearest_pow (unsigned int num)
{
	unsigned int j, k;
	(j = num & 0xFFFF0000) || (j = num);
	(k = j & 0xFF00FF00) || (k = j);
	(j = k & 0xF0F0F0F0) || (j = k);
	(k = j & 0xCCCCCCCC) || (k = j);
	(j = k & 0xAAAAAAAA) || (j = k);
	return j << 1;
}

void RenderText::generateFontTexture() {
	QFontMetrics metric(f);

	int fontHeight = metric.height();

	const char startChar = ' ';
	const char endChar = '~';

	int heightPow2 = nearest_pow(fontHeight);

	for (char c = startChar; c <= endChar; ++c) {
		if (m_characters.find(c) != m_characters.end())
			continue;
		QChar ch(c);
		FontChar *fc = new FontChar;
		
		int width = metric.width(c);
		int widthPow2 = nearest_pow(width);

		float s0 = 0.0;
		float s1 = (float)width / widthPow2;

		float t0 = 0.0;
		float t1 = (float)metric.height() / heightPow2;
		fc->width = width;

		QImage finalImage(widthPow2, heightPow2, QImage::Format_ARGB32);
		finalImage.fill(Qt::transparent);
		QPainter painter;
		painter.begin(&finalImage);
		painter.setRenderHints(QPainter::HighQualityAntialiasing | QPainter::TextAntialiasing);

		painter.setFont(f);
		painter.setPen(Qt::black);

		painter.drawText(0, metric.ascent(), QString(c));
		painter.end();
		glGenTextures(1, &fc->textureID);
		glBindTexture(GL_TEXTURE_2D, fc->textureID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		finalImage = QGLWidget::convertToGLFormat(finalImage);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, finalImage.width(), finalImage.height(),
			0, GL_RGBA, GL_UNSIGNED_BYTE, finalImage.bits());
		struct textVertData
		{
			float x;
			float y;
			float u;
			float v;
		};
		// we are creating a billboard with two triangles so we only need the
		// 6 verts, (could use index and save some space but shouldn't be too much of an
		// issue
		textVertData d[6];
		// load values for triangle 1
		d[0].x=0;
		d[0].y=0;
		d[0].u=s0;
		d[0].v=t0;

		d[1].x=fc->width;
		d[1].y=0;
		d[1].u=s1;
		d[1].v=t0;

		d[2].x=0;
		d[2].y=fontHeight;
		d[2].u=s0;
		d[2].v=t1;
		// load values for triangle two
		d[3].x=0;
		d[3].y=0+fontHeight;
		d[3].u=s0;
		d[3].v=t1;


		d[4].x=fc->width;
		d[4].y=0;
		d[4].u=s1;
		d[4].v=t0;


		d[5].x=fc->width;
		d[5].y=fontHeight;
		d[5].u=s1;
		d[5].v=t1;


		glGenVertexArrays(1, &fc->vao);
		glBindVertexArray(fc->vao);
		// set the vertex data (2 for x,y 2 for u,v)
		glGenBuffers(1, &fc->vbo);
		glBindBuffer(GL_ARRAY_BUFFER, fc->vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(d), d, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(textVertData), BUFFER_OFFSET(0));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(textVertData), BUFFER_OFFSET(2 * sizeof(float)));
		// say how many indecis to be rendered
		unsigned int indices[6];
		for (int i = 0; i < 6; i++)
			indices[i] = i;
		glGenBuffers(1, &fc->vbo_indices);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fc->vbo_indices);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

		// now unbind
		glBindVertexArray(0);
		m_characters[c] = fc;
	}
}

void RenderText::clear() {
	for (auto it = m_characters.begin(); it != m_characters.end(); ++it) {
		auto p = *it;
		if (p->textureID) glDeleteTextures(1, &p->textureID);
		if (p->vbo) glDeleteBuffers(1, &p->vbo);
		if (p->vbo_indices) glDeleteBuffers(1, &p->vbo_indices);
		if (p->vao) glDeleteVertexArrays(1, &p->vao);
		delete p;
	}
	m_characters.clear();
}
