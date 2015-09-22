#ifndef RENDERTEXT_H
#define RENDERTEXT_H

#include "../GL_Extension/Angel.h"
#include <QtGui/QFontMetrics>
#include <vector>
#include <QtCore/QHash>

typedef struct {
	int width;
	GLuint textureID;
	GLuint vao;
	GLuint vbo, vbo_indices;
}FontChar;

class RenderText {
public:
	RenderText() : f("Times", 18){}
	~RenderText() { clear();  }
	QFont f;
	QHash<char, FontChar *> m_characters;
	void generateFontTexture();
	void clear();
};

#endif
