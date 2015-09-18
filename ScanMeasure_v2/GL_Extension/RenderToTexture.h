#ifndef RENDER_TO_TEXTURE_H
#define RENDER_TO_TEXTURE_H

#include "Angel.h"

class RenderToTexture
{
private:
	GLuint frameBufferName;
	GLuint renderedTexture;
	GLuint depthRenderBuffer;
	int width, height;

public:
	RenderToTexture() : width(1024), height(768) {
		glGenFramebuffers(1, &frameBufferName);
		glGenTextures(1, &renderedTexture);
		glGenRenderbuffers(1, &depthRenderBuffer);
	}

	void setWidthAndHeight(int w, int h) { width = w;  height = h;  }
	void bindFrameBuffer();
	void generateTexture();
	void generateDepthBuffer();
	bool renderTexture();
};


#endif