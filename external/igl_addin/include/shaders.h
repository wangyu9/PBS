/*
 *  shaders.h
 *  GLUTviewer
 *
 *  Created by Denis Kovacs on 4/10/08.
 *  Copyright 2008 New York University. All rights reserved.
 *
 */

#ifndef SHADERS_H
#define SHADERS_H

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#else
#   ifdef _WIN32
#       include <windows.h>
#   endif
#   include <GL/gl.h>
#endif

// added by wangyu:

//#include <string>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif
	
	struct GLSL_Program {
		GLuint v, f, g;  // vertex/fragment shaders
		GLuint p;     // program
	};
	
	struct GLSL_Attrib{
		std::string name;
		GLuint id;
	};

	void printShaderInfoLog(GLuint obj);
	void printProgramInfoLog(GLuint obj);
	
	GLuint loadShader(const char *filename, GLuint type);
	
	struct GLSL_Program loadShaderProgram(const char *vertFilename, const char *fragFilename);
	struct GLSL_Program loadShaderProgramWithAttribs(const char *vertFilename, const char *fragFilename, std::vector<struct GLSL_Attrib> attribs);//added by wangyu
	struct GLSL_Program loadShaderProgramStr(const char *vertShader, const char *fragShader);
	struct GLSL_Program initShaderProgram(GLuint v, GLuint f);
	struct GLSL_Program initShaderProgramWithAttribs(GLuint v, GLuint f, std::vector<struct GLSL_Attrib> attribs);//added by wangyu
	struct GLSL_Program initGShaderProgram(GLuint v, GLuint g, GLuint f, GLenum primIn, GLenum primOut, GLint maxVerticesOut);
	struct GLSL_Program loadGShaderProgram(const char *vertFilename, const char *geomFilename, const char *fragFilename, GLenum primIn, GLenum primOut, GLint maxVerticesOut);
	
	void deinitShaderProgram(struct GLSL_Program p);
	
#ifdef __cplusplus
}
#endif


#endif