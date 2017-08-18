#ifndef __tga_h__
#define __tga_h__

// Functions to read and write a TGA file

#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif

typedef struct {
  
  GLsizei  width;
  GLsizei  height;
  GLint    components;
  GLenum   format;
  
  GLsizei  cmapEntries;
  GLenum   cmapFormat;
  GLubyte *cmap;
  
  GLubyte *pixels;
  
} gliGenericImage;

typedef struct {
  unsigned char idLength;
  unsigned char colorMapType;
  
  /* The image type. */
#define TGA_TYPE_MAPPED 1
#define TGA_TYPE_COLOR 2
#define TGA_TYPE_GRAY 3
#define TGA_TYPE_MAPPED_RLE 9
#define TGA_TYPE_COLOR_RLE 10
#define TGA_TYPE_GRAY_RLE 11
  unsigned char imageType;
  
  /* Color Map Specification. */
  /* We need to separately specify high and low bytes to avoid endianness
   and alignment problems. */
  unsigned char colorMapIndexLo, colorMapIndexHi;
  unsigned char colorMapLengthLo, colorMapLengthHi;
  unsigned char colorMapSize;
  
  /* Image Specification. */
  unsigned char xOriginLo, xOriginHi;
  unsigned char yOriginLo, yOriginHi;
  
  unsigned char widthLo, widthHi;
  unsigned char heightLo, heightHi;
  
  unsigned char bpp;
  
  /* Image descriptor.
   3-0: attribute bpp
   4:   left-to-right ordering
   5:   top-to-bottom ordering
   7-6: zero
   */
#define TGA_DESC_ABITS 0x0f
#define TGA_DESC_HORIZONTAL 0x10
#define TGA_DESC_VERTICAL 0x20
  unsigned char descriptor;
  
} TgaHeader;

typedef struct {
  unsigned int extensionAreaOffset;
  unsigned int developerDirectoryOffset;
#define TGA_SIGNATURE "TRUEVISION-XFILE"
  char signature[16];
  char dot;
  char null;
} TgaFooter;

extern gliGenericImage *gliReadTGA(FILE *fp, char *name, int hflip, int vflip);
extern int gliVerbose(int newVerbose);

void writeTGA( gliGenericImage* image, FILE *fp);

////////////////////////////////////////////////////////////////////////////////
//
// WARNING
//
// THIS DOES NOT DEAL WITH VERTICALLY FLIPPED DATA CORRECTLY
//
////////////////////////////////////////////////////////////////////////////////

/* This file is derived from (actually an earlier version of)... */

/* The GIMP -- an image manipulation program
 * Copyright (C) 1995 Spencer Kimball and Peter Mattis
 *
 * $Id: tga.cpp,v 1.1.2.5 2007-05-10 02:10:07 elif Exp $
 * TrueVision Targa loading and saving file filter for the Gimp.
 * Targa code Copyright (C) 1997 Raphael FRANCOIS and Gordon Matzigkeit
 *
 * The Targa reading and writing code was written from scratch by
 * Raphael FRANCOIS <fraph@ibm.net> and Gordon Matzigkeit
 * <gord@gnu.ai.mit.edu> based on the TrueVision TGA File Format
 * Specification, Version 2.0:
 *
 *   <URL:ftp://ftp.truevision.com/pub/TGA.File.Format.Spec/>
 *
 * It does not contain any code written for other TGA file loaders.
 * Not even the RLE handling. ;)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

typedef struct {
  unsigned char *statebuf;
  int statelen;
  int laststate;
} RLEstate;

int std_fread(RLEstate *rleInfo, unsigned char *buf, size_t datasize, size_t nelems, FILE *fp);
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

#define RLE_PACKETSIZE 0x80

/* Decode a bufferful of file. */
int rle_fread(RLEstate *rleInfo, unsigned char *vbuf, size_t datasize, size_t nelems, FILE *fp);


gliGenericImage * gliReadTGA(FILE *fp, char *name, int hflip, int vflip);

int gliVerbose(int newVerbose);


// added 10/2005, Denis Zorin
// a very simple TGA output, supporting 
// uncompressed luminance RGB and RGBA 
// G22.2270 students: this is C (no C++) 
// so this is not the style I would encourage
// you to use; I used it for consistency 
// with the rest of the code in this file 



// this makes sure that 
// image size is written in correct format 
// and byte order (least first)
void write16bit(int n, FILE* fp);



void writeTGA( gliGenericImage* image, FILE *fp);

#endif /* __tga_h__ */
