// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_REANTTWEAKBAR_XML_SERIALIZATION_H
#define IGL_REANTTWEAKBAR_XML_SERIALIZATION_H
#include "../igl_inline.h"
#include "XMLSerializer.h"

//#ifndef IGL_HEADER_ONLY
//#define IGL_HEADER_ONLY
//#define TO_REMOVE_IGL_HEADER_ONLY
////wangyu #undef IGL_HEADER_ONLY
//#endif

#include <libigl_removed/ReAntTweakBar.h> // modified by wangyu #include <igl/ReAntTweakBar.h>

// Forward declarations
namespace igl
{
  class ReTwBar;
};
namespace tinyxml2
{
  class XMLDocument;
};

namespace igl
{
  
//  namespace
//  {
  
//    IGL_INLINE bool save_ReAntTweakBar(::igl::ReTwBar* bar, const char* file_name);
//    IGL_INLINE bool save_ReAntTweakBar(::igl::ReTwBar* bar, tinyxml2::XMLDocument* doc);
//    IGL_INLINE bool load_ReAntTweakBar(::igl::ReTwBar* bar, const char *file_name);
//    IGL_INLINE bool load_ReAntTweakBar(::igl::ReTwBar* bar, tinyxml2::XMLDocument* doc);
    
    
	IGL_INLINE bool save_ReAntTweakBar(::igl::ReTwBar* bar, const char* file_name);
    
	IGL_INLINE bool save_ReAntTweakBar(::igl::ReTwBar* bar, tinyxml2::XMLDocument* doc);
    
	IGL_INLINE bool load_ReAntTweakBar(::igl::ReTwBar* bar, const char *file_name);

	IGL_INLINE bool load_ReAntTweakBar(::igl::ReTwBar* bar, tinyxml2::XMLDocument* doc);
    
//  }
}
#ifdef TO_REMOVE_IGL_HEADER_ONLY
#undef IGL_HEADER_ONLY
//wangyu
#endif

#endif
