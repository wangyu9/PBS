// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
/* ---------------------------------------------------------------------------
 // XMLSerialization.h
 // Author: Christian Schüller <schuellchr@gmail.com>
 ------------------------------------------------------------------------------
 Inherit from this class to have the easiest way to serialize your user defined class.
 
 1) Pass the default name of your class to the base constructor.
 2) Override InitSerialization() and add your variables to serialize like:
 xmlSerializer->Add(var1,"name1");
 xmlSerializer->Add(var2,"name2");

 Workaround for Visual Studio run time debugger inspection problem:
 Copy and implement all the functions, splitting them into a source and header file.
 Restrictions on Native C++ Expressions (Anonymous Namespaces):
 http://msdn.microsoft.com/en-us/library/0888kc6a%28VS.80%29.aspx
 ----------------------------------------------------------------------------*/
#ifndef IGL_XML_SERIALIZATION_H
#define IGL_XML_SERIALIZATION_H

#include "XMLSerializer.h" //wangyu #include <igl/xml/XMLSerializer.h>

namespace igl
{
  //namespace
  //{
    class XMLSerializer;

    class XMLSerialization : public igl::XMLSerializable
    {
    public:
      XMLSerializer* xmlSerializer;

      /**
       * Default implementation of XMLSerializable interface
       */
      virtual void Init();
      virtual bool Serialize(tinyxml2::XMLDocument* doc,tinyxml2::XMLElement* element);
      virtual bool Deserialize(tinyxml2::XMLDocument* doc,const tinyxml2::XMLElement* element);

      /**
      * Default constructor, destructor, assignment and copy constructor
      */
      XMLSerialization(const std::string& name);
      ~XMLSerialization();
      XMLSerialization(const XMLSerialization& obj);
      XMLSerialization& operator=(const XMLSerialization& obj);

      /**
      * Function which must be overriden in the subclass if you dont use
      * heap allocations (new) to create new instances.
      * It will get called if the assignment operator or copy constructor
      * is involved to update the references to the new copied data structures
      *
      * Add in this fucntion all the variables you wanna serialize like:
      * xmlSerializer->Add(var1);
      * xmlSerializer->Add(varw);
      * ...
      */
      virtual void InitSerialization();

      /**
       * Following functions can be overwritten to handle the specific events.
       * Return false to prevent serialization of object.
       */
      virtual bool BeforeSerialization();
      virtual void AfterSerialization();
      virtual bool BeforeDeserialization();
      virtual void AfterDeserialization();

    private:
      void initXMLSerializer();
    };


  //}
}

#endif
