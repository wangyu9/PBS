// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
/* ---------------------------------------------------------------------------
 // XMLSerializer.h
 // Author: Christian Schüller <schuellchr@gmail.com>
 -----------------------------------------------------------------------------
 
 Header library which allows to save and load a serialization of basic c++ data
 types like char, char*, std::string, bool, uint, int, float, double to and
 from a xml file.  Containers like std::vector, std::std::pair, Eigen dense and
 sparse matrices are supported as well as combinations of them (like
 vector<pair<string,bool>> or vector<vector<int>>).
 
 To serialize an arbitrary object use the XMLSerializable interface or even
 simpler the XMLSerialization class.
 
 The serialized objects are organised in groups in the xml file and have
 their own names which must be unique within one group.
 
 You can find examples how to use it in the test case class XMLSerializerTest.
 
 -----------------------------------------------------------------------------
TODOs:
* handle memory leak when deserializing to pointers
* loops of pointers and from multiple objects
* NULL pointers
* Versioning
 -----------------------------------------------------------------------------
Bugs:
* Doesn't handle RowMajor Eigen matrices
 ----------------------------------------------------------------------------- */
#ifndef IGL_XML_SERIALIZER_H
#define IGL_XML_SERIALIZER_H

#include <igl/igl_inline.h>
#include "XMLSerializable.h"//changed by wangyu #include <igl/xml/XMLSerializable.h>

#include <iostream>
//#include <array>
#include <vector>
#include <set>
#include <map>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "tinyxml2.h"

namespace igl
{
  //namespace
  //{
    // Utility functions
    void EncodeXMLElementName(std::string& name);
    void DecodeXMLElementName(std::string& name);
    void ReplaceSubString(std::string& str, const std::string& search, const std::string& replace);
    
    // Forward declaration
    class XMLSerializer;
    
    /**
     * class XMLSerializableObject
     * internal usage
     */
    class XMLSerializableObject : public ::igl::XMLSerializable
    {
    public:
      
      XMLSerializableObject(const std::string& name, const std::string& group);
      virtual ~XMLSerializableObject();
      
      // set attribute conversion functions
      void SetAttribute(tinyxml2::XMLElement* element, const char* name, char& dest);
      void SetAttribute(tinyxml2::XMLElement* element, const char* name, char*& dest);
      void SetAttribute(tinyxml2::XMLElement* element, const char* name, std::string& dest);
      void SetAttribute(tinyxml2::XMLElement* element, const char* name, bool& dest);
      void SetAttribute(tinyxml2::XMLElement* element, const char* name, unsigned int& dest);
      void SetAttribute(tinyxml2::XMLElement* element, const char* name, int& dest);
      void SetAttribute(tinyxml2::XMLElement* element, const char* name, float& dest);
      void SetAttribute(tinyxml2::XMLElement* element, const char* name, double& dest);
      
      // get attribute conversion functions
      void GetAttribute(const char* src, char& dest);
      void GetAttribute(const char* src, char*& dest);
      void GetAttribute(const char* src, std::string& dest);
      void GetAttribute(const char* src, bool& dest);
      void GetAttribute(const char* src, unsigned int& dest);
      void GetAttribute(const char* src, int& dest);
      void GetAttribute(const char* src, float& dest);
      void GetAttribute(const char* src, double& dest);
      
      // Initialization
      
      // Basic data types
      using XMLSerializable::Init;
      void Init(char& val);
      void Init(char*& val);
      void Init(std::string& val);
      void Init(bool& val);
      void Init(unsigned int& val);
      void Init(int& val);
      void Init(float& val);
      void Init(double& val);
      
      // XMLSerializable*
      template<typename T>
      void Init(T& obj);
      template<typename T>
      void Init(T*& obj);
      
      // STL containers
      /*template<typename T, int S>
       void Init(std::array<T,S>& obj);*/
      template<typename T0, typename T1>
      void Init(std::pair<T0,T1>& obj);
      template<typename T>
      void Init(std::vector<T>& obj);
      template<typename T>
      void Init(std::set<T>& obj);
      template<typename T0, typename T1>
      void Init(std::map<T0,T1>& obj);
      
      // Eigen types
      template<typename T, int R, int C>
      void Init(Eigen::Matrix<T,R,C>& obj);
      template<typename T>
      void Init(Eigen::SparseMatrix<T>& obj);
      
      // Serialization
      
      // Basic data types
      using XMLSerializable::Serialize;
      bool Serialize(char& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(char*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(std::string& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(std::string*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(bool obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(bool*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(unsigned int& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(unsigned int*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(int& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(int*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(float& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(float*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(double& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(double*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      
      // XMLSerializable*
      template<typename T>
      bool Serialize(T& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool Serialize(T*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      
      // STL containers
      
      /*template<typename T, size_t S>
       bool Serialize(std::array<T,S>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
       template<typename T, size_t S>
       bool Serialize(std::array<T,S>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);*/
      
      template<typename T0, typename T1>
      bool Serialize(std::pair<T0,T1>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      template<typename T0, typename T1>
      bool Serialize(std::pair<T0,T1>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      
      template<typename T>
      bool Serialize(std::vector<T>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool Serialize(std::vector<T>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(std::vector<bool>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(std::vector<bool>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(std::vector<int>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(std::vector<int>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);

      template<typename T>
      bool Serialize(std::set<T>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool Serialize(std::set<T>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);

      template<typename T0, typename T1>
      bool Serialize(std::map<T0,T1>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      template<typename T0, typename T1>
      bool Serialize(std::map<T0,T1>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(std::map<int,int>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool Serialize(std::map<int,int>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      
      // Eigen types
      template<typename T, int R, int C>
      bool Serialize(Eigen::Matrix<T,R,C>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      template<typename T, int R, int C>
      bool Serialize(Eigen::Matrix<T,R,C>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      
      template<typename T>
      bool Serialize(Eigen::SparseMatrix<T>& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool Serialize(Eigen::SparseMatrix<T>*& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      
      // Deserialization
      
      // Basic data types
      using XMLSerializable::Deserialize;
      bool Deserialize(char& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(char*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(std::string& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(std::string*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(bool& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(bool*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(unsigned int& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(unsigned int*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(int& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(int*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(float& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(float*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(double& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(double*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      
      // XMLSerializable*
      template<typename T>
      bool Deserialize(T& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool Deserialize(T*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      
      // STL containers
      
      /*template<typename T, size_t S>
       bool Deserialize(std::array<T,S>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
       template<typename T, size_t S>
       bool Deserialize(std::array<T,S>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);*/
      
      template<typename T0, typename T1>
      bool Deserialize(std::pair<T0,T1>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      template<typename T0, typename T1>
      bool Deserialize(std::pair<T0,T1>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      
      template<typename T>
      bool Deserialize(std::vector<T>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool Deserialize(std::vector<T>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(std::vector<bool>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(std::vector<bool>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(std::vector<int>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(std::vector<int>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);

      template<typename T>
      bool Deserialize(std::set<T>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool Deserialize(std::set<T>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);

      template<typename T0, typename T1>
      bool Deserialize(std::map<T0,T1>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      template<typename T0, typename T1>
      bool Deserialize(std::map<T0,T1>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(std::map<int,int>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      bool Deserialize(std::map<int,int>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      
      // Eigen types
      template<typename T, int R, int C>
      bool Deserialize(Eigen::Matrix<T,R,C>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      template<typename T, int R, int C>
      bool Deserialize(Eigen::Matrix<T,R,C>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      
      template<typename T>
      bool Deserialize(Eigen::SparseMatrix<T>& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool Deserialize(Eigen::SparseMatrix<T>*& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
      
    private:
      
      template<typename T>
      bool setElementAttribute(T& obj, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      template<typename T>
      bool getElementAttribute(T& obj, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element, const std::string& name);
    };
    
    /**
     * class XMLSerializableInstance
     * internal usage
     */
    template<typename T>
    class XMLSerializableInstance : public XMLSerializableObject
    {
    public:
      
      T& Object;
      T DefaultValue;
      
      XMLSerializableInstance(T& obj, const std::string& name, const std::string group);
      XMLSerializableInstance(T& obj, const std::string& name, const std::string group, T defaultValue);
      ~XMLSerializableInstance();
      
      // XMLSerializable interface implementation
      void Init();
      bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
      bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);
    };
    
    /**
     * struct XMLSerializerGroup
     * internal usage
     */
    struct XMLSerializerGroup
    {
      std::string Name;
      std::vector<XMLSerializable*>* Objects;
    };
    
    /**
     * class XMLSerializer
     * This is the core class which takes care of saving and loading of serialization of object structures.
     */
    class XMLSerializer
    {
    public:
      
      /**
       * Serializes an object to a file
       */
      //template<typename T>
      //static bool SaveObject(T& object, const char* filename);
      //template<typename T>
      //static bool SaveObject(T& object, const std::string& objectName, const std::string& groupName, const char* filename, bool overwrite);
      //
      ///**
      // * Loads the serialization of an object from a file.
      // */
      //template<typename T>
      //static bool LoadObject(T& object, const char* filename);
      //template<typename T>
      //static bool LoadObject(T& object, const std::string& objectName, const std::string& groupName, const char* filename);
      
		template<typename T>
		static bool SaveObject(T& object, const char* filename)//wangyu
		{
			return SaveObject(object, "Object", "Serialization", filename, true);
		}

		template<typename T>
		static bool SaveObject(T& object, const std::string& objectName, const std::string& groupName, const char* filename, bool overwrite)//wangyu
		{
			bool result = true;
			XMLSerializer* serializer = new XMLSerializer(groupName);
			result &= serializer->Add(object, objectName);
			result &= serializer->Save(objectName, groupName, filename, overwrite);
			delete serializer;
			return result;
		}

		template<typename T>
		static bool LoadObject(T& object, const char* filename)
		{
			return LoadObject(object, "Object", "Serialization", filename);
		}

		template<typename T>
		static bool LoadObject(T& object, const std::string& objectName, const std::string& groupName, const char* filename)
		{
			bool result = true;
			XMLSerializer* serializer = new XMLSerializer(groupName);
			result &= serializer->Add(object, objectName);
			result &= serializer->Load(objectName, groupName, filename);
			delete serializer;
			return result;
		}



      /**
       * Constructor which sets the default group
       */
      XMLSerializer(const std::string& defaultGroup);
      ~XMLSerializer();
      
      /**
       * Save the serialization of all groups to file.
       * Parameter overwrite specifies if file gets overwritten or updated
       */
      bool Save(const char* filename, bool overwrite);
      bool Save(const std::string& groupName, const char* filename, bool overwrite);
      bool Save(const std::string& objectName, const std::string& groupName, const char* filename, bool overwrite);
      
      /**
       * Save the serialization of all groups to a XMLDocument instance.
       */
      bool SaveToXMLDoc(tinyxml2::XMLDocument* doc);
      bool SaveToXMLDoc(const std::string& groupName, tinyxml2::XMLDocument* doc);
      bool SaveToXMLDoc(const std::string& objectName, const std::string& groupName, tinyxml2::XMLDocument* doc);
      
      /**
       * Save the serialization of a group with a new provided name to given XMLElement instance.
       */
      bool SaveGroupToXMLElement(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      bool SaveGroupToXMLElement(const std::string& groupName, tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element, const std::string& name);
      
      /**
       * Load the serialization from a file.
       */
      bool Load(const char* filename);
      bool Load(const std::string& groupName, const char* filename);
      bool Load(const std::string& objectName, const std::string& groupName, const char* filename);
      
      /**
       * Load the serialization from an XMLDocument instance.
       */
      bool LoadFromXMLDoc(tinyxml2::XMLDocument* doc);
      bool LoadFromXMLDoc(const std::string& groupName, tinyxml2::XMLDocument* doc);
      bool LoadFromXMLDoc(const std::string& objectName, const std::string& groupName, tinyxml2::XMLDocument* doc);
      
      /**
       * Load the serialization from a XMLElement instance to given group.
       */
      bool LoadGroupFromXMLElement(const std::string& groupName, tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);
      
      /**
       * Set/Get current group. Every object which is added afterwards will be in this group, except it specifies another group.
       */
      void SetCurrentGroup(const std::string& group);
      std::string GetCurrentGroup();
      
      /**
       * Add an object to the serializer. Can be simple types like char, char*, string, unsigned int, int, float, double or containers like std::array, std::pair, std::vector.
       * Also Eigen dense or sparse matrices are supported and all objects of type Serializable* and combinations of thoses types like vector<vector>, vector<pair> or even vector<pair<vector,Serializable*>>>.
       * Also pointers to those objects can be used (for instance like vector<vector<pair<int,float>*>*>).
       * char* is also possible as base type and represents a array of chars, but be carefull that the pointer is not just a copy but a valid instance in the current programm scope.
       */
      
      // Basic types
      bool Add(char& obj, const std::string& name);
      bool Add(char*& obj, const std::string& name);
      bool Add(std::string& obj, const std::string& name);
      bool Add(bool& obj, const std::string& name);
      bool Add(unsigned int& obj, const std::string& name);
      bool Add(int& obj, const std::string& name);
      bool Add(float& obj, const std::string& name);
      bool Add(double& obj, const std::string& name);
      
      bool Add(char& obj, const std::string& name, char defaultValue);
      bool Add(char*& obj, const std::string& name, char* defaultValue);
      bool Add(std::string& obj, const std::string& name, std::string defaultValue);
      bool Add(bool& obj, const std::string& name, bool defaultValue);
      bool Add(unsigned int& obj, const std::string& name, unsigned int defaultValue);
      bool Add(int& obj, const std::string& name, int defaultValue);
      bool Add(float& obj, const std::string& name, float defaultValue);
      bool Add(double& obj, const std::string& name, double defaultValue);
      
      // XMLSerializable*
      template<typename T>
      bool Add(T& object, const std::string& name);
      template<typename T>
      bool Add(T& object, const std::string& name, T defaultValue);
      
      // STL containers
      /*template<typename T, size_t S>
       bool Add(std::array<T,S>& obj, const std::string& name);*/
      template<typename T0, typename T1>
      bool Add(std::pair<T0,T1>& obj, const std::string& name);
      template<typename T>
      bool Add(std::vector<T>& obj, const std::string& name);
      template<typename T>
      bool Add(std::set<T>& obj, const std::string& name);
      template<typename T0, typename T1>
      bool Add(std::map<T0,T1>& obj, const std::string& name);
      
      // Eigen types
      template<typename T, int R, int C>
      bool Add(Eigen::Matrix<T,R,C>& obj, const std::string& name);
      template<typename T>
      bool Add(Eigen::SparseMatrix<T>& obj, const std::string& name);
      
    private:
      
      std::map<std::string,XMLSerializerGroup*>::iterator currentGroup;
      std::map<std::string,XMLSerializerGroup*> groups;
      
      template<typename T>
      bool add(T& object, const std::string& name);
      template<typename T>
      bool add(T& object, const std::string& name, T defaultValue);
      bool addObjectToGroup(XMLSerializable* object, const std::string& group);
      bool addObjectToGroup(XMLSerializable* object, std::map<std::string,XMLSerializerGroup*>::iterator it);
      std::map<std::string,XMLSerializerGroup*>::iterator setGetGroup(const std::string& group);
      tinyxml2::XMLDocument* openDoc(const char* filename);
      tinyxml2::XMLElement* findAddGroup(tinyxml2::XMLDocument* doc, const char* groupName);
    };
    

  //}
}
#endif
