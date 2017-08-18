#include "XMLSerialization.h"

namespace igl{
	// Implementation

	void XMLSerialization::Init()
	{
	}

	bool XMLSerialization::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
	{
		bool serialized = false;

		if (this->BeforeSerialization())
		{
			if (xmlSerializer == NULL)
			{
				xmlSerializer = new XMLSerializer(Name);
				this->InitSerialization();
			}
			serialized = xmlSerializer->SaveGroupToXMLElement(doc, element, Name);
			this->AfterSerialization();
		}

		return serialized;
	}

	bool XMLSerialization::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
	{
		bool serialized = false;

		if (this->BeforeDeserialization())
		{
			if (xmlSerializer == NULL)
			{
				xmlSerializer = new XMLSerializer(Name);
				this->InitSerialization();
			}
			serialized = xmlSerializer->LoadGroupFromXMLElement(Name, doc, element);
			this->AfterDeserialization();
		}

		return serialized;
	}

	void XMLSerialization::InitSerialization()
	{
		std::cout << "You have to override InitSerialization()" << "\n";
		//assert(false);
	}

	XMLSerialization::XMLSerialization(const std::string& name)
	{
		Name = name;
		xmlSerializer = NULL;
	}

	XMLSerialization::~XMLSerialization()
	{
		if (xmlSerializer != NULL)
			delete xmlSerializer;
		xmlSerializer = NULL;
	}

	XMLSerialization::XMLSerialization(const XMLSerialization& obj)
	{
		Name = obj.Name;
		xmlSerializer = NULL;
	}

	XMLSerialization& XMLSerialization::operator=(const XMLSerialization& obj)
	{
		if (this != &obj)
		{
			Name = obj.Name;
			if (xmlSerializer != NULL)
			{
				delete xmlSerializer;
				xmlSerializer = NULL;
			}
		}
		return *this;
	}

	bool XMLSerialization::BeforeSerialization()
	{
		return true;
	}

	void XMLSerialization::AfterSerialization()
	{
	}

	bool XMLSerialization::BeforeDeserialization()
	{
		return true;
	}

	void XMLSerialization::AfterDeserialization()
	{
	}
}