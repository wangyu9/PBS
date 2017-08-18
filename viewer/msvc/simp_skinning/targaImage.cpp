// this targa image reader is transformed from "more opengl game programming" book

#include "targaImage.h"
#include <fstream>
using namespace std;


void cpmem(void *dst, void * src, int numBytes)
{
    if (numBytes < 1) return;

    unsigned char * d = static_cast<unsigned char *>(dst);
    unsigned char * s = static_cast<unsigned char *>(src);
    for (int i=0; i<numBytes; i++)
        d[i] = s[i];
}

 TargaImage::TargaImage():pImageData(0)
 {
 }

TargaImage::~TargaImage()
 {
     release();
 }

// loading and unloading
bool TargaImage::load(const char* filename)
{
    ifstream imgFile(filename, ios_base::binary);

	if (!imgFile)
		return false;

	tgaheader_t tgaHeader;

	// read the TGA header
	imgFile.read(reinterpret_cast<char*>(&tgaHeader),sizeof(tgaheader_t));

	// see if the image type is one that we support (RGB, RGB RLE, GRAYSCALE, GRAYSCALE RLE)
	if ( ((tgaHeader.imageTypeCode != TGA_RGB) && (tgaHeader.imageTypeCode != TGA_GRAYSCALE) &&
		 (tgaHeader.imageTypeCode != TGA_RGB_RLE) && (tgaHeader.imageTypeCode != TGA_GRAYSCALE_RLE)) ||
		 tgaHeader.colorMapType != 0)
	{
		imgFile.close();
		return false;
	}

	// get image width and height
	width = tgaHeader.width;
	height = tgaHeader.height;

	// colormode -> 3 = BGR, 4 = BGRA
	int colorMode = tgaHeader.bpp / 8;

	// we don't handle less than 24 bit
	if (colorMode < 3)
	{
		imgFile.close();
		return false;
	}

	imageSize = width * height * colorMode;

	// allocate memory for TGA image data
	pImageData = new unsigned char[imageSize];

	// skip past the id if there is one
	if (tgaHeader.idLength > 0)
		imgFile.seekg(tgaHeader.idLength, ios_base::cur);

	// read image data
	if (tgaHeader.imageTypeCode == TGA_RGB || tgaHeader.imageTypeCode == TGA_GRAYSCALE)
	{
	    imgFile.read(reinterpret_cast<char*>(pImageData), imageSize);
		//fread(pImageData, 1, m_imageSize, pFile);
	}
	else
	{
		// this is an RLE compressed image
		unsigned char id;
		unsigned char length;
		rgba_t color = { 0, 0, 0, 0 };
		unsigned int i = 0;

		while (i < imageSize)
		{
			imgFile.read(reinterpret_cast<char*>(&id),1);

			// see if this is run length data
			if (id >= 128)// & 0x80)
			{
				// find the run length
				length = (unsigned char)(id - 127);

				// next 3 (or 4) bytes are the repeated values
				imgFile.read(reinterpret_cast<char*>(&color.b),1);
				imgFile.read(reinterpret_cast<char*>(&color.g),1);
				imgFile.read(reinterpret_cast<char*>(&color.r),1);

				if (colorMode == 4)
                    imgFile.read(reinterpret_cast<char*>(&color.a),1);

				// save everything in this run
				while (length > 0)
				{
					pImageData[i++] = color.b;
					pImageData[i++] = color.g;
					pImageData[i++] = color.r;

					if (colorMode == 4)
						pImageData[i++] = color.a;

					--length;
				}
			}
			else
			{
				// the number of non RLE pixels
				length = static_cast<unsigned char>(id + 1);

				while (length > 0)
				{
					imgFile.read(reinterpret_cast<char*>(&color.b),1);
                    imgFile.read(reinterpret_cast<char*>(&color.g),1);
                    imgFile.read(reinterpret_cast<char*>(&color.r),1);

					if (colorMode == 4)
						imgFile.read(reinterpret_cast<char*>(&color.a),1);

					pImageData[i++] = color.b;
					pImageData[i++] = color.g;
					pImageData[i++] = color.r;

					if (colorMode == 4)
						pImageData[i++] = color.a;

					--length;
				}
			}
		}
	}

	switch(tgaHeader.imageTypeCode)
	{
	case TGA_RGB:
	case TGA_RGB_RLE:
		if (3 == colorMode)
		{
			imageDataFormat = IMAGE_RGB;
			imageDataType = IMAGE_DATA_UNSIGNED_BYTE;
			colorDepth = 24;
		}
		else
		{
			imageDataFormat = IMAGE_RGBA;
			imageDataType = IMAGE_DATA_UNSIGNED_BYTE;
			colorDepth = 32;
		}
		break;

	case TGA_GRAYSCALE:
	case TGA_GRAYSCALE_RLE:
		imageDataFormat = IMAGE_LUMINANCE;
		imageDataType = IMAGE_DATA_UNSIGNED_BYTE;
		colorDepth = 8;
		break;
	}

	if ((tgaHeader.imageDesc & TOP_LEFT) == TOP_LEFT)
		flipVertical();

	// swap the red and blue components in the image data
	swapRedBlue();

	return (pImageData);
}

void TargaImage::release()
{
    delete [] pImageData;
    pImageData = 0;
}

// flips image vertically
bool TargaImage::flipVertical()
{
   	if (!pImageData)
		return false;

	if (colorDepth == 32)
	{
		rgba_t* tmpBits = new rgba_t[width];
		if (!tmpBits)
			return false;

		int lineWidth = width * 4;

		rgba_t* top = (rgba_t*)pImageData;
		rgba_t* bottom = (rgba_t*)(pImageData + lineWidth * (height-1));

		for (int i = 0; i < (height / 2); ++i)
		{
			cpmem(tmpBits, top, lineWidth);
			cpmem(top, bottom, lineWidth);
			cpmem(bottom, tmpBits, lineWidth);

			top = (rgba_t*)((unsigned char*)top + lineWidth);
			bottom = (rgba_t*)((unsigned char*)bottom - lineWidth);
		}

		delete [] tmpBits;
		tmpBits = 0;
	}
	else if (colorDepth == 24)
	{
		rgb_t* tmpBits = new rgb_t[width];
		if (!tmpBits)
			return false;

		int lineWidth = width * 3;

		rgb_t* top = (rgb_t*)pImageData;
		rgb_t* bottom = (rgb_t*)(pImageData + lineWidth * (height-1));

		for (int i = 0; i < (height / 2); ++i)
		{
			cpmem(tmpBits, top, lineWidth);
			cpmem(top, bottom, lineWidth);
			cpmem(bottom, tmpBits, lineWidth);

			top = (rgb_t*)((unsigned char*)top + lineWidth);
			bottom = (rgb_t*)((unsigned char*)bottom - lineWidth);
		}

		delete [] tmpBits;
		tmpBits = 0;
	}

	return true;
}

bool TargaImage::convertRGBAToRGB()
{
   	if ((colorDepth == 32) && (imageDataFormat == IMAGE_RGBA))
	{
		rgb_t* newImage = new rgb_t[width * height];

		if (!newImage)
			return false;

		rgb_t* dest = newImage;
		rgba_t* src = (rgba_t*)pImageData;

		for (int x = 0; x < height; x++)
		{
			for (int y = 0; y < width; y++)
			{
				dest->r = src->r;
				dest->g = src->g;
				dest->b = src->b;

				++src;
				++dest;
			}
		}

		delete [] pImageData;
		pImageData = (unsigned char*)newImage;

		colorDepth = 24;
		imageDataType = IMAGE_DATA_UNSIGNED_BYTE;
		imageDataFormat = IMAGE_RGB;

		return true;
	}

	return false;
}

bool TargaImage::convertRGBToRGBA(unsigned char alphaValue)
{
   	if ((colorDepth == 24) && (imageDataFormat == IMAGE_RGB))
	{
		rgba_t* newImage = new rgba_t[width * height];

		if (!newImage)
			return false;

		rgba_t* dest = newImage;
		rgb_t* src = (rgb_t*)pImageData;

		for (int x = 0; x < height; x++)
		{
			for (int y = 0; y < width; y++)
			{
				dest->r = src->r;
				dest->g = src->g;
				dest->b = src->b;
				dest->a = alphaValue;

				++src;
				++dest;
			}
		}

		delete [] pImageData;
		pImageData = (unsigned char*)newImage;

		colorDepth = 32;
		imageDataType = IMAGE_DATA_UNSIGNED_BYTE;
		imageDataFormat = IMAGE_RGBA;

		return true;
	}

	return false;
}

void TargaImage::swapRedBlue()
{
  	switch (colorDepth)
	{
	case 32:
		{
			unsigned char temp;
			rgba_t* source = (rgba_t*)pImageData;

			for (int pixel = 0; pixel < (width * height); ++pixel)
			{
				temp = source[pixel].b;
				source[pixel].b = source[pixel].r;
				source[pixel].r = temp;
			}
		} break;
	case 24:
		{
			unsigned char temp;
			rgb_t* source = (rgb_t*)pImageData;

			for (int pixel = 0; pixel < (width * height); ++pixel)
			{
				temp = source[pixel].b;
				source[pixel].b = source[pixel].r;
				source[pixel].r = temp;
			}
		} break;
	default:
		// ignore other color depths
		break;
	}
}
