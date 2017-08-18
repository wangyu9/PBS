#ifndef TARGA_IMAGE_H
#define TARGA_IMAGE_H

struct rgba_t
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;
};

struct rgb_t
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

enum TGATypes
{
  TGA_NODATA = 0,
  TGA_INDEXED = 1,
  TGA_RGB = 2,
  TGA_GRAYSCALE = 3,
  TGA_INDEXED_RLE = 9,
  TGA_RGB_RLE = 10,
  TGA_GRAYSCALE_RLE = 11
};

// Image Data Formats
#define	IMAGE_RGB       0
#define IMAGE_RGBA      1
#define IMAGE_LUMINANCE 2

// Image data types
#define	IMAGE_DATA_UNSIGNED_BYTE 0

// Pixel data transfer from file to screen:
// These masks are AND'd with the imageDesc in the TGA header,
// bit 4 is left-to-right ordering
// bit 5 is top-to-bottom
#define BOTTOM_LEFT  0x00	// first pixel is bottom left corner
#define BOTTOM_RIGHT 0x10	// first pixel is bottom right corner
#define TOP_LEFT     0x20	// first pixel is top left corner
#define TOP_RIGHT    0x30	// first pixel is top right corner

// TGA header
struct tgaheader_t
{
  unsigned char  idLength;
  unsigned char  colorMapType;
  unsigned char  imageTypeCode;
  unsigned char  colorMapSpec[5];
  unsigned short xOrigin;
  unsigned short yOrigin;
  unsigned short width;
  unsigned short height;
  unsigned char  bpp;
  unsigned char  imageDesc;
};

class TargaImage
{
public:
    TargaImage();
    virtual ~TargaImage();

    // loading and unloading
    bool load(const char* filename);
    void release();

    // flips image vertically
    bool flipVertical();

    unsigned short getWidth() { return width; }
    unsigned short getHeight() { return height; }
    unsigned char  getImageFormat() { return imageDataFormat; }
    unsigned char  getColorDepth() { return colorDepth; }
    unsigned char  getChanels() { return colorDepth / 8 ; }

    // converts RGB format to RGBA format and vice versa
    bool convertRGBAToRGB();
    bool convertRGBToRGBA(unsigned char alphaValue);

    // returns the current image data
    unsigned char* getImage() { return pImageData; }


private:
    void swapRedBlue();

    unsigned char   colorDepth;
    unsigned char   imageDataType;
    unsigned char   imageDataFormat;
    unsigned char*  pImageData;
    unsigned short  width;
    unsigned short  height;
    unsigned long   imageSize;
};

#endif
