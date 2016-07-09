//
//  minimal bmp support that only handles 24 bit color images
//

#ifndef BMP_FILE_FORMAT_H
#define BMP_FILE_FORMAT_H

#include <iostream>
#include <fstream>
#include <tuple>
#include <vector>
#include <exception>

#include "linalg.h"   
using namespace linalg::aliases; // int2 byte3 

inline std::vector<byte3> monotorgb(const std::vector<unsigned char> &src)
{
	std::vector<byte3> dst(src.size());
	std::transform(src.begin(), src.end(), dst.begin(), [](unsigned char c){return byte3(c, c, c); });
	return dst;
}
inline std::vector<byte3> shorttorgb_r(const std::vector<unsigned short> &src)
{
	std::vector<byte3> dst(src.size());
	// this color ramp is a bit of a hack right now, just trying to visualize output for sanity check   object at about 50cm away.
	auto map = [](unsigned short c){return (c==0)?byte3(0,0,0): byte3(c > 512 ? 0 : std::min(255, 512 - c), c > 512 ? 0 : std::min(255, (512 - c)*2), c > 768 ? 0 : std::min(255, (768 - c)/2)); };
	std::transform(src.begin(), src.end(), dst.begin(),map);
	return dst;
}

struct BMPHeader{
	// char bm[2];  // not part of struct due to 4 byte alignment of the int members
	int   filesize;              // 54 + dim.x * dim.y * (bpp/8)
	int   whatever        = 0;   // 4 bytes can be 0000
	int   start           = 54;  // 54
	int   dib_header_size = 40;  // 40  size of remainder of this struct, which is same as windows.h -> wingdi.h -> BITMAPINFOHEADER or struct tagBITMAPINFOHEADER
	int2  dim;                   //  w,h width,height
	short color_planes    = 1;   // 1
	short bpp             = 24;  // 24  bits per pixel
	int   pixarraycomp    = 0;   // 0  compression flag   BI_RGB in wingdi.h
	int   imagesizebytes;        // w*h*bpp/8  or filesize-54
	int   pix_per_meter_x = 0;   // 2835 or 0
	int   pix_per_meter_y = 0;   // 2835 or 0
	int   palettesize     = 0;   // 0
	int   colors_important= 0;   // 0
	BMPHeader(){};
	BMPHeader(int2 dim) :dim(dim), imagesizebytes(dim.x*dim.y * 24 / 8), filesize(dim.x*dim.y * 24 / 8 + 54){}
};
static_assert(sizeof(BMPHeader) == 54 - 2, "incorrect header size");

struct BMPImage
{
	int2 dim;   // w,h
	std::vector<byte3> image;
};
inline BMPImage BMPRead(const char *filename)
{
	std::ifstream file(filename, std::ios::binary);
	if (!file.is_open())
		throw(std::exception((std::string("BMPRead failed to open: ") + filename).c_str()));
	char bm[2]; // "BM"
	file.read(bm, 2);
	BMPHeader header;
	file.read((char*)&header, sizeof(header));
	std::vector<byte3> image(header.imagesizebytes/3);
	file.read((char*)image.data(), header.imagesizebytes);
	for (auto &pixel: image)
		std::swap(pixel.x,pixel.z); // bgr to rgb
	return { header.dim, image };
}

inline void BMPWrite(const char *filename, std::vector<byte3> image, int2 dim)    // pass image by value since we need to swap bytes
{
	std::ofstream file(filename, std::ios::binary | std::ios::trunc );
	if (!file.is_open())
		throw(std::exception("unable to open a file for binary write"));
	file.write("BM", 2);
	BMPHeader header(dim);
	file.write(reinterpret_cast<const char*>(&header), sizeof(header));
	for (auto &pixel : image)
		std::swap(pixel.x, pixel.z); // rgb to bgr 
	file.write(reinterpret_cast<const char*>(image.data()), header.imagesizebytes);
	file.close();
}

template<class T> inline std::vector<T> & FlipV(std::vector<T> &image, int2 dim)  // ugh
{
	for (int y = 0; y < dim.y / 2; y++)	for (int x = 0; x < dim.x; x++)
		std::swap(image[y*dim.x + x], image[(dim.y - 1 - y)*dim.x + x]);
	return image;
}

inline std::vector<byte3>  ShortToRGB(const std::vector<unsigned short> &src, std::function<unsigned char(short)> f)
{
	auto ColorEncode = [&f](unsigned short s) ->byte3 {
		return byte3(f(s), 
			(s & 1) >> 0 | (s & 4) >> 1 | (s & 16) >> 2 | (s & 64 ) >> 3 | (s & 256) >> 4 | (s & 1024) >> 5 | (s & 4096) >> 6 | (s & 16384) >> 7 ,
			(s & 2) >> 1 | (s & 8) >> 2 | (s & 32) >> 3 | (s & 128) >> 4 | (s & 512) >> 5 | (s & 2048) >> 6 | (s & 8192) >> 7 | (s & 32768) >> 8
			//reinterpret_cast<unsigned char*>(&s)[0], reinterpret_cast<unsigned char*>(&s)[1]
			);
	};
	std::vector<byte3> image(src.size());
	for (unsigned int i = 0; i < src.size(); i++)
	{
		image[i] = ColorEncode(src[i]);
	}
	return image;
}
inline void BMPFromShortR(const char *filename, const std::vector<unsigned short> &src, int2 dim, std::function<unsigned char(short)> f = [](short s)->unsigned char{return (s >= 512) ? 0 : 255 - ((s >> 1) & 255); })
{
	BMPWrite(filename, FlipV(ShortToRGB(src, f),dim), dim);
}
inline void BMPFromShortC(const char *filename, const std::vector<unsigned short> &src, int2 dim, std::function<unsigned char(short)> f = [](short s)->unsigned char{return (s >= 1024) ? 255 : ((s >> 2) & 255); })
{
	BMPWrite(filename, FlipV(ShortToRGB(src, f),dim), dim);
}

inline std::vector<unsigned short> RGBToShort(const std::vector<byte3> &src,int2 dim)
{
	std::vector<unsigned short> data(src.size());
	auto ColorDecode = [](byte3 c) -> unsigned short{
		unsigned short r = 0;
		for (int i = 0; i < 8; i++)
		{
			r |=  (c.y&(1 << i)) << i;
			r |=  (c.z&(1 << i)) << (i+1);
		}
		return r; //  c.y + (c.z << 8);
	};
	for (unsigned int i = 0; i < src.size(); i++)
	{
		data[i] = ColorDecode(src[i]);
	}
	FlipV(data, dim);
	return data;
}






#endif // BMP_FILE_FORMAT_H
