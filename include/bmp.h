//
//  minimal bmp support that only handles 24 bit color images
//

#ifndef BMP_FILE_FORMAT_H
#define BMP_FILE_FORMAT_H

#include <iostream>
#include <fstream>
#include <tuple>
#include <vector>
#include <vecmatquat.h>  // int2

struct BMPHeader{
	// char bm[2];  // not part of struct due to 4 byte alignment of the int members
	int   filesize;              // 54 + dim.x * dim.y * (bpp/8)
	int   whatever        = 0;   // 4 bytes can be 0000
	int   start           = 54;  // 54
	int   dib_header_size = 40;  // 40
	int2  dim;                   //  w,h 
	short color_planes    = 1;   // 1
	short bpp             = 24;  // 24  bits per pixel
	int   pixarraycomp    = 0;   // 0
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
	std::vector<ubyte3> image;
};
BMPImage BMPRead(const char *filename)
{
	std::ifstream file(filename, std::ios::binary);
	char bm[2]; // "BM"
	file.read(bm, 2);
	BMPHeader header;
	file.read((char*)&header, sizeof(header));
	std::vector<ubyte3> image(header.imagesizebytes/3);
	file.read((char*)image.data(), header.imagesizebytes);
	for (auto &pixel: image)
		std::swap(pixel.x,pixel.z); // bgr to rgb
	return { header.dim, image };
}

void BMPWrite(const char *filename, std::vector<ubyte3> image, int2 dim)    // pass image by value since we need to swap bytes
{
	std::ofstream file(filename, std::ios::binary | std::ios::trunc );
	if (!file.is_open())
		throw("unable to open a file for binary write");
	file.write("BM", 2);
	BMPHeader header(dim);
	file.write(reinterpret_cast<const char*>(&header), sizeof(header));
	for (auto &pixel : image)
		std::swap(pixel.x, pixel.z); // rgb to bgr 
	file.write(reinterpret_cast<const char*>(image.data()), header.imagesizebytes);
	file.close();
}


#endif // BMP_FILE_FORMAT_H
