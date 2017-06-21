
#pragma once
#ifndef HTML_REFLECT_H
#define HTML_REFLECT_H

#include "misc.h"
#include "minixml.h"
#include "tcpsocket.h"
#include "bmp.h"
#include "misc_image.h"



//  Reflection to html forms, and assignment from urls
template<class T> xmlNode  to_html(const bool  &d, const char *name) { xmlNode n("p"); n.body = ToString() << name << " " << (int)d; return n; }

template<class T> typename std::enable_if<!std::is_class<T>::value, xmlNode>::type  
to_html(T   d, const char *name) 
{ 
	xmlNode n("p"); n.body = ToString() << name << " " << d; return n; 
}

xmlNode to_html(std::string d, const char * name) { xmlNode n("p");n.body = std::string(name) + " " + d;return n; }

template<class T> xmlNode to_html(const std::vector<T>  &array, const char *name) 
{ 
	xmlNode n("blockquote"); 
	n.body = ToString() << name << " "; 
	for (const auto &d : array) 
		n.children.push_back(to_html(d, "child_i"));  
	return n; 
}

struct html_encoder 
{ 
	xmlNode &n; 
	template<class T, class... TS> void operator () (const char * name, const T & field, TS...) {n.children.push_back(to_html(field, name)); } 
};

template<class T> typename std::enable_if<std::is_class<T>::value, xmlNode>::type 
to_html(const T & o, const char* name) 
{ 
	xmlNode n("UL");
	n.body = name; 
	visit_fields(const_cast<T &>(o), html_encoder{ n }); 
	return n; 
}


xmlNode to_form(const std::string &d, const char *url, const char *name,const char *form_input_type)
{
	xmlNode n("form");
	n.attributes.push_back({ "action",url });
	n.attributes.push_back({ "target","output" });
	n.children.push_back(xmlNode("b"));n.children.back().body = name;
	n.children.push_back(xmlNode("input")); n.children.back().attributes.push_back({ "name",name });n.children.back().attributes.push_back({ "type", form_input_type });n.children.back().attributes.push_back({ "value",d.c_str() });
	n.children.push_back(xmlNode("button")); n.children.back().attributes.push_back({ "type", "submit" });n.children.back().body = "submit";
	return n;
}

xmlNode to_form(const std::string &d, const char *url, const char *name) { return to_form(d, url, name, "text"); }
template<class T> typename std::enable_if<std::is_arithmetic<T>::value, xmlNode>::type  to_form(const T   &d, const char* url, const char *name)
{
	return to_form((ToString() << d).str(), url, name,"number");
}
xmlNode to_form(const float &d, const char *url, const char *name) 
{ 
	auto n = to_form((ToString() << d).str(), url, name, "number"); 
	n.child("input").attributes.push_back({ "step","any" });
	return n; 
}
xmlNode to_form(const bool &d, const char *url, const char *name)
{
	xmlNode n("form");
	n.attributes.push_back({ "action",url });
	n.attributes.push_back({ "target","output" });
	n.children.push_back(xmlNode("b"));n.children.back().body = name;

	for (bool buttonstate : { false,true })
	{
		xmlNode t("i"); t.body = buttonstate ? " true:" : " false:";
		n.children.push_back(t);
		xmlNode r("input"); 
		r.attributes.push_back({ "name",name });
		r.attributes.push_back({ "type", "radio" });
		r.attributes.push_back({ "value",buttonstate?"1":"0" });
		if(buttonstate==d)
			r.attributes.push_back({ "checked","true" });
		n.children.push_back(r);
	}
	n.children.push_back(xmlNode("button")); n.children.back().attributes.push_back({ "type", "submit" });n.children.back().body = "submit";
	return n;
}
template<class T> typename std::enable_if<std::is_arithmetic<T>::value, xmlNode>::type  to_form(const T   &d, const char* url, const char *name,linalg::vec<T,2> range)
{
	auto n = to_form(d, url, name);
	n.child("input").attributes.push_back({ "min",(ToString() << range.x).str().c_str() });
	n.child("input").attributes.push_back({ "max",(ToString() << range.y).str().c_str() });
	return n;
}

template<class T> xmlNode to_form(const std::vector<T>  &array, const char* url, const char *name)
{
	xmlNode n("BLOCKQUOTE");
	int i = 0;
	for (const auto &d : array)
		n.children.push_back(to_form(d, url, (ToString() << name << '.' << i++).str().c_str()));
	return n;
}

struct to_form_encoder 
{ 
	xmlNode &n;const char *url; std::string pathname; 
	template<class T, class... TS> void operator () (const char * name, const T & field, TS...) { n.children.push_back(to_form(field, url, (pathname + "." + name).c_str())); } 
	template<class T, class... TS> void operator () (const char * name, const T & field, linalg::vec<T, 2> range,TS...) { n.children.push_back(to_form(field, url, (pathname + "." + name).c_str(),range)); }
};
template<class T> typename std::enable_if<std::is_class<T>::value, xmlNode>::type to_form(const T & o, const char *url, const char* pathname) { xmlNode n("blockquote");visit_fields(const_cast<T &>(o), to_form_encoder{ n,url,pathname }); return n; }

template<class T> xmlNode to_form_html_doc(const T& o, const char *url = "http://localhost:12345/")
{
	xmlNode root("html");
	root.children.push_back("head");
	root.children.push_back("body");
	root.child("body").children.push_back("p");
	root.child("body").children.push_back(to_form(o, url, ""));
	xmlNode iframe("iframe");
	iframe.attributes.push_back({ "name","output" });
	iframe.attributes.push_back({ "height","600" });
	iframe.attributes.push_back({ "width","1000" });
	root.child("body").children.push_back(std::move(iframe));
	return root;
}



template<class T> typename std::enable_if<!std::is_class<T>::value, std::string>::type  assigner(T &d, const char *value) { if (*value && *value == '=') value++; d = FromString(value); return ToString() << d; }
std::string assignerb(bool &d, const char *value) { if (*value && *value == '=') value++; d = FromString(value); return ToString() << d; }
std::string  assigner(std::string &d, const char *value) { if (*value && *value == '=') value++; d = value; return d; }
template<class T> std::string assigner(std::vector<T> &array, const char *str)
{
	std::cout << "vector assigner " << str << std::endl;
	while (*str && (*str == '.' || *str == '/' || *str == '?'))
		str++;
	const char *t = str;
	while (isdigit(*t))
		t++;
	std::string idx(str, t);
	int i = FromString(idx);
	std::cout << "assigning " << i << " with " << t << std::endl;
	return assigner((T&)array[i], t);
	std::cout << "done\n";
}
struct assigner_encoder {
	const char *str;
	ToString &ts;
	template<class T, class... TS> void operator () (const char * fname, T & field, TS...)
	{
		std::cout << "visiting fields " << fname << "  with " << str << std::endl;
		if (!strncmp(fname, str, strlen(fname)))
			ts << assigner(field, str + strlen(fname));
		std::cout << "done\n";
	}
};
template<class T> typename std::enable_if<std::is_class<T>::value, std::string>::type assigner(T & o, const char* str)
{
	ToString ts;
	while (*str && (*str == '.' || *str == '/' || *str == '?'))
		str++;
	visit_fields(o, assigner_encoder{ str,ts });
	return ts;
}

inline SOCKET http_reply_image(SOCKET s,Image<byte3> image)
{
	for (auto &pixel : image.raster)
		std::swap(pixel.x, pixel.z); // rgb to bgr 
	FlipV(image.raster, image.dim());
	BMPHeader bmp_header(image.dim());
	int content_size = 2 + sizeof(bmp_header) + product(image.dim())*sizeof(*image.raster.data());
	std::string http_header = ToString() << "HTTP/1.0 200 OK\nAccess-Control-Allow-Origin: *\nContent-Length: " << content_size << "\nContent Type: image/bmp\n\n";
	int rc = send(s, http_header.c_str(), http_header.length(), 0);
	rc = send(s, "BM", 2, 0);
	rc = send(s, reinterpret_cast<const char*>(&bmp_header), sizeof(bmp_header), 0);
	rc = send(s, reinterpret_cast<const char*>(image.raster.data()), product(image.dim())*sizeof(*image.raster.data()), 0);//bmp_header.imagesizebytes
	return s;
}

#endif  HTML_REFLECT_H
