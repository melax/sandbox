//
//  a small xml parser 
// doesn't fully meet spec
//

#ifndef SANDBOX_MINI_XML_PARSE_H
#define SANDBOX_MINI_XML_PARSE_H

#include <string>
#include <iostream>
#include <fstream>
#include <strstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <exception>

class xmlNode
{
public:
	class Attribute
	{
	  public:
		std::string key;
		std::string value;
	};
	std::string tag;
	std::vector<Attribute> attributes;  
	std::vector<xmlNode>  children;
	std::string body;
	xmlNode *        haschild(const char *s)          { for (unsigned int i = 0; i < children.size(); i++)if (s == children[i].tag)return &children[i]; return NULL; }
	const xmlNode *  haschild(const char *s) const    { for (unsigned int i = 0; i < children.size(); i++)if (s == children[i].tag)return &children[i]; return NULL; }
	xmlNode &        child(const char *s)             { if (auto c=haschild(s)) return *c;  children.push_back(xmlNode(s)); return children.back(); }
	const xmlNode &  child(const char *s)    const    { if (auto c=haschild(s)) return *c;  throw(std::exception((std::string("xml tree missing expected node ")+s+" in "+tag).c_str())); }
	Attribute *      hasAttribute(const char *s)      { auto iter = std::find_if(attributes.begin(), attributes.end(), [s](Attribute&a){return s == a.key; }); return (iter != attributes.end()) ? &*iter : NULL; }
	const Attribute *hasAttribute(const char *s)const { for(unsigned int i=0;i<attributes.size();i++)if(s==attributes[i].key)return &attributes[i];return NULL;}
	std::string &    attribute(const char *s)         { Attribute *a = hasAttribute(s); if (a)return a->value; attributes.push_back({s,""}); return attributes.back().value; }
	std::string      attribute(const char *s)const    { auto a = hasAttribute(s);  return (a) ? a->value : ""; }

	xmlNode(const char *_tag):tag(_tag){}
	xmlNode(){}
	//xmlNode(const xmlNode &a) = delete;
	~xmlNode(){}
};

inline int         IsOneOf    (const char  e, const char *p        ) { while (*p) if (*p++ == e) return 1; return 0; }
inline const char *SkipChars  (const char *s, const char *delimeter) { while (*s &&  IsOneOf(*s, delimeter)){ s++; } return s; }
inline const char *SkipToChars(const char *s, const char *tokens   ) { while (*s && !IsOneOf(*s, tokens   )){ s++; } return s; }

inline std::string NextToken(const char *&s)
{
	const char *t;
	std::string token("");
	s = SkipChars(s, " \t\n\r");
	t = s;
	if (!*s) 
	{
		token = "";
		return token;
	}
	if (*t == '\"') 
	{
		s = t = t + 1;
		s = SkipToChars(s, "\"");
		token = std::string(std::string(t, s));
		if (*s) s++;
		return token;
	}
	if (IsOneOf(*t, "<>!?=/")) 
	{
		s++;
		token = std::string(t, t + 1);
		return token;
	}
	s = SkipToChars(s, "<>!?=/ \r\t\n");
	token = std::string(t, s);
	s = SkipChars(s, " \t\n\r");
	return token;
}

inline  xmlNode XMLParse(const char *&s) 
{
	std::string token;
	while (token != "<" || IsOneOf(*s, "!?")) 
	{
		token=NextToken(s);
		assert(*s);
	}
	xmlNode elem(NextToken(s).c_str());
	while (*s && (token=NextToken(s)) != ">" && token != "/") 
	{
		std::string &newval = elem.attribute(token.c_str());
		token = NextToken(s);
		assert(token == "=");
		newval = NextToken(s).c_str();
	}
	if (token == "/") 
	{
		token = NextToken(s);
		assert(token == ">");
		return elem; // no children
	}
	assert(token == ">");
	const char *t = SkipChars(s, " \t\n\r");;
	token = NextToken(s);
	while (token != "<" || *s != '/') 
	{
		if (token == "<") 
		{
			s = t;// rewind a bit
			elem.children.push_back(XMLParse(s));
			t = SkipChars(s, " \t\n\r");;
			token = NextToken(s);
		}
		else 
		{
			
			s = SkipToChars(s, "<");
			elem.body = elem.body + std::string(t, s);
			t = s;
			token = NextToken(s);
		}
	}
	assert(*s == '/');
	token = NextToken(s);
	token = NextToken(s);
	assert(token == elem.tag);
	token = NextToken(s);
	assert(token == ">");
	return elem;
}


inline xmlNode XMLParseFile(const char *filename) 
{
	if (!filename || !*filename) return NULL;
	std::ifstream file(filename, std::ios::binary|std::ios::ate); 
	if (!file.is_open())
		throw(std::exception((std::string("File Not Found: ") + filename).c_str()));
	auto len = file.tellg();
	file.seekg(0, std::ios::beg);
	std::string mem((size_t)len+1,'\0');
	file.read(&mem[0], len);
	file.close();
	const char *s = mem.data();
	return XMLParse(s);
}


inline void XMLSaveFile(const xmlNode &elem, FILE *fp)
{
	static int depth = 0;
	int singleline = (elem.children.size() == 0 && elem.body.size()<60);
	auto indent = [fp](){for (int i = 0; i < depth; i++) fprintf(fp, " "); };
	indent();
	fprintf(fp, "<%s", (const char*)elem.tag.c_str());
	for (unsigned int i = 0; i<elem.attributes.size(); i++) {
		fprintf(fp, " %s=\"%s\"", (const char*)elem.attributes[i].key.c_str(), (const char*)elem.attributes[i].value.c_str());
	}
	fprintf(fp, (singleline) ? ">" : ">\n");
	depth += 2;
	for (unsigned int i = 0; i<elem.children.size(); i++) {
		XMLSaveFile(elem.children[i], fp);
	}
	if (elem.body != "") {
		if (!singleline) indent();
		fprintf(fp, "%s%s", elem.body.c_str(), (singleline) ? "" : "\n");
	}
	depth -= 2;
	if (!singleline) indent();
	fprintf(fp, "</%s>\n", (const char*)elem.tag.c_str());
}
inline void XMLSaveFile(const xmlNode &elem, const char *filename)
{
	FILE *fp=NULL;
	fopen_s(&fp, filename, "w");
	assert(fp);
	if (&elem != NULL)
		XMLSaveFile(elem, fp);
	fclose(fp);
}



template<class T>
inline std::vector<T> ArrayImport(std::string str)
{
	std::vector<T> a;
	std::istringstream s(str);
	T e;
	while (((s >> e), s.good()))
	{
		a.push_back(e);
		s.ignore(16, '\n');
	}
	return a;
}

//  ToString() - convenience class/function for using << overloads to generate strings inline within a single expression
// Example Usage:   return ToString() << "test" << my_vec3 << " more " << my_float_var << " and so on";
//            or:   set_thermometer_gui_string( ToString() << (temperature*9/5+32) ); 
struct ToString
{
	std::ostringstream o;
	std::string str(){ return o.str(); }
	operator std::string(){ return o.str(); }
	template<class T>ToString &operator<<(const T &t) { o << t; return *this; }
};

// StringTo and FromString  - convenience class/function for using >> overloads to convert from strings inline within a single expression
//    f(StringTo<float3>("6 7 8"));  // if f() is overloaded specify type with StringTo<>()
//    g(FromString("9"));            // works if g isn't overloaded such as it takes a float
//
template<class T> inline  T StringTo(std::string s) { T v; std::istringstream i(std::move(s)); i >> v; return v; }

struct FromString
{
	const std::string &s;
	FromString(const std::string &s) :s(s){}
	template<class T> operator T (){ return StringTo<T>(s); }
	operator std::string(){ return s; }
};




#endif  // SANDBOX_MINI_XML_PARSE_H
