//
//  test html reflection - browser html form based gui
//
//  no debugger? no gui? no problem!
//  rather than integrating a scripting language, command console, or adding gui elements to a small 
//  opengl program, all various tweakable parameters and data is put in a single tuple and
//  exposed through a generated html form based web page.
//  this exploits the same unobtrusive visit_fields() mechanism used for json serialization and sterling's other 
//  gui component generation system.
//  the program opens up a socket, effectively becoming a web server, 
//  and listens for requests and reacts accordingly. 
//  from the browser it is possible to modify various runtime settings including exposed data for 
//  objects in the application (in this case a simple 2D polygon object).
//  

#define NOMINMAX
#include <iostream>
#include <algorithm>

#include "../include/misc_image.h"
#include "../include/geometric.h"
#include "../include/html_reflect.h"

#include "../include/json.h"
#include "../include/misc_json.h"
#include "../include/glwin.h"        // in order to provide some simple example visuals that can be tweaked in html-form 

template<class F> void visit_fields(int3   &v, F f) { f("x", v.x); f("y", v.y); f("z", v.z); }
template<class F> void visit_fields(float3 &v, F f) { f("x", v.x); f("y", v.y); f("z", v.z); }
template<class F> void visit_fields(float2 &v, F f) { f("x", v.x); f("y", v.y); }
template<class F> void visit_fields(int2   &v, F f) { f("x", v.x); f("y", v.y); }
struct RuntimeParams
{
	bool continue_execution;
	int connections_seen, frame_id;
	std::string helpstring;
};
template<class F> void visit_fields(RuntimeParams &s, F f)
{ 
	f("continue_execution", s.continue_execution); 
	f("connections_seen"  , s.connections_seen  );
	f("frame_id", s.frame_id);
	f("helpstring", s.helpstring);
}

struct PolygonObject
{
	std::vector<float3> verts;  // just positions
	float3 color;
	float spin;
	float angle;
	template<class F> void visit_fields(F f) { f("verts", verts); f("color", color);f("spin", spin), f("angle",angle );}
}; 



struct    // anonymous  has to be global struct since local struct/class cant have member templates in msvc2015
{ 
	float red, green, blue; 
	template<class F> void visit_fields(F f) { f("red", red);f("green", green);f("blue", blue); }
} clear_color = { 0.0f,0.0f, 0.25f};

struct AnotherTestStruct // just to show some more examples of reflection
{
	float a, b;
	int3 v;
	std::vector<int> nums;
	struct { int c, d; template<class F> void visit_fields(F f) { f("c", c);f("d", d); } } nested_member;
	template<class F> void visit_fields(F f) { f("a", a); f("b", b, float2(0, 1)); f("v", v); f("nums", nums); f("nested_member", nested_member); }
} more_test_cases = { 13,17,{ 97,98,99 } ,{ 9,8,7,6 } ,{1024,1025 }};


int main(int argc, char *argv[]) try
{

	Image<byte3>  image({ 256,256 });
	for (auto p : rect_iteration(image.dim()))
		image.pixel(p) = byte3(p.x, p.y, 128);
	SOCKET sock = start_server(12345);

	PolygonObject my_polygon = { { { -0.4f, -0.3f, 0.0f },{ 0.4f, -0.3f, 0.0f },{ 0.0f,  0.5f, 0.0f } },{ 1.0f,0.0f,0.0f } , 0.1f,0.0f };
	RuntimeParams runtime_settings = { true,0,0,  "tweak various runtime parameters from your browser!"};
	bool use_meta_refresh = false;
	int3 test_int3 = { 100,200,300 };
	bool extra_bool = false;
	int extra_int = 202;
	auto unused = std::tie(extra_bool, "extra_bool", extra_int, "extra_int", test_int3, "test_int3", more_test_cases,"more_test_cases");
	auto all_my_params = std::tie(runtime_settings, "runtime_settings" , clear_color, "clear_color" , my_polygon,"my_polygon" , use_meta_refresh,"use_meta_refresh", unused,"unused" );

	GLWin glwin("enter url http://localhost:12345 in your browser");
	std::cout << "open up a browser and enter URL:  http://localhost:12345/ " << std::endl;

	while (runtime_settings.continue_execution && glwin.WindowUp())
	{
		if (auto s = serverside_get_request(sock))
		{
			std::cout << s.message << "--------" << std::endl;
			std::cout << "url: \"" << s.url << "\"" << std::endl;
			if (s.command == "")
			{
				auto htmldoc = to_form_html_doc(all_my_params,"params");
				htmldoc.child("body").child("p").body = "generated html form from internal parameters and data.<br> *<a href=/pic>pic test</a> *<a href=/screenshot>screenshot</a> *<a href=.clear_color.green=1>go green</a>";
				xmlNode refresh("meta");
				refresh.attributes.push_back({ "http-equiv","refresh" });
				refresh.attributes.push_back({ "content","10" });
				if (use_meta_refresh)
					htmldoc.child("head").children.push_back(refresh);  // will cause browser to refresh every 10 seconds, after enabled, you may need to manually refresh once so that paage with this meta gets loaded 
				std::string foodoc = ToString() << htmldoc;
				std::cout << "foodoc: " << foodoc << std::endl;
				http_reply_text(s.socket, foodoc, "html");

			}
			else if (s.command == "pic")
			{
				http_reply_image(s.socket, image);
			}
			else if (s.command == "screenshot")
			{
				Image<byte3> cimage(glwin.res);
				glReadPixels(0, 0, glwin.res.x, glwin.res.y, GL_RGB, GL_UNSIGNED_BYTE, cimage.raster.data());
				FlipV(cimage.raster, cimage.dim());
				http_reply_image(s.socket, cimage);
			}
			else   
			{
				for (auto a : s.args)     // for each ':' separated argument 
				{
					auto res = assigner(all_my_params, a.c_str());
					std::cout << "result of interpretor: " << res << " from "<< a << std::endl;
				}
				if(!s.args.size())
				{
					auto aresult = assigner(all_my_params, s.command.c_str() );
					std::cout << " assigner result:  " << aresult << std::endl;
				}
				auto paramjson = to_json(all_my_params);
				std::string paramsjsontxt = ToString() << json::tabbed(to_json(all_my_params),4,1);   // to show new internal state in a easy to read json format
				std::cout << paramsjsontxt << std::endl;
				std::string params_as_html = ToString() << to_html(all_my_params, "params");   // show new internal state with typical html formatting (not an html form)
				std::cout << params_as_html << std::endl;
				http_reply_text(s.socket, ToString() << "got it thanks\n" << "\n-------\n" 
				                                     << "<pre>\n" << paramsjsontxt << "\n</pre>\n" 
				                                     << "\n-------\n" << s.message << "---------\n" 
				                                     << params_as_html << "\n--------\n" << "ok" 
				                                     << "\n--------\n");
			}
			closesocket(s.socket);
			std::cout << "done service this frame\n";
			runtime_settings.connections_seen++; 
		}

		glClearColor(clear_color.red,clear_color.green,clear_color.blue,0.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		// animate and update our 'game' object:
		glRotatef(my_polygon.angle += my_polygon.spin, 0, 0, 1);
		glBegin(GL_TRIANGLES);
		glColor3fv(my_polygon.color);
		for(auto &p:my_polygon.verts)
			glVertex3fv(p);
		glEnd();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		glColor3f(1, 1, 0.5f);
		glwin.PrintString({ 0,0 }, "%s", runtime_settings.helpstring.c_str());
		glPopAttrib();
		glwin.SwapBuffers();

		runtime_settings.frame_id++;
	}
}
catch (const char *c)
{
	MessageBox(GetActiveWindow(), "FAIL", c, 0);
}
catch (std::exception e)
{
	MessageBox(GetActiveWindow(), "FAIL", e.what(), 0);
}
