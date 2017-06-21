//
// some tcp/ip socket code useful for reference
//
#pragma once
#ifndef TCP_SOCKET_CONVENIENCE_WRAPPER
#define TCP_SOCKET_CONVENIENCE_WRAPPER

#include <vector>
#include <exception>

#include "misc.h"
#define NOMINMAX
#include <winsock.h>
#pragma comment(lib,"ws2_32.lib")
#define MSG_WAITALL 0x8   // omg, MS forgot to define MSG_WAITALL in winsock.h (http://www.windows-tech.info/14/89f04a075d8e9df7.php):




inline bool reporterror(const char *file, int line, std::string e)
{
	std::string s = std::string(file) + ":" + std::to_string(line) + " " + e;
	std::cerr << "FAIL:  " << s << "\n";
	throw(std::exception(s.c_str()));
	return 0;
}
#define RESOURCE_ERROR(m)  reporterror(__FILE__,__LINE__,m)

inline void perror(char *) { printf("%d is the error", WSAGetLastError()); }

#  define DEFAULT_PORT  80




inline SOCKET ConnectTCP(const char *machine = "localhost", int port = DEFAULT_PORT)
{
	// On WINDOWS be sure to call WSAStartup winsock before calling this routine
	struct  hostent *phe;
	SOCKADDR_IN addr;  /* DESTination Socket INternet */
	SOCKET sock = INVALID_SOCKET;

	if ((sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET)
		RESOURCE_ERROR("socket creation");
	// todo:  add somthing here that when destructed will close this socket if we throw before we return 

	addr.sin_family = AF_INET;

	if (isdigit(machine[0]))
	{
		unsigned int address = inet_addr(machine);
		phe = gethostbyaddr((char *)&address, 4, AF_INET);
		if (phe) {
			memcpy((char *)&(addr.sin_addr), phe->h_addr, phe->h_length);
		}
		else {
			// just hack it in
			memcpy((char *)&(addr.sin_addr), (char *)&address, 4);
		}
	}
	else
	{
		(phe = gethostbyname(machine)) || RESOURCE_ERROR(std::string("unable to get hostbyname ") + machine + "error: " + std::to_string(WSAGetLastError()));  // note need to closesocket(sock);
		memcpy((char FAR *)&(addr.sin_addr), phe->h_addr, phe->h_length);
	}

	addr.sin_port = htons(port);        /* Convert to network ordering */

	if (connect(sock, (PSOCKADDR)&addr, sizeof(addr)) < 0)  RESOURCE_ERROR("connect() failed"); //  note need to closesocket(sock);
	return sock;
}

inline int PollRead(SOCKET s,long usec_timeout=0)
{
	if (s == INVALID_SOCKET) return 0;
	fd_set re, wr, ex;
	struct timeval tm;
	tm.tv_sec = 0;
	tm.tv_usec = usec_timeout;
	FD_ZERO(&re);
	FD_ZERO(&wr);
	FD_ZERO(&ex);
	FD_SET(s, &re);
	int rc = select(s + 1, &re, &wr, &ex, &tm);
	if (rc == SOCKET_ERROR) return 0;
	//printf("select:  rc=%d  readfdset=%d\n",rc,FD_ISSET(s,&re));
	return (rc && FD_ISSET(s, &re));
}



inline void read(void *dst, int n, SOCKET s) { int r; (r = recv(s, (char*)dst, n, MSG_WAITALL)) == n || RESOURCE_ERROR(std::string("recv error ") + std::to_string(WSAGetLastError()) + " wanted:" + std::to_string(n) + " rv:" + std::to_string(r)); }

template<class T>
T read(SOCKET s) { T t; read(&t, sizeof(t), s); return t; }


template<class T>
std::vector<T> readarray(SOCKET s)
{
	auto n = read<int32_t>(s); // should be 32 bit
	std::vector<T> a(n);
	read(a.data(), n * sizeof(T), s);
	return a;
}

inline std::vector<char> testserver(const char* a="GET foo")
{
	std::vector<char> testimage(32 * 32*3, 0);
	auto s = ConnectTCP("localhost", 21212);  // on some systems, low numbered ports are reserved.
	send(s, a, strlen(a), 0);
	read(testimage.data(), 32*32*3, s);
	return testimage;
}

// 	WSADATA wsdata;
//	if (int status = WSAStartup(MAKEWORD(1, 1), &wsdata))   RESOURCE_ERROR(std::to_string(status) + " error from wsastartup");

//

inline SOCKET start_server(int port = 80) 
{
	// On WINDOWS be sure to call WSAStartup winsock before calling this routine
	int status;
	WSADATA whatever;
	if ((status = WSAStartup(MAKEWORD(1, 1), &whatever)) != 0) 
	{
		throw(std::exception("FAIL:  WSAStartup socket startup"));
		return INVALID_SOCKET;
	}

	SOCKADDR_IN listen_addr;  /* Local socket - internet style */
	listen_addr.sin_family = AF_INET;
	listen_addr.sin_addr.s_addr = INADDR_ANY;
	listen_addr.sin_port = htons(port);        /* Convert to network ordering */
	auto listen_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_sock == INVALID_SOCKET) 
	{
		throw(std::exception((std::string("FAIL: socket create: ")+std::to_string(WSAGetLastError())).c_str()));
		return INVALID_SOCKET;
	}
	if (bind(listen_sock, (struct sockaddr FAR *) &listen_addr, sizeof(listen_addr)) == -1) {
		perror("ERROR:  bind:  ");
		throw(std::exception((std::string("FAIL: socket bind: ") + std::to_string(WSAGetLastError())).c_str()));
		return INVALID_SOCKET;
	}
	if (listen(listen_sock, 4) < 0) 
	{
		perror("listen:  ");
		// printf("%d is the error", WSAGetLastError());
		WSACleanup();
		throw(std::exception((std::string("FAIL: socket listen: ") + std::to_string(WSAGetLastError())).c_str()));
		return INVALID_SOCKET;
	}
	return listen_sock;
}

inline void http_reply_text(SOCKET s, std::string reply, const char *type = "plain")
{
	std::string http_header = ToString() << "HTTP/1.0 200 OK\nAccess-Control-Allow-Origin: *\nContent-Length: " << reply.length() << "\nContent Type: text/" << type << "\n\n";
	int rc = send(s, http_header.c_str(), http_header.length(), 0);
	rc = send(s, reply.c_str(), reply.length(), 0);
}


inline std::pair<SOCKET,std::string> serverside_get_request(SOCKET listen_sock)
{
	if (listen_sock == INVALID_SOCKET) 
	{
		return{ INVALID_SOCKET,"" };
	}
	if (!PollRead(listen_sock)) 
		return{ INVALID_SOCKET,"" };
	std::cout << "listen_sock polled for read\n";
	// SOCKET s = Accept();
	SOCKADDR_IN accept_addr;    // Accept socket address - internet style 
	int accept_addr_len;        // Accept socket address length 
	accept_addr_len = sizeof(accept_addr);
	SOCKET s = accept(listen_sock, (struct sockaddr FAR *) &accept_addr,
		(int FAR *) &accept_addr_len);
	if (s == INVALID_SOCKET)
	{
		std::cout << "error "<< s << "on accept listen_sock\n";
		perror("accept:  \n");
		// printf("%d is the error", WSAGetLastError());
		//WSACleanup(); // should we throw?  
		return{ INVALID_SOCKET,"" };// INVALID_SOCKET;
	}
	if (!PollRead(s,1000)) 
	{ 
		int i=10;
		while (!PollRead(s,100000) && --i)
			std::cout << '.'; 
		std::cout << " \ngot an accepted socket but nothing to read when poll select "<< i << " attempts remain\n";
		if (i == 0)
		{
			http_reply_text(s, "wtf - no request??");
			closesocket(s);
			return{ INVALID_SOCKET,"" };
		}
	}
	char buf[2048];
	int rc = 0;
	buf[rc] = '\0';
	int bc = 0;
	while (PollRead(s,1000) && (bc = recv(s, buf + rc, 1024 - rc, 0)) >0) {
		
		rc = rc + bc;
	}
	buf[rc] = '\0';
	if (bc < 0)
		int k = 3;
	return{ s,std::string(buf) };
}


inline void do_server_thing(SOCKET listen_sock,std::string reply) 
{
	auto request = serverside_get_request(listen_sock);
	if (request.first == INVALID_SOCKET)
		return;
	std::string http_header = ToString() << "HTTP/1.0 200 OK\nAccess-Control-Allow-Origin: *\nContent-Length: "<< reply.length() << "\nContent Type: text/plain\n\n";
	int rc = send(request.first, http_header.c_str(), http_header.length(), 0);
	rc = send(request.first, reply.c_str(), reply.length(), 0);
	closesocket(request.first);
}


#endif  TCP_SOCKET_CONVENIENCE_WRAPPER
