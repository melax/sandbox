//
// some tcp/ip socket code useful for reference
//
#include <vector>
#include <exception>

#  include <winsock.h>
#pragma comment(lib,"ws2_32.lib")
#define MSG_WAITALL 0x8   // omg, MS forgot to define MSG_WAITALL in winsock.h (http://www.windows-tech.info/14/89f04a075d8e9df7.php):



#define RESOURCE_ERROR(m)  reporterror(__FILE__,__LINE__,m)
inline bool reporterror(const char *file, int line, std::string e)
{
	std::string s = std::string(file) + ":" + std::to_string(line) + " " + e;
	std::cerr << "FAIL:  " << s << "\n";
	throw(std::exception(s.c_str()));
	return 0;
}

void perror(char *) { printf("%d is the error", WSAGetLastError()); }

#  define DEFAULT_PORT  80




SOCKET ConnectTCP(const char *machine = "localhost", int port = DEFAULT_PORT)
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

int PollRead(SOCKET s)
{
	if (s == INVALID_SOCKET) return 0;
	fd_set re, wr, ex;
	struct timeval tm;
	tm.tv_sec = 0;
	tm.tv_usec = 0;
	FD_ZERO(&re);
	FD_ZERO(&wr);
	FD_ZERO(&ex);
	FD_SET(s, &re);
	int rc = select(s + 1, &re, &wr, &ex, &tm);
	if (rc == SOCKET_ERROR) return 0;
	//printf("select:  rc=%d  readfdset=%d\n",rc,FD_ISSET(s,&re));
	return (rc && FD_ISSET(s, &re));
}



void read(void *dst, int n, SOCKET s) { int r; (r = recv(s, (char*)dst, n, MSG_WAITALL)) == n || RESOURCE_ERROR(std::string("recv error ") + std::to_string(WSAGetLastError()) + " wanted:" + std::to_string(n) + " rv:" + std::to_string(r)); }

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

std::vector<char> testserver(const char* a="GET foo")
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

SOCKET start_server(int port = 80) 
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

void do_server_thing(SOCKET listen_sock,std::string reply) 
{
	if (listen_sock == INVALID_SOCKET) 
	{
		return;
	}
	if (!PollRead(listen_sock)) 
		return; 

	// SOCKET s = Accept();
	SOCKADDR_IN accept_addr;    // Accept socket address - internet style 
	int accept_addr_len;        // Accept socket address length 
	accept_addr_len = sizeof(accept_addr);
	SOCKET s = accept(listen_sock, (struct sockaddr FAR *) &accept_addr,
		(int FAR *) &accept_addr_len);
	if (s < 0) {
		perror("accept:  ");
		// printf("%d is the error", WSAGetLastError());
		WSACleanup(); // should we throw?  
		return;// INVALID_SOCKET;
	}


	if (s == INVALID_SOCKET) { return; }
	if (!PollRead(s)) { closesocket(s);return; }
	char buf[2048];
	int rc = 0;
	buf[rc] = '\0';
	int bc = 0;
	while (PollRead(s) && (bc = recv(s, buf + rc, 1024 - rc, 0)) >0) {
		rc = rc + bc;
	}
	buf[rc] = '\0';


	char outbuf[256];
	sprintf(outbuf, "HTTP/1.0 200 OK\nContent Type: text/plain\n\n");
	rc = send(s, outbuf, strlen(outbuf), 0);
	sprintf(outbuf, "%s\n",reply.c_str());
	rc = send(s, outbuf, strlen(outbuf), 0);
	closesocket(s);
}
