#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H
#pragma once
	

#include "NetClient.hpp"
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <functional>

template<class Tin, class Tout>
class UDPSocket : public NetClient
{
	public:
	   //typedef void(*client_cb)(Tin & data);
	   //typedef std::function<void(const Tin&)> client_cb;

	private:
		int port;
		Tin data;

		struct sockaddr_in serveraddr;
		std::function<void(const Tin &)> cb;
		
	public:
	   

		UDPSocket(const int port, const char * server_ip, std::function<void(const Tin&)> cb) : port(port), cb(cb)
		{

			sh = socket(AF_INET,SOCK_DGRAM,0);
			if(sh < 0) {
                printf("Failed to create UDP Socket for port %d\n",port);
				return;
			}

			memset(&serveraddr, 0, sizeof(serveraddr));

			// Filling server information
			serveraddr.sin_family    = AF_INET; // IPv4
			serveraddr.sin_port = htons(port);

			if (inet_aton(server_ip , &serveraddr.sin_addr) == 0) 
			{
				fprintf(stderr, "inet_aton() failed for IP Address %s\n",server_ip);
				return;

			}
		
		}

		virtual ~UDPSocket()
		{
			close(sh);
			sh=-1;
		}

		virtual void read_cb(void * ctx) {
			(void)ctx;
			socklen_t len=sizeof(serveraddr);
			if(recvfrom(sh,(void*)&data,sizeof(Tin),0, (struct sockaddr *) &serveraddr, &len)>0) {
				cb(data);
			}
			else {
				printf("Failed to read from UDP Socket for port %d\n",port);
			}
		}

		bool send(Tout & data) {
			int rc= sendto(sh,(void*)&data,sizeof(data),0, (const struct sockaddr*)&serveraddr,sizeof(serveraddr));
			if(rc <0) {
				printf("Failed to send from UDP Socket for port %d\n",port);
				return false;
			}
			return true;
		}



};
#endif // UDP_SOCKET_H
