#ifndef NETTHREAD_H
#define NETTHREAD_H

#pragma once
	
#include "NetClient.hpp"
#include <thread>
#include <vector>

class NetThread  
{
	private:
		std::thread *thread;

		std::vector<NetClient *> clients;

		bool running; 

		int epoll_fd;

	public:

		NetThread();
		~NetThread();

		void start(void);
		void stop(void);

		bool register_client(NetClient & client);

		NetThread & operator+(NetClient& client) { register_client(client); return *this; };

		void run(void);

};
#endif