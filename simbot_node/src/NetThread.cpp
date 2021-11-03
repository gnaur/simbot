#include "NetThread.hpp"  
#include <sys/epoll.h>
#include <unistd.h>

#define MAX_EVENTS 32

NetThread::NetThread() : thread(NULL), running(false), epoll_fd(-1)
{
	epoll_fd = epoll_create1(0);
}
	
NetThread::~NetThread()
{
	stop();
}

void NetThread::start(void) {
    if(!running) {
        thread = new std::thread(&NetThread::run,this);
    }
}

void NetThread::stop(void) {

    if(running) {
        running = false;
        thread->join();
    }
    for (auto it=clients.begin(); it != clients.end(); it++) {
        epoll_ctl(epoll_fd,EPOLL_CTL_DEL,(*it)->getSH(),NULL);
    }
    close(epoll_fd);
    epoll_fd = -1;

    delete thread;
    thread = NULL;
}

void NetThread::run(void) {
    struct epoll_event events[MAX_EVENTS];
    while(running){
        int rc=epoll_wait(epoll_fd,events,MAX_EVENTS,100);
        if(rc == -1){

            switch(errno) {
                case EINTR:
                    break;

                default:
                    printf("Failed to polls socket handles. errno= %d\n",errno);
                    break;

            }

        }
        else {
            for (int e=0; e < rc; e++) {
                NetClient * client = (NetClient*)events[e].data.ptr;
                if(events[e].events & EPOLLIN) {
                    client->read_cb(&events[e]);
                }
                else {
                     printf("Error event for fd=%d\n",client->getSH());
                }
            }
        }
    }
}

bool NetThread::register_client(NetClient & client) { 

    clients.push_back(&client); 

    struct epoll_event event={ EPOLLIN | EPOLLRDHUP | EPOLLPRI | EPOLLERR,
        {
            (void*)&client
        }
     };

    return epoll_ctl(epoll_fd,EPOLL_CTL_ADD,client.getSH(),&event) ==0;

    

}
