#ifndef NETCLIENT_H
#define NETCLIENT_H
#pragma once

class NetClient {

    public:

        NetClient() : sh(-1) {
            
        };

        virtual ~NetClient() {};

        typedef void (*sock_notify_cb)(void * ctx);

        int getSH(void) { return sh; };

        virtual void read_cb(void * ctx) = 0; 

    protected:
        int sh;

    
};

#endif // NETCLIENT_H

