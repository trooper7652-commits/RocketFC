#pragma once
// Tiny non-blocking TCP helpers for host_live.cpp, in their own translation
// unit so <winsock2.h>/<windows.h> macros (ERROR, INPUT, min/max, ...) never
// meet the Arduino mocks or the firmware sources.
#include <cstdint>

typedef long long net_sock;  // -1 = none
const net_sock NET_NONE = -1;

bool net_init();
net_sock net_listen(int port);           // 127.0.0.1:port, non-blocking
net_sock net_accept(net_sock listener);  // NET_NONE if nobody is waiting
// > 0 bytes read, 0 nothing available, -1 peer closed / error
int net_recv(net_sock s, char* buf, int n);
// Sends everything it can without blocking for long; returns false if the
// peer is gone.
bool net_send(net_sock s, const void* p, int n);
void net_close(net_sock s);
