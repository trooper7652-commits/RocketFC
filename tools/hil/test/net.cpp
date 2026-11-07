// See net.h.
#include "net.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET raw_sock;
static bool wouldBlock() { return WSAGetLastError() == WSAEWOULDBLOCK; }
static void setNonBlocking(raw_sock s) {
  u_long one = 1;
  ioctlsocket(s, FIONBIO, &one);
}
static void closeRaw(raw_sock s) { closesocket(s); }
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int raw_sock;
static const raw_sock INVALID_SOCKET = -1;
static bool wouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK; }
static void setNonBlocking(raw_sock s) { fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK); }
static void closeRaw(raw_sock s) { close(s); }
#endif

#include <chrono>
#include <thread>

bool net_init() {
#ifdef _WIN32
  WSADATA w;
  return WSAStartup(MAKEWORD(2, 2), &w) == 0;
#else
  return true;
#endif
}

net_sock net_listen(int port) {
  raw_sock s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == INVALID_SOCKET) return NET_NONE;
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof(one));
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons((unsigned short)port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(s, (sockaddr*)&a, sizeof(a)) != 0 || listen(s, 1) != 0) {
    closeRaw(s);
    return NET_NONE;
  }
  setNonBlocking(s);
  return (net_sock)s;
}

net_sock net_accept(net_sock l) {
  raw_sock c = accept((raw_sock)l, nullptr, nullptr);
  if (c == INVALID_SOCKET) return NET_NONE;
  setNonBlocking(c);
  int one = 1;  // USB-like latency: no Nagle batching of 40-byte packets
  setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
  return (net_sock)c;
}

int net_recv(net_sock s, char* buf, int n) {
  const int k = (int)recv((raw_sock)s, buf, n, 0);
  if (k > 0) return k;
  if (k == 0) return -1;  // orderly close
  return wouldBlock() ? 0 : -1;
}

bool net_send(net_sock s, const void* p, int n) {
  const char* c = (const char*)p;
  const auto give_up = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  while (n > 0) {
    const int k = (int)send((raw_sock)s, c, n, 0);
    if (k > 0) {
      c += k;
      n -= k;
    } else if (k < 0 && wouldBlock()) {
      if (std::chrono::steady_clock::now() > give_up) return true;  // reader stalled: drop
      std::this_thread::yield();
    } else {
      return false;
    }
  }
  return true;
}

void net_close(net_sock s) {
  if (s != NET_NONE) closeRaw((raw_sock)s);
}
