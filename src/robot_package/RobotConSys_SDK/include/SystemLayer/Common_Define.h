#ifndef COMMON_DEFINE_H
#define COMMON_DEFINE_H

#include <stdint.h>

// #define INVALID_SOCKET_RECV     0x7fffffff
// #define SOCKET_REMOTE_CLOSED    0x80000001
#define SOCKET_REMOTE_CLOSED    -1      //远程关闭

#define SOCKET_READ_FINISH      0       //socket继续读完
#define SOCKET_READ_AGAIN       1       //继续读
#define INVALID_SOCKET_RECV     2       //无效的接收

#ifdef __linux__
#ifndef DECLSPEC_DLLEXPORT
#define DECLSPEC_DLLEXPORT
#endif

#define PATH_SEPARATOR          '/'
#define PATH_SEP_STRING         "/"

#define STDCALL
#define THREAD_ROUTINE_RETURN   void*

#define SOCKET_FD               int
#define INVALID_SOCKET_FD       -1

#define UART_FD                 int
#define UART_FAIL               -1

#elif _WIN32 
#ifndef DECLSPEC_DLLEXPORT
#define DECLSPEC_DLLEXPORT      __declspec(dllexport)
#endif

#define PATH_SEPARATOR          '\\'
#define PATH_SEP_STRING         "\\"

#define STDCALL                 _stdcall
#define THREAD_ROUTINE_RETURN   uint32_t

#define SOCKET_FD               uint64_t
#define INVALID_SOCKET_FD       (uint64_t)(~0)

#define UART_FD                 void*
#define UART_FAIL               NULL

#endif

#endif