#ifndef TCPDEV_STRUCT_H
#define TCPDEV_STRUCT_H

#include <stdio.h>
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include "Common_Define.h"
#include "TCPDev_TypeDef.h"

namespace DevLayer{

struct DECLSPEC_DLLEXPORT Msg_Pag{
    uint32_t label  = 0;    // 消息类型：参考地址，利用地址表示消息类型，0-999为用户类型，1000及以上为设备类型
    uint32_t devIndex = 0;  // 设备序号
    uint32_t type   = 0;    // 消息类型：0:指令 1:文件，需要保存到本地
    uint32_t index  = 0;    // 消息序号：消息包较大需要拆包时使用，从1开始计数
    uint32_t count  = 0;    // 消息总数：消息包较大需要拆包时使用
    std::string content;    // 消息内容：单个消息包最多1024个字节
    void print(const char* str = "");
};

class DECLSPEC_DLLEXPORT TCPDev_Connect{
public:
    TCPDev_Connect();
    ~TCPDev_Connect();

    int createThread(SOCKET_FD fd, THREAD_ROUTINE_RETURN (STDCALL *start_routine)(void*), void* arg);
    void setAuthority(SOCKET_FD fd, AUTHORITY_TYPE authority);
    void insert(SOCKET_FD fd);
    void insertPipe(SOCKET_FD fd, SOCKET_FD fdPipe);
    void erase(SOCKET_FD fd);
    int size();
    void clear();
    bool find(SOCKET_FD fd);
    bool find(AUTHORITY_TYPE authority);
    AUTHORITY_TYPE getAuthority(SOCKET_FD fd);
    std::vector<SOCKET_FD> getFD();

    void openHeartBeat();     //心跳监控
    void closeHeartBeat();
private:
    std::mutex m_connectMutex;
    std::map<SOCKET_FD, AUTHORITY_TYPE> m_connectAuthority;
    std::map<SOCKET_FD, void*> m_connectThread;
    std::map<SOCKET_FD, SOCKET_FD> m_connectPipe;

    volatile bool m_bOpenHeartBeat = true;
    void* m_threadHeartBeat;
	static THREAD_ROUTINE_RETURN (STDCALL cycleTaskHeartBeat)(void* lpParameter);
};

}

#endif