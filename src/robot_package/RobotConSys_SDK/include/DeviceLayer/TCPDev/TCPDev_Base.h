#ifndef TCPDEV_BASE_H
#define TCPDEV_BASE_H

#include <stdint.h>
#include <string>
#include "TCPDev_Struct.h"

namespace DevLayer{
class DECLSPEC_DLLEXPORT TCPDev_Base{
public:
    TCPDev_Base();
    ~TCPDev_Base();

    bool online();
protected:
    // common api
    bool getStartFlag();    // 判断TCP通讯是否建立，若返回true，服务端则表示accept线程已建立，客户端则表示已连接到服务端
    int send_cmd(SOCKET_FD fd, uint32_t label, uint32_t devIndex, std::string cmd);
    int send_file(SOCKET_FD fd, uint32_t label, uint32_t devIndex, const char* filePath);

    // client api
    int initClient(const char* ip, int port, const char* tmpDir);
    void closeClient();
    SOCKET_FD getSocketFdClient();  // 返回客户端自身的文件描述符

    // server api
    int initServer(int port, const char* tmpDir);
    void closeServer();
    std::vector<SOCKET_FD> getConnectFD();  // 返回已连接的客户端文件描述符
    AUTHORITY_TYPE getConnectAuthority(SOCKET_FD fd);  // 根据客户端文件描述符获取客户端的权限类型
    bool findConnectAuthority(AUTHORITY_TYPE authority); // 判断是否存在已连接的某权限类型的客户端
    void setConnectAuthority(SOCKET_FD fd, AUTHORITY_TYPE authority);    // 根据客户端文件描述符设置客户端的权限类型

private:
    // common variable
    bool m_startFlag = false;   // 启动标志位
    std::string m_tmpDir;
    void* m_tcp;
    TCPCONDEV_TYPE m_devType;
    int m_pipeFd;

    std::mutex m_sendMutex;
    int send_package(SOCKET_FD fd, Msg_Pag& pag);

    /// <summary>
    /// 从TCP读取消息、提取一个消息包
    /// </summary>
    /// <param name="fd">读文件描述符 </param>
    /// <param name="pag">将要处理的一个数据包</param>
    /// <param name="rest_pag_str">未处理消息</param>
    /// <returns>小于零：错误，线程退出
    ///          等于零：tcp无可读数据
    ///          大于零：tcp继续读取或者rest_pag_str中有未处理的包   
    /// </returns>
    int recv_package(SOCKET_FD fd, Msg_Pag& pag, std::string& rest_pag_str);

    static THREAD_ROUTINE_RETURN (STDCALL recv_cmd_file)(void* lpParameter);

    // client variable
    void* m_threadRecv;
    void* m_threadHeartBeat;    
    bool m_bOpenHeartBeat;
    virtual void excuteDevCMD(uint32_t label, uint32_t devIndex, std::string& cmd){};
    virtual void excuteDevFile(uint32_t label, uint32_t devIndex, const char* filePath){};
    static THREAD_ROUTINE_RETURN(STDCALL heart_beat_fun)(void* lpParameter);

    // server variable
    void* m_threadAccept;    // accept线程
    static THREAD_ROUTINE_RETURN (STDCALL acceptConnect)(void* lpParameter);
    TCPDev_Connect m_connect;    // 连接到服务端的所有客户端的文件描述符、权限和线程管理
    virtual void excuteDevCMD(SOCKET_FD fd, uint32_t label, uint32_t devIndex, std::string& cmd){};
    virtual void excuteDevFile(SOCKET_FD fd, uint32_t label, uint32_t devIndex, const char* filePath){};
};

}

#endif