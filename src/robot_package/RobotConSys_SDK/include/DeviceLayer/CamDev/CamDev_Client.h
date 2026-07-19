#ifndef CAMDEV_CLIENT_H
#define CAMDEV_CLIENT_H

#include "Common_Define.h"
#include "CamDev_TypeDef.h"

namespace DevLayer{

class DECLSPEC_DLLEXPORT CamDev_Client{
public:
    CamDev_Client();
    ~CamDev_Client();

    int init(const char* ip, int port);
    void close();

    int capture(uint8_t* imgBuf, CAM_DEV_IMG_TYPE type);

private:
    void* m_dev;
};

}

#endif