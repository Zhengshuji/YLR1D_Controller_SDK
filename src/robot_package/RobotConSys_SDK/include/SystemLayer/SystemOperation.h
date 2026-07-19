#ifndef SYSTEM_OPERATION_H
#define SYSTEM_OPERATION_H

#include "Common_Define.h"

namespace SysLayer{

DECLSPEC_DLLEXPORT int createDir(const char* path);
DECLSPEC_DLLEXPORT void removeDir(const char* path);

class DECLSPEC_DLLEXPORT CLoadLibrary{
public:
    CLoadLibrary();
    ~CLoadLibrary();
    int open(const char* path, const char* name);
    void* loadFunc(const char* funcName);
    void loadErrorPrint();
    void close();

private:
    void* m_handle;
};

}

#endif