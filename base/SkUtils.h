#include "PxPhysicsAPI.h"
#include "string"
#include <stdexcept>
class SkErrorCallback : public physx::PxErrorCallback
{
public:
    virtual void reportError(physx::PxErrorCode::Enum code, const char *message, const char *file,
                             int line)
    {
        if (code < 0)
        {
            return;
        }
        std::string type = "";
        switch (code)
        {
        case physx::PxErrorCode::eNO_ERROR:
            type = "NO_ERROR";
            break;
        case physx::PxErrorCode::eDEBUG_INFO:
            type = "DEBUG_INFO";
            break;
        case physx::PxErrorCode::eDEBUG_WARNING:
            type = "DEBUG_WARNING";
            break;
        case physx::PxErrorCode::eINVALID_PARAMETER:
            type = "INVALID_PARAMETER";
            break;
        case physx::PxErrorCode::eINVALID_OPERATION:
            type = "INVALID_OPERATION";
            break;
        case physx::PxErrorCode::eOUT_OF_MEMORY:
            type = "OUT_OF_MEMORY";
            break;
        case physx::PxErrorCode::eINTERNAL_ERROR:
            type = "INTERNAL_ERROR";
            break;
        case physx::PxErrorCode::eABORT:
            type = "ABORT";
            break;
        case physx::PxErrorCode::ePERF_WARNING:
            type = "PERF_WARNING";
            break;
        default:
            break;
        }
        fprintf(stderr,"DEBUG %s in %s,%d: %s...\n",type.c_str(),file,line,message);
    }
};
#define Error(str) { throw new std::runtime_error(str);}
