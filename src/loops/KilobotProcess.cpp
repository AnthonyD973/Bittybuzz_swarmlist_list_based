#include <sys/mman.h> // shm_open, shm_unlink
#include <fcntl.h>    // O_RDWR, O_CREAT, S_IRUSR, S_IWUSR
#include <unistd.h>   // ftruncate
#include <cerrno>     // strerror, errno

#include "KilobotProcess.h"

/****************************************/
/****************************************/

using argos::CARGoSException; // Required because of the THROW_ARGOSEXCEPTION macro.

swlexp::KilobotProcess::KilobotProcess(
    argos::CLoopFunctions& loopFunc,
    argos::UInt32 id,
    const std::string& controllerId,
    const argos::CVector3& position,
    const argos::CQuaternion& orientation)

    : m_loopFunc(&loopFunc)
    , m_controllerId(controllerId)
{

    // Create and place kilobot. This also creates the process itself.
    argos::CKilobotEntity* kb =
        new argos::CKilobotEntity(
            "kb" + std::to_string(id),
            m_controllerId,
            position,
            orientation
        );
    m_loopFunc->AddEntity(*kb);

    // Create experiment data
    m_expDataName = "/exp_data" + std::to_string(id);
    m_expDataFd = ::shm_open(m_expDataName.c_str(), O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    int truncRet = ::ftruncate(m_expDataFd, sizeof(exp_data_t));
    if (truncRet >= 0) {
        m_expData = reinterpret_cast<exp_data_t*>(
            ::mmap(NULL,
                   sizeof(exp_data_t),
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED,
                   m_expDataFd,
                   0));
    }
    else {
        THROW_ARGOSEXCEPTION("Error acquiring experiment data: " << ::strerror(errno));
    }
}

/****************************************/
/****************************************/

swlexp::KilobotProcess::KilobotProcess(swlexp::KilobotProcess&& other)
 : m_loopFunc(std::move(other.m_loopFunc))
 , m_controllerId(std::move(other.m_controllerId))
 , m_expDataName(std::move(other.m_expDataName))
 , m_expDataFd(std::move(other.m_expDataFd))
 , m_expData(std::move(other.m_expData))
{
    other.m_loopFunc  = nullptr;
    other.m_expDataFd = -1;
    other.m_expData   = nullptr;
}

/****************************************/
/****************************************/

swlexp::KilobotProcess& swlexp::KilobotProcess::operator=(swlexp::KilobotProcess&& other) {
    if (&other != this) {
        // Close current process resources, if we ourselves aren't empty.
        if (m_expDataFd >= 0) {
            ::munmap(m_expData, sizeof(exp_data_t));
            ::close(m_expDataFd);
            ::shm_unlink(m_expDataName.c_str());
        }

        // Copy other's resources.
        m_loopFunc     = std::move(other.m_loopFunc);
        m_controllerId = std::move(other.m_controllerId);
        m_expDataName  = std::move(other.m_expDataName);
        m_expDataFd    = std::move(other.m_expDataFd);
        m_expData      = std::move(other.m_expData);

        // Empty the other object.
        other.m_loopFunc  = nullptr;
        other.m_expDataFd = -1;
        other.m_expData   = nullptr;
    }
    return *this;
}

/****************************************/
/****************************************/

swlexp::KilobotProcess::~KilobotProcess() {
    if (m_expDataFd >= 0) {
        ::munmap(m_expData, sizeof(exp_data_t));
        ::close(m_expDataFd);
        ::shm_unlink(m_expDataName.c_str());
    }
}

/****************************************/
/****************************************/

void swlexp::KilobotProcess::reset() {
    // Fill the experiment data with zeroes
    for (argos::UInt32 i = 0; i < sizeof(exp_data_t); ++i) {
        ((char*)m_expData)[i] = 0;
    }
}