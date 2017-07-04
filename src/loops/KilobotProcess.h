/**
 * @file KilobotProcess.h
 * @brief Definition of the KilobotProcess class, which wraps the creation
 * and destruction of Kilobot subprocesses (RAII).
 */

#ifndef KILOBOT_PROCESS_H
#define KILOBOT_PROCESS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>

#include "include/exp_data.h"

namespace swlexp {

    /**
     * Wraps the creation and destruction of Kilobot subprocesses (RAII).
     */
    class KilobotProcess {

    // ==============================
    // =          METHODS           =
    // ==============================

    public:

        /**
         * Main constructor. Creates the kilobot subprocess and its resources.
         * @param[in,out] loopFunc Loop functions object where to
         * add/remove kilobots.
         * @param[in] id The kilobot's number. Should be unique.
         * @param[in] controllerId ID of the kilobot's controller. Must match
         * one of those in the ARGoS file.
         * @param[in] position Initial position of the kilobot.
         * @param[in] orientation Initial orientation of the kilobot.
         */
        KilobotProcess(argos::CLoopFunctions& loopFunc,
                       argos::UInt32 id,
                       const std::string& controllerId,
                       const argos::CVector3& position,
                       const argos::CQuaternion& orientation);

        KilobotProcess(KilobotProcess&& other);
        KilobotProcess& operator=(KilobotProcess&& other);
        KilobotProcess(const KilobotProcess&) = delete;
        KilobotProcess& operator=(const KilobotProcess&) = delete;

        /**
         * Destructor. Frees the ressources.
         */
        ~KilobotProcess();


        // Getters
        
        inline
        std::string getControllerId() { return m_controllerId; }

        inline
        exp_data_t* getExpData() { return m_expData; }

        inline
        const exp_data_t* getExpData() const { return m_expData; }

        // Other function

        void reset();

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

        argos::CLoopFunctions* m_loopFunc; ///< Loop functions object where to add/remove kilobots.
        std::string m_controllerId;        ///< ID of the kilobot's controller. Must match one of those in the ARGoS file.
        std::string m_expDataName;         ///< Name of the shared memory map.
        int m_expDataFd;                   ///< File descriptor for the memory map file.
        exp_data_t* m_expData;             ///< Pointer to the shared memory maps.

    };

}

#endif // !KILOBOT_PROCESS_H