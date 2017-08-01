#ifndef ROBOT_PLACER_H
#define ROBOT_PLACER_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <string>

#include "include.h"

namespace swlexp {

    class RobotPlacer {

    // ==============================
    // =          METHODS           =
    // ==============================

    public:

        /**
         * Places the robots depending on the topology.
         */
        void placeRobots(std::string topology, argos::UInt32 numRobots, argos::CLoopFunctions& loops);

        /**
         * Determines which robot currently is the farthest from the
         * arena's origin.
         * Throws an ARGoS exception if there are no foot-bots on the arena.
         * @return The robot farthest away from the arena's origin.
         */
        argos::CEntity& findFarthestFromOrigin();

    private:

        /**
         * Places a certain number of robots in a line.
         * @param[in] numRobots The number of robots to place.
         */
        void _placeLine(argos::UInt32 numRobots);

        /**
         * Places a certain number of robots in one large cluster in
         * such a way that robots have a high number of neighbors in their
         * communication range.
         * @param[in] numRobots The number of robots to place.
         */
        void _placeScaleFree(argos::UInt32 numRobots);

        /**
         * Places a certain number of robots randomly inside a square.
         * @param[in] numRobots The number of robots to place.
         */
        void _placeCluster(argos::UInt32 numRobots);

        /**
         * Places a certain number of robots uniformly in an area.
         * @param[in] numRobots The number of robots to place.
         * @param[in] area The area to place the robots in.
         */
        void _placeUniformly(argos::UInt32 numRobots, argos::CRange<argos::Real> area);

        /**
         * Places the walls of the arena.
         * @param[in] numRobots The number of robots to place.
         * @param[in] density Density of the robots.
         */
        void _placeWalls(argos::UInt32 numRobots);

    // ==============================
    // =       STATIC METHODS       =
    // ==============================

    public:

        /**
         * Instance getter.
         */
        inline static
        RobotPlacer& getInst() { return c_inst; }

        static
        argos::Real getRabRange();

        static
        std::string getControllerName();

    // ==============================
    // =         ATTRIBUTES         =
    // ==============================

    private:

        argos::CLoopFunctions* m_loops; ///< Loop functions to use to place the robots.

    // ==============================
    // =       STATIC MEMBERS       =
    // ==============================

    private:
        static RobotPlacer c_inst; ///< Instance.

    };

}

#endif // !ROBOT_PLACER_H