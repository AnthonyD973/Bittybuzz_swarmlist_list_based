#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <list>

#include "RobotPlacer.h"

namespace swlexp {
    RobotPlacer RobotPlacer::c_inst;
}

static       argos::Real   rabRange         = 0.00;
static const std::string   FB_CONTROLLER    = "fb_ctrl";
static const argos::UInt32 MAX_PLACE_TRIALS = 20;
static const argos::UInt32 MAX_ROBOT_TRIALS = 20000;
static const argos::Real   FOOTBOT_RADIUS   = 0.085036758f;
static const argos::Real   FB_AREA          = ARGOS_PI * argos::Square(0.085036758f);
static const argos::Real   WALL_THICKNESS   = 0.1;
static const argos::Real   WALL_HEIGHT      = 2.0;
static const argos::Real   DENSITY          = 0.1;

/****************************************/
/****************************************/

using argos::CARGoSException; // Required because of the THROW_ARGOSEXCEPTION macro.

argos::CEntity& swlexp::RobotPlacer::findFarthestFromOrigin() {
    argos::CSpace::TMapPerType& entities =
        argos::CSimulator::GetInstance().
            GetSpace().GetEntitiesByType("foot-bot");

    argos::Real largestDistance = 0.0;
    argos::CFootBotEntity* farthestRobot = nullptr;
    for (auto it = entities.begin(); it != entities.end(); ++it) {
        argos::CFootBotEntity* fbe =
            argos::any_cast<argos::CFootBotEntity*>(it->second);
        argos::Real distance =
            fbe->GetEmbodiedEntity().
            GetOriginAnchor().Position.Length();

        if (distance > largestDistance) {
            largestDistance = distance;
            farthestRobot = fbe;
        }
    }
    if (farthestRobot != nullptr) {
        return *farthestRobot;
    }
    else {
        THROW_ARGOSEXCEPTION("No foot-bots on the arena.");
    }
}

/****************************************/
/****************************************/

void swlexp::RobotPlacer::placeRobots(
        std::string topology,
        argos::UInt32 numRobots,
        argos::CLoopFunctions& loops) {
    m_loops = &loops;

    // Place robots.
    if (topology == "line") {
        rabRange = 0.19;
        RobotPlacer::getInst()._placeLine(numRobots);
    }
    else if (topology == "scalefree") {
        rabRange = 0.19;
        RobotPlacer::getInst()._placeScaleFree(numRobots);
    }
    else if (topology == "cluster") {
        rabRange = Sqrt(FB_AREA / DENSITY) * 2.0;
        RobotPlacer::getInst()._placeCluster(numRobots);
    }
    else {
        THROW_ARGOSEXCEPTION("Unknown topology: " << topology);
    }
}

/****************************************/
/****************************************/

void swlexp::RobotPlacer::_placeLine(argos::UInt32 numRobots) {
    static const argos::Real X_SPACING = 0.00, Y_SPACING = -(rabRange - 0.01);
    // static const argos::Real X_BASEPOS = 0.00, Y_BASEPOS = 0.00, Z_BASEPOS = 0.00;

    // // Resize arena.
    // const argos::Real X_SIZE = 1.0 + X_BASEPOS + (numRobots) * X_SPACING;
    // const argos::Real Y_SIZE = 1.0 + Y_BASEPOS + (numRobots) * Y_SPACING;
    // const argos::Real Z_SIZE = 1.0;
    // const argos::CVector3 ARENA_SIZE = argos::CVector3(X_SIZE, Y_SIZE, Z_SIZE);
    // const argos::CVector3 ARENA_CENTER = argos::CVector3(
    //     (X_BASEPOS + X_SIZE) / 2,
    //     (Y_BASEPOS + Y_SIZE) / 2,
    //     (Z_BASEPOS + Z_SIZE) / 2);

    // GetSpace().SetArenaSize(ARENA_SIZE);
    // GetSpace().SetArenaCenter(ARENA_CENTER);

    // Place the footbots
    argos::Real baseX = X_SPACING * (numRobots / 2.0);
    argos::Real baseY = - Y_SPACING * (numRobots / 2.0);
    for (argos::UInt32 i = 0; i < numRobots; ++i) {
        argos::CVector3 pos = argos::CVector3(baseX + i * X_SPACING, baseY + i * Y_SPACING, 0);
        argos::CQuaternion orient = argos::CQuaternion();

        argos::CFootBotEntity* fb = new argos::CFootBotEntity(
            "fb" + std::to_string(i),
            FB_CONTROLLER,
            pos,
            orient,
            rabRange,
            getPacketSize());
        
        m_loops->AddEntity(*fb);
    }
}

/****************************************/
/****************************************/

void swlexp::RobotPlacer::_placeCluster(argos::UInt32 un_robots) {
   /* Calculate side of the region in which the robots are scattered */
   argos::Real fHalfSide = Sqrt((FB_AREA * un_robots) / DENSITY) / 2.0f;
   argos::CRange<argos::Real> cAreaRange(-fHalfSide, fHalfSide);
   /* Place robots */
   _placeUniformly(un_robots, cAreaRange);
}

/****************************************/
/****************************************/

struct SFData {
   
   struct SEntry {
      argos::UInt32 Conns;
      argos::CVector3& Pos;
      SEntry(argos::UInt32 un_conns,
             argos::CVector3& c_pos) :
         Conns(un_conns),
         Pos(c_pos) {}
   };

   SFData() :
      TotConns(0),
      RNG(argos::CRandom::CreateRNG("argos")) {}

   ~SFData() {
      while(!Data.empty()) {
         delete Data.front();
         Data.pop_front();
      }
   }

   void Insert(argos::CFootBotEntity& c_entity) {
      /* Two connections to be added: entity <-> pivot */
      TotConns += 2;
      Data.push_back(
         new SEntry(1,
                    c_entity.GetEmbodiedEntity().
                    GetOriginAnchor().Position));
   }

   SEntry* Pick() {
      if(Data.size() > 1) {
         /* More than 1 element stored, look for the pivot */
         argos::UInt32 x = RNG->Uniform(argos::CRange<argos::UInt32>(0, TotConns));
         argos::UInt32 unSum = 0;
         std::list<SEntry*>::iterator it = Data.begin();
         while(it != Data.end() && unSum <= x) {
            unSum += (*it)->Conns;
            ++it;
         }
         if(it != Data.end()) {
            --it;
            return *it;
         }
         else {
            return Data.back();
         }
      }
      else if(Data.size() == 1) {
         /* One element stored, just return that one */
         return Data.front();
      }
      else THROW_ARGOSEXCEPTION("SFData::Pick(): empty structure");
   }

private:

   std::list<SEntry*> Data;
   argos::UInt32 TotConns;
   argos::CRandom::CRNG* RNG;
   
};

static argos::Real GenerateCoordinate(argos::CRandom::CRNG* pc_rng) {
   const argos::Real HALF_SF_RANGE = (rabRange / Sqrt(2)) * 0.5f;
   argos::Real v = pc_rng->Uniform(argos::CRange<argos::Real>(-HALF_SF_RANGE, HALF_SF_RANGE));
   if(v > 0.0) v += HALF_SF_RANGE;
   else v -= HALF_SF_RANGE;
   return v;
}

void swlexp::RobotPlacer::_placeScaleFree(argos::UInt32 un_robots) {
   /* Data structures for the insertion of new robots */
   argos::UInt32 unRobotTrials, unPlaceTrials;
   argos::CFootBotEntity* pcFB;
   std::ostringstream cFBId;
   argos::CVector3 cFBPos;
   argos::CQuaternion cFBRot;
   SFData sData;
   SFData::SEntry* psPivot;
   bool bDone;
   /* Create a RNG (it is automatically disposed of by ARGoS) */
   argos::CRandom::CRNG* pcRNG = argos::CRandom::CreateRNG("argos");
   /* Add first robot in the origin */
   /* Create the robot in the origin and add it to ARGoS space */
   pcFB = new argos::CFootBotEntity(
      "fb0",
      FB_CONTROLLER,
      argos::CVector3(),
      argos::CQuaternion(),
      rabRange,
      getPacketSize());
   m_loops->AddEntity(*pcFB);
   sData.Insert(*pcFB);
   /* Add other robots */
   for(argos::UInt32 i = 1; i < un_robots; ++i) {
      /* Make the id */
      cFBId.str("");
      cFBId << "fb" << i;
      /* Create the robot in the origin and add it to ARGoS space */
      pcFB = new argos::CFootBotEntity(
         cFBId.str(),
         FB_CONTROLLER,
         argos::CVector3(),
         argos::CQuaternion(),
         rabRange,
         getPacketSize());
      m_loops->AddEntity(*pcFB);
      /* Retry choosing a pivot until you get a position or have an error */
      unRobotTrials = 0;
      do {
         /* Choose a pivot */
         ++unRobotTrials;
         psPivot = sData.Pick();
         cFBRot.FromAngleAxis(pcRNG->Uniform(argos::CRadians::UNSIGNED_RANGE),
                              argos::CVector3::Z);
         /* Try placing a robot close to this pivot */
         unPlaceTrials = 0;
         do {
            ++unPlaceTrials;
            /* Pick a position within the range of the pivot */
            cFBPos.Set(GenerateCoordinate(pcRNG),
                       GenerateCoordinate(pcRNG),
                       0.0f);
            cFBPos += psPivot->Pos;
            /* Try placing the robot */
            bDone = m_loops->MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
         }
         while(!bDone && unPlaceTrials <= MAX_PLACE_TRIALS);
      } while(!bDone && unRobotTrials <= MAX_ROBOT_TRIALS);
      /* Was the robot placed successfully? */
      if(!bDone) {
         THROW_ARGOSEXCEPTION(__FUNCTION__ << ": Can't place " << cFBId.str());
      }
      /* Yes, insert it in the data structure */
      ++psPivot->Conns;
      sData.Insert(*pcFB);
   }
}

/****************************************/
/****************************************/

void swlexp::RobotPlacer::_placeWalls(argos::UInt32 un_robots) {
   /* Calculate arena side */
   argos::Real fArenaSide =
      rabRange *
      Sqrt((25.0 * ARGOS_PI * un_robots) /
           ((100.0 - 4.0 * ARGOS_PI) * DENSITY));
   argos::Real fArenaSide2 = fArenaSide / 2.0;
   argos::Real fArenaSide5 = fArenaSide / 5.0;
   argos::Real fArenaSide10 = fArenaSide / 10.0;
   /* Place the north wall */
   m_loops->AddEntity(
      *new argos::CBoxEntity("wall_north",
                      argos::CVector3(fArenaSide2, 0, 0),
                      argos::CQuaternion(),
                      false,
                      argos::CVector3(WALL_THICKNESS, fArenaSide, WALL_HEIGHT)));
   /* Place the south wall */
   m_loops->AddEntity(
      *new argos::CBoxEntity("wall_south",
                      argos::CVector3(-fArenaSide2, 0, 0),
                      argos::CQuaternion(),
                      false,
                      argos::CVector3(WALL_THICKNESS, fArenaSide, WALL_HEIGHT)));
   /* Place the west wall */
   m_loops->AddEntity(
      *new argos::CBoxEntity("wall_west",
                      argos::CVector3(0, fArenaSide2, 0),
                      argos::CQuaternion(),
                      false,
                      argos::CVector3(fArenaSide, WALL_THICKNESS, WALL_HEIGHT)));
   /* Place the east wall */
   m_loops->AddEntity(
      *new argos::CBoxEntity("wall_east",
                      argos::CVector3(0, -fArenaSide2, 0),
                      argos::CQuaternion(),
                      false,
                      argos::CVector3(fArenaSide, WALL_THICKNESS, WALL_HEIGHT)));
   /* Place the NW column */
   m_loops->AddEntity(
      *new argos::CCylinderEntity("col_nw",
                           argos::CVector3(fArenaSide5, fArenaSide5, 0),
                           argos::CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
   /* Place the NE column */
   m_loops->AddEntity(
      *new argos::CCylinderEntity("col_ne",
                           argos::CVector3(fArenaSide5, -fArenaSide5, 0),
                           argos::CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
   /* Place the SW column */
   m_loops->AddEntity(
      *new argos::CCylinderEntity("col_sw",
                           argos::CVector3(-fArenaSide5, fArenaSide5, 0),
                           argos::CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
   /* Place the SE column */
   m_loops->AddEntity(
      *new argos::CCylinderEntity("col_se",
                           argos::CVector3(-fArenaSide5, -fArenaSide5, 0),
                           argos::CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
   /* Calculate side of the region in which the robots are scattered */
   argos::CRange<argos::Real> cAreaRange(-fArenaSide2, fArenaSide2);
   /* Place robots */
   _placeUniformly(un_robots, cAreaRange);
}

/****************************************/
/****************************************/

void swlexp::RobotPlacer::_placeUniformly(argos::UInt32 un_robots,
                            argos::CRange<argos::Real> c_area_range) {
   argos::UInt32 unTrials;
   argos::CFootBotEntity* pcFB;
   std::ostringstream cFBId;
   argos::CVector3 cFBPos;
   argos::CQuaternion cFBRot;
   /* Create a RNG (it is automatically disposed of by ARGoS) */
   argos::CRandom::CRNG* pcRNG = argos::CRandom::CreateRNG("argos");
   /* For each robot */
   for(size_t i = 0; i < un_robots; ++i) {
      /* Make the id */
      cFBId.str("");
      cFBId << "fb" << i;
      /* Create the robot in the origin and add it to ARGoS space */
      pcFB = new argos::CFootBotEntity(
         cFBId.str(),
         FB_CONTROLLER,
         argos::CVector3(),
         argos::CQuaternion(),
         rabRange,
         getPacketSize());
      m_loops->AddEntity(*pcFB);
      /* Try to place it in the arena */
      unTrials = 0;
      bool bDone;
      do {
         /* Choose a random position */
         ++unTrials;
         cFBPos.Set(pcRNG->Uniform(c_area_range),
                    pcRNG->Uniform(c_area_range),
                    0.0f);
         cFBRot.FromAngleAxis(pcRNG->Uniform(argos::CRadians::UNSIGNED_RANGE),
                              argos::CVector3::Z);
         bDone = m_loops->MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
      } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
      if(!bDone) {
         THROW_ARGOSEXCEPTION(__FUNCTION__ << ": Can't place " << cFBId.str());
      }
   }
}

/****************************************/
/****************************************/

argos::Real swlexp::RobotPlacer::getRabRange() {
    return rabRange;
}

/****************************************/
/****************************************/

std::string swlexp::RobotPlacer::getControllerName() {
    return FB_CONTROLLER;
}