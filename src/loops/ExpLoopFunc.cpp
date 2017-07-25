#include <algorithm>
#include <cstdio>
#include <list>

#include "ExpLoopFunc.h"

namespace swlexp {
    const argos::Real   ExpLoopFunc::c_RAB_RANGE   = 0.19;
    argos::UInt16       ExpLoopFunc::c_packetSize;
}

static const std::string   FB_CONTROLLER    = "fb_ctrl";
static const argos::UInt32 MAX_PLACE_TRIALS = 20;
static const argos::UInt32 MAX_ROBOT_TRIALS = 20000;
static const argos::Real   FOOTBOT_RADIUS   = 0.085036758f;
static const argos::Real   SF_RANGE         = swlexp::ExpLoopFunc::getRabRange() / Sqrt(2);
static const argos::Real   HALF_SF_RANGE    = SF_RANGE * 0.5f;
static const argos::Real   WALL_THICKNESS   = 0.1;
static const argos::Real   WALL_HEIGHT      = 2.0;
static const argos::Real   DENSITY          = 0.1;

/****************************************/
/****************************************/

swlexp::ExpLoopFunc::ExpLoopFunc()
{ }

/****************************************/
/****************************************/

swlexp::ExpLoopFunc::~ExpLoopFunc() {

}

/****************************************/
/****************************************/

using argos::CARGoSException; // Required because of the THROW_ARGOSEXCEPTION macro.

void swlexp::ExpLoopFunc::Init(argos::TConfigurationNode& t_tree) {

    // Get experiment params
    argos::TConfigurationNode controllers        = argos::GetNode(GetSimulator().GetConfigurationRoot(), "controllers");
    argos::TConfigurationNode footbot_controller = argos::GetNode(controllers,        "footbot_controller");
    argos::TConfigurationNode sensors            = argos::GetNode(footbot_controller, "sensors");
    argos::TConfigurationNode rab                = argos::GetNode(sensors,            "range_and_bearing");
    argos::GetNodeAttributeOrDefault(rab, "packet_drop_prob", m_msgDropProb, 0.0);

    argos::TConfigurationNode params             = argos::GetNode(footbot_controller, "params");
    argos::GetNodeAttribute(params, "packet_size", c_packetSize);

    argos::GetNodeAttribute(t_tree, "res", m_expResName);
    argos::GetNodeAttribute(t_tree, "log", m_expLogName);
    argos::GetNodeAttribute(t_tree, "fb_csv", m_expFbCsvName);
    argos::GetNodeAttribute(t_tree, "realtime_output_file", m_expRealtimeOutputName);
    argos::GetNodeAttribute(t_tree, "fb_status_log_delay", m_expStatusLogDelay);
    argos::UInt32 numRobots;
    argos::GetNodeAttribute(t_tree, "num_robots", numRobots);
    std::string topology;
    argos::GetNodeAttribute(t_tree, "topology", topology);
    if (topology == "line") {
        _placeLine(numRobots);
    }
    else if (topology == "scalefree") {
        _placeScaleFree(numRobots);
    }
    else {
        THROW_ARGOSEXCEPTION("Unknown topology: " << topology);
    }

    // Open files.
    m_expFbCsv.open(m_expFbCsvName, std::ios::trunc);
    if (m_expFbCsv.fail()) {
        THROW_ARGOSEXCEPTION("Could not open CSV file \"" << m_expFbCsvName << "\".");
    }
    if (m_expRealtimeOutputName != "") {
        m_expRealtimeOutput.open(m_expRealtimeOutputName, std::ios::trunc);
        if (m_expRealtimeOutput.fail()) {
            THROW_ARGOSEXCEPTION("Could not open realtime CSV file \"" << m_expFbCsvName << "\".");
        }
    }
    else {
        argos::LOG << "No realtime file used.\n";
    }
    m_expRes.open(m_expResName, std::ios::trunc);
    if (m_expRes.fail()) {
        THROW_ARGOSEXCEPTION("Could not open CSV file \"" << m_expResName << "\".");
    }
    m_expLog.open(m_expLogName, std::ios::trunc);
    if (m_expLog.fail()) {
        THROW_ARGOSEXCEPTION("Could not open log file \"" << m_expLogName << "\".");
    }

    // Write experiment params to result and log files.
    m_expRes << '\n' <<
                topology      << c_CSV_DELIM <<
                numRobots     << c_CSV_DELIM <<
                m_msgDropProb;

    m_expLog << "---EXPERIMENT START---\n"
                "Topology: " << topology << "\n" <<
                "Number of robots: " << numRobots << "\n"
                "Drop probability: " << (m_msgDropProb * 100) << "%\n";

    argos::LOG << "--------------------------------------\n"
                  "TOPOLOGY: " << topology << "\n" <<
                  "NUMBER OF ROBOTS: " << numRobots << "\n"
                  "PACKET DROP PROBABILITY: " << (m_msgDropProb * 100) << "%\n"
                  "--------------------------------------\n";
    m_expLog.flush();

    // Setup realtime output.
    m_timeSinceLastRealtimeOutput = std::time(NULL);
    swlexp::FootbotController::writeStatusLogs(m_expRealtimeOutput, false);
    m_expRealtimeOutput.flush();

    // Write header in the status logs file and perform the first status log.
    swlexp::FootbotController::writeStatusLogHeader(m_expFbCsv);
    swlexp::FootbotController::writeStatusLogs(m_expFbCsv, true);
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::Destroy() {
    m_expRes.flush();
    m_expLog.flush();
    m_expFbCsv.flush();
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::PostStep() {
    static argos::UInt32 callsTillStatusLog = m_expStatusLogDelay - 1;

    // Write to realtime status log every 5 minutes.
    static const argos::UInt32 DELAY_FOR_REALTIME_LOG = 300;
    std::time_t time = std::time(NULL);
    if (time - m_timeSinceLastRealtimeOutput >= DELAY_FOR_REALTIME_LOG) {
        m_timeSinceLastRealtimeOutput = time;
        swlexp::FootbotController::writeStatusLogs(m_expRealtimeOutput, false);
        m_expRealtimeOutput.flush();
    }

    if (callsTillStatusLog == 0) {
        callsTillStatusLog = m_expStatusLogDelay;
        swlexp::FootbotController::writeStatusLogs(m_expFbCsv, true);
    }

    --callsTillStatusLog;
}

/****************************************/
/****************************************/

bool swlexp::ExpLoopFunc::IsExperimentFinished() {
    bool isFinished = swlexp::FootbotController::isConsensusReached();
    if (isFinished) {
        _finishExperiment(swlexp::ExpLoopFunc::ExitCode::NORMAL);
    }
    return isFinished;
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::_placeLine(argos::UInt32 numRobots) {
    static const argos::Real X_SPACING = 0.00, Y_SPACING = 0.18;
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
    //     std::cout.flush();

    // GetSpace().SetArenaSize(ARENA_SIZE);
    // GetSpace().SetArenaCenter(ARENA_CENTER);

    // Place the footbots
    argos::Real baseX = - X_SPACING * (numRobots / 2.0);
    argos::Real baseY = - Y_SPACING * (numRobots / 2.0);
    for (argos::UInt32 i = 0; i < numRobots; ++i) {
        argos::CVector3 pos = argos::CVector3(baseX + i * X_SPACING, baseY + i * Y_SPACING, 0);
        argos::CQuaternion orient = argos::CQuaternion();

        argos::CFootBotEntity* fb = new argos::CFootBotEntity(
            "fb" + std::to_string(i),
            FB_CONTROLLER,
            pos,
            orient,
            c_RAB_RANGE,
            c_packetSize);
        
        AddEntity(*fb);
    }
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
   argos::Real v = pc_rng->Uniform(argos::CRange<argos::Real>(-HALF_SF_RANGE, HALF_SF_RANGE));
   if(v > 0.0) v += HALF_SF_RANGE;
   else v -= HALF_SF_RANGE;
   return v;
}

void swlexp::ExpLoopFunc::_placeScaleFree(argos::UInt32 un_robots) {
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
      c_RAB_RANGE,
      c_packetSize);
   AddEntity(*pcFB);
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
         c_RAB_RANGE,
         c_packetSize);
      AddEntity(*pcFB);
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
            bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
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

void swlexp::ExpLoopFunc::_placeWalls(argos::UInt32 un_robots) {
   /* Calculate arena side */
   argos::Real fArenaSide =
      c_RAB_RANGE *
      Sqrt((25.0 * ARGOS_PI * un_robots) /
           ((100.0 - 4.0 * ARGOS_PI) * DENSITY));
   argos::Real fArenaSide2 = fArenaSide / 2.0;
   argos::Real fArenaSide5 = fArenaSide / 5.0;
   argos::Real fArenaSide10 = fArenaSide / 10.0;
   /* Place the north wall */
   AddEntity(
      *new argos::CBoxEntity("wall_north",
                      argos::CVector3(fArenaSide2, 0, 0),
                      argos::CQuaternion(),
                      false,
                      argos::CVector3(WALL_THICKNESS, fArenaSide, WALL_HEIGHT)));
   /* Place the south wall */
   AddEntity(
      *new argos::CBoxEntity("wall_south",
                      argos::CVector3(-fArenaSide2, 0, 0),
                      argos::CQuaternion(),
                      false,
                      argos::CVector3(WALL_THICKNESS, fArenaSide, WALL_HEIGHT)));
   /* Place the west wall */
   AddEntity(
      *new argos::CBoxEntity("wall_west",
                      argos::CVector3(0, fArenaSide2, 0),
                      argos::CQuaternion(),
                      false,
                      argos::CVector3(fArenaSide, WALL_THICKNESS, WALL_HEIGHT)));
   /* Place the east wall */
   AddEntity(
      *new argos::CBoxEntity("wall_east",
                      argos::CVector3(0, -fArenaSide2, 0),
                      argos::CQuaternion(),
                      false,
                      argos::CVector3(fArenaSide, WALL_THICKNESS, WALL_HEIGHT)));
   /* Place the NW column */
   AddEntity(
      *new argos::CCylinderEntity("col_nw",
                           argos::CVector3(fArenaSide5, fArenaSide5, 0),
                           argos::CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
   /* Place the NE column */
   AddEntity(
      *new argos::CCylinderEntity("col_ne",
                           argos::CVector3(fArenaSide5, -fArenaSide5, 0),
                           argos::CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
   /* Place the SW column */
   AddEntity(
      *new argos::CCylinderEntity("col_sw",
                           argos::CVector3(-fArenaSide5, fArenaSide5, 0),
                           argos::CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
   /* Place the SE column */
   AddEntity(
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

void swlexp::ExpLoopFunc::_placeUniformly(argos::UInt32 un_robots,
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
         c_RAB_RANGE,
         c_packetSize);
      AddEntity(*pcFB);
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
         bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
      } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
      if(!bDone) {
         THROW_ARGOSEXCEPTION(__FUNCTION__ << ": Can't place " << cFBId.str());
      }
   }
}

/****************************************/
/****************************************/

void swlexp::ExpLoopFunc::_finishExperiment(swlexp::ExpLoopFunc::ExitCode exitCode) {
    m_expLog << "---END---\n";

    if (exitCode == NORMAL) {
        const argos::UInt32 NUM_FOOTBOTS =
            swlexp::FootbotController::getNumControllers();
        const argos::UInt64 NUM_MSGS_TX =
            swlexp::FootbotController::getTotalNumMessagesTx();
        const argos::UInt64 NUM_MSGS_RX =
            swlexp::FootbotController::getTotalNumMessagesRx();
        double bwTx = ((double)NUM_MSGS_TX / GetSpace().GetSimulationClock() /
                    NUM_FOOTBOTS) * c_packetSize;
        double bwRx = ((double)NUM_MSGS_RX / GetSpace().GetSimulationClock() /
                    NUM_FOOTBOTS) * c_packetSize;

        m_expRes << c_CSV_DELIM << GetSpace().GetSimulationClock() <<
                    c_CSV_DELIM << NUM_MSGS_TX <<
                    c_CSV_DELIM << NUM_MSGS_RX <<
                    c_CSV_DELIM << bwTx <<
                    c_CSV_DELIM << bwRx;
        m_expRes.flush();
        m_expLog << "Consensus (ts): " << GetSpace().GetSimulationClock() << "\n"
                    "Msgs sent (total): " << NUM_MSGS_TX << "\n"
                    "Msgs received (total): " << NUM_MSGS_RX << "\n"
                    "Avg. sent bandwidth (B/(timestep*foot-bot)): " << bwTx << "\n"
                    "Avg. received bandwidth (B/(timestep*foot-bot)): " << bwRx << "\n"
                    "\n";
        m_expLog.flush();

        swlexp::FootbotController::writeStatusLogs(m_expFbCsv, true);
        m_expFbCsv.flush();

        argos::LOG << "Experiment finished normally in " << GetSpace().GetSimulationClock() <<
                    " timesteps. See \"" << m_expLogName << "\" for results.\n";
    }
    else {
        remove(m_expFbCsvName.c_str());
        remove(m_expResName.c_str());
        m_expLog      << "[ERROR] " << _exitCodeToString(exitCode) << "\n";
        m_expLog.flush();
    }
}

/****************************************/
/****************************************/

std::string swlexp::ExpLoopFunc::_exitCodeToString(swlexp::ExpLoopFunc::ExitCode exitCode) {
    if (exitCode == NORMAL) {
        return "";
    }
    else {
        THROW_ARGOSEXCEPTION("Unknown exit code: " << exitCode);
    }
}

/****************************************/
/****************************************/

using swlexp::ExpLoopFunc;

REGISTER_LOOP_FUNCTIONS(ExpLoopFunc, "exp_loop_func");
