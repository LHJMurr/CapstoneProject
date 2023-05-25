#ifndef STATE_BEHAVIOR_H
#define STATE_BEHAVIOR_H

#include "RobotState.h"
#include "MovementManager.h"
#include "ArmManager.h"

class corner {
  public:
    static void main(int commands[], int turnTimings[], int numCommands, int outSpeed, int inSpeed, MovementManager* m, RobotState* rs);
  private:
    static int commandIdx;
};

class forwards {
  public:
    static void main(int fullSpeed, int turnSpeed, MovementManager* m, RobotState* rs);
};

class pickup {
  public:
    static void main(int adjustSpeed, int outSpeed, int inSpeed, ArmManager* a, MovementManager* m, RobotState* rs);
  private:
    static void alignJuicebox(int adjustSpeed, MovementManager* m);
    static void alignAngle(bool checkRight, bool* cornerFound, int adjustSpeed, MovementManager* m, int distanceEpsilon);
};

class dropoff {
  public:
    static void main(int outSpeed, int inSpeed, ArmManager* a, MovementManager* m, RobotState* rs);
};

class crawl {
  public:
    static void main(int crawlSpeed, int turnSpeed, MovementManager* m, RobotState* rs);
};

class done {
  public:
    static void main(InterfaceManager* ui, RobotState* rs);
};

#endif
