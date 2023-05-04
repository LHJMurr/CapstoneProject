#ifndef STATE_BEHAVIOR_H
#define STATE_BEHAVIOR_H

#include "RobotState.h"
#include "MovementManager.h"
#include "ArmManager.h"

class corner {
  public:
    static void main(int commands[], int numCommands, int turnSpeed, MovementManager* m, RobotState* rs);
  private:
    static int commandIdx;
};

class forwards {
  public:
    static void main(int fullSpeed, int turnSpeed, MovementManager* m, RobotState* rs);
};

class pickup {
  public:
    static void main(int adjustSpeed, int turnSpeed, ArmManager* a, MovementManager* m, RobotState* rs);
  private:
    static void alignJuicebox(int adjustSpeed, MovementManager* m);
    static void alignAngle(bool checkRight, bool* cornerFound, int adjustSpeed, MovementManager* m);
};

class dropoff {
  public:
    static void main(int turnSpeed, ArmManager* a, MovementManager* m, RobotState* rs);
};

class crawl {
  public:
    static void main(int crawlSpeed, int turnSpeed, MovementManager* m, RobotState* rs);
};

class done {
  public:
    static void main();
};

#endif
