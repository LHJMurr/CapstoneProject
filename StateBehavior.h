#ifndef STATE_BEHAVIOR_H
#define STATE_BEHAVIOR_H

#include "RobotState.h"
#include "MovementManager.h"
#include "ArmManager.h"
#include "UserInterface.h"

class corner {
  public:
    static void main(int commands[], int turnAdjustments[], int turnTimings[], int outSpeed, int inSpeed, MovementManager* m, RobotState* rs);
  private:
    static int commandIdx;
};

class forwards {
  public:
    static void main(int fullSpeed, int turnSpeed, MovementManager* m, RobotState* rs);
};

class pickup {
  public:
    static void main(int fullSpeed, ArmManager* a, MovementManager* m, RobotState* rs);
};

class dropoff {
  public:
    static void main(int fullSpeed, ArmManager* a, MovementManager* m, RobotState* rs);
};

class crawl {
  public:
    static void main(int crawlSpeed, int turnSpeed, MovementManager* m, RobotState* rs);
};

class done {
  public:
    static void main(InterfaceManager* ui, MovementManager* m, RobotState* rs, Instructions* inst);
};

#endif
