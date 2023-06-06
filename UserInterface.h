#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

class InterfaceManager {
  public:
    InterfaceManager(int dL, int sL, int iB, int bD);
    void initialize();
    void blinkDoneLight(bool state, int numTimes); // Sets doneLight's state then blinks the it n times asynchronously.
    bool wasPressed();
    int stateLight;
  private:
    int doneLight; // Lights up solid IF the robot is in the DONE state.
    int interfaceButton; // Allows input from the user. IF in the DONE state, press the interface button n times with in quick succession to give the robot a command string and start.
    int blinkDelay; // Delay between blinks.
};

class Instructions {
  public:
    Instructions();
    void setCommands(int roll_1_result, int roll_2_result);
    int commandInstructions[8];
    int getNumCommands() const;
  private:
    const int numCommandsRoll_1 = 4;
    const int numCommandsRoll_2 = 4;
    const int roll_1[6][4] = {
                              {0, 1, 0, 1},
                              {0, 1, 1, 1},
                              {0, 1, -1, 1},
                              {1, 1, 1, 1},
                              {-1, 1, 1, 1},
                              {-1, 1, -1, 1}
                              };
    const int roll_2[6][4] = {
                              {0, 1, 0, 1},
                              {0, 1, 1, 1},
                              {0, 1, -1, 1},
                              {1, 1, 1, 1},
                              {-1, 1, 1, 1},
                              {-1, 1, -1, 1}
                              };
};

#endif
