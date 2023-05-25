#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

class InterfaceManager {
  public:
    InterfaceManager(int dL, int iB, int bD);
    void initialize();
    void blinkDoneLight(bool state, int numTimes); // Sets doneLight's state then blinks the it n times asynchronously.
    bool wasPressed();
  private:
    int doneLight; // Lights up solid IF the robot is in the DONE state.
    int interfaceButton; // Allows input from the user. IF in the DONE state, press the interface button n times with in quick succession to give the robot a command string and start.
    int blinkDelay; // Delay between blinks.
};

#endif
