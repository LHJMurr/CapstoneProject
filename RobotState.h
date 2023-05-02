#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

// States
class RobotState {
  public:
    void changeState(int newState);
    int getState() const;
    RobotState();

    // State Values
    static const int CORNER = 0;
    static const int FORWARDS = 1;
    static const int PICKUP = 2;
    static const int DROPOFF = 3; 
    static const int CRAWL = 4;
    static const int DONE = 5;
    
  private:
    int currentState; 
};

#endif
