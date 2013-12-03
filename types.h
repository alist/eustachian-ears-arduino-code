enum SystemStatus {
  unknownStatus = 0,
  faultStatus,
  goodStatus
};

enum SessionState{
  pendingState = 0,
  estopState,
  idleState,
  beginState,
  activeState,
  endState
};

const int D10 = 10; //pin 30 from the atmega //arduino somehow maps
const int D11 = 11; //pin 12 from the atmega //arduino somehow maps
