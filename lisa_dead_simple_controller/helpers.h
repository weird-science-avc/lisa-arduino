struct Location {
  float x;
  float y;
};

struct Position {
  float x;
  float y;
  float r;
};

struct Vector {
  float r;
  float d;
};

void serialPrintLocation(char* prefix, Location l);
void serialPrintPosition(char* prefix, Position p);

Vector vectorToTarget(Position position, Location target);
bool haveArrived(float distance);

Position calculateNewPosition(Position position, float distance, float turnRadius);
float calculateNewVelocity(float speed, float distance);
int calculateNewSteering(float steering, float angle);

float turnRadiusFromSteering(int steering);
void setVelocity(float speed);
void setSteering(int steering);
