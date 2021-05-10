//Pin moteurs
#define SPEEDPIN1 2
#define SPEEDPIN2 4
#define DIRPIN11 8
#define DIRPIN12 9
#define DIRPIN21 12
#define DIRPIN22 13

void updateMotor(bool moteur, bool dir, int vel) {

  if (moteur) {
    analogWrite(SPEEDPIN1, vel);
    if (dir) {
      digitalWrite(DIRPIN11, LOW);
      digitalWrite(DIRPIN12, HIGH);
    }
    else{
      digitalWrite(DIRPIN11, HIGH);
      digitalWrite(DIRPIN12, LOW);
    }
  }
  else {
    analogWrite(SPEEDPIN2, vel);
    if (dir) {
      digitalWrite(DIRPIN21, HIGH);
      digitalWrite(DIRPIN22, LOW);
    }
    else{
      digitalWrite(DIRPIN21, LOW);
      digitalWrite(DIRPIN22, HIGH);
    }
  }
}
