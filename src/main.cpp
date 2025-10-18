#include <Arduino.h>
#include <Servo.h>

class Control {
  Servo servo;

  private:
    float x0, x1, x2, x3;
    float t1, t2, t3;
    float vmax, amax;
    bool parametersCalculated = false;
    bool isMoving = false;
    unsigned long startTime;

    int degreesToMicroseconds(float degrees) {
      return map(degrees * 10, 0, 1800, 1000, 2000);
    }

    float calculatePosition(float t) {
      if (t <= t1) {
        return x0 + (amax * t * t) / 2;
      }
      else if (t <= t1 + t2) {
        return x1 + vmax * (t - t1);
      } 
      else if (t <= t1 + t2 + t3) {
        float t_brake = t - t1 - t2;
        return x2 + vmax * t_brake - (amax * t_brake * t_brake) / 2;
      } 
      else { return x3; }
    }

  public:
    Control() {
      servo.attach(9);
    }

    void setParamets(float x0, float x3, float vamax, float amax) {
      this->x0 = x0;
      this->x3 = x3;
      this->vmax = vmax;
      this->amax = amax;
      parametersCalculated = false;
    }

  void calculatedParameters() {
          t1 = vmax / amax;
          x1 = x0 + (amax * pow(t1, 2)) / 2;
          double brakingDistance = (amax * pow(t1, 2)) / 2;
          double distConstSpeed = x1 - x0 - (x1 - x0) - brakingDistance;
          double timeConst = distConstSpeed / vmax;
          x2 = x1 + (vmax * timeConst);
          parametersCalculated = true;
  }

  void startMovement() {
      if (!parametersCalculated) {
        calculatedParameters();
      }
      startTime = millis();
      isMoving = true;
    }

    void update() {
      if (!isMoving) return;
      
      unsigned long currentTime = millis();
      float elapsedTime = (currentTime - startTime) / 1000.0;
      
      float position_deg = calculatePosition(elapsedTime);
      
      position_deg = constrain(position_deg, 0, 180);
      
      int pulseWidth = degreesToMicroseconds(position_deg);
      servo.writeMicroseconds(pulseWidth);
      
      Serial.print("Time: "); Serial.print(elapsedTime);
      Serial.print("s, Position: "); Serial.print(position_deg);
      Serial.print("°, Pulse: "); Serial.println(pulseWidth);
      
      if (elapsedTime >= (t1 + t2 + t3)) {
        isMoving = false;
        servo.write(round(x3));
        Serial.println("Movement completed");
      }
    }

    bool movementCompleted() {
      return !isMoving;
    }
    
    void moveTo(float target, float speed, float acceleration) {
      setParamets(x3, target, speed, acceleration);
      startMovement();
    }
};

Control servoController;

void setup() {
  Serial.begin(9600);
  Serial.println("Servo Control Started");
  
  servoController.setParamets(0, 90, 60, 30);
  delay(1000);
  servoController.startMovement();
}



void loop() {
  servoController.update();
  
  if (servoController.movementCompleted()) {
    delay(2000);
    static int movementCount = 0;
    
    /*switch(movementCount) {
        servoController.moveTo(180, 20, 20);
        movementCount++;
        break;
      case 1:
        servoController.moveTo(45, 35, 35);
        movementCount++;
        break;
      case 2:
        servoController.moveTo(135, 15, 15);
        movementCount++;
        break;
      case 3:
        servoController.moveTo(0, 90, 40);
        movementCount = 0;
        break;
    }*/
   servoController.moveTo(175, 50, 50);
   servoController.moveTo(5, -50, -50);
  }
}
