
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <iostream>
#include <iomanip>
#include <array>
#include <string>

const int TIME_STEP {64};
const double MAX_SPEED {6.28};
const int SENSORS_COUNT {8};

int main(int argc, char **argv) {
  webots::Robot robot {};
  // name of distance sensor can be found on https://cyberbotics.com/doc/guide/epuck?version=R2023a
  // we dont use it tho so idk why its there
  std::array<webots::DistanceSensor* ,SENSORS_COUNT> ps {};
  std::array<std::string, SENSORS_COUNT> psNames {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

  for (int i = 0; i <SENSORS_COUNT; i++) {
    ps[i] = robot.getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }

  webots::Motor *leftMotor = robot.getMotor("left wheel motor");
  webots::Motor *rightMotor = robot.getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  while (robot.step(TIME_STEP)!= -1) {
    std::array <double, SENSORS_COUNT> psValues {};
    for (int i = 0; i < SENSORS_COUNT; i++) {
      psValues[i] = ps[i]->getValue();
    }

    // sets n.o. digits to three in the output using iomanip
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time: " << robot.getTime() << " ";

    std::cout << "DistanceSensor: ";
    for (const auto &elem : psValues) {
      std::cout << elem << " ";
    }
    // std::cout << std::endl;

    const double obstacleThreshold {80.0};
    bool rightObstacle {psValues[0] > obstacleThreshold || psValues[1] > obstacleThreshold || psValues[2] > obstacleThreshold};
    bool leftObstacle {psValues[5] > obstacleThreshold || psValues[6] > obstacleThreshold || psValues[7] > obstacleThreshold};

    std::cout << "leftObstacle: " << leftObstacle << " rightObstacle: " << rightObstacle << " ";


    double speedScale {0.5};
    double leftSpeed {speedScale * MAX_SPEED};
    double rightSpeed {speedScale * MAX_SPEED};

    if (leftObstacle) {
      //turn right
      leftSpeed = speedScale * MAX_SPEED;
      rightSpeed = -speedScale * MAX_SPEED;
    } else if (rightObstacle) {
      //turn left
      leftSpeed = -speedScale * MAX_SPEED;
      rightSpeed = speedScale * MAX_SPEED;
    }
    std::cout << "leftSpeed: " << leftSpeed << " rightSpeed: " << rightSpeed << std::endl;
    // move forward
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
    // notice in simuation the robot is more sensitive to the wall than the boxes- this is because of the light levels and other external factors
  }

  return 0;
}
