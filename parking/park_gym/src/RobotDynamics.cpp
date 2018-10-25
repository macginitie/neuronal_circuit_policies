#include "RobotDynamics.h"
#include <cmath>
#include <iostream>

RobotDynamics::RobotDynamics() { 
  Reset(); 
}

void RobotDynamics::Reset() {
  totalTime = 0;
  x = 0;
  y = 0;
  v_is = 0;
  v_want = 0;
  theta = 0;
  w_is = 0;
  w_want = 0;

  while (!linearActuatorCommandQueue.empty()) {
    linearActuatorCommandQueue.pop();
  }
  while (!angularActuatorCommandQueue.empty()) {
    angularActuatorCommandQueue.pop();
  }
}

void RobotDynamics::CheckActuatorCommandQueue() {
  if (linearActuatorCommandQueue.size() > 0) {
    std::pair<float, float> firstItemLinearActuator =
        linearActuatorCommandQueue.front();
    if (firstItemLinearActuator.first <= totalTime) {
      v_want = firstItemLinearActuator.second;
      linearActuatorCommandQueue.pop();
    }
  }

  if (angularActuatorCommandQueue.size() > 0) {
    std::pair<float, float> firstItemAngularActuator =
        angularActuatorCommandQueue.front();
    if (firstItemAngularActuator.first <= totalTime) {
      w_want = firstItemAngularActuator.second;
      angularActuatorCommandQueue.pop();
    }
  }
}

void RobotDynamics::DoSimulationStep(float deltaT) {
  totalTime += deltaT;

  CheckActuatorCommandQueue();

  bool accelerate = v_want >= v_is;

  if (accelerate) {
    // Accelerate to v_want
    v_is += Acceleration * deltaT;
    if (v_is > v_want)
      v_is = v_want;
  } else {
    // Decelerate to v_want
    v_is -= Acceleration * deltaT;
    if (v_is < v_want)
      v_is = v_want;
  }

  bool angularAccelerate = w_want >= w_is;

  if (angularAccelerate) {
    // Accelerate to w_want
    w_is += AngularAcceleration * deltaT;
    if (w_is > w_want)
      w_is = w_want;
  } else {
    // Decelerate to w_want
    w_is -= AngularAcceleration * deltaT;
    if (w_is < w_want)
      w_is = w_want;
  }

  x += deltaT * std::cos(theta) * v_is;
  y += deltaT * std::sin(theta) * v_is;

  theta += deltaT * w_is;
  if (theta > M_PI / 2)
    theta -= M_PI;
  if (theta < -M_PI / 2)
    theta += M_PI;
}

void RobotDynamics::SendCommands(float linearVelocity, float angularVelocity) {

  std::pair<float, float> enqueItemLinearActuator =
      std::make_pair(totalTime + linearActuatorDelay, linearVelocity);
  linearActuatorCommandQueue.push(enqueItemLinearActuator);

  std::pair<float, float> enqueItemAngularActuator =
      std::make_pair(totalTime + angularActuatorDelay, angularVelocity);
  angularActuatorCommandQueue.push(enqueItemAngularActuator);
}
