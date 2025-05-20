#pragma once

#include <Arduino.h>
#include <mcp2515_can.h>
#include "Lib_Var.hpp"

class CAN
{
public:
  CAN(uint8_t cs);

  void begin();
  bool available();
  Var::Label getLatestLabel();

  void sendFlight(uint8_t flightMode, uint16_t flightTime, bool doLogging, char ident);
  void receiveFlight(uint8_t *flightMode, uint16_t *flightTime, bool *doLogging, char *ident);

  void sendTrajectory(bool isFalling, float altitude);
  void receiveTrajectory(bool *isFalling, float *altitude);

  void receiveSutegomaTemperature(float *ventPortTemperature, float *tankAtmosphereTemperature);
  void receiveSutegomaPerformance(uint32_t *time, float *taskRate);

  void sendValveMode(bool isLaunchMode);
  void receiveValveMode(bool *isLaunchMode);

  void sendIgnition(bool isIgnition);
  void receiveIgnition(bool *isIgnition);

  void sendValveDataPart1(int16_t motorTemperature, int16_t mcuTemperature, int16_t current, uint16_t inputVoltage);
  void receiveValveDataPart1(float *motorTemperature, float *mcuTemperature, float *current, float *inputVoltage);

  void sendValveDataPart2(int16_t currentPosition, int16_t currentDesiredPosition, int16_t currentVelocity);
  void receiveValveDataPart2(float *currentPosition, float *currentDesiredPosition, float *currentVelocity);

  void sendValveDataPart3(int16_t currentPositionSupply, int16_t voltageSupply);
  void receiveValveDataPart3(float *currentPositionSupply, float *voltageSupply);

  void sendDynamics(float force, float jerk);
  void receiveDynamics(float *force, float *jerk);

private:
  mcp2515_can *_can;

  uint32_t _latestLabel;
  uint8_t _latestData[8];
};
