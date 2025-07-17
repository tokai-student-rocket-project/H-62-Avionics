#include "Lib_CAN.hpp"

CAN::CAN(uint8_t cs)
{
  _can = new mcp2515_can(cs);
}

void CAN::begin()
{
  _can->begin(CAN_125KBPS, MCP_8MHz);
}

bool CAN::available()
{
  bool isAvailable = _can->checkReceive() == CAN_MSGAVAIL;

  if (isAvailable)
  {
    uint8_t len = 0;
    _can->readMsgBuf(&len, _latestData);
    _latestLabel = _can->getCanId();
  }

  return isAvailable;
}

Var::Label CAN::getLatestLabel()
{
  return static_cast<Var::Label>(_latestLabel);
}

void CAN::sendFlight(uint8_t flightMode, uint16_t flightTime, bool doLogging, char ident)
{
  uint8_t data[5];
  data[0] = flightMode;
  memcpy(data + 1, &flightTime, 2);
  data[3] = doLogging;
  data[4] = ident;

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::FLIGHT_DATA), 0, 5, data);
}

void CAN::receiveFlight(uint8_t *flightMode, uint16_t *flightTime, bool *doLogging, char *ident)
{
  *flightMode = _latestData[0];
  memcpy(flightTime, _latestData + 1, 2);
  *doLogging = _latestData[3];
  *ident = _latestData[4];
}

void CAN::sendTrajectory(bool isFalling, float altitude)
{
  uint8_t data[5];
  data[0] = isFalling;
  memcpy(data + 1, &altitude, 4);

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::TRAJECTORY_DATA), 0, 5, data);
}

void CAN::receiveTrajectory(bool *isFalling, float *altitude)
{
  *isFalling = _latestData[0];
  memcpy(altitude, _latestData + 1, 4);
}

void CAN::receiveSutegomaTemperature(float *ventPortTemperature, float *tankAtmosphereTemperature)
{
  memcpy(ventPortTemperature, _latestData + 0, 4);
  memcpy(tankAtmosphereTemperature, _latestData + 4, 4);
}

void CAN::receiveSutegomaPerformance(uint32_t *time, float *taskRate)
{
  memcpy(time, _latestData + 0, 4);
  memcpy(taskRate, _latestData + 4, 4);
}

void CAN::sendValveMode(bool isLaunchMode)
{
  uint8_t data[1];
  data[0] = isLaunchMode;

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::VALVE_MODE), 0, 1, data);
}

void CAN::receiveValveMode(bool *isLaunchMode)
{
  *isLaunchMode = _latestData[0];
}

void CAN::sendIgnition(bool isIgnition)
{
  uint8_t data[1];
  data[0] = isIgnition;

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::GSE_SIGNAL), 0, 1, data);
}

void CAN::receiveIgnition(bool *isIgnition)
{
  *isIgnition = _latestData[0];
}

void CAN::sendBusMonitor(float busVoltage, float busCurrent, float busPower, float busTemperature)
{
  int16_t busVoltage_int = static_cast<int16_t>(busVoltage * 100.0);
  int16_t busCurrent_int = static_cast<int16_t>(busCurrent * 100.0);
  int16_t busPower_int = static_cast<int16_t>(busPower * 100.0);
  int16_t busTemperature_int = static_cast<int16_t>(busTemperature * 100.0);

  uint8_t data[8];
  memcpy(data, &busVoltage_int, 2);
  memcpy(data + 2, &busCurrent_int, 2);
  memcpy(data + 4, &busPower_int, 2);
  memcpy(data + 6, &busTemperature_int, 2);

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::MONITOR_BUS), 0, 8, data);
}

void CAN::receiveBusMonitor(float *busVoltage, float *busCurrent, float *busPower, float *busTemperature)
{
  int16_t busVoltage_int, busCurrent_int, busPower_int, busTemperature_int;

  memcpy(&busVoltage_int, _latestData + 0, 2);
  memcpy(&busCurrent_int, _latestData + 2, 2);
  memcpy(&busPower_int, _latestData + 4, 2);
  memcpy(&busTemperature_int, _latestData + 6, 2);

  *busVoltage = static_cast<float>(busVoltage_int) / 100.0;
  *busCurrent = static_cast<float>(busCurrent_int) / 100.0;
  *busPower = static_cast<float>(busPower_int) / 1000.0;
  *busTemperature = static_cast<float>(busTemperature_int) / 100.0;

  Serial.print(">busVoltage_V: ");
  Serial.println(*busVoltage);
  Serial.print(">busCurrent_mA: ");
  Serial.println(*busCurrent);
  Serial.print(">busPower_mW: ");
  Serial.println(*busPower);
  Serial.print(">busTemperature_C: ");
  Serial.println(*busTemperature);
}

void CAN::sendValveDataPart1(int16_t motorTemperature, int16_t mcuTemperature, int16_t current, uint16_t inputVoltage)
{
  uint8_t data[8];
  memcpy(data, &motorTemperature, 2);
  memcpy(data + 2, &mcuTemperature, 2);
  memcpy(data + 4, &current, 2);
  memcpy(data + 6, &inputVoltage, 2);

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::VALVE_DATA_PART_1), 0, 8, data);
}

void CAN::receiveValveDataPart1(float *motorTemperature, float *mcuTemperature, float *current, float *inputVoltage)
{
  int16_t motorTemperatureRaw, mcuTemperatureRaw, currentRaw, inputVoltageRaw;

  memcpy(&motorTemperatureRaw, _latestData, 2);
  memcpy(&mcuTemperatureRaw, _latestData + 2, 2);
  memcpy(&currentRaw, _latestData + 4, 2);
  memcpy(&inputVoltageRaw, _latestData + 6, 2);

  *motorTemperature = (float)motorTemperatureRaw / 100.0;
  *mcuTemperature = (float)mcuTemperatureRaw / 100.0;
  *current = (float)currentRaw;
  *inputVoltage = (float)inputVoltageRaw / 1000.0;
}

void CAN::sendValveDataPart2(int16_t currentPosition, int16_t currentDesiredPosition, int16_t currentVelocity)
{
  uint8_t data[6];
  memcpy(data, &currentPosition, 2);
  memcpy(data + 2, &currentDesiredPosition, 2);
  memcpy(data + 4, &currentVelocity, 2);

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::VALVE_DATA_PART_2), 0, 6, data);
}

void CAN::receiveValveDataPart2(float *currentPosition, float *currentDesiredPosition, float *currentVelocity)
{
  int16_t currentPositionRaw, currentDesiredPositionRaw, currentVelocityRaw;

  memcpy(&currentPositionRaw, _latestData, 2);
  memcpy(&currentDesiredPositionRaw, _latestData + 2, 2);
  memcpy(&currentVelocityRaw, _latestData + 4, 2);

  *currentPosition = (float)currentPositionRaw / 100.0;
  *currentDesiredPosition = (float)currentDesiredPositionRaw / 100.0;
  *currentVelocity = (float)currentVelocityRaw / 10.0;

  Serial.print(">currentPosition_Main: ");
  Serial.println(*currentPosition);
}

void CAN::sendValveDataPart3(int16_t motorTemperature, int16_t mcuTemperature, int16_t current, int16_t inputVoltage)
{
  uint8_t data[8];
  memcpy(data, &motorTemperature, 2);
  memcpy(data + 2, &mcuTemperature, 2);
  memcpy(data + 4, &current, 2);
  memcpy(data + 6, &inputVoltage, 2);

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::VALVE_DATA_PART_3), 0, 8, data);
}

void CAN::receiveValveDataPart3(float *motorTemperature, float *mcuTemperature, float *current, float *inputVoltage)
{
  int16_t motorTemperatureRaw, mcuTemperatureRaw, currentRaw, inputVoltageRaw;

  memcpy(&motorTemperatureRaw, _latestData, 2);
  memcpy(&mcuTemperatureRaw, _latestData + 2, 2);
  memcpy(&currentRaw, _latestData + 4, 2);
  memcpy(&inputVoltageRaw, _latestData + 6, 2);

  *motorTemperature = (float)motorTemperatureRaw / 100.0;
  *mcuTemperature = (float)mcuTemperatureRaw / 100.0;
  *current = (float)currentRaw;
  *inputVoltage = (float)inputVoltageRaw / 1000.0;
}

void CAN::sendValveDataPart4(int16_t currentPosition, int16_t currentDesiredPosition, int16_t currentVelocity)
{
  uint8_t data[6];
  memcpy(data, &currentPosition, 2);
  memcpy(data + 2, &currentDesiredPosition, 2);
  memcpy(data + 4, &currentVelocity, 2);

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::VALVE_DATA_PART_4), 0, 6, data);
}

void CAN::receiveValveDataPart4(float *currentPosition, float *currentDesiredPosition, float *currentVelocity)
{
  int16_t currentPositionRaw, currentDesiredPositionRaw, currentVelocityRaw;

  memcpy(&currentPositionRaw, _latestData, 2);
  memcpy(&currentDesiredPositionRaw, _latestData + 2, 2);
  memcpy(&currentVelocityRaw, _latestData + 4, 2);

  *currentPosition = (float)currentPositionRaw / 100.0;
  *currentDesiredPosition = (float)currentDesiredPositionRaw / 100.0;
  *currentVelocity = (float)currentVelocityRaw / 10.0;

  Serial.print(">currentPosition_Supply: ");
  Serial.println(*currentPosition);
}

void CAN::sendDynamics(float force, float jerk)
{
  uint8_t data[8];
  memcpy(data + 0, &force, 4);
  memcpy(data + 4, &jerk, 4);

  _can->sendMsgBuf(static_cast<uint32_t>(Var::Label::DYNAMICS_DATA), 0, 8, data);
}

void CAN::receiveDynamics(float *force, float *jerk)
{
  memcpy(force, _latestData + 0, 4);
  memcpy(jerk, _latestData + 4, 4);
}
