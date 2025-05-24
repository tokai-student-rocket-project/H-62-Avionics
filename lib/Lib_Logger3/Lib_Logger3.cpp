#include "Lib_Logger3.hpp"


Logger::Logger(uint32_t csFram0, uint32_t csFram1, uint32_t csFram2) {
  _fram0 = new FRAM(csFram0);
  _fram1 = new FRAM(csFram1);
  _fram2 = new FRAM(csFram2);
}


void Logger::reset() {
  _offset = 0;
}


void Logger::dump() {
  _fram0->dump();
  _fram1->dump();
  _fram2->dump();
}


void Logger::clear() {
  _fram0->setWriteEnable();
  _fram0->clear();
  _fram1->setWriteEnable();
  _fram1->clear();
  _fram2->setWriteEnable();
  _fram2->clear();
}


uint32_t Logger::write(const uint8_t* data, uint32_t size) {
  if (_offset + size >= FRAM::LENGTH * 3) {
    return size;
  }
  else if (_offset + size >= FRAM::LENGTH * 2) {
    uint32_t writeAddress = _offset - FRAM::LENGTH * 2;
    _fram2->setWriteEnable();
    _fram2->write(writeAddress, data, size);
  }
  else if (_offset + size >= FRAM::LENGTH * 1) {
    uint32_t writeAddress = _offset - FRAM::LENGTH;
    _fram1->setWriteEnable();
    _fram1->write(writeAddress, data, size);
  }
  else {
    uint32_t writeAddress = _offset;
    _fram0->setWriteEnable();
    _fram0->write(writeAddress, data, size);
  }

  _offset += size;

  ////////////////////////////////////////////////////////////
  Serial.print("[");
  Serial.print(getUsage(), 0);
  Serial.print("% | ");
  Serial.print(_offset);
  Serial.print("b / ");
  Serial.print(FRAM::LENGTH * 2);
  Serial.print("b] ");

  for (uint32_t i = 0; i < size; i++) {
    Serial.print(data[i], HEX);
    Serial.print((data[i] == 0x00) ? "\n" : " ");
  }
  ////////////////////////////////////////////////////////////

  return size;
}


uint32_t Logger::getOffset() {
  return _offset;
}


float Logger::getUsage() {
  return ((float)_offset / (float)(FRAM::LENGTH * 3)) * 100.0;
}

uint8_t Logger::framNumber() {
  if (_offset >= FRAM::LENGTH * 3) {
    return -1;
  }
  else if (_offset >= FRAM::LENGTH * 2) {
    return 2;
  }
  else if (_offset >= FRAM::LENGTH * 1) {
    return 1;
  }
  else {
    return 0;
  }
}
