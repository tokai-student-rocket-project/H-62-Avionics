#include "Lib_Telemeter.hpp"


void Telemeter::initialize(int32_t frequency, int32_t bandwidth) {
  LoRa.begin(frequency);
  LoRa.setSignalBandwidth(bandwidth);
}


void Telemeter::reserveData(const uint8_t* data, uint32_t size) {
  if (reservedSize + size > 4096) {
    return;
  }

  memcpy(reservedData + reservedSize, data, size);
  reservedSize += size;
}


void Telemeter::sendReservedData() {
  LoRa.beginPacket();
  LoRa.write(reservedData, reservedSize);
  LoRa.endPacket(true);

  reservedSize = 0;
}
