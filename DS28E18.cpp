// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include "DS28E18.h"

// for Particle support
// yield() is not a standard function, but instead wraps Particle process
// https://community.particle.io/t/syscall-yield-operation/40708/2
#if defined(PLATFORM_ID)  // Only defined if a Particle device
inline void yield() {
	Particle.process();
}
#elif ARDUINO >= 100
#include "Arduino.h"
#else
extern "C" {
#include "WConstants.h"
}
#endif

#define IDXCHECK if(index > (_devices.size() - 1)) return false;

// OneWire commands
DS28E18Device::DS28E18Device(OneWire* wire, DeviceAddress deviceAddress) {
  _wire = wire;
  bcopy(deviceAddress, _deviceAddress, sizeof(DeviceAddress));
  _sequenceLoaded = false;
  _lastResultByte = 0;
  _lastError = 0;
  _spd = 0x01;
}

void DS28E18Device::firstInit() {
  uint8_t cmd[] = {DS28E18WriteGPIOConfiguration,0x0b,0x03,0xa5,0x0f};
  _wire->reset();
  _wire->skip();
  uint8_t result[2];
  uint8_t result_len = 2;
  write_cmd_(cmd, sizeof(cmd), result, &result_len);
  Serial.printf("Init Resp: %d %02x\r\n", result_len, result[0]);
}

DeviceAddress* DS28E18Device::getAddress() {
  return &_deviceAddress;
}

bool DS28E18Device::hasAddress(DeviceAddress deviceAddress) {
  return !bcmp(_deviceAddress, deviceAddress, sizeof(DeviceAddress));
}

uint16_t DS28E18Device::getStatus() {
  uint8_t cmd[] = {DS28E18DeviceStatus};
  uint8_t result[4];
  uint8_t result_len = 4;
  if(!write_cmd(cmd, 1, result, &result_len)) {
    _lastError = CMD_ERROR >> 8;
    return CMD_ERROR;
  }
  Serial.printf("Status Resp: %02x %02x %02x %02x\r\n", result[0], result[1], result[2], result[3]);
  if((result_len == 0xFF) || (result_len < 2)) {
    _lastError = NO_RESULT_ERROR >> 8;
    return NO_RESULT_ERROR;
  }
  _lastResultByte = result[0];
  if(result[0] != 0xAA) {
    _lastError = RESPONSE_ERROR >> 8;
    return RESPONSE_ERROR | result[0];
  }
  if(result[1] & DS28E18_POR_BITMASK) {
    _sequenceLoaded = false;
  }
  _lastError = SUCCESS;
  return result[1];
}

bool DS28E18Device::load_sequencer(uint8_t* sequence, uint16_t sequenceStart, uint16_t sequenceLen) {
  uint8_t sequencer_cmd[3 + sequenceLen];
  sequencer_cmd[0] = DS28E18WriteSequencer;
  sequencer_cmd[1] = sequenceStart & 0xFF;
  sequencer_cmd[2] = (sequenceStart >> 8) & 0x01;
  for(int i = 0; i < sequenceLen; ++i) {
    sequencer_cmd[3 + i] = sequence[i];
  }
  uint8_t result;
  uint8_t result_len = 1;
  if(!write_cmd(sequencer_cmd, sizeof(sequencer_cmd), &result, &result_len)) {
    Serial.println("seq load failed");
    _lastError = CMD_ERROR >> 8;
    return false;
  }
  if(result_len != 1) {
    Serial.println("seq load no result");
    _lastError = NO_RESULT_ERROR >> 8;
    return false;
  }
  _lastResultByte = result;
  if(result != 0xAA) {
    Serial.printf("seq failed load Resp: %02x\r\n", result);
    _lastError = RESPONSE_ERROR >> 8;
    return false;
  }
  Serial.println("seq loaded");
  _sequenceLoaded = true;
  _lastError = SUCCESS;
  return true;
}

bool DS28E18Device::run_sequencer(uint16_t sequenceStart, uint16_t sequenceLen, uint32_t waitTime) {
  if(!_sequenceLoaded) {
    Serial.println("Sequence not loaded");
    return false;
  }
  uint8_t sequencer_cmd[4];
  sequencer_cmd[0] = DS28E18RunSequencer;
  sequencer_cmd[1] = sequenceStart & 0xFF;
  sequencer_cmd[2] = (sequenceStart >> 8) & 0x01;
  sequencer_cmd[2] |= (sequenceLen << 1) & 0xFE;
  sequencer_cmd[3] = (sequenceLen >> 7) & 0x03;
  uint8_t result[3];
  uint8_t result_len = sizeof(result);
  if(!write_cmd(sequencer_cmd, sizeof(sequencer_cmd), result, &result_len, waitTime)) {
    Serial.println("seq run failed");
    _lastError = CMD_ERROR >> 8;
    return false;
  }
  if((result_len != 1) && (result_len != 3)) {
    Serial.println("seq run no result");
    _lastError = NO_RESULT_ERROR >> 8;
    return false;
  }
  _lastResultByte = result[0];
  if((result_len == 3) && (result[0] == 0x88)) {
    Serial.printf("seq run NACK result %02x\r\n", result[2], result[1]);
    _lastError = NACK_ERROR >> 8;
    _nackAddr = (uint16_t)result[1] << 8 | result[2];
    if(_nackAddr == 0) {
      _nackAddr = 512;
    }
    return false;
  }
  if((result_len == 1) && (result[0] == 0x44)) { // POR during sequence
    Serial.printf("seq POR\r\n");
    _lastError = POR_ERROR >> 8;
    _sequenceLoaded = false;
    return false;
  }
  if((result_len == 1) && (result[0] != 0xAA)) {
    Serial.printf("seq run bad result %02x\r\n", result[0]);
    _lastError = RESPONSE_ERROR >> 8;
    return false;
  }
  _lastError = SUCCESS;
  return true;
}

bool DS28E18Device::read_sequencer(uint16_t start, uint16_t len, uint8_t* result) {
  uint8_t sequencer_cmd[3];
  sequencer_cmd[0] = DS28E18ReadSequencer;
  sequencer_cmd[1] = start & 0xFF;
  sequencer_cmd[2] = (start >> 8) & 0x01;
  sequencer_cmd[2] |= (len << 1) & 0xFE;
  uint8_t rec[len + 1];
  uint8_t rec_len = sizeof(rec);
  if(!write_cmd(sequencer_cmd, sizeof(sequencer_cmd), rec, &rec_len)) {
    Serial.println("seq read failed");
    return false;
  }
  if((rec_len == 0xFF) || (rec_len == 0x00)) {
    Serial.println("seq read no result");
    return false;
  }
  if((rec_len >= 1) && (rec[0] != 0xAA)) {
    Serial.printf("seq read bad result %02x\r\n", rec[0]);
    return false;
  }
  for(uint8_t i = 0; i < min((int)len, (int)rec_len - 1); ++i) {
    result[i] = rec[i + 1];
  }
  return true;
}

uint16_t DS28E18Device::getNackAddr() {
  return _nackAddr;
}

uint8_t DS28E18Device::getError() {
  return _lastError;
}

uint8_t DS28E18Device::getResultByte() {
  return _lastResultByte;
}

bool DS28E18Device::write_cmd(uint8_t* cmd, uint8_t cmdLen, uint8_t* result, uint8_t* resultLen, uint32_t waitTime) {
  _wire->reset();
  _wire->select(_deviceAddress);
  return write_cmd_(cmd, cmdLen, result, resultLen, waitTime);
}

bool DS28E18Device::write_cmd_(uint8_t* cmd, uint8_t cmdLen, uint8_t* result, uint8_t* resultLen, uint32_t waitTime) {
  uint8_t cmd_buf[cmdLen + 2];
  cmd_buf[0] = DS28E18CommandCode;
  cmd_buf[1] = cmdLen;
  bcopy(cmd, &cmd_buf[2], cmdLen);
  //for(uint8_t i = 0; i < cmdLen; ++i) {
  //  cmd_buf[i + 2] = cmd[i];
  //}
  uint16_t crc = _wire->crc16(cmd_buf, sizeof(cmd_buf));
  _wire->write_bytes(cmd_buf, sizeof(cmd_buf));
  uint16_t neg_crc;
  uint8_t b[2];
  _wire->read_bytes(b, 2);
  neg_crc = (uint16_t)b[1] << 8 | b[0];
  if((crc ^ neg_crc) != 0xFFFF) { 
    Serial.printf("CRC: %04x != %04x\r\n", crc, neg_crc);
    return false;
  //} else {
  //  Serial.println("crc match");
  }
  _wire->write(0xAA);
  delayMicroseconds(max(1000U, waitTime));
  _wire->read(); // dummy read
  uint8_t rec_len = _wire->read(); // length
  if(rec_len == 0xFF) {
    Serial.println("cmd no Result");
    return false;
  }
  uint8_t rec[1 + rec_len]; // len + result
  rec[0] = rec_len;
  _wire->read_bytes(&rec[1], rec_len);
  _wire->read_bytes(b, 2);
  neg_crc = (uint16_t)b[1] << 8 | b[0];
  crc = _wire->crc16(rec, sizeof(rec));
  if((crc ^ neg_crc) != 0xFFFF) { 
    Serial.printf("CRC2: %04x != %04x\r\n", crc, neg_crc);
    return false;
  //} else {
  //  Serial.println("crc2 match");
  }
  int max_result_size = 0;
  if(resultLen != NULL) {
    max_result_size = *resultLen;
    *resultLen = rec_len;
  }
  if(result != NULL) {
    for(uint8_t i = 0; i < min((int)rec_len, max_result_size); ++i) {
      result[i] = rec[i + 1];
    }
  }
  return true;
}

int16_t DS28E18Device::getExecutionTime(uint8_t* sequence, uint16_t sequenceLen) {
  // time table as on page 44 of datasheet
  uint8_t time_by_spd[] = {33,12,8,0, // 02, 03
                          136,45,25,0, //E3, D3,D4
                          123,42,25,17, // C0
                          35,15,10,8}; // 01, 80
  uint16_t i = 0;
  uint16_t time_us = 0;
  while(i < sequenceLen) {
    switch(sequence[i]) {
      // i2c commands
      case 0x02:
      case 0x03:
        if(_spd > 2) {
          Serial.println("Sequence calc wrong spd");
          return -3;
        }
        time_us += time_by_spd[0 * 4 + _spd];
        break;
      case 0xE3:
      case 0xD3:
      case 0xD4:
        if(_spd > 2) {
          Serial.println("Sequence calc wrong spd");
          return -3;
        }
        i++;
        if((i + sequence[i]) >= sequenceLen) {
          Serial.println("Sequence to short");
          return -1;
        }
        time_us += sequence[i] * time_by_spd[1 * 4 + _spd];
        i += sequence[i];
        break;
      // spi commands
      case 0xC0:
        {
        if(_spd > 3) {
          Serial.println("Sequence calc wrong spd");
          return -3;
        }
        if((i + 2) >= sequenceLen) {
          Serial.println("Sequence to short");
          return -1;
        }
        i++;
        uint8_t rl = sequence[i];
        i++;
        uint8_t wl = sequence[i];
        if((i + rl + wl) >= sequenceLen) {
          Serial.println("Sequence to short");
          return -1;
        }
        time_us += (rl + wl) * time_by_spd[2 * 4 + _spd];
        i += (rl + wl);
        }
        break;
      case 0xB0:
        {
        if((i + 2) >= sequenceLen) {
          Serial.println("Sequence to short");
          return -1;
        }
        i++;
        uint8_t rl = sequence[i];
        i++;
        uint8_t wl = sequence[i];
        time_us += (wl + rl) * 26;
        if((i + rl + wl) >= sequenceLen) {
          Serial.println("Sequence to short");
          return -1;
        }
        if(wl > 0){
          i += (wl - 1) / 8 + 1;
        }
        if(rl > 0){
          i += (rl - 1) / 8 + 1;
        }
        }
        break;
      case 0x01:
      case 0x80:
        if(_spd > 2) {
          Serial.println("Sequence calc wrong spd");
          return -3;
        }
        time_us += sequence[i] * time_by_spd[1 * 4 + _spd];
        break;
      case 0xDD:
        i++;
        if((i + sequence[i]) >= sequenceLen) {
          Serial.println("Sequence to short");
          return -1;
        }
        time_us += pow(2, sequence[i]) * 1248;
        break;
      case 0xCC:
      case 0xBB:
        time_us += 6;
        break;
      case 0xD1:
      case 0x1D:
        time_us += 8;
        i++;
        break;
      case 0xE2:
      case 0x2E:
        time_us += 10;
        i += 2;
        break;
      default:
        Serial.println("Sequence cmd unknown");
        return -2;
        break;
    }
    i++;
  }
  return time_us;
}



DS28E18::DS28E18() {
	_useExternalPullup = false;
  _initTries = 0;
}

DS28E18::DS28E18(OneWire* oneWire) : DS28E18() {
	setOneWire(oneWire);
}

/*
 * Constructs DS28E18 with strong pull-up turned on. Strong pull-up is mandated in datasheet for parasitic
 * power (2 wires) setup. 
 */
DS28E18::DS28E18(OneWire* oneWire, uint8_t pullupPin, bool negatePullup) : DS28E18(oneWire) {
	setPullupPin(pullupPin, negatePullup);
}

void DS28E18::setPullupPin(uint8_t pullupPin, bool negatePullup) {
	_useExternalPullup = true;
	_pullupPin = pullupPin;
  _negatePullup = negatePullup;
	pinMode(pullupPin, OUTPUT);
	deactivateExternalPullup();
}

void DS28E18::setOneWire(OneWire* oneWire) {
	_wire = oneWire;
	_devices.clear();
}

// initialise the bus
bool DS28E18::begin(void) {
	DeviceAddress deviceAddress;

	_wire->reset_search();
	_devices.clear(); // Reset the number of devices when we enumerate wire devices
  bool init_need = false;
	while (_wire->search(deviceAddress)) {
		if (validAddress(deviceAddress)) {
			if (deviceAddress[0] == DS28E18MODEL) {
        _devices.push_back(new DS28E18Device(_wire, deviceAddress));
        if(deviceAddress[7] == DS28E18EMPTYCRC) {
          init_need = true;
        }
      }
		}
	}
  if(init_need) {
    if(_initTries++ < 3) {
      Serial.println("First init");
      _devices.at(0)->firstInit();
      return begin(); // we recurse max tree times to init
    } else {
      return false;
    }
  }
  if(!_devices.size()) {
    return false;
  }
  // we have to query status to clear possible POR
  for(int i = 0; i < _devices.size(); ++i) {
    uint8_t s = _devices.at(i)->getStatus();
    Serial.printf("status %d: %02x\r\n", i, s);
  }
  return true;
}

// returns the number of devices found on the bus
uint8_t DS28E18::getDeviceCount(void) {
	return _devices.size();
}

uint8_t DS28E18::getStatus(DeviceAddress deviceAddress) {
  for(auto it : _devices) {
    if(it->hasAddress(deviceAddress)) {
      return it->getStatus();
    }
  }
  return 0xFF;
}

uint8_t DS28E18::getStatus(uint8_t index) {
  IDXCHECK
  return _devices.at(index)->getStatus();
}

bool DS28E18::getAddress(uint8_t* deviceAddress, uint8_t index) {
  IDXCHECK
  bcopy(_devices.at(index)->getAddress(), deviceAddress, sizeof(DeviceAddress));
  return true;
}

int DS28E18::getIdByAddress(DeviceAddress deviceAddress) {
  int i = 0;
  for(auto it : _devices) {
    if(it->hasAddress(deviceAddress)) {
      return i;
    }
    i++;
  }
  return -1;
}

DS28E18Device* DS28E18::getDevByAddress(DeviceAddress deviceAddress) {
  for(auto it : _devices) {
    if(it->hasAddress(deviceAddress)) {
      return it;
    }
  }
  return nullptr;
}

bool DS28E18::load_sequencer(DeviceAddress deviceAddress, uint8_t* sequence, uint16_t sequenceStart, uint16_t sequenceLen) {
  if(DS28E18Device* d = getDevByAddress(deviceAddress)) {
    return d->load_sequencer(sequence, sequenceStart, sequenceLen);
  }
  return false;
}

bool DS28E18::load_sequencer(uint8_t index, uint8_t* sequence, uint16_t sequenceStart, uint16_t sequenceLen) {
  IDXCHECK
  return _devices.at(index)->load_sequencer(sequence, sequenceStart, sequenceLen);
}

bool DS28E18::run_sequencer(DeviceAddress deviceAddress, uint16_t sequenceStart, uint16_t sequenceLen, uint32_t waitTime) {
  if(DS28E18Device* d = getDevByAddress(deviceAddress)) {
    return d->run_sequencer(sequenceStart, sequenceLen, waitTime);
  }
  return false;
}

bool DS28E18::run_sequencer(uint8_t index, uint16_t sequenceStart, uint16_t sequenceLen, uint32_t waitTime) {
  IDXCHECK
  return _devices.at(index)->run_sequencer(sequenceStart, sequenceLen, waitTime);
}

bool DS28E18::read_sequencer(DeviceAddress deviceAddress, uint16_t start, uint16_t len, uint8_t* result) {
  if(DS28E18Device* d = getDevByAddress(deviceAddress)) {
    return d->read_sequencer(start, len, result);
  }
  return false;
}

bool DS28E18::read_sequencer(uint8_t index, uint16_t start, uint16_t len, uint8_t* result) {
  IDXCHECK
  return _devices.at(index)->read_sequencer(start, len, result);

}

bool DS28E18::MPR_sensor_init(DeviceAddress deviceAddress) {
  uint8_t sequence[] = {  0x02, // start
                          0xE3, 0x04, 0x30, 0xaa, 0x00, 0x00, // write addr(18w) AA 00 00
                          0x03, // stop
                          0xDD, 0x03, // delay(8)
                          0x02, // start
                          0xE3, 0x01, 0x31, // write addr(18r)
                          0xD3, 0x04, 0xFF, 0xFF, 0xFF, 0xFF,  // read 4 byte (addr is 16)
                          0x03 }; // stop
  Serial.printf("execution time: %d\r\n", getDevByAddress(deviceAddress)->getExecutionTime(sequence, sizeof(sequence)));
  return load_sequencer(deviceAddress, sequence, 0, sizeof(sequence));
}

bool DS28E18::MPR_sensor_init(uint8_t index) {
  IDXCHECK
  return MPR_sensor_init(*(_devices.at(index)->getAddress()));
}

bool DS28E18::MPR_sensor_measure(DeviceAddress deviceAddress) {
  activateExternalPullup();
  bool ret = run_sequencer(deviceAddress, 0 ,8 , 220); // calculated 204 us
  return ret;
}

bool DS28E18::MPR_sensor_measure(uint8_t index) {
  IDXCHECK
  return MPR_sensor_measure(*(_devices.at(index)->getAddress()));
}

bool DS28E18::MPR_sensor_read_result(DeviceAddress deviceAddress) {
  bool ret = run_sequencer(deviceAddress, 10 ,11 , 260); // calculated 249 us
  deactivateExternalPullup();
  return ret;
}

bool DS28E18::MPR_sensor_read_result(uint8_t index) {
  IDXCHECK
  return MPR_sensor_read_result(*(_devices.at(index)->getAddress()));
}

bool DS28E18::MPR_sensor_get_result(DeviceAddress deviceAddress, uint8_t &status, uint32_t &value) {
  uint8_t result[4];
  if(!read_sequencer(deviceAddress, 16, 4, result)) {
    return false;
  }
  status = result[0];
  //value = *((uint32_t*)result) & 0x00FFFFFF;
  value = (uint32_t)result[1] << 16 | (uint32_t)result[2] << 8 | (uint32_t)result[3];
  return true;
}

bool DS28E18::MPR_sensor_get_result(uint8_t index, uint8_t &status, uint32_t &value) {
  IDXCHECK
  return MPR_sensor_get_result(*(_devices.at(index)->getAddress()), status, value);
}

bool DS28E18::MPR_sensor_measure_result(DeviceAddress deviceAddress, uint8_t &status, uint32_t &value) {
  if(!run_sequencer(deviceAddress, 0 ,21, 10000)) {
    return false;
  }
  return MPR_sensor_get_result(deviceAddress, status, value);
}

bool DS28E18::MPR_sensor_measure_result(uint8_t index, uint8_t &status, uint32_t &value) {
  IDXCHECK
  return MPR_sensor_measure_result(*(_devices.at(index)->getAddress()), status, value);
}

void DS28E18::activateExternalPullup() {
	if (_useExternalPullup)
		digitalWrite(_pullupPin, _negatePullup ? HIGH : LOW);
}

void DS28E18::deactivateExternalPullup() {
	if (_useExternalPullup)
		digitalWrite(_pullupPin, _negatePullup ? LOW : HIGH);
}

// returns true if address is valid
bool DS28E18::validAddress(const uint8_t* deviceAddress) {
	return (_wire->crc8((uint8_t*)deviceAddress, 7) == deviceAddress[7]);
}

