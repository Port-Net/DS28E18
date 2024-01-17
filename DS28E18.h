#ifndef DS28E18_h
#define DS28E18_h

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// set to true to include code for new and delete operators
#ifndef REQUIRESNEW
#define REQUIRESNEW false
#endif

#include <inttypes.h>
#ifdef __STM32F1__
#include <OneWireSTM.h>
#else
#include <OneWire.h>
#endif

#include <vector>

// Model IDs
#define DS28E18MODEL 0x56  
#define DS28E18EMPTYCRC 0xB2  
#define DS28E18_POR_BITMASK 0x02

#define DS28E18CommandCode            0x66

#define DS28E18WriteSequencer         0x11
#define DS28E18ReadSequencer          0x22
#define DS28E18RunSequencer           0x33
#define DS28E18WriteConfiguration     0x55
#define DS28E18ReadConfiguration      0x6A
#define DS28E18WriteGPIOConfiguration 0x83
#define DS28E18ReadGPIOConfiguration  0x7C
#define DS28E18DeviceStatus           0x7A

// For readPowerSupply on oneWire bus
// definition of nullptr for C++ < 11, using official workaround:
// http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2431.pdf
#if __cplusplus < 201103L
const class
{
public:
	template <class T>
	operator T *() const {
		return 0;
	}

	template <class C, class T>
	operator T C::*() const {
		return 0;
	}

private:
	void operator&() const;
} nullptr = {};
#endif

typedef uint8_t DeviceAddress[8];


class DS28E18Device {
public:
  enum error_t {
    SUCCESS = 0,
    CMD_ERROR = 0x100,
    NO_RESULT_ERROR = 0x200,
    RESPONSE_ERROR = 0x300,
    NACK_ERROR = 0x400,
    POR_ERROR = 0x500
  };
  DS28E18Device(OneWire* wire, DeviceAddress deviceAddress);
  void firstInit(void);
  DeviceAddress* getAddress(void);
  bool hasAddress(DeviceAddress deviceAddress);
  uint16_t getStatus(void);
  bool load_sequencer(uint8_t* sequence, uint16_t sequenceStart, uint16_t sequenceLen);
  bool run_sequencer(uint16_t sequenceStart, uint16_t sequenceLen, uint8_t waitTime);
  bool read_sequencer(uint16_t start, uint16_t len, uint8_t* result);
  uint16_t getNackAddr(void);
  uint8_t getError(void);
  uint8_t getResultByte(void);
  int16_t getExecutionTime(uint8_t* sequence, uint16_t sequenceLen);

private:
  bool write_cmd(uint8_t* cmd, uint8_t cmdLen, uint8_t* result = NULL, uint8_t* resultLen = NULL, uint8_t waitTime = 1);
  bool write_cmd_(uint8_t* cmd, uint8_t cmdLen, uint8_t* result = NULL, uint8_t* resultLen = NULL, uint8_t waitTime = 1);
  
  OneWire* _wire;
  DeviceAddress _deviceAddress;
  bool _sequenceLoaded;
  uint16_t _nackAddr;
  uint8_t _lastError;
  uint8_t _lastResultByte;
  uint8_t _spd;
};

class DS28E18 {
public:
	DS28E18();
	DS28E18(OneWire*);
	DS28E18(OneWire*, uint8_t pullupPin, bool negatePullup = false);

	void setPullupPin(uint8_t pullupPin, bool negatePullup = false);
	void setOneWire(OneWire*);

	// initialise bus
	bool begin(void);
  uint8_t getDeviceCount(void);
  uint8_t getStatus(DeviceAddress addr);
  uint8_t getStatus(uint8_t index);
  bool getAddress(uint8_t* deviceAddress, uint8_t index);
  bool load_sequencer(DeviceAddress deviceAddress, uint8_t* sequence, uint16_t sequenceStart, uint16_t sequenceLen);
  bool load_sequencer(uint8_t index, uint8_t* sequence, uint16_t sequenceStart, uint16_t sequenceLen);
  bool run_sequencer(DeviceAddress deviceAddress, uint16_t sequenceStart, uint16_t sequenceLen, uint8_t waitTime);
  bool run_sequencer(uint8_t index, uint16_t sequenceStart, uint16_t sequenceLen, uint8_t waitTime);
  bool read_sequencer(DeviceAddress deviceAddress, uint16_t start, uint16_t len, uint8_t* result);
  bool read_sequencer(uint8_t index, uint16_t start, uint16_t len, uint8_t* result);
  bool load_MPR_sequencer(DeviceAddress deviceAddress);
  bool load_MPR_sequencer(uint8_t index);
  bool run_MPR_sequencer(DeviceAddress deviceAddress);
  bool run_MPR_sequencer(uint8_t index);
  bool read_MPR_result(DeviceAddress deviceAddress, uint8_t &status, uint32_t &value);
  bool read_MPR_result(uint8_t index, uint8_t &status, uint32_t &value);

  	// returns true if address is valid
	bool validAddress(const uint8_t*);

private:
	// External pullup control
  int getIdByAddress(DeviceAddress deviceAddress);
  DS28E18Device* getDevByAddress(DeviceAddress deviceAddress);
	void activateExternalPullup(void);
	void deactivateExternalPullup(void);

  //bool write_cmd(uint8_t* cmd, uint8_t cmdLen, uint8_t* result = NULL, uint8_t* resultLen = NULL, uint8_t waitTime = 1);

	// parasite power on or off
	bool _parasite;

	// external pullup
	bool _useExternalPullup;
	uint8_t _pullupPin;
  bool _negatePullup;

	// Take a pointer to one wire instance
	OneWire* _wire;
  uint8_t _initTries;
  std::vector<DS28E18Device*> _devices;
};

#endif