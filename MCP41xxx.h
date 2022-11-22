//
// Created by Andrew Ward on 11/21/22.
//

#ifndef MCP41XXX_H
#define MCP41XXX_H

#include <Arduino.h>

class MCP41xxx {
  public:
	// These are the pins used to connect to the chip
    MCP41xxx(int CS, int CLK, int MOSI);
	// Sets the given pot (1 or 2) to a given value (0-255)
    void setPot(byte pot, byte value);
  private:
    void transferSPI(byte data);
    int _CS, _CLK, _MOSI;
};

#endif // MCP41XXX_H
