//
// Created by Andrew Ward on 11/21/22.
//

#ifndef MCP41XXX_H
#define MCP41XXX_H

#include <Arduino.h>

#define MCP_NOP 0b00000000
#define MCP_WRITE 0b00010001
#define MCP_SHTDWN 0b00100001



class MCP41xxx {
  public:
	// These are the pins used to connect to the chip
    //    MCP41xxx(int CS);
        MCP41xxx(int CS, int CLK, int MOSI);
	// Sets the given pot (1 or 2) to a given value (0-255)
    void setPot(byte value);
    void setPot(byte pot, byte value);
  private:
    void transferSPI(byte data);
    int _CS, _CLK, _MOSI;
};

#endif // MCP41XXX_H
