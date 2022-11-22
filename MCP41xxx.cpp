//
// Created by Andrew Ward on 11/21/22.
//

#include "MCP41xxx.h"

/**
 * for using the MCP41xxx digital potentiometer with SPI.
 */

MCP41xxx::MCP41xxx(int CS, int CLK, int MOSI) {
  _CS = CS;
  _CLK = CLK;
  _MOSI = MOSI;

  pinMode(_CS, OUTPUT);
  pinMode(_CLK, OUTPUT);
  pinMode(_MOSI, OUTPUT);
  digitalWrite(_CLK, LOW);
}

void MCP41xxx::setPot(byte pot, byte value) {
  digitalWrite(_CS, LOW);
  transferSPI((pot & B11) | B00010000);
  transferSPI(value);
  digitalWrite(_CS, HIGH);
}

void MCP41xxx::transferSPI(byte data) {
  for (byte i=1; i<=8; i++) {
    digitalWrite(_MOSI, (data >> (8-i)) & 1 ? HIGH : LOW);
    digitalWrite(_CLK, HIGH);
    digitalWrite(_CLK, LOW);
  }
}
