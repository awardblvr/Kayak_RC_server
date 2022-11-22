//
// Created by Andrew Ward on 11/21/22.
//

#include "MCP41xxx.h"

/**
 * for using the MCP41xxx digital potentiometer with SPI.
 */


/*
 * Define the MCP41100 OP command bits (only one POT)
 * Note: command byte format xxCCxxPP, CC command, PP pot number (01 if selected)
 *              xxCCxxPP
 * MCP_NOP    0b00000000
 * MCP_WRITE  0b00010001
 * MCP_SHTDWN 0b00100001
 *
 * The MCP41xxx is SPI-compatible. The hardware is managed by
 *  sending two bytes:
 *  BYTE 1 - is a command byte (NOP, WRITE, SHUTDOWN)
 *  BYTE 2 - is the data byte (new setpoint value 0-255).
 *
 * C1 C0  Command   Command Summary                              P1* P0  Potentiometer Selections
 *  0  0  None      No Command will be executed.                 0   0  Dummy Code: Neither Potentiometer affected.
 *  0  1  Write     Data Write the data contained in Data        0   1  Command executed on Potentiometer 0.
 *                  Byte to the potentiometer(s) determined      1   0  Command executed on Potentiometer 1.
 *                  by the potentiometer selection bits.         1   1  Command executed on both Potentiometers.
 *  1  0  Shutdown  Potentiometer(s) determined by
 *                  potentiometer selection bits will enter     * P1 is Don't Care for MCP41xxx
 *                  Shutdown Mode. Data bits for this
 *                  command are ‘don’t cares’.
 *  1  1  None      No Command will be executed.
 *
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

void MCP41xxx::setPot(byte value) {
  digitalWrite(_CS, LOW);
  //transferSPI((pot & 0b11) | 0b00010000);
  transferSPI(MCP_WRITE);
  transferSPI(value);
  digitalWrite(_CS, HIGH);
}

void MCP41xxx::setPot(byte pot, byte value) {
  digitalWrite(_CS, LOW);
  //transferSPI((pot & 0b11) | 0b00010000);
  transferSPI((pot & 0b11) | MCP_WRITE);
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
