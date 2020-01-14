/**
 *  _   _        _                            _
 * | | | |      |_|                          | |
 * | |_| |_____  _ _____           _     _ __| |
 * | |_  | ___ || |  _  \  _____  \ \  / // _  |
 * | | | | ____|| | |_| | (_____)  \ \/ /( (_| |
 * |_| |_|_____)|_|___  |           \__/  \____|
 *                  __| | Haute Ecole d'Ingenieurs
 *                 |___/  et de Gestion - Vaud
 *
 * @title    Logiciel pour carte servomoteur
 * @context  Coupe suisse de robotique 2007
 * @author   Patrick Gerber <patrick.gerber@heig-vd.ch>
 * @file     main.c
 * @language ASCII/C
 * @svn      $Id: main.c 162 2007-03-26 17:04:15Z ychevall@heig-vd.ch $
 *
 */

/**
 * Includes files
 */
#ifndef __C300__
#define __C300__
#include "types.h"
#include <C8051F300.H>

#endif
#include "i2c.h"
#include "scso_cmd_list.h"

/**
 * Constants
 */
#define I2C_ADDR 0x11

/**
 * Hardware Specifications
 */
sbit LED = P0 ^ 6;
sbit TEST_POINT = P0 ^ 5;
sbit SERVO1 = P0 ^ 4;
sbit SERVO2 = P0 ^ 2;
sbit SERVO3 = P0 ^ 3;

/**
 * SFR's
 */
sfr16 PCA0 = 0xF9;
sfr16 PCA0CP0 = 0xFB;
sfr16 PCA0CP1 = 0xE9;
sfr16 PCA0CP2 = 0xEB;

/**
 * Constants
 */
extern code char sine[256];

/**
 * Mermory  Map
 */
typedef struct {
  uint8 broadcast : 8;

  uint8 servo0_mode : 2; //!< 0x0 PosA, 0x1 PosB, 0x2 PosC, 0x3 Oscillation
  uint8 servo1_mode : 2; //!< 0x0 PosA, 0x1 PosB, 0x2 PosC, 0x3 Oscillation
  uint8 servo2_mode : 2; //!< 0x0 PosA, 0x1 PosB, 0x2 PosC, 0x3 Oscillation
  uint8 leds_mode : 2;   //!< 0x1 Led is free, 0x0 Led is in internal mode

  uint8 servo0_power : 2; //!< 0x0 Off 0x1 On
  uint8 servo1_power : 2; //!< 0x0 Off 0x1 On
  uint8 servo2_power : 2; //!< 0x0 Off 0x1 On
  uint8 na0 : 2;

  uint8 servo0_a : 8;     //!< Servo 0: Pre programmed position A [-100..100 %]
  uint8 servo0_b : 8;     //!< Servo 0: Pre programmed position B [-100..100 %]
  uint8 servo0_c : 8;     //!< Servo 0: Pre programmed position C [-100..100 %]
  uint8 servo0_speed : 8; //!< Servo 0: Speed [0..100 %]
  uint8 servo0_tosc : 8;  //!< Servo 0: Oscillation period [0..100 dsec]

  uint8 servo1_a : 8;     //!< Servo 0: Pre programmed position A [-100..100 %]
  uint8 servo1_b : 8;     //!< Servo 0: Pre programmed position B [-100..100 %]
  uint8 servo1_c : 8;     //!< Servo 0: Pre programmed position C [-100..100 %]
  uint8 servo1_speed : 8; //!< Servo 0: Speed [0..100 %]
  uint8 servo1_tosc : 8;  //!< Servo 0: Oscillation period [0..100 dsec]

  uint8 servo2_a : 8;     //!< Servo 0: Pre programmed position A [-100..100 %]
  uint8 servo2_b : 8;     //!< Servo 0: Pre programmed position B [-100..100 %]
  uint8 servo2_c : 8;     //!< Servo 0: Pre programmed position C [-100..100 %]
  uint8 servo2_speed : 8; //!< Servo 0: Speed [0..100 %]
  uint8 servo2_tosc : 8;  //!< Servo 0: Oscillation period [0..100 dsec]
} memory_map;

/**
 * Global variables
 */
memory_map g;

void init_memory_map() {
  g.servo0_mode = 0;
  g.servo1_mode = 0;
  g.servo2_mode = 0;
  g.leds_mode = 0;

  g.servo0_power = 0;
  g.servo1_power = 0;
  g.servo2_power = 0;

  g.servo0_a = 0;
  g.servo0_b = 0;
  g.servo0_c = 0;
  g.servo0_speed = 0;
  g.servo0_tosc = 0;

  g.servo1_a = 0;
  g.servo1_b = 0;
  g.servo1_c = 0;
  g.servo1_speed = 0;
  g.servo1_tosc = 0;

  g.servo2_a = 0;
  g.servo2_b = 0;
  g.servo2_c = 0;
  g.servo2_speed = 0;
  g.servo2_tosc = 0;
}

/**
 * Hardware Initialisations
 */
void PCA_Init() {
  PCA0MD &= ~0x40;
  PCA0MD = 0x03;
  PCA0CN = 0x40; // Active le module PCA

  PCA0CPM0 = 0xC2; // P1.0 Mode PWM 16bit
  PCA0CPM1 = 0xC2;
  PCA0CPM2 = 0xC2;
}

void Timer_Init() {
  TCON = 0x10;
  TMOD = 0x01;
  TL0 = 0x19;
  TH0 = 0xE0;
}

void Oscillator_Init() { OSCICN = 0x06; }

void Interrupts_Init() {
  EIE1 = 0x08;
  IE = 0x82;
}

void Port_IO_Init() {
  P0MDOUT = 0x1C;
  XBR1 = 0xC4;
  XBR2 = 0x40;
}

void init_device(void) {

  PCA_Init();
  Port_IO_Init();
  Oscillator_Init();
  Timer_Init();
  Interrupts_Init();
}

/**
 * Set Duty Cycle of Servo 0
 */
void set_dt0(char dt) {
  PCA0CP0 = 0xFFFF - 4593 - 20 * dt;
  PCA0CPM0 = 0xC2;
}

/**
 * Set Duty Cycle of Servo 1
 */
void set_dt1(char dt) {
  PCA0CP1 = 0xFFFF - 4593 - 20 * dt;
  PCA0CPM1 = 0xC2;
}

/**
 * Set Duty Cycle of Servo 2
 */
void set_dt2(char dt) {
  PCA0CP2 = 0xFFFF - 4593 - 20 * dt;
  PCA0CPM2 = 0xC2;
}

/**
 * Servo 0 Power OFF
 */
void servo0_poweroff() {
  PCA0CP0 = 0xFFFF;
  PCA0CPM0 = 0x82;
}

/**
 * Servo 1 Power OFF
 */
void servo1_poweroff() {
  PCA0CP1 = 0xFFFF;
  PCA0CPM1 = 0x82;
}

/**
 * Servo 2 Power OFF
 */
void servo2_poweroff() {
  PCA0CP2 = 0xFFFF;
  PCA0CPM2 = 0x82;
}

/**
 * PCA ISR
 */
void pca_isr() interrupt 9 {
  if (g.leds_mode == 0)
    LED = ~LED;
  PCA0 = 0x10BD; // Periode 20ms
  CF = 0;
}

/**
 * Timer 0 ISR.
 */
void timer0_isr() interrupt 1 {
  static unsigned char i0 = 0, i1 = 0, i2 = 0;
  static unsigned char j0 = 0, j1 = 0, j2 = 0;

  if (g.servo0_power == 1 && g.servo0_mode == 0x3 && i0++ == g.servo0_tosc) {
    i0 = 0;
    j0++;
    if (g.servo0_b >= g.servo0_a)
      set_dt0(g.servo0_a + (g.servo0_b - g.servo0_a) / 2 +
              (char)(((int)(g.servo0_b - g.servo0_a) * (int)(sine[j0])) >> 8));
    else
      set_dt0(g.servo0_b + (g.servo0_a - g.servo0_b) / 2 +
              (char)(((int)(g.servo0_a - g.servo0_b) * (int)(sine[j0])) >> 8));
  }

  if (g.servo1_power == 1 && g.servo1_mode == 0x3 && i1++ == g.servo1_tosc) {
    i1 = 0;
    j1++;
    if (g.servo1_b >= g.servo1_a)
      set_dt1(g.servo1_a + (g.servo1_b - g.servo1_a) / 2 +
              (char)(((int)(g.servo1_b - g.servo1_a) * (int)(sine[j1])) >> 8));
    else
      set_dt1(g.servo1_b + (g.servo1_a - g.servo1_b) / 2 +
              (char)(((int)(g.servo1_a - g.servo1_b) * (int)(sine[j1])) >> 8));
  }

  if (g.servo2_power == 1 && g.servo2_mode == 0x3 && i2++ == g.servo2_tosc) {
    i2 = 0;
    j2++;
    if (g.servo2_b >= g.servo2_a)
      set_dt2(g.servo2_a + (g.servo2_b - g.servo2_a) / 2 +
              (char)(((int)(g.servo2_b - g.servo2_a) * (int)(sine[j2])) >> 8));
    else
      set_dt2(g.servo2_b + (g.servo2_a - g.servo2_b) / 2 +
              (char)(((int)(g.servo2_a - g.servo2_b) * (int)(sine[j2])) >> 8));
  }
  TL0 = 0x64;
  TH0 = 0xFF;
  TF0 = 0;
}

/**
 * Callback I2C
 */
void i2c_callback(uint8 reg, uint8 *buffer) {
  switch (reg) {
  case SCSO_REG_BROADCAST:
    break;

  case SCSO_REG_SERVO0_POWERON:
    if (buffer[0] == 0xAF)
      g.servo0_power = 1;
    PCA0CPM0 = 0xC2;
    break;
  case SCSO_REG_SERVO1_POWERON:
    if (buffer[0] == 0xAF)
      g.servo1_power = 1;
    PCA0CPM1 = 0xC2;
    break;
  case SCSO_REG_SERVO2_POWERON:
    if (buffer[0] == 0xAF)
      g.servo2_power = 1;
    PCA0CPM2 = 0xC2;
    break;
  case SCSO_REG_SERVO0_POWEROFF:
    if (buffer[0] == 0xFA)
      g.servo0_power = 0;
    servo0_poweroff();
    break;
  case SCSO_REG_SERVO1_POWEROFF:
    if (buffer[0] == 0xFA)
      g.servo1_power = 0;
    servo1_poweroff();
    break;
  case SCSO_REG_SERVO2_POWEROFF:
    if (buffer[0] == 0xFA)
      g.servo2_power = 0;
    servo2_poweroff();
    break;

  case SCSO_REG_SERVO0_A:
    g.servo0_a = buffer[0];
    break;
  case SCSO_REG_SERVO0_B:
    g.servo0_b = buffer[0];
    break;
  case SCSO_REG_SERVO0_C:
    g.servo0_c = buffer[0];
    break;

  case SCSO_REG_SERVO1_A:
    g.servo1_a = buffer[0];
    break;
  case SCSO_REG_SERVO1_B:
    g.servo1_b = buffer[0];
    break;
  case SCSO_REG_SERVO1_C:
    g.servo1_c = buffer[0];
    break;

  case SCSO_REG_SERVO2_A:
    g.servo2_a = buffer[0];
    break;
  case SCSO_REG_SERVO2_B:
    g.servo2_b = buffer[0];
    break;
  case SCSO_REG_SERVO2_C:
    g.servo2_c = buffer[0];
    break;

  case SCSO_REG_SERVO0_GOTO_A:
    set_dt0(g.servo0_a);
    g.servo0_mode = 0x0;
    break;
  case SCSO_REG_SERVO0_GOTO_B:
    set_dt0(g.servo0_b);
    g.servo0_mode = 0x1;
    break;
  case SCSO_REG_SERVO0_GOTO_C:
    set_dt0(g.servo0_c);
    g.servo0_mode = 0x2;
    break;

  case SCSO_REG_SERVO1_GOTO_A:
    set_dt1(g.servo1_a);
    g.servo1_mode = 0x0;
    break;
  case SCSO_REG_SERVO1_GOTO_B:
    set_dt1(g.servo1_b);
    g.servo1_mode = 0x1;
    break;
  case SCSO_REG_SERVO1_GOTO_C:
    set_dt1(g.servo1_c);
    g.servo1_mode = 0x2;
    break;

  case SCSO_REG_SERVO2_GOTO_A:
    set_dt2(g.servo2_a);
    g.servo2_mode = 0x0;
    break;
  case SCSO_REG_SERVO2_GOTO_B:
    set_dt2(g.servo2_b);
    g.servo2_mode = 0x1;
    break;
  case SCSO_REG_SERVO2_GOTO_C:
    set_dt2(g.servo2_c);
    g.servo2_mode = 0x2;
    break;

  case SCSO_REG_SERVO0_OSC:
    g.servo0_mode = 0x3;
    break;
  case SCSO_REG_SERVO1_OSC:
    g.servo1_mode = 0x3;
    break;
  case SCSO_REG_SERVO2_OSC:
    g.servo2_mode = 0x3;
    break;

  case SCSO_REG_SERVO0_TOSC:
    g.servo0_tosc = buffer[0];
    break;
  case SCSO_REG_SERVO1_TOSC:
    g.servo1_tosc = buffer[0];
    break;
  case SCSO_REG_SERVO2_TOSC:
    g.servo2_tosc = buffer[0];
    break;
  }
}

int main(void) {
  init_device();
  init_memory_map();
  i2c_init(I2C_ADDR, &g, sizeof g);
  i2c_set_callback_mode(&i2c_callback);

  servo0_poweroff();
  servo1_poweroff();
  servo2_poweroff();

#if 0
  g.servo1_power = 1;
  PCA0CPM1 = 0xC2;
  set_dt1(-127);
#endif
  for (;;)
    ;
}