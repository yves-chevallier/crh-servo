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
 * @title    Logiciel de contr�le de moteur pour la carte "motionboard"
 * @context  Coupe suisse de robotique 2007
 * @author   Y. Chevallier <nowox@kalios.ch>
 * @file     scso_cmd_list.h
 * @language ASCII/C
 * @svn      $Id: main.c 105 2005-12-14 16:53:46Z Canard $
 * @desc     Ce fichier contient les adresses des registres de la carte
 *           servo.
 */
#ifndef __scso_524jdzq4834h__
#define __scso_524jdzq4834h__

/**
 * Command list
 */
#define SCSO_REG_BROADCAST          0x00
#define SCSO_REG_MODE               0x01
#define SCSO_REG_POWER              0x02
#define SCSO_REG_SERVO0_A           0x03
#define SCSO_REG_SERVO0_B           0x04
#define SCSO_REG_SERVO0_C           0x05
#define SCSO_REG_SERVO0_SPEED       0x06
#define SCSO_REG_SERVO0_SSC         0x07
#define SCSO_REG_SERVO1_A           0x08
#define SCSO_REG_SERVO1_B           0x09
#define SCSO_REG_SERVO1_C           0x0A
#define SCSO_REG_SERVO1_SPEED       0x0B
#define SCSO_REG_SERVO1_SSC         0x0C
#define SCSO_REG_SERVO2_A           0x0D
#define SCSO_REG_SERVO2_B           0x0E
#define SCSO_REG_SERVO2_C           0x0F
#define SCSO_REG_SERVO2_SPEED       0x10
#define SCSO_REG_SERVO2_SSC         0x11
                                  //0x12
                                  //0x13
                                  //0x14
                                  //0x15
                                  //0x16
                                  //0x17
                                  //0x18
                                  //0x19
                                  //0x1A
        		                  //0x1B
                                  //0x1C
                                  //0x1D
                                  //0x1E
                                  //0x1F
#define SCSO_REG_SERVO0_GOTO_A      0x20
#define SCSO_REG_SERVO0_GOTO_B      0x21
#define SCSO_REG_SERVO0_GOTO_C      0x22
#define SCSO_REG_SERVO0_OSC         0x23
#define SCSO_REG_SERVO0_TOSC        0x24
                                  //0x25
                                  //0x26
                                  //0x27
                                  //0x28
                                  //0x29
                                  //0x2A
        			              //0x2B
                                  //0x2C
                                  //0x2D
                                  //0x2E
                                  //0x2F
#define SCSO_REG_SERVO1_GOTO_A      0x30
#define SCSO_REG_SERVO1_GOTO_B      0x31
#define SCSO_REG_SERVO1_GOTO_C      0x32
#define SCSO_REG_SERVO1_OSC         0x33
#define SCSO_REG_SERVO1_TOSC        0x34
                                  //0x35
                                  //0x36
                                  //0x37
                                  //0x38
                                  //0x39
                                  //0x3A
                                  //0x3B
                                  //0x3C
                                  //0x3D
                                  //0x3E
                                  //0x3F
#define SCSO_REG_SERVO2_GOTO_A      0x40
#define SCSO_REG_SERVO2_GOTO_B      0x41
#define SCSO_REG_SERVO2_GOTO_C      0x42
#define SCSO_REG_SERVO2_OSC         0x43
#define SCSO_REG_SERVO2_TOSC        0x44
                                  //0x45
                                  //0x46
                                  //0x47
                                  //0x48
                                  //0x49
                                  //0x4A
        			              //0x4B
                                  //0x4C
                                  //0x4D
                                  //0x4E
                                  //0x4F
#define SCSO_REG_SERVO0_POWERON     0x50
#define SCSO_REG_SERVO1_POWERON     0x51
#define SCSO_REG_SERVO2_POWERON     0x52
                                  //0x51
                                  //0x52
                                  //0x53
                                  //0x54
                                  //0x55
                                  //0x56
                                  //0x57
                                  //0x58
                                  //0x59
                                  //0x5A
                                  //0x5B
                                  //0x5C
                                  //0x5D
                                  //0x5E
                                  //0x5F
#define SCSO_REG_SERVO0_POWEROFF    0x60
#define SCSO_REG_SERVO1_POWEROFF    0x61
#define SCSO_REG_SERVO2_POWEROFF    0x62
                                  //0x63
                                  //0x64
                                  //0x65
                                  //0x66
                                  //0x67
                                  //0x68
                                  //0x69
                                  //0x6A
                                  //0x6B
                                  //0x6C
                                  //0x6D
                                  //0x6E
                                  //0x6F
                                  //0x70
                                  //0x71
                                  //0x72
                                  //0x73
                                  //0x74
                                  //0x75
                                  //0x76
                                  //0x77
                                  //0x78
                                  //0x79
                                  //0x7A
        			              //0x7B
                                  //0x7C
                                  //0x7D
                                  //0x7E
                                  //0x7F
                                  //0x80
                                  //0x81
                                  //0x82
                                  //0x83
                                  //0x84
                                  //0x85
                                  //0x86
                                  //0x87
                                  //0x88
                                  //0x89
                                  //0x8A
        		            	  //0x8B
                                  //0x8C
                                  //0x8D
                                  //0x8E
                                  //0x8F
                                  //0x90
                                  //0x91
                                  //0x92
                                  //0x93
                                  //0x94
                                  //0x95
                                  //0x96
                                  //0x97
                                  //0x98
                                  //0x99
                                  //0x9A
        			              //0x9B
                                  //0x9C
                                  //0x9D
                                  //0x9E
                                  //0x9F
                                  //0xA0
                                  //0xA1
                                  //0xA2
                                  //0xA3
                                  //0xA4
                                  //0xA5
                                  //0xA6
                                  //0xA7
                                  //0xA8
                                  //0xA9
                                  //0xAA
        			              //0xAB
                                  //0xAC
                                  //0xAD
                                  //0xAE
                                  //0xAF
                                  //0xB0
                                  //0xB1
                                  //0xB2
                                  //0xB3
                                  //0xB4
                                  //0xB5
                                  //0xB6
                                  //0xB7
                                  //0xB8
                                  //0xB9
                                  //0xBA
        			              //0xBB
                                  //0xBC
                                  //0xBD
                                  //0xBE
                                  //0xBF
                                  //0xC0
                                  //0xC1
                                  //0xC2
                                  //0xC3
                                  //0xC4
                                  //0xC5
                                  //0xC6
                                  //0xC7
                                  //0xC8
                                  //0xC9
                                  //0xCA
        			              //0xCB
                                  //0xCC
                                  //0xCD
                                  //0xCE
                                  //0xCF
                                  //0xD0
                                  //0xD1
                                  //0xD2
                                  //0xD3
                                  //0xD4
                                  //0xD5
                                  //0xD6
                                  //0xD7
                                  //0xD8
                                  //0xD9
                                  //0xDA
        			              //0xDB
                                  //0xDC
                                  //0xDD
                                  //0xDE
                                  //0xDF
                                  //0xE0
                                  //0xE1
                                  //0xE2
                                  //0xE3
                                  //0xE4
                                  //0xE5
                                  //0xE6
                                  //0xE7
                                  //0xE8
                                  //0xE9
                                  //0xEA
        			              //0xEB
                                  //0xEC
                                  //0xED
                                  //0xEE
                                  //0xEF
                                  //0xF0
                                  //0xF1
                                  //0xF2
                                  //0xF3
                                  //0xF4
                                  //0xF5
                                  //0xF6
                                  //0xF7
                                  //0xF8
                                  //0xF9
                                  //0xFA
        			              //0xFB
                                  //0xFC
                                  //0xFD
                                  //0xFE
                                  //0xFF
#endif
