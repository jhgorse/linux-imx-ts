/*
 * Maxim MAX17055 Fuel Gauge driver header file
 *
 * Author: Kerem Sahin <kerem.sahin@maximintegrated.com>
 * Copyright (C) 2016 Maxim Integrated
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MAX17055_BATTERY_H_
#define __MAX17055_BATTERY_H_

#define MAX17055_TABLE_SIZE             48
#define MAX17055_TABLE_SIZE_IN_BYTES    (2 * MAX17055_TABLE_SIZE)

/* Model loading options */
#define MODEL_LOADING_OPTION1           1
#define MODEL_LOADING_OPTION2           2
#define MODEL_LOADING_OPTION3           3

/* Model lock/unlock */
#define MODEL_UNLOCK1   0X0059
#define MODEL_UNLOCK2   0X00C4
#define MODEL_LOCK1     0X0000
#define MODEL_LOCK2     0X0000

/* VFSOC0/QH0 lock/unlock */
#define VFSOC0_QH0_LOCK     0x0000
#define VFSOC0_QH0_UNLOCK   0x0080

#define MODEL_SCALING        1

struct max17055_platform_data {
    u16 designcap;
    u16 ichgterm;
    u16 vempty;
    int vcharge;

    u16 learncfg;
    u16 relaxcfg;
    u16 config;
    u16 config2;
    u16 fullsocthr;
    u16 tgain;
    u16 toff;
    u16 curve;
    u16 rcomp0;
    u16 tempco;
    u16 qrtable00;
    u16 qrtable10;
    u16 qrtable20;
    u16 qrtable30;

    u16 dpacc;
    u16 modelcfg;

    u16 model_data[MAX17055_TABLE_SIZE];
    int (*get_charging_status)(void);
    int model_option;

    /*
     * rsense in miliOhms.
     * default 10 (if rsense = 0) as it is the recommended value by
     * the datasheet although it can be changed by board designers.
     */
    unsigned int rsense;
    int volt_min;   /* in mV */
    int volt_max;   /* in mV */
    int temp_min;   /* in DegreC */
    int temp_max;   /* in DegreeC */
    int soc_max;    /* in percent */
    int soc_min;    /* in percent */
    int curr_max;   /* in mA */
    int curr_min;   /* in mA */
};

enum max17055_register{
    MAX17055_STATUS_REG                 = 0x00,
    MAX17055_VALRTTH_REG                = 0x01,
    MAX17055_TALRTTH_REG                = 0x02,
    MAX17055_SALRTTH_REG                = 0x03,
    MAX17055_REPCAP_REG                 = 0x05,
    MAX17055_REPSOC_REG                 = 0x06,
    MAX17055_TEMP_REG                   = 0x08,
    MAX17055_VCELL_REG                  = 0x09,
    MAX17055_CURRENT_REG                = 0x0A,
    MAX17055_AVGCURRENT_REG             = 0x0B,
    MAX17055_MIXCAP_REG                 = 0x0F,

    MAX17055_FULLCAPREP_REG             = 0x10,
    MAX17055_TTE_REG                    = 0X11,
    MAX17055_QRTABLE00_REG              = 0x12,
    MAX17055_FULLSOCTHR_REG             = 0x13,
    MAX17055_CYCLES_REG                 = 0x17,
    MAX17055_DESIGNCAP_REG              = 0x18,
    MAX17055_AVGVCELL_REG               = 0x19,
    MAX17055_MAXMINVOLT_REG             = 0x1B,
    MAX17055_CONFIG_REG                 = 0x1D,
    MAX17055_ICHGTERM_REG               = 0x1E,

    MAX17055_VERSION_REG                = 0x21,
    MAX17055_QRTABLE10_REG              = 0x22,
    MAX17055_FULLCAPNOM_REG             = 0x23,
    MAX17055_LEARNCFG_REG               = 0x28,
    MAX17055_RELAXCFG_REG               = 0x2A,
    MAX17055_TGAIN_REG                  = 0x2C,
    MAX17055_TOFF_REG                   = 0x2D,

    MAX17055_QRTABLE20_REG              = 0x32,
    MAX17055_RCOMP0_REG                 = 0x38,
    MAX17055_TEMPCO_REG                 = 0x39,
    MAX17055_VEMPTY_REG                 = 0x3A,
    MAX17055_FSTAT_REG                  = 0x3D,

    MAX17055_QRTABLE30_REG              = 0x42,
    MAX17055_DQACC_REG                  = 0x45,
    MAX17055_DPACC_REG                  = 0x46,
    MAX17055_VFSOC0_REG                 = 0x48,
    MAX17055_QH0_REG                    = 0x4C,
    MAX17055_QH_REG                     = 0x4D,

    MAX17055_VFSOC0_QH0_LOCK_REG        = 0x60,
    MAX17055_LOCK1_REG                  = 0x62,
    MAX17055_LOCK2_REG                  = 0x63,

    MAX17055_MODELDATA_START_REG        = 0x80,

    MAX17055_IALRTTH_REG                = 0xB4,
    MAX17055_CURVE_REG                  = 0xB9,
    MAX17055_HIBCFG_REG                 = 0xBA,
    MAX17055_CONFIG2_REG                = 0xBB,

    MAX17055_MODELCFG_REG               = 0xDB,

    MAX17055_OCV_REG                    = 0xFB,
    MAX17055_VFSOC_REG                  = 0xFF,
};

#endif
