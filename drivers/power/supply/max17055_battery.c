/*
 * Maxim MAX17055 IC Fuel Gauge driver
 *
 * Author: Mahir Ozturk <mahir.ozturk@maximintegrated.com>
 *         Kerem Sahin <kerem.sahin@maximintegrated.com>
 * Copyright (C) 2016 Maxim Integrated
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

 //FW Version 1.0.9
 
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/power/max17055_battery.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define DRV_NAME "max17055"


/* CONFIG register bits */
#define MAX17055_CONFIG_ALRT_EN         (1 << 2)
#define MAX17055_CONFIG2_LDMDL          (1 << 5)

/* STATUS register bits */
#define MAX17055_STATUS_BST             (1 << 3)
#define MAX17055_STATUS_POR             (1 << 1)

/* MODELCFG register bits */
#define MAX17055_MODELCFG_REFRESH       (1 << 15)

/* TALRTTH register bits */
#define MIN_TEMP_ALERT                  0
#define MAX_TEMP_ALERT                  8

/* FSTAT register bits */
#define MAX17055_FSTAT_DNR              (1)

/* STATUS interrupt status bits */
#define MAX17055_STATUS_ALRT_CLR_MASK   (0x88BB)
#define MAX17055_STATUS_SOC_MAX_ALRT    (1 << 14)
#define MAX17055_STATUS_TEMP_MAX_ALRT   (1 << 13)
#define MAX17055_STATUS_VOLT_MAX_ALRT   (1 << 12)
#define MAX17055_STATUS_SOC_MIN_ALRT    (1 << 10)
#define MAX17055_STATUS_TEMP_MIN_ALRT   (1 << 9)
#define MAX17055_STATUS_VOLT_MIN_ALRT   (1 << 8)
#define MAX17055_STATUS_CURR_MAX_ALRT   (1 << 6)
#define MAX17055_STATUS_CURR_MIN_ALRT   (1 << 2)

#define MAX17055_VMAX_TOLERANCE     50 /* 50 mV */

enum chip_id {
    ID_MAX17055,
};

struct max17055_priv {
    struct i2c_client       *client;
    struct device           *dev;
    struct regmap           *regmap;
    struct power_supply     *battery;
    struct max17055_platform_data   *pdata;
    struct work_struct      init_worker;
    struct attribute_group  *attr_grp;
};

static inline int max17055_lsb_to_uvolts(struct max17055_priv *priv, int lsb)
{
    return lsb * 625 / 8; /* 78.125uV per bit */
}

static int max17055_raw_current_to_uamps(struct max17055_priv *priv, u32 curr)
{
    int res = curr;

    /* Negative */
    if (res & 0x8000)
        res |= 0xFFFF0000;

    res *= 1562500 / (priv->pdata->rsense * 1000);
    return res;
}

static enum power_supply_property max17055_battery_props[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_CYCLE_COUNT,
    POWER_SUPPLY_PROP_VOLTAGE_MAX,
    POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_VOLTAGE_AVG,
    POWER_SUPPLY_PROP_VOLTAGE_OCV,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_NOW,
    POWER_SUPPLY_PROP_CHARGE_COUNTER,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
    POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CURRENT_AVG,
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
};

static int max17055_poll_flag_clear(struct regmap *map, int regno, int mask, int timeout)
{
    u32 data;
    int ret;

    do
    {
        msleep(50);
        ret = regmap_read(map, regno, &data);
        if(ret < 0)
            return ret;

        if(!(data & mask))
            return 0;

        timeout -= 50;
    } while(timeout > 0);

    return -ETIME;
}

static inline void max17055_unlock_model(struct max17055_priv *priv)
{
    struct regmap *map = priv->regmap;

    regmap_write(map, MAX17055_LOCK1_REG, MODEL_UNLOCK1);
    regmap_write(map, MAX17055_LOCK2_REG, MODEL_UNLOCK2);
}

static inline void max17055_lock_model(struct max17055_priv *priv)
{
    struct regmap *map = priv->regmap;

    regmap_write(map, MAX17055_LOCK1_REG, MODEL_LOCK1);
    regmap_write(map, MAX17055_LOCK2_REG, MODEL_LOCK2);
}

static ssize_t max17055_log_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct max17055_priv *priv = dev_get_drvdata(dev);
    int rc = 0, reg = 0;
    u32 val = 0;

    for(reg = 0; reg < 0xE0; reg++)
    {
        regmap_read(priv->regmap, reg, &val);
        rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "0x%04X,", val);

        if(reg == 0x4F)
            reg += 0x60;

        if(reg == 0xBF)
            reg += 0x10;
    }

    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "\n");

    return rc;
}

static DEVICE_ATTR(log, S_IRUGO, max17055_log_show, NULL);

static struct attribute* max17055_attr[] = {
    &dev_attr_log.attr,
    NULL
};

static struct attribute_group max17055_attr_group = {
    .attrs = max17055_attr,
};

static int max17055_get_temperature(struct max17055_priv *priv, int *temp)
{
    int ret;
    u32 data;
    struct regmap *map = priv->regmap;

    ret = regmap_read(map, MAX17055_TEMP_REG, &data);
    if (ret < 0)
        return ret;

    *temp = data;
    /* The value is signed. */
    if (*temp & 0x8000)
        *temp |= 0xFFFF0000;

    /* The value is converted into centigrade scale */
    /* Units of LSB = 1 / 256 degree Celsius */
    *temp >>= 8;

    return 0;
}

static int max17055_get_temperature_limit(struct max17055_priv *priv, int *temp, int shift)
{
    int ret;
    u32 data;
    struct regmap *map = priv->regmap;

    ret = regmap_read(map, MAX17055_TALRTTH_REG, &data);
    if (ret < 0)
        return ret;

    *temp = data >> shift;
    /* The value is signed */
    if(*temp & 0x80)
        *temp |= 0xFFFFFF00;

    /* LSB is 1DegreeC */
    return 0;
}

static int max17055_get_battery_health(struct max17055_priv *priv, int *health)
{
    int temp, vavg, vbatt, ret;
    u32 val;

    ret = regmap_read(priv->regmap, MAX17055_AVGVCELL_REG, &val);
    if (ret < 0)
        goto health_error;

    /* bits [0-3] unused */
    vavg = max17055_lsb_to_uvolts(priv, val);
    /* Convert to millivolts */
    vavg /= 1000;

    ret = regmap_read(priv->regmap, MAX17055_VCELL_REG, &val);
    if (ret < 0)
        goto health_error;

    /* bits [0-3] unused */
    vbatt = max17055_lsb_to_uvolts(priv, val);
    /* Convert to millivolts */
    vbatt /= 1000;

    if (vavg < priv->pdata->volt_min) {
        *health = POWER_SUPPLY_HEALTH_DEAD;
        goto out;
    }

    if (vbatt > priv->pdata->volt_max + MAX17055_VMAX_TOLERANCE) {
        *health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        goto out;
    }

    ret = max17055_get_temperature(priv, &temp);
    if (ret < 0)
        goto health_error;

    if (temp <= priv->pdata->temp_min) {
        *health = POWER_SUPPLY_HEALTH_COLD;
        goto out;
    }

    if (temp >= priv->pdata->temp_max) {
        *health = POWER_SUPPLY_HEALTH_OVERHEAT;
        goto out;
    }

    *health = POWER_SUPPLY_HEALTH_GOOD;

out:
    return 0;

health_error:
    return ret;
}

static int max17055_write_verify_reg(struct regmap *map, u8 reg, u32 value)
{
    int retries = 8;
    int ret;
    u32 read_value;

    do {
        ret = regmap_write(map, reg, value);
        msleep(3);
        regmap_read(map, reg, &read_value);
        if (read_value != value) {
            ret = -EIO;
            retries--;
        }
    } while (retries && read_value != value);

    if (ret < 0)
        pr_err("Couldn't verify write 0x%4x to reg 0x%2x\n", value, reg);

    return ret;
}

static inline void max17055_write_model_data(struct max17055_priv *priv,
                    u8 addr, int size)
{
    struct regmap *map = priv->regmap;
    int i;

    for (i = 0; i < size; i++)
        regmap_write(map, addr + i, priv->pdata->model_data[i]);
}

static inline void max17055_read_model_data(struct max17055_priv *priv,
                    u8 addr, u16 *data, int size)
{
    struct regmap *map = priv->regmap;
    int i;
    u32 singledata;

    for (i = 0; i < size; i++){
        regmap_read(map, addr + i, &singledata);
        data[i] = singledata;
    }
}

static inline int max17055_model_data_compare(struct max17055_priv *priv,
                    u16 *data1, u16 *data2, int size)
{
    int i;

    if (memcmp(data1, data2, size)) {
        dev_err(&priv->client->dev, "%s compare failed\n", __func__);
        for (i = 0; i < size; i++)
            dev_info(&priv->client->dev, "0x%4x, 0x%4x", data1[i], data2[i]);
        dev_info(&priv->client->dev, "\n");
        return -EINVAL;
    }
    return 0;
}

static int max17055_load_model(struct max17055_priv *priv)
{
    int ret;
    int table_size = ARRAY_SIZE(priv->pdata->model_data);
    u16 *temp_data;

    temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
    if (!temp_data)
        return -ENOMEM;

    max17055_unlock_model(priv);
    max17055_write_model_data(priv, MAX17055_MODELDATA_START_REG, table_size);
    max17055_read_model_data(priv, MAX17055_MODELDATA_START_REG,
        temp_data, table_size);

    ret = max17055_model_data_compare(
        priv,
        priv->pdata->model_data,
        temp_data,
        table_size);

    max17055_lock_model(priv);
    kfree(temp_data);

    return ret;
}

static int max17055_get_property(struct power_supply *psy,
                   enum power_supply_property psp,
                   union power_supply_propval *val)
{
    struct max17055_priv *priv = power_supply_get_drvdata(psy);
    struct regmap *regmap = priv->regmap;
    struct max17055_platform_data *pdata = priv->pdata;
    unsigned int reg;
    int ret;

    switch (psp) {
    case POWER_SUPPLY_PROP_PRESENT:
        ret = regmap_read(regmap, MAX17055_STATUS_REG, &reg);
        if (ret < 0)
            return ret;
        if (reg & MAX17055_STATUS_BST)
            val->intval = 0;
        else
            val->intval = 1;
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        ret = regmap_read(regmap, MAX17055_CYCLES_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = reg;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MAX:
        ret = regmap_read(regmap, MAX17055_MAXMINVOLT_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = reg >> 8;
        val->intval *= 20000; /* Units of LSB = 20mV */
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
        ret = regmap_read(regmap, MAX17055_VEMPTY_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = reg >> 7;
        val->intval *= 10000; /* Units of LSB = 10mV */
        break;
    case POWER_SUPPLY_PROP_STATUS:
        if (pdata && pdata->get_charging_status)
            val->intval = pdata->get_charging_status();
        else
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = regmap_read(regmap, MAX17055_VCELL_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = max17055_lsb_to_uvolts(priv, reg);
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_AVG:
        ret = regmap_read(regmap, MAX17055_AVGVCELL_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = max17055_lsb_to_uvolts(priv, reg);
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_OCV:
        ret = regmap_read(regmap, MAX17055_OCV_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = max17055_lsb_to_uvolts(priv, reg);
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        ret = regmap_read(regmap, MAX17055_REPSOC_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = reg >> 8; /* RepSOC LSB: 1/256 % */
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        ret = regmap_read(regmap, MAX17055_FULLCAPREP_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = (reg * 1000) >> 1; /* FullCAPRep LSB: 0.5 mAh */
        break;
    case POWER_SUPPLY_PROP_CHARGE_COUNTER:
        ret = regmap_read(regmap, MAX17055_QH_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = ((s16)reg * 1000) >> 1; /* QH LSB: 0.5 mAh */
        break;
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        ret = regmap_read(regmap, MAX17055_REPCAP_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = (reg * 1000) >> 1; /* RepCAP LSB: 0.5 mAh */
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = max17055_get_temperature(priv, &val->intval);
        if (ret < 0)
            return ret;

        val->intval *= 10; /* Convert 1DegreeC LSB to 0.1DegreeC LSB */
        break;
    case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
        ret = max17055_get_temperature_limit(priv, &val->intval, MIN_TEMP_ALERT);
        if (ret < 0)
            return ret;

        val->intval *= 10; /* Convert 1DegreeC LSB to 0.1DegreeC LSB */
        break;
    case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
        ret = max17055_get_temperature_limit(priv, &val->intval, MAX_TEMP_ALERT);
        if (ret < 0)
            return ret;

        val->intval *= 10; /* Convert 1DegreeC LSB to 0.1DegreeC LSB */
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        ret = max17055_get_battery_health(priv, &val->intval);
        if (ret < 0)
            return ret;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = regmap_read(regmap, MAX17055_CURRENT_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = max17055_raw_current_to_uamps(priv, reg);
        break;
    case POWER_SUPPLY_PROP_CURRENT_AVG:
        ret = regmap_read(regmap, MAX17055_AVGCURRENT_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = max17055_raw_current_to_uamps(priv, reg);
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
        ret = regmap_read(regmap, MAX17055_TTE_REG, &reg);
        if (ret < 0)
            return ret;

        val->intval = (reg * 45) >> 3; /* TTE LSB: 5.625 sec */
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static irqreturn_t max17055_irq_handler(int id, void *dev)
{
    struct max17055_priv *priv = dev;
    u32 val;

    /* Check alert type */
    regmap_read(priv->regmap, MAX17055_STATUS_REG, &val);

    if (val & MAX17055_STATUS_SOC_MAX_ALRT)
        dev_info(priv->dev, "Alert: SOC MAX!\n");
    if (val & MAX17055_STATUS_SOC_MIN_ALRT)
        dev_info(priv->dev, "Alert: SOC MIN!\n");
    if (val & MAX17055_STATUS_TEMP_MAX_ALRT)
        dev_info(priv->dev, "Alert: TEMP MAX!\n");
    if (val & MAX17055_STATUS_TEMP_MIN_ALRT)
        dev_info(priv->dev, "Alert: TEMP MIN!\n");
    if (val & MAX17055_STATUS_VOLT_MAX_ALRT)
        dev_info(priv->dev, "Alert: VOLT MAX!\n");
    if (val & MAX17055_STATUS_VOLT_MIN_ALRT)
        dev_info(priv->dev, "Alert: VOLT MIN!\n");
    if (val & MAX17055_STATUS_CURR_MAX_ALRT)
        dev_info(priv->dev, "Alert: CURR MAX!\n");
    if (val & MAX17055_STATUS_CURR_MIN_ALRT)
        dev_info(priv->dev, "Alert: CURR MIN!\n");

    /* Clear alerts */
    regmap_write(priv->regmap, MAX17055_STATUS_REG,
                  val & MAX17055_STATUS_ALRT_CLR_MASK);

    power_supply_changed(priv->battery);
    return IRQ_HANDLED;
}

static int max17055_verify_model_lock(struct max17055_priv *priv)
{
    int i;
    int table_size = ARRAY_SIZE(priv->pdata->model_data);
    u16 *temp_data;
    int ret = 0;

    temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
    if (!temp_data)
        return -ENOMEM;

    max17055_read_model_data(priv, MAX17055_MODELDATA_START_REG, temp_data,
                table_size);
    for (i = 0; i < table_size; i++)
        if (temp_data[i])
            ret = -EINVAL;

    kfree(temp_data);
    return ret;
}

static void max17055_set_alert_thresholds(struct max17055_priv *priv)
{
    struct max17055_platform_data *pdata = priv->pdata;
    struct regmap *regmap = priv->regmap;
    u32 val;

    /* Set VAlrtTh */
    val = (pdata->volt_min / 20);
    val |= ((pdata->volt_max / 20) << 8);
    regmap_write(regmap, MAX17055_VALRTTH_REG, val);

    /* Set TAlrtTh */
    val = pdata->temp_min & 0xFF;
    val |= ((pdata->temp_max & 0xFF) << 8);
    regmap_write(regmap, MAX17055_TALRTTH_REG, val);

    /* Set SAlrtTh */
    val = pdata->soc_min;
    val |= (pdata->soc_max << 8);
    regmap_write(regmap, MAX17055_SALRTTH_REG, val);

    /* Set IAlrtTh */
    val = (pdata->curr_min * pdata->rsense / 400) & 0xFF;
    val |= (((pdata->curr_max * pdata->rsense / 400) & 0xFF) << 8);
    regmap_write(regmap, MAX17055_IALRTTH_REG, val);
}

static int max17055_init(struct max17055_priv *priv)
{
    struct regmap *regmap = priv->regmap;
    struct max17055_platform_data *pdata = priv->pdata;
    int ret;
    unsigned int reg;
    u32 vfsoc, fgrev, hibcfg, dpacc;

    ret = regmap_read(regmap, MAX17055_VERSION_REG, &fgrev);
    if (ret < 0)
        return -ENODEV;

    dev_info(priv->dev, "IC Version: 0x%04x\n", fgrev);

    /* Step 0: Check for POR */
    /* Skip load model if POR bit is cleared */
    regmap_read(regmap, MAX17055_STATUS_REG, &reg);
    /* Skip load custom model */
    if (!(reg & MAX17055_STATUS_POR)){
        dev_info(priv->dev, "POR is not set. Skipping initialization...\n");
        return 0;
    }

    /* Step 1: Check if FStat.DNR == 0 */
    ret = max17055_poll_flag_clear(regmap, MAX17055_FSTAT_REG, MAX17055_FSTAT_DNR, 500);
    if (ret < 0){
        dev_err(priv->dev, "Unsuccessful init: Data Not Ready!\n");
        return ret;
    }

    /* Step 2: Initialize configuration */
    
    /* Force exit from hibernate */
    regmap_read(regmap, MAX17055_HIBCFG_REG, &hibcfg);
    regmap_write(regmap, 0x60, 0x90);
    regmap_write(regmap, MAX17055_HIBCFG_REG, 0x0);
    regmap_write(regmap, 0x60, 0x0);

    switch (pdata->model_option) {
    case MODEL_LOADING_OPTION1:
        /* Step 2.1: Option 1 EZ Config */
        regmap_write(regmap, MAX17055_DESIGNCAP_REG, pdata->designcap);
        regmap_write(regmap, MAX17055_DQACC_REG, pdata->designcap >> 5);
        regmap_write(regmap, MAX17055_ICHGTERM_REG, pdata->ichgterm);
        regmap_write(regmap, MAX17055_VEMPTY_REG, pdata->vempty);

        if (pdata->vcharge > 4275){
            dpacc = (pdata->designcap >> 5) * 0xC800 / pdata->designcap;
            regmap_write(regmap, MAX17055_DPACC_REG, dpacc);
            regmap_write(regmap, MAX17055_MODELCFG_REG, 0x8400);
        }
        else{
            dpacc = (pdata->designcap >> 5) * 0xAC6A / pdata->designcap;
            regmap_write(regmap, MAX17055_DPACC_REG, dpacc);
            regmap_write(regmap, MAX17055_MODELCFG_REG, 0x8000);
        }

        /* Poll ModelCFG.ModelRefresh bit for clear */
        ret = max17055_poll_flag_clear(regmap, MAX17055_MODELCFG_REG, MAX17055_MODELCFG_REFRESH, 500);
        if(ret < 0){
            dev_err(priv->dev, "Option1 model refresh not completed!\n");
            return ret;
        }
        break;
    case MODEL_LOADING_OPTION2:
        /* Step 2.2: Option 2 Custom Short INI without OCV Table */
        regmap_write(regmap, MAX17055_DESIGNCAP_REG, pdata->designcap);
        regmap_write(regmap, MAX17055_DQACC_REG, pdata->designcap >> 4);
        regmap_write(regmap, MAX17055_ICHGTERM_REG, pdata->ichgterm);
        regmap_write(regmap, MAX17055_VEMPTY_REG, pdata->vempty);
        max17055_write_verify_reg(regmap, MAX17055_LEARNCFG_REG, pdata->learncfg); /* Optional */

        regmap_write(regmap, MAX17055_DPACC_REG, pdata->dpacc);
        regmap_write(regmap, MAX17055_MODELCFG_REG, pdata->modelcfg);

        /* Poll ModelCFG.ModelRefresh bit for clear */
        ret = max17055_poll_flag_clear(regmap, MAX17055_MODELCFG_REG, MAX17055_MODELCFG_REFRESH, 500);
        if(ret < 0){
            dev_err(priv->dev, "Option2 model refresh not completed!\n");
            return ret;
        }

        regmap_write(regmap, MAX17055_RCOMP0_REG, pdata->rcomp0);
        regmap_write(regmap, MAX17055_TEMPCO_REG, pdata->tempco);
        regmap_write(regmap, MAX17055_QRTABLE00_REG, pdata->qrtable00);
        regmap_write(regmap, MAX17055_QRTABLE10_REG, pdata->qrtable10);
        regmap_write(regmap, MAX17055_QRTABLE20_REG, pdata->qrtable20);  /* Optional */
        regmap_write(regmap, MAX17055_QRTABLE30_REG, pdata->qrtable30);  /* Optional */

        break;
    case MODEL_LOADING_OPTION3:
        /* Step 3.3: Option 3 Custom Full INI with OCV Table */
        
        /* Step 3.3.1a :  Write LearnCFG with LS 7 */
        max17055_write_verify_reg(regmap, MAX17055_LEARNCFG_REG, pdata->learncfg|0x0070);
        
        /* Steps 3.3.1-3: Unlock model access, write/read/verify custom model,
                          lock model access */                         
        ret = max17055_load_model(priv);
        if(ret){
            dev_err(priv->dev, "Option3 model table write unsuccessful!\n");
            return ret;
        }

        /* Steps 3.3.4: Verify that model access is locked */
        ret = max17055_verify_model_lock(priv);
        if(ret){
            dev_err(priv->dev, "Option3 model unlock unsuccessful!\n");
            return ret;
        }

        /* Step 3.3.5 Write custom paramaters */
        regmap_write(regmap, MAX17055_DESIGNCAP_REG, pdata->designcap);
        regmap_write(regmap, MAX17055_DQACC_REG, pdata->designcap >> 4);
        regmap_write(regmap, MAX17055_DPACC_REG, 0xC80);
        regmap_write(regmap, MAX17055_ICHGTERM_REG, pdata->ichgterm);
        regmap_write(regmap, MAX17055_VEMPTY_REG, pdata->vempty);
        regmap_write(regmap, MAX17055_RCOMP0_REG, pdata->rcomp0);
        regmap_write(regmap, MAX17055_TEMPCO_REG, pdata->tempco);
        regmap_write(regmap, MAX17055_QRTABLE00_REG, pdata->qrtable00);
        regmap_write(regmap, MAX17055_QRTABLE10_REG, pdata->qrtable10);

        /* Update required capacity registers */
        regmap_write(regmap, MAX17055_REPCAP_REG, 0);
        regmap_read(regmap, MAX17055_VFSOC_REG, &vfsoc);
        max17055_write_verify_reg(regmap, MAX17055_VFSOC0_REG, vfsoc);
        max17055_write_verify_reg(regmap, MAX17055_FULLCAPREP_REG, pdata->designcap);
        max17055_write_verify_reg(regmap, MAX17055_FULLCAPNOM_REG, pdata->designcap);

        /* Optional */
        regmap_write(regmap, MAX17055_RELAXCFG_REG, pdata->relaxcfg);
        regmap_write(regmap, MAX17055_CONFIG_REG, pdata->config);
        regmap_write(regmap, MAX17055_CONFIG2_REG, pdata->config2);
        regmap_write(regmap, MAX17055_FULLSOCTHR_REG, pdata->fullsocthr);
        regmap_write(regmap, MAX17055_TGAIN_REG, pdata->tgain);
        regmap_write(regmap, MAX17055_TOFF_REG, pdata->toff);
        regmap_write(regmap, MAX17055_CURVE_REG, pdata->curve);

        /* Step 3.3.5b Prepare to Load Model */
        regmap_write(regmap, MAX17055_REPCAP_REG , 0x0000); 
        regmap_write(regmap, MAX17055_MIXCAP_REG, pdata->designcap);
        
        /* Step 3.3.6 Initiate model loading */
        regmap_read(regmap, MAX17055_CONFIG2_REG, &reg);
        regmap_write(regmap, MAX17055_CONFIG2_REG, reg | MAX17055_CONFIG2_LDMDL); /* Set Config2.LdMdl bit */

        
        /* Step 3.3.7 Poll the Config2.LdMdl=0 */
        ret = max17055_poll_flag_clear(regmap, MAX17055_CONFIG2_REG, MAX17055_CONFIG2_LDMDL, 5000);
        if(ret < 0){
            dev_err(priv->dev, "Option3 LdMdl not completed!\n");
            return ret;
        }
        
        /* Step 3.3.7b :  Write LearnCFG with LS 0 */
        max17055_write_verify_reg(regmap, MAX17055_LEARNCFG_REG, pdata->learncfg&0xFF8F);
        
        max17055_write_verify_reg(regmap, MAX17055_QRTABLE20_REG, pdata->qrtable20);
        max17055_write_verify_reg(regmap, MAX17055_QRTABLE30_REG, pdata->qrtable30);

        break;
    default:
        dev_err(priv->dev, "Undefined model option: %d\n", pdata->model_option);
    }

    /* Restore original HibCfg */
    regmap_write(regmap, MAX17055_HIBCFG_REG, hibcfg);

    /* Optional step - alert threshold initialization */
    max17055_set_alert_thresholds(priv);

    /* Clear Status.POR */
    regmap_read(regmap, MAX17055_STATUS_REG, &reg);
    max17055_write_verify_reg(regmap, MAX17055_STATUS_REG, reg & ~MAX17055_STATUS_POR);

    return 0;
}

static void max17055_init_worker(struct work_struct *work)
{
    struct max17055_priv *priv = container_of(work,
                            struct max17055_priv,
                            init_worker);

    max17055_init(priv);
}

static struct max17055_platform_data *max17055_parse_dt(struct device *dev)
{
    struct device_node *np = dev->of_node;
    struct max17055_platform_data *pdata;
    struct property *prop;
    int ret;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata)
        return NULL;

    ret = of_property_read_u32(np, "talrt-min", &pdata->temp_min);
    if (ret)
        pdata->temp_min = -128; /* DegreeC */ /* Disable alert */

    ret = of_property_read_u32(np, "talrt-max", &pdata->temp_max);
    if (ret)
        pdata->temp_max = 127; /* DegreeC */ /* Disable alert */

    ret = of_property_read_u32(np, "valrt-min", &pdata->volt_min);
    if (ret)
        pdata->volt_min = 0; /* mV */ /* Disable alert */

    ret = of_property_read_u32(np, "valrt-max", &pdata->volt_max);
    if (ret)
        pdata->volt_max = 5100; /* mV */ /* Disable alert */

    ret = of_property_read_u32(np, "ialrt-min", &pdata->curr_min);
    if (ret)
        pdata->curr_min = -5120; /* mA */ /* Disable alert */

    ret = of_property_read_u32(np, "ialrt-max", &pdata->curr_max);
    if (ret)
        pdata->curr_max = 5080; /* mA */ /* Disable alert */

    ret = of_property_read_u32(np, "salrt-min", &pdata->soc_min);
    if (ret)
        pdata->soc_min = 0; /* Percent */ /* Disable alert */

    ret = of_property_read_u32(np, "salrt-max", &pdata->soc_max);
    if (ret)
        pdata->soc_max = 255; /* Percent */ /* Disable alert */


    ret = of_property_read_u32(np, "rsense", &pdata->rsense);
    if (ret)
        pdata->rsense = 10;

    ret = of_property_read_u32(np, "model-option", &pdata->model_option);
    if (ret)
        pdata->model_option = 1;


    ret = of_property_read_u16(np, "vempty", &pdata->vempty);
    if (ret)
        pdata->vempty = 0xA561;

    ret = of_property_read_u16(np, "ichgterm", &pdata->ichgterm);
    if (ret)
        pdata->ichgterm = 0x0640;

    ret = of_property_read_u16(np, "designcap", &pdata->designcap);
    if (ret)
        pdata->designcap = 0x0BB8;

    ret = of_property_read_u32(np, "vcharge", &pdata->vcharge);
    if (ret)
        pdata->vcharge = 4300;


    ret = of_property_read_u16(np, "rcomp0", &pdata->rcomp0);
    if (ret)
        pdata->rcomp0 = 0x0070;

    ret = of_property_read_u16(np, "tempco", &pdata->tempco);
    if (ret)
        pdata->tempco = 0x223E;

    ret = of_property_read_u16(np, "learncfg", &pdata->learncfg);
    if (ret)
        pdata->learncfg = 0xC482;

    ret = of_property_read_u16(np, "qrtable00", &pdata->qrtable00);
    if (ret)
        pdata->qrtable00 = 0x1050;

    ret = of_property_read_u16(np, "qrtable10", &pdata->qrtable10);
    if (ret)
        pdata->qrtable10 = 0x2013;

    ret = of_property_read_u16(np, "qrtable20", &pdata->qrtable20);
    if (ret)
        pdata->qrtable20 = 0x0B04;

    ret = of_property_read_u16(np, "qrtable30", &pdata->qrtable30);
    if (ret)
        pdata->qrtable30 = 0x0885;

    ret = of_property_read_u16(np, "dpacc", &pdata->dpacc);
    if (ret)
        pdata->dpacc = 0x0C80;

    ret = of_property_read_u16(np, "modelcfg", &pdata->modelcfg);
    if (ret)
        pdata->modelcfg = 0x8000;


    ret = of_property_read_u16(np, "relaxcfg", &pdata->relaxcfg);
    if (ret)
        pdata->relaxcfg = 0x2039;

    ret = of_property_read_u16(np, "config", &pdata->config);
    if (ret)
        pdata->config = 0x2210;

    ret = of_property_read_u16(np, "config2", &pdata->config2);
    if (ret)
        pdata->config2 = 0x0658;

    ret = of_property_read_u16(np, "fullsocthr", &pdata->fullsocthr);
    if (ret)
        pdata->fullsocthr = 0x5F05;

    ret = of_property_read_u16(np, "tgain", &pdata->tgain);
    if (ret)
        pdata->tgain = 0xEE56;

    ret = of_property_read_u16(np, "toff", &pdata->toff);
    if (ret)
        pdata->toff = 0x1DA4;

    ret = of_property_read_u16(np, "curve", &pdata->curve);
    if (ret)
        pdata->curve = 0x0025;

    prop = of_find_property(np, "model-data", NULL);
    if (prop && prop->length == MAX17055_TABLE_SIZE_IN_BYTES) {
        ret = of_property_read_u16_array(np, "model-data",
                        pdata->model_data,
                        MAX17055_TABLE_SIZE);
        if (ret)
            dev_warn(dev, "failed to get model_data %d\n", ret);
    }

    return pdata;
}

static const struct regmap_config max17055_regmap = {
    .reg_bits   = 8,
    .val_bits   = 16,
    .val_format_endian = REGMAP_ENDIAN_NATIVE,
};

static const struct power_supply_desc max17055_fg_desc = {
	.name           = "max17055_battery",
	.type           = POWER_SUPPLY_TYPE_BATTERY,
	.properties     = max17055_battery_props,
	.num_properties = ARRAY_SIZE(max17055_battery_props),
	.get_property   = max17055_get_property,
};

static int max17055_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    /*struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);*/
    struct max17055_priv *priv;
    struct power_supply_config psy_cfg = {};
    int ret;

    /*if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
        return -EIO;*/

    priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    if (client->dev.of_node)
        priv->pdata = max17055_parse_dt(&client->dev);
    else
        priv->pdata = client->dev.platform_data;

    priv->dev   = &client->dev;

    i2c_set_clientdata(client, priv);

    priv->client = client;
    priv->regmap = devm_regmap_init_i2c(client, &max17055_regmap);
    if (IS_ERR(priv->regmap))
        return PTR_ERR(priv->regmap);

    INIT_WORK(&priv->init_worker, max17055_init_worker);
    schedule_work(&priv->init_worker);

    psy_cfg.drv_data = priv;
    priv->battery = power_supply_register(&client->dev,
					  &max17055_fg_desc, &psy_cfg);

    if (IS_ERR(priv->battery)) {
    	ret = PTR_ERR(priv->battery);
	dev_err(&client->dev, "failed to register battery: %d\n", ret);
	goto err_supply;
    }
    
    if (client->irq) {
        ret = devm_request_threaded_irq(priv->dev, client->irq,
                        NULL,
                        max17055_irq_handler,
                        IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                        priv->battery->desc->name,
                        priv);
        if (ret) {
            dev_err(priv->dev, "Failed to request irq %d\n", client->irq);
            goto err_irq;
        }
        else {
            regmap_update_bits(priv->regmap, MAX17055_CONFIG_REG,
                        MAX17055_CONFIG_ALRT_EN, MAX17055_CONFIG_ALRT_EN);
        }
    }

    /* Create max17055 sysfs attributes */
    priv->attr_grp = &max17055_attr_group;
    ret = sysfs_create_group(&priv->dev->kobj, priv->attr_grp);
    if(ret){
        dev_err(priv->dev, "Failed to create attribute group [%d]\n", ret);
        priv->attr_grp = NULL;
        goto err_attr;
    }

    return 0;

err_irq:
    power_supply_unregister(priv->battery);
err_supply:
    cancel_work_sync(&priv->init_worker);
err_attr:
    sysfs_remove_group(&priv->dev->kobj, priv->attr_grp);
    return ret;
}

static int max17055_remove(struct i2c_client *client)
{
    struct max17055_priv *priv = i2c_get_clientdata(client);

    cancel_work_sync(&priv->init_worker);
    sysfs_remove_group(&priv->dev->kobj, priv->attr_grp);
    power_supply_unregister(priv->battery);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max17055_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);

    if (client->irq) {
        disable_irq(client->irq);
        enable_irq_wake(client->irq);
    }

    return 0;
}

static int max17055_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);

    if (client->irq) {
        disable_irq_wake(client->irq);
        enable_irq(client->irq);
    }

    return 0;
}

static SIMPLE_DEV_PM_OPS(max17055_pm_ops,
             max17055_suspend, max17055_resume);
#define MAX17055_PM_OPS (&max17055_pm_ops)
#else
#define MAX17055_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id max17055_match[] = {
    { .compatible = "maxim,max17055", },
    { },
};
MODULE_DEVICE_TABLE(of, max17055_match);

static const struct i2c_device_id max17055_id[] = {
    { "max17055", ID_MAX17055 },
    { },
};
MODULE_DEVICE_TABLE(i2c, max17055_id);

static struct i2c_driver max17055_i2c_driver = {
    .driver = {
        .name       = DRV_NAME,
        .of_match_table = of_match_ptr(max17055_match),
        .pm     = MAX17055_PM_OPS,
    },
    .probe      = max17055_probe,
    .remove     = max17055_remove,
    .id_table   = max17055_id,
};
module_i2c_driver(max17055_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kerem Sahin <kerem.sahin@maximintegrated.com>");
MODULE_AUTHOR("Mahir Ozturk <mahir.ozturk@maximintegrated.com>");
MODULE_DESCRIPTION("Maxim Max17055 Fuel Gauge driver");
