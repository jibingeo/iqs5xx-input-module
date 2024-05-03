/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT azoteq_iqs5xx

#include "input_iqs5xx.h"
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(azoteq_iqs5xx, CONFIG_ZMK_LOG_LEVEL);

// Default config
struct iqs5xx_reg_config iqs5xx_reg_config_default() {
  struct iqs5xx_reg_config regconf;

  regconf.activeRefreshRate = 1;
  regconf.idleRefreshRate = 50;
  regconf.singleFingerGestureMask = GESTURE_SINGLE_TAP;
  regconf.multiFingerGestureMask = GESTURE_SCROLLG | GESTURE_TWO_FINGER_TAP;
  regconf.tapTime = 250;
  regconf.tapDistance = 50;
  regconf.touchMultiplier = 0;
  regconf.debounce = 0;
  regconf.i2cTimeout = 4;
  regconf.filterSettings =
      MAV_FILTER /*| IIR_FILTER | IIR_SELECT static mode */;
  regconf.filterDynBottomBeta = 22;
  regconf.filterDynLowerSpeed = 19;
  regconf.filterDynUpperSpeed = 140;

  regconf.xyConfig = PALM_DETECT | SWITCH_XY_AXIS | FLIP_Y | FLIP_X;

  regconf.initScrollDistance = 25;

  return regconf;
}

/**
 * @brief Read from the iqs550 chip via i2c
 * example: iqs5xx_seq_read(dev, GestureEvents0_adr, buffer, 44)
 *
 * @param dev Pointer to device driver MASTER
 * @param start start address for reading
 * @param  pointer to buffer to be read into
 * @param len number of bytes to read
 * @return int
 */
static int iqs5xx_seq_read(const struct device *dev, const uint16_t start,
                           uint8_t *read_buf, const uint8_t len) {
  const struct iqs5xx_data *data = dev->data;
  uint16_t nstart = (start << 8) | (start >> 8);
  while (1) {
    int err = i2c_write_read(data->i2c, AZOTEQ_IQS5XX_ADDR, &nstart,
                             sizeof(nstart), read_buf, len);
    if (!err)
      break;
    LOG_ERR("read error");
  }
  return 0;
}

/**
 * @brief Write to the iqs550 chip via i2c
 * example: iqs5xx_write(dev, GestureEvents0_adr, buffer, 44)
 * @param dev Pointer to device driver MASTER
 * @param start address of the i2c slave
 * @param buf Buffer to be written
 * @param len number of bytes to write
 * @return int
 */
static int iqs5xx_write(const struct device *dev, const uint16_t start_addr,
                        const uint8_t *buf, uint32_t num_bytes) {

  const struct iqs5xx_data *data = dev->data;

  uint8_t addr_buffer[2];
  struct i2c_msg msg[2];

  addr_buffer[1] = start_addr & 0xFF;
  addr_buffer[0] = start_addr >> 8;
  msg[0].buf = addr_buffer;
  msg[0].len = 2U;
  msg[0].flags = I2C_MSG_WRITE;

  msg[1].buf = (uint8_t *)buf;
  msg[1].len = num_bytes;
  msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

  while (1) {
    int err = i2c_transfer(data->i2c, msg, 2, AZOTEQ_IQS5XX_ADDR);
    if (!err)
      break;
    LOG_ERR("write error");
  }
  return 0;
}

/**
 * @brief Read data from IQS5XX
 */
static int iqs5xx_sample_fetch(const struct device *dev) {
  uint8_t buffer[44];

  struct iqs5xx_data *data = dev->data;

  int res = iqs5xx_seq_read(dev, GestureEvents0_adr, buffer, 44);
  iqs5xx_write(dev, END_WINDOW, 0, 1);
  if (res < 0) {
    LOG_ERR("\ntrackpad res: %d", res);
    return res;
  }

  // Gestures
  data->raw_data.gestures0 = buffer[0];
  data->raw_data.gestures1 = buffer[1];
  // System info
  data->raw_data.system_info0 = buffer[2];
  data->raw_data.system_info1 = buffer[3];
  // Number of fingers
  data->raw_data.finger_count = buffer[4];
  // Relative X position
  data->raw_data.rx = buffer[5] << 8 | buffer[6];
  // Relative Y position
  data->raw_data.ry = buffer[7] << 8 | buffer[8];

  // Fingers
  for (int i = 0; i < 5; i++) {
    const int p = 9 + (7 * i);
    // Absolute X
    data->raw_data.fingers[i].ax = (buffer[p + 0] << 8 | buffer[p + 1]);
    // Absolute Y
    data->raw_data.fingers[i].ay = (buffer[p + 2] << 8 | buffer[p + 3]);
    // Touch strength
    data->raw_data.fingers[i].strength = buffer[p + 4] << 8 | buffer[p + 5];
    // Area
    data->raw_data.fingers[i].area = buffer[p + 6];
  }
  // LOG_INF("trackpad result %d", data ->raw_data.finger_count);
  if (data->raw_data.finger_count) {
    // LOG_INF("trackpad result %d", data ->raw_data.finger_count);
    // LOG_INF("trackpad result %d,%d",
    // data->raw_data.fingers[0].ax,data->raw_data.fingers[0].ay);
    //  LOG_INF("trackpad result %d, %d", data->raw_data.fingers[0].strength,
    //  data->raw_data.fingers[0].area);
  }

  return 0;
}

static void set_int(const struct device *dev, const bool en) {
  const struct iqs5xx_config *config = dev->config;
  int err = gpio_pin_interrupt_configure(config->dr_port, config->dr_pin,
                                         en ? GPIO_INT_EDGE_TO_ACTIVE
                                            : GPIO_INT_DISABLE);
  if (err) {
    LOG_ERR("can't set interrupt");
  }
}

static void iqs5xx_thread(void *arg, void *unused2, void *unused3) {

  const struct device *dev = arg;
  ARG_UNUSED(unused2);
  ARG_UNUSED(unused3);
  struct iqs5xx_data *data = dev->data;
  /* Initialize the event */

  // Initialize device registers - may be overwritten later in trackpad.c
  k_sem_take(&data->gpio_sem, K_FOREVER);
  // Disable interrupt
  set_int(dev, false);
  struct iqs5xx_reg_config const iqs5xx_registers = iqs5xx_reg_config_default();
  int err = iqs5xx_registers_init(dev, &iqs5xx_registers);
  if (err) {
    LOG_ERR("Failed to initialize IQS5xx registers!\r\n");
  }

  while (1) {
    set_int(dev, true);
    k_sem_take(&data->gpio_sem, K_FOREVER);
    set_int(dev, false);
    iqs5xx_sample_fetch(dev);

    bool hasGesture = false;
    switch (data->raw_data.gestures0) {
    case GESTURE_SINGLE_TAP:
      input_report_key(dev, INPUT_BTN_0, 1, true, K_NO_WAIT);
      input_report_key(dev, INPUT_BTN_0, 0, true, K_NO_WAIT);
      hasGesture = true;
      break;
    default:
    }
    switch (data->raw_data.gestures1) {
    case GESTURE_TWO_FINGER_TAP:
      input_report_key(dev, INPUT_BTN_1, 1, true, K_NO_WAIT);
      input_report_key(dev, INPUT_BTN_1, 0, true, K_NO_WAIT);
      hasGesture = true;
      break;
    case GESTURE_SCROLLG:
      input_report_rel(dev, INPUT_REL_WHEEL, data->raw_data.ry, false,
                       K_FOREVER);
      input_report_rel(dev, INPUT_REL_HWHEEL, data->raw_data.rx, true,
                       K_FOREVER);
      hasGesture = true;
      break;
    default:
    }
    if (!hasGesture) {
      input_report_rel(dev, INPUT_REL_X, data->raw_data.rx, false, K_FOREVER);
      input_report_rel(dev, INPUT_REL_Y, data->raw_data.ry, true, K_FOREVER);
    }
    // k_msgq_put(&my_msgq, &edata, K_FOREVER);
    LOG_INF("Read %i %i %i %i %i", data->raw_data.fingers[0].ax,
            data->raw_data.fingers[0].ay, data->raw_data.rx, data->raw_data.ry,
            data->raw_data.gestures0);
  }
}

/**
 * @brief Called when data ready pin goes active. Releases the semaphore
 * allowing thread to run.
 *
 * @param dev
 * @param cb
 * @param pins
 */
static void iqs5xx_callback(const struct device *dev, struct gpio_callback *cb,
                            uint32_t pins) {
  struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, dr_cb);
  k_sem_give(&data->gpio_sem);
}

/**
 * @brief Sets registers to initial values
 *
 * @param dev
 * @return >0 if error
 */
int iqs5xx_registers_init(const struct device *dev,
                          const struct iqs5xx_reg_config *config) {
  uint8_t buf;
  // 16 or 32 bit values must be swapped to big endian
  uint8_t wbuff[16];

  // Reset device
  buf = RESET_TP;
  iqs5xx_write(dev, SystemControl1_adr, &buf, 1);
  iqs5xx_write(dev, END_WINDOW, 0, 1);
  k_msleep(200);

  int err = 0;

  // Set active refresh rate
  *((uint16_t *)wbuff) = SWPEND16(config->activeRefreshRate);
  err |= iqs5xx_write(dev, ActiveRR_adr, wbuff, 2);

  // Set idle refresh rate
  *((uint16_t *)wbuff) = SWPEND16(config->idleRefreshRate);
  err |= iqs5xx_write(dev, IdleRR_adr, wbuff, 2);

  // Set single finger gestures
  err |= iqs5xx_write(dev, SFGestureEnable_adr,
                      &config->singleFingerGestureMask, 1);
  // Set multi finger gestures
  err |= iqs5xx_write(dev, MFGestureEnable_adr, &config->multiFingerGestureMask,
                      1);

  *((uint16_t *)wbuff) = (uint16_t)SWPEND16(768);
  err |= iqs5xx_write(dev, XResolution_adr, wbuff, 2);

  *((uint16_t *)wbuff) = (uint16_t)SWPEND16(512);
  err |= iqs5xx_write(dev, YResolution_adr, wbuff, 2);

  buf = 0;
  err |= iqs5xx_write(dev, StationaryTouchThr_adr, &buf, 1);
  // Set tap time
  *((uint16_t *)wbuff) = SWPEND16(config->tapTime);
  err |= iqs5xx_write(dev, TapTime_adr, wbuff, 2);

  // Set tap distance
  *((uint16_t *)wbuff) = SWPEND16(config->tapDistance);
  err |= iqs5xx_write(dev, TapDistance_adr, wbuff, 2);

  // Set touch multiplier
  err |= iqs5xx_write(dev, GlobalTouchSet_adr, &config->touchMultiplier, 1);

  // Set debounce settings
  err |= iqs5xx_write(dev, ProxDb_adr, &config->debounce, 1);
  err |= iqs5xx_write(dev, TouchSnapDb_adr, &config->debounce, 1);

  // wbuff[0] = ND_ENABLE;
  wbuff[0] = ND_ENABLE;
  // Set noise reduction
  err |= iqs5xx_write(dev, HardwareSettingsA_adr, wbuff, 1);

  // Set i2c timeout
  err |= iqs5xx_write(dev, I2CTimeout_adr, &config->i2cTimeout, 1);

  // Set filter settings
  err |= iqs5xx_write(dev, FilterSettings0_adr, &config->filterSettings, 1);

  // err |= iqs5xx_write(dev, XYStaticBeta_adr, &config->filterStatBottomBeta,
  // 1); LOG_ERR("DynamicBottomBeta_adr: %i", err);

  err |=
      iqs5xx_write(dev, DynamicBottomBeta_adr, &config->filterDynBottomBeta, 1);

  err |=
      iqs5xx_write(dev, DynamicLowerSpeed_adr, &config->filterDynLowerSpeed, 1);

  err |= iqs5xx_write(dev, XYConfig0_adr, &config->xyConfig, 1);

  *((uint16_t *)wbuff) = SWPEND16(config->filterDynUpperSpeed);
  err |= iqs5xx_write(dev, DynamicUpperSpeed_adr, wbuff, 2);

  // Set initial scroll distance
  *((uint16_t *)wbuff) = SWPEND16(config->initScrollDistance);
  err |= iqs5xx_write(dev, ScrollInitDistance_adr, wbuff, 2);

  *((uint16_t *)wbuff) = SWPEND16(100);
  err |= iqs5xx_write(dev, SwipeInitTime_adr, wbuff, 2);
  *((uint16_t *)wbuff) = SWPEND16(100);
  err |= iqs5xx_write(dev, SwipeInitDistance_adr, wbuff, 2);

  iqs5xx_write(dev, END_WINDOW, 0, 1);

  if (err)
    LOG_ERR("initialize error %d", err);
  else
    LOG_INF("initialized");
  return err;
}

static int iqs5xx_init(const struct device *dev) {
  struct iqs5xx_data *data = dev->data;
  const struct iqs5xx_config *config = dev->config;
  if (!device_is_ready(data->i2c)) {
    LOG_WRN("i2c bus not ready!");
    return -EINVAL;
  }
  // Blocking semaphore as a flag for sensor read
  k_sem_init(&data->gpio_sem, 0, 1);

  int err = 0;
  // Configure data ready pin
  gpio_pin_configure(config->dr_port, config->dr_pin,
                     GPIO_INPUT | config->dr_flags);
  // Initialize interrupt callback
  gpio_init_callback(&data->dr_cb, iqs5xx_callback, BIT(config->dr_pin));
  // Add callback
  err |= gpio_add_callback(config->dr_port, &data->dr_cb);
  // Configure data ready interrupt
  set_int(dev, true);

  return err;
}

#define IQS5XX_INST(n)                                                         \
  static struct iqs5xx_data iqs5xx_data_##n = {                                \
      .i2c = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(n)))};                           \
  static const struct iqs5xx_config iqs5xx_config_##n = {                      \
      .dr_port = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(n), dr_gpios)),        \
      .dr_pin = DT_INST_GPIO_PIN(n, dr_gpios),                                 \
      .dr_flags = DT_INST_GPIO_FLAGS(n, dr_gpios),                             \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, iqs5xx_init, NULL, &iqs5xx_data_##n,                \
                        &iqs5xx_config_##n, POST_KERNEL,                       \
                        CONFIG_INPUT_INIT_PRIORITY, NULL);                     \
  K_THREAD_DEFINE(thread1, 1024, iqs5xx_thread, DEVICE_DT_GET(DT_DRV_INST(n)), \
                  NULL, NULL, K_PRIO_COOP(10), 0, 0);                          \

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INST)
