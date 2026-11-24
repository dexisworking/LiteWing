/**
 * LiteWing ESP32 Gesture Glove Controller - Hardware Configuration
 */

#ifndef __GLOVE_CONFIG_H__
#define __GLOVE_CONFIG_H__

/* LiteWing Drone Wi-Fi SoftAP Configuration */
#define DRONE_SSID_PREFIX      "LiteWing"
#define DRONE_WIFI_PASSWORD    "12345678"
#define DRONE_IP_ADDRESS       "192.168.43.42"
#define DRONE_UDP_PORT         2391

/* Transmission Cadence */
#define GLOVE_SEND_INTERVAL_MS 20      /* 50 Hz streaming rate */

/* Pin Assignments for ESP32 DevKit V1 */
#define PIN_I2C_SDA            21      /* MPU6050 SDA */
#define PIN_I2C_SCL            22      /* MPU6050 SCL */

#define PIN_DEADMAN_ADC        34      /* LDR or Flex Sensor (ADC1_CH6) */
#define PIN_EMERGENCY_BTN      4       /* Emergency Stop Button (Active LOW) */
#define PIN_MODE_BTN           5       /* Mode Toggle Button (Active LOW) */
#define PIN_STATUS_LED         2       /* Onboard Status Indicator LED */

/* Deadman Threshold */
#define DEADMAN_ADC_THRESHOLD  1200    /* Raw ADC threshold for hand engagement (0-4095) */
#define DEADMAN_INVERT         false   /* true if covered LDR = lower voltage */

/* Attitude Angles & Deadzones */
#define GESTURE_DEADZONE_DEG   3.0f    /* ±3 deg neutral deadzone around level hand */
#define GESTURE_MAX_ANGLE_DEG  25.0f   /* Max roll/pitch angle sent to drone */

/* Throttle Mapping */
#define THROTTLE_MIN           0
#define THROTTLE_HOVER_EST     36000   /* Approximate hover thrust */
#define THROTTLE_MAX           55000   /* Max allowable gesture thrust */

/* Complementary Filter Constants */
#define COMP_FILTER_ALPHA      0.96f   /* Gyro weight vs Accel weight */

#endif /* __GLOVE_CONFIG_H__ */
