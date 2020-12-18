#ifndef BOARD_H_
#define BOARD_H_

#define FW_VERSION 0x0100 // upper byte is major, lower is minor

#define MAGNETIC_DECLINATION 4.26f
#define BOOST_EN  PB6
#define UART_INH  -1
#define UART_SEL_A  -1
#define UART_SEL_B  -1
#define DRIVER_EN  PA3
#define ENABLE_DRIVER  LOW
#define DISABLE_DRIVER  HIGH
#define LED_RED PH0
#define BUTTON PA5
#define CALIBRATION_OFFSET 0x0100 //Needed for writing cal values in eeprom
#define OLED_MPU_I2C_EN PA0
#define BAT_MON PA4

float static input_calib[]={
    2.016,
    2.016,
    1.5685,
    1.413666667,
    1.32975,
    1.28,
    1.245,
    1.22,
    1.20125,
    1.186666667,
    1.175,
    1.165454545,
    1.156666667,
    1.15,
    1.143571429,
    1.138666667,
    1.133125,
    1.129411765,
    1.125555556,
    1.122105263,
    1.1195,
    1.116666667,
    1.113636364,
    1.11173913,
    1.109583333,
    1.1072,
    1.105384615,
    1.102962963,
    1.101428571,
    1.10215353,
    1.100928681,
    1.099782854
};

#define GPS_EN PB5
#define BAT_MON_CALIB 1.14

#endif
