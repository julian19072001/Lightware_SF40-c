#ifndef _SF40_H_
#define _SF40_H_

    #include <stdio.h>
    #include <stdint.h>
    #include <stdbool.h>
    #include <string.h>

    #include "../RPI-serial/RPIserial.h"

    #define MAX_RESPONSE_SIZE 1028
    #define STARTBIT 0XAA

    #define MODEL_NUMBER        "SF40"
    #define LIDAR_VOLTAGE(counts)    ((uint32_t)counts / 4095.0) * 2.048 * 5.7


    // Commands for the Lidar
    #define LIDAR_PRODUCT_NAME      0
    #define LIDAR_HARDWARE_VERSION  1    
    #define LIDAR_FIRMWARE_VERSION  2  
    #define LIDAR_SERIAL_NUMBER     3
    #define LIDAR_USER_DATA         9
    #define LIDAR_TOKEN             10
    #define LIDAR_SAVE_PARAMETERS   12
    #define LIDAR_RESET             14
    #define LIDAR_INCOMING_VOLTAGE  20    
    #define LIDAR_STREAM            30
    #define LIDAR_DISTANCE_OUTPUT   48 
    #define LIDAR_LASER_FIRING      50
    #define LIDAR_TEMPRATURE        55
    #define LIDAR_BAUD_RATE         90
    #define LIDAR_DISTANCE          105
    #define LIDAR_MOTOR_STATE       106
    #define LIDAR_MOTOR_VOLTAGE     107
    #define LIDAR_OUTPUT_RATE       108
    #define LIDAR_FORWARD_OFFSET    109
    #define LIDAR_REVOLUTIONS       110
    #define LIDAR_ALARM_STATE       111
    #define LIDAR_ALARM_1           112
    #define LIDAR_ALARM_2           113
    #define LIDAR_ALARM_3           114
    #define LIDAR_ALARM_4           115
    #define LIDAR_ALARM_5           116
    #define LIDAR_ALARM_6           117
    #define LIDAR_ALARM_7           118

    // Lidar baudrates
    #define LIDAR_115K2             4
    #define LIDAR_230K4             5
    #define LIDAR_460K8             6
    #define LIDAR_921K6             7

    // Lidar output rates in point per second
    #define LIDAR_20010_PPS         0
    #define LIDAR_10005_PPS         1
    #define LIDAR_6670_PPS          2
    #define LIDAR_2001_PPS          3

    // Motor states
    #define MOTOR_PRE_STARTUP       1
    #define MOTOR_WAIT_ON_REVS      2
    #define MOTOR_NORMAL            3
    #define MOTOR_ERROR             4

    typedef struct streamOutput{
        uint8_t     alarmState;             // State of each alarm as described in Alarm state [111]
        uint16_t    pps;                    // Points per second
        int16_t     forwardOffset;          // Orientation offset as described in Forward offset [109]
        int16_t     motorVoltage;           // Motor voltage as described in Motor voltage [107]
        uint8_t     revolutionIndex;        // Increments as each new revolution begins. Note that this value wraps to 0 after 255.
        uint16_t    pointTotal;             // Total number of points this revolution.
        uint16_t    pointCount;             // Number of points in this packet.
        uint16_t    pointStartIndex;        // Index of the first point in this packet.
        int16_t     pointDistances[200];    // Array of distances [cm] for each point.
    }streamOutput_t;

    typedef struct readDistance{
        int16_t     averageDistance;    // Average distance [cm]
        int16_t     closestDistance;    // Closest distance [cm]
        int16_t     furthestDistance;   // Furthest distance [cm]
        int16_t     angle;              // Angle to closest distance [10ths of a degree]
        uint32_t    calculationTime;    // Calculation time [us]
    }readDistance_t;

    typedef struct writeDistance{
        int16_t     direction;          // Direction [degrees]
        int16_t     width;              // Angular width [degrees]
        int16_t     minimumDistance;    // Minimum distance [cm]
    }writeDistance_t;

    typedef union{
        uint8_t byte;
        struct{
            uint8_t alarm1 : 1;
            uint8_t alarm2 : 1;
            uint8_t alarm3 : 1;
            uint8_t alarm4 : 1;
            uint8_t alarm5 : 1;
            uint8_t alarm6 : 1;
            uint8_t alarm7 : 1;
            uint8_t alarmAny : 1;
        };
    } alarms;

    typedef struct alarm{
        uint8_t enabled;        // 1 means enabled, 0 means disabled.
        int16_t direction;      // Primary direction in degrees.
        int16_t width;          // Angular width in degrees around the primary direction.
        int16_t distance;       // Distance at which alarm is triggered.
    }alarm_t;

#endif