#ifndef PROFIKUM_H
#define PROFIKUM_H

#pragma once

#include "Profinet.h"
#include "profikum_com.h"

class SerialConnection;

class Profikum final
{
public:
    Profikum();
    ~Profikum();
    
    Profikum (const Profikum&) = delete;
    Profikum& operator= (const Profikum&) = delete;

    bool InitializeProfinet(const std::string_view& mainNetworkInterface="wlan0");
    bool StartProfinet();
    bool ExportGDSML(const char* filename) const;
    void RunController(const std::string& serialPort="/dev/ttyAMA0");
private:
    profinet::LoggerType logger;
    void Log(profinet::LogLevel logLevel, const char* format, ...) noexcept;    
    bool profinetInitialized{false};
    profinet::Profinet profinet;
    
    //Outputs
    volatile int16_t speedLeft{0};
    volatile int16_t speedRight{0};
    
    // Inputs
    //Acceleration
    volatile int16_t accelerationX{0};
    volatile int16_t accelerationY{0};
    volatile int16_t accelerationZ{0};
    // Gyro 
    volatile int16_t gyroX{0};
    volatile int16_t gyroY{0};
    volatile int16_t gyroZ{0};
    // Magnetometer 
    volatile int16_t magnetometerX{0};
    volatile int16_t magnetometerY{0};
    volatile int16_t magnetometerZ{0};
    // ultrasound
    volatile int16_t rightDistance{-1};
    volatile int16_t leftDistance{-1};
    // encoder
    volatile int16_t leftEncoderCounts{0};
    volatile int16_t rightEncoderCounts{0};
    volatile int16_t leftEncoderCountsPerSecond{0};
    volatile int16_t rightEncoderCountsPerSecond{0};

    // Parameters encoder
    static constexpr uint16_t countsPerRotation{910};
    static constexpr uint16_t wheelRadius_mm{19};

    std::unique_ptr<profinet::ProfinetControl> profinetInstance;

    bool SendSerial(SerialConnection& serialConnection);
    bool ReceiveSerial(SerialConnection& serialConnection);
    bool InterpretCommand(profikum::com::ProfikumOutput command, int16_t value);

    static bool IsInterfaceOnline(std::string interface);
};

#endif
