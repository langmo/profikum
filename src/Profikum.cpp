#include "Profikum.h"
#include "SerialConnection.h"
#include "Device.h"
#include "gsdmltools.h"
#include "standardconversions.h"
#include "profikum_com.h"
#include <cstdint>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cstdarg>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h> // inet_ntop
#include <unistd.h>

Profikum::Profikum() : logger{profinet::logging::CreateConsoleLogger<profinet::LogLevel::logInfo>()}
{

}

Profikum::~Profikum()
{

}

void Profikum::Log(profinet::LogLevel logLevel, const char* format, ...) noexcept
{
   if(!logger)
      return;
   va_list args;
   std::string message;

   va_start (args, format);
   message.resize (vsnprintf (0, 0, format, args));
   vsnprintf (&message[0], message.size () + 1, format, args);
   va_end (args);
   logger(logLevel, std::move(message));
}

bool Profikum::InitializeProfinet(const std::string_view& mainNetworkInterface)
{
    profinet.GetProperties().mainNetworkInterface = mainNetworkInterface;
    profinet::Device& device = profinet.GetDevice();

    device.properties.vendorName = "FH Technikum Wien";
    // ID of the device, which is unique for the given vendor.
    device.properties.deviceID = 0x0002;
    device.properties.deviceName = "Profikum";
    device.properties.deviceInfoText = "A small mobile robot controlled by Profinet.";
    device.properties.deviceProductFamily = "robots";
    // profinet name
    device.properties.stationName = "profikum";        
    device.properties.numSlots = 4;
    
    // Current software version of device.
    device.properties.swRevMajor = 0;
    device.properties.swRevMinor = 1;
    device.properties.swRevPatch = 1;
    // Current hardware version of device.
    device.properties.hwRevMajor = 1;
    device.properties.hwRevMinor = 0;

    /* Note: You need to read out the actual hardware serial number instead */
    device.properties.serialNumber = "000-000-000";

    /* GSDML tag: OrderNumber */
    device.properties.orderID = "123 444 555";

    /* GSDML tag: ModuleInfo / Name */
    device.properties.productName = "Profikum Robot";

    /* GSDML tag: MinDeviceInterval */
    profinet.GetProperties().cycleTimeUs= 8000;
    device.properties.minDeviceInterval = 32*32; /* 32 ms. One "unit" corresponds to 1/32 ms */
    device.properties.defaultMautype = 0x10; /* Copper 100 Mbit/s Full duplex */

    // Motor module
    auto motorModuleWithPlugInfo = device.modules.Create(0x00000040, 1);
    if(!motorModuleWithPlugInfo)
        return false;
    auto& [motorPlugInfo, motorModule]{*motorModuleWithPlugInfo};
    motorModule.properties.name = "Motors";
    motorModule.properties.infoText = "Moule allows control of the two motors of the Profikum bot.";
    profinet::Submodule* motorSubmodule = motorModule.submodules.Create(0x00000140);
    motorSubmodule->properties.name = "Motors submodule";
    motorModule.properties.infoText = "Submoule allows control of the two motors of the Profikum bot.";
    // Inputs
    auto leftSpeedSetCallback = [this](int16_t value) -> void
        {
            std::unique_lock lock{mutex};
            speedLeft = value;
        };
    profinet::Input* left = motorSubmodule->inputs.Create<int16_t, sizeof(int16_t)>(leftSpeedSetCallback);
    left->properties.description = "Speed of left motor, in mm/s. Max. is about +-60mm/s, but depends on the specific motor, battery levels, ...";

    auto rightSpeedSetCallback = [this](int16_t value) -> void
        {
            std::unique_lock lock{mutex};
            speedRight = value;
        };
    profinet::Input* right = motorSubmodule->inputs.Create<int16_t, sizeof(int16_t)>(rightSpeedSetCallback);
    right->properties.description = "Speed of right motor, in mm/s. Max. is about +-60mm/s, but depends on the specific motor, battery levels, ...";
    
    // IMU module
    auto imuModuleWithPlugInfo = device.modules.Create(0x00000041, std::vector<uint16_t>{2,3,4});
    if(!imuModuleWithPlugInfo)
        return false;
    auto& [imuPlugInfo, imuModule]{*imuModuleWithPlugInfo};
    imuModule.properties.name = "IMU sensors";
    imuModule.properties.infoText = "Moule allows reading of the different IMU sensors.";
    //Acceleration
    profinet::Submodule* accelerationSubmodule = imuModule.submodules.Create(0x00000151);
    accelerationSubmodule->properties.name = "Acceleration";
    accelerationSubmodule->properties.infoText = "Acceleration sensor";
    auto accelerationXGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return accelerationX;
        };
    profinet::Output* output = accelerationSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(accelerationXGetCallback);
    output->properties.description = "Acceleration in X direction";
    auto accelerationYGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return accelerationY;
        };
    output = accelerationSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(accelerationYGetCallback);
    output->properties.description = "Acceleration in Y direction";
    auto accelerationZGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return accelerationZ;
        };
    output = accelerationSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(accelerationZGetCallback);
    output->properties.description = "Acceleration in Z direction";
    // Gyro 
    profinet::Submodule* gyroSubmodule = imuModule.submodules.Create(0x00000152);
    gyroSubmodule->properties.name = "Gyro";
    gyroSubmodule->properties.infoText = "Gyro sensor";
    auto gyroXGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return gyroX;
        };
    output = gyroSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(gyroXGetCallback);
    output->properties.description = "Gyro in X direction";
    auto gyroYGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return gyroY;
        };
    output = gyroSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(gyroYGetCallback);
    output->properties.description = "Gyro in Y direction";
    auto gyroZGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return gyroZ;
        };
    output = gyroSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(gyroZGetCallback);
    output->properties.description = "Gyro in Z direction";
    // Magnetometer 
    profinet::Submodule* magnetometerSubmodule = imuModule.submodules.Create(0x00000153);
    magnetometerSubmodule->properties.name = "Magnetometer";
    magnetometerSubmodule->properties.infoText = "Magnetormeter sensor";
    auto magnetometerXGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return magnetometerX;
        };
    output = magnetometerSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(magnetometerXGetCallback);
    output->properties.description = "Magnetometer in X direction";
    auto magnetometerYGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return magnetometerY;
        };
    output = magnetometerSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(magnetometerYGetCallback);
    output->properties.description = "Magnetometer in Y direction";
    auto magnetometerZGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return magnetometerZ;
        };
    output = magnetometerSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(magnetometerZGetCallback);
    output->properties.description = "Magnetometer in Z direction";

    // Ultrasound module
    auto ultrasoundModuleWithPlugInfo = device.modules.Create(0x00000042, std::vector<uint16_t>{2,3,4});
    if(!ultrasoundModuleWithPlugInfo)
        return false;
    auto& [ultrasoundPlugInfo, ultrasoundModule]{*ultrasoundModuleWithPlugInfo};
    ultrasoundModule.properties.name = "Ultrasound distance sensor";
    ultrasoundModule.properties.infoText = "Distances are measured in mm.";
    profinet::Submodule* ultrasoundSubmodule = ultrasoundModule.submodules.Create(0x00000142);
    ultrasoundSubmodule->properties.name = "Ultrasound distance sensor";
    ultrasoundSubmodule->properties.infoText = "Distances are measured in mm.";
    //Outputs
    //distance in mm
    auto rightDistanceGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return rightDistance;
        };
    output = ultrasoundSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(rightDistanceGetCallback);
    output->properties.description = "Right distance measured by ultrasound sensor in mm.";

    auto leftDistanceGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return leftDistance;
        };
    output = ultrasoundSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(leftDistanceGetCallback);
    output->properties.description = "Left distance measured by ultrasound sensor in mm.";

    // Encoder module
    auto encoderModuleWithPlugInfo = device.modules.Create(0x00000043, std::vector<uint16_t>{2,3,4});
    if(!encoderModuleWithPlugInfo)
        return false;
    auto& [encoderPlugInfo, encoderModule]{*encoderModuleWithPlugInfo};
    encoderModule.properties.name = "Encoder";
    encoderModule.properties.infoText = "Encoder for the two motors.";
    profinet::Submodule* encoderSubmodule = encoderModule.submodules.Create(0x00000143);
    encoderSubmodule->properties.name = "Motor Encoder";
    encoderSubmodule->properties.infoText = "Encoder for the two motors.";
    //Outputs total travelled distance
    auto leftMillimetersGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return leftEncoderMillimeters;
        };
    output = encoderSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(leftMillimetersGetCallback);
    output->properties.description = "Total distance, in mm, travelled by the left motor, as measured by the encoder.";
    auto rightMillimetersGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return rightEncoderMillimeters;
        };
    output = encoderSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(rightMillimetersGetCallback);
    output->properties.description = "Total distance, in mm, travelled by the right motor, as measured by the encoder.";
    
    // outputs speed
    auto leftSpeedGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return leftEncoderMillimetersPerSecond;
        };
    output = encoderSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(leftSpeedGetCallback);
    output->properties.description = "Speed of left wheel, in mm/s, as measured by encoder.";
    auto rightSpeedGetCallback = [this]() -> int16_t
        {
            std::unique_lock lock{mutex};
            return rightEncoderMillimetersPerSecond;
        };
    output = encoderSubmodule->outputs.Create<int16_t, sizeof(int16_t)>(rightSpeedGetCallback);
    output->properties.description = "Speed of right wheel, in mm/s, as measured by encoder.";

    profinetInitialized = true;
    return true;
}

bool Profikum::IsInterfaceOnline(std::string interface)
{
    struct ifreq ifr;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
   {
      return false;
   }
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(sock, SIOCGIFADDR, &ifr) < 0) 
    {
        close(sock);
        return false;
    }

    struct sockaddr_in* addr = (struct sockaddr_in*)&ifr.ifr_addr;
    uint32_t ipRaw = ntohl (addr->sin_addr.s_addr);
    close(sock);
    if(ipRaw == 0)
        return false;
    else
        return true;
}

bool Profikum::StartProfinet()
{
    if(!profinetInitialized)
    {
        Log(profinet::logError, "Profinet not yet initialized.");
        return false;
    }
    while(!IsInterfaceOnline(profinet.GetProperties().mainNetworkInterface))
    {
        Log(profinet::logWarning, "Network interface %s not yet running. Retrying in 2s...", profinet.GetProperties().mainNetworkInterface.c_str());
        std::this_thread::sleep_for (std::chrono::seconds(2));
    }
    profinetInstance = profinet.Initialize(logger);
    if(!profinetInstance)
    {
        Log(profinet::logError, "Could not initialize Profinet.");
        return false;
    }
    return profinetInstance->Start();
}

bool Profikum::SendSerial(SerialConnection& serialConnection)
{
    int16_t speedLeftTemp;
    int16_t speedRightTemp;
    {
        std::unique_lock lock{mutex};
        speedLeftTemp = speedLeft;
        speedLeft = INT16_MAX;
        speedRightTemp = speedRight;
        speedRight = INT16_MAX;

    }
    uint8_t buffer[4];
    if(speedLeftTemp != INT16_MAX)
    {
        // Left motor
        buffer[0] = profikum::com::FromProfikumInput(profikum::com::ProfikumInput::leftMotorSetSpeed);
        profinet::toProfinet<int16_t, sizeof(int16_t)>(buffer+1, 2, speedLeftTemp);
        
        buffer[3] = profikum::com::stopByte;
        if(!serialConnection.Send(buffer, 4))
        {
            return false;
        }
    }

    if(speedRightTemp != INT16_MAX)
    {
        // Right motor
        buffer[0] = profikum::com::FromProfikumInput(profikum::com::ProfikumInput::rightMotorSetSpeed);
        profinet::toProfinet<int16_t, sizeof(int16_t)>(buffer+1, 2, speedRightTemp);
        
        if(!serialConnection.Send(buffer, 4))
        {
            return false;
        }
    }
    return true;
}
bool Profikum::ReceiveSerial(SerialConnection& serialConnection)
{
    static uint8_t buffer[4];
    static size_t bufferPos{0};
    std::size_t numBytes{};
    while(true)
    {
        serialConnection.Read(buffer+bufferPos, 1, &numBytes);
        if(numBytes <= 0)
            break;
        else if(bufferPos == 0 && buffer[0] != profikum::com::stopByte)
            continue;
        bufferPos++;
        if(bufferPos == 4)
        {
            int16_t val;
            if(profinet::fromProfinet<int16_t, sizeof(int16_t)>(buffer+2, 2, &val))
                InterpretCommand(profikum::com::ToProfikumOutput(buffer[1]), val);
            else
                InterpretCommand(profikum::com::ProfikumOutput::error, 0);
            bufferPos=0;
        }
    }
    return true;
}
void Profikum::RunController(const std::string& serialPort)
{
    SerialConnection serialConnection{};
    while(!serialConnection.Connect(serialPort))
    {
        Log(profinet::logError, "Could not establish serial connection. Retrying in 2s...");
        std::this_thread::sleep_for (std::chrono::seconds(2));
    }
    while(true)
    {
        SendSerial(serialConnection);
        ReceiveSerial(serialConnection);
        // Sleep for 5ms, just to don't block the profinet stack thread completely.
        std::this_thread::sleep_for (std::chrono::milliseconds(5));
    }
}

bool Profikum::InterpretCommand(profikum::com::ProfikumOutput command, int16_t value)
{
    std::unique_lock lock{mutex};
    switch(command)
    {
        case profikum::com::ProfikumOutput::accelerationX:
            accelerationX = value;
            return true;
        case profikum::com::ProfikumOutput::accelerationY:
            accelerationY = value;
            return true;
        case profikum::com::ProfikumOutput::accelerationZ:
            accelerationZ = value;
            return true;
        case profikum::com::ProfikumOutput::gyroX:
            gyroX = value;
            return true;
        case profikum::com::ProfikumOutput::gyroY:
            gyroY = value;
            return true;
        case profikum::com::ProfikumOutput::gyroZ:
            gyroZ = value;
            return true;
        case profikum::com::ProfikumOutput::magnetometerX:
            magnetometerX = value;
            return true;
        case profikum::com::ProfikumOutput::magnetometerY:
            magnetometerY = value;
            return true;
        case profikum::com::ProfikumOutput::magnetometerZ:
            magnetometerZ = value;
            return true;
        case profikum::com::ProfikumOutput::rightUltrasoundDistance:
            rightDistance = value;
            return true;
        case profikum::com::ProfikumOutput::leftUltrasoundDistance:
            leftDistance = value;
            return true;
        case profikum::com::ProfikumOutput::leftEncoderMillimeters:
            leftEncoderMillimeters = value;
            return true;
        case profikum::com::ProfikumOutput::rightEncoderMillimeters:
            rightEncoderMillimeters = value;
            return true;
        case profikum::com::ProfikumOutput::leftEncoderMillimetersPerSecond:
            leftEncoderMillimetersPerSecond = value;
            return true;
        case profikum::com::ProfikumOutput::rightEncoderMillimetersPerSecond:
            rightEncoderMillimetersPerSecond = value;
            return true;
        default:
            return false;
    }
}

bool Profikum::ExportGDSML(const char* folderName) const
{
    std::string folder{folderName};
    return profinet::gsdml::CreateGsdml(profinet, folder);
}
