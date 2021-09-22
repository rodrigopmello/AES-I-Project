#pragma once

// EPOS Smart Transducer Declarations

#include <smartdata.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <unistd.h>
#include <string>
#include <stdlib.h>

#include <sstream>
#include <bits/stdc++.h>




#define _UTIL




template<unsigned long _UNIT>
class Transducer: public SmartData, public Observed
{
public:
    static const unsigned long UNIT = _UNIT;

    enum : unsigned int { SENSOR = 1 << 0, ACTUATOR = 1 << 1 };
    typedef unsigned int Type;
    static const Type TYPE = SENSOR | ACTUATOR;

    typedef typename Unit::Get<_UNIT>::Type Value;

    typedef _UTIL::Observer Observer;
    typedef _UTIL::Observed Observed;

protected:
    Transducer() {}

public:
    virtual ~Transducer() {}

    virtual Value sense() = 0;
    virtual void actuate(const Value & value) {}

    Power_Mode power() { return Power_Mode::FULL; }
    void power(const Power_Mode & mode) {}
};

class Dummy_Transducer: public Transducer<SmartData::Unit::I64>
{
    friend Responsive_SmartData<Dummy_Transducer>;

public:
    static const bool active = true;
    static const Uncertainty UNCERTAINTY = UNKNOWN;
    static const Type TYPE = SENSOR | ACTUATOR;

public:
    Dummy_Transducer(const Device_Id & dev): _value(0) {
        if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }
        
        memset(&servaddr, 0, sizeof(servaddr));
        memset(&cliaddr, 0, sizeof(cliaddr));

        // Filling server information
        servaddr.sin_family    = AF_INET; // IPv4
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(5050);

        // Bind the socket with the server address
        if ( bind(sockfd, (const struct sockaddr *)&servaddr, 
                sizeof(servaddr)) < 0 )
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }




    }

    std::string hexToASCII(std::string hex)
    {
        // initialize the ASCII code string as empty.
        std::string ascii = "";
        for (size_t i = 0; i < hex.length(); i += 2)
        {
            // extract two characters from hex string
            std::string part = hex.substr(i, 2);
            
            // change it into base 16 and
            // typecast as the character
            char ch = std::stoul(part, nullptr, 16);
            
            // add this char to final ASCII string
            ascii += ch;
        }
            return ascii;
    }

    
    virtual Value sense() { 

        char buffer[12];        
        int n = 0;

        socklen_t len;
    
        len = sizeof(cliaddr);
        n = recvfrom(sockfd, (char *)buffer, 12, 
                    MSG_DONTWAIT, ( struct sockaddr *) &cliaddr,
                    &len);
        buffer[n] = '\0';
        const char* tempBuf = (char*) buffer;

        std::stringstream ss;

        const char* it = tempBuf;
        const char* end = tempBuf + std::strlen(tempBuf);

        for (; it != end; ++it)
            ss << std::hex << unsigned(*it);

        long long int result;
        
        ss >> result;
        std::string hexString = ss.str();
        
        std::cout << "Hex value: " << std::hex << ss.str() << std::endl;

        long long int number = (long long int)strtoll(hexString.c_str(), NULL, 16);
    
        std::cout << "Hex number: " << number << std::endl;

        std::cout << "Hex text: " << hexToASCII(hexString) << std::endl;
        
        if (n > 0){ 
            std::cout <<  "Valor sobrescrito " << number << std::endl;
            _value = number;
        }

        n = 0;
        
        
        // _value = 0x6d28612c3129;        
        return _value; 
    }
    virtual void actuate(const Value & value) { _value = value; }

private:
    Value _value;
    sockaddr_in servaddr, cliaddr;
    int sockfd;
   
};

#ifdef __ACCELEROMETER_H
#include __ACCELEROMETER_H
#endif

#ifdef __GYROSCOPE_H
#include __GYROSCOPE_H
#endif

#ifdef __THERMOMETER_H
#include __THERMOMETER_H
#endif

#ifdef __HYGROMETER_H
#include __HYGROMETER_H
#endif

#ifdef __CO2_SENSOR_H
#include __CO2_SENSOR_H
#endif

#ifdef __PLUVIOMETER_H
#include __PLUVIOMETER_H
#endif

#ifdef __PRESSURE_SENSOR_H
#include __PRESSURE_SENSOR_H
#endif

#ifdef __KEYPAD_H
#include __KEYPAD_H
#endif

#ifdef __THERMISTOR_SENSOR_H
#include __THERMISTOR_SENSOR_H
#endif

#ifdef __ENCODER_SENSOR_H
#include __ENCODER_SENSOR_H
#endif


using Antigravity = Responsive_SmartData<Dummy_Transducer>;
using Antigravity_Proxy = Interested_SmartData<Dummy_Transducer::Unit::Wrap<Dummy_Transducer::UNIT>>;

#ifdef __ACCELEROMETER_H
using Acceleration = Responsive_SmartData<Accelerometer>;
using Acceleration_Proxy = Interested_SmartData<Accelerometer::Unit::Wrap<Accelerometer::UNIT>>;
#endif

#ifdef __GYROSCOPE_H
using Angular_Velocity = Responsive_SmartData<Gyroscope>;
using Angular_Velocity_Proxy = Interested_SmartData<Gyroscope::Unit::Wrap<Gyroscope::UNIT>>;
#endif

#ifdef __THERMOMETER_H
using Temperature = Responsive_SmartData<Thermometer>;
using Temperature_Proxy = Interested_SmartData<Thermometer::Unit::Wrap<Thermometer::UNIT>>;
#endif

#ifdef __HYGROMETER_H
using Relative_Humidity = Responsive_SmartData<Hygrometer>;
using Relative_Humidity_Proxy = Interested_SmartData<Hygrometer::Unit::Wrap<Hygrometer::UNIT>>;
#endif

#ifdef __CO2_SENSOR_H
using CO2_Concentration = Responsive_SmartData<CO2_Sensor>;
using CO2_Concentration_Proxy = Interested_SmartData<CO2_Sensor::Unit::Wrap<CO2_Sensor::UNIT>>;
#endif

#ifdef __PLUVIOMETER_H
using Precipitation = Responsive_SmartData<Pluviometer>;
using Precipitation_Proxy = Interested_SmartData<Pluviometer::Unit::Wrap<Pluviometer::UNIT>>;
#endif

#ifdef __PRESSURE_SENSOR_H
using Atmospheric_Pressure = Responsive_SmartData<Pressure_Sensor>;
using Atmospheric_Pressure_Proxy = Interested_SmartData<Pressure_Sensor::Unit::Wrap<Pressure_Sensor::UNIT>>;
#endif

#ifdef __KEYPAD_H
using Smart_Key = Responsive_SmartData<Keypad>;
using Smart_Key_Proxy = Interested_SmartData<Keypad::Unit::Wrap<Keypad::UNIT>>;
#endif

#ifdef __ENCODER_SENSOR_H
using Encoder = Responsive_SmartData<Encoder_Sensor>;
using Encoder_Proxy = Interested_SmartData<Encoder_Sensor::Unit::Wrap<Encoder_Sensor::UNIT>>;
#endif

#ifdef __THERMISTOR_SENSOR_H
using Thermistor = Responsive_SmartData<Thermistor_Sensor>;
using Thermistor_Proxy = Interested_SmartData<Thermistor_Sensor::Unit::Wrap<Thermistor_Sensor::UNIT>>;
#endif
