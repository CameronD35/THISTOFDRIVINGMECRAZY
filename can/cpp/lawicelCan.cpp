#include <windows.h>
#include <lawicel_can.h>
#include <iostream>
#include <thread>
#include <cmath>
#include <stdexcept>
#include <stdexcpt.h>

#define CANUSB_ACCEPTANCE_CODE_ALL 0x00000000
#define CANUSB_ACCEPTANCE_MASK_ALL 0xFFFFFFFF
#define CANUSB_FLAG_TIMESTAMP 0x0001

class TofSensor {
public:
    TofSensor(const char* port = "COM6", const char* bitrate = "1000") 
        : handle(-1) {
        
        // Open CAN interface with 1Mbps bitrate
        handle = canusb_Open(port, bitrate, 
                            CANUSB_ACCEPTANCE_CODE_ALL, 
                            CANUSB_ACCEPTANCE_MASK_ALL, 
                            CANUSB_FLAG_TIMESTAMP);
        
        if(handle <= 0) {
            throw std::runtime_error("CAN interface open failed");
        }
    }

    ~TofSensor() {
        if(handle > 0) {
            canusb_Close(handle);
        }
    }

    void startMeasurements() {
        sendRemoteFrame(START);
    }

    void stopMeasurements() {
        sendRemoteFrame(STOP);
    }

    void collectMeasurements() {
        CANMsg msg;
        int result = canusb_Read(handle, &msg);
        
        if(result > 0) {
            interpretMeasurements(msg);
        }
        else if(result == ERROR_CANUSB_TIMEOUT) {
            std::cerr << "Data not received\n";
        }
    }

private:
    CANHANDLE handle;
    static constexpr uint32_t START = 0x08;
    static constexpr uint32_t STOP = 0x09;

    void sendRemoteFrame(uint32_t can_id) {
        CANMsg frame = {};
        frame.id = can_id;
        frame.flags = CANMSG_RTR;
        frame.len = 0;

        int result = canusb_Write(handle, &frame);
        if(result <= 0) {
            std::cerr << "Frame send failed\n";
            canusb_Close(handle);
            handle = -1;
        }
    }

    void interpretMeasurements(const CANMsg& msg) {
        // Implementation matches Python version
        double distance = (msg.data[1] << 16) | (msg.data[2] << 8) | msg.data[3];
        distance /= 16384.0;

        double amplitude = (msg.data[4] << 8) | msg.data[5];
        amplitude /= 16.0;

        uint8_t signal_quality = msg.data[6];
        int16_t status = (msg.data[7] << 8) | msg.data[8];
        
        // Print or process measurements
        std::cout << "Distance: " << distance 
                  << " | Amplitude: " << amplitude
                  << " | Quality: " << static_cast<int>(signal_quality)
                  << " | Status: " << status << "\n";
    }
};

int main() {
    try {
        TofSensor sensor;
        sensor.startMeasurements();

        for(int i = 0; i < 100; ++i) {
            sensor.collectMeasurements();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        sensor.stopMeasurements();
    }
    catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
