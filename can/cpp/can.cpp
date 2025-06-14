#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <conio.h>
#include <tchar.h>
#include <string>
#include <thread>
#include <fstream>
#include <chrono>

#include <windows.h>

#define OUTPUT_FILE_PATH "D:/Data/TOF/"
#define BUFFER_SIZE 100

namespace asio = boost::asio;
using asio::serial_port;

HANDLE fileHandle;
typedef const wchar_t* win_string;

struct DataStruct {

    double distance;
    int amplitude;
    short memsAngle;
    uint16_t signalQuality;
    int16_t status;
    std::chrono::milliseconds timestamp;

};

class TofSensor {
    public:
        TofSensor(asio::io_context& io, std::string outputFile, const std::string& port = "COM6") // might need to change port, should be this since running off windows os?
            : serial_(io, port), timer_(io) {

                // initialize output filepath for TOF
                filepath = outputFile;

                // Open a file and initialize it with a row of entries then closes it
                fileStream.open(filepath, std::ios::app);
                fileStream << "distance,amplitude,signal_quality,status,timestamp\n";
                fileStream.close();

                // Config of serial ports for SLCAN
                // 1Mbps, 8 character bits, no stop bits, no parity, no flow control
                serial_.set_option(serial_port::baud_rate(1000000));
                serial_.set_option(serial_port::character_size(8));
                serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
                serial_.set_option(serial_port::parity(serial_port::parity::none));
                serial_.set_option(serial_port::flow_control(serial_port::flow_control::none));

                send_command("S8\r"); // Set bitrate to 1mbps
                send_command("O\r"); // opening the channel


            }

            void startMeasurements() {
                fileStream.open(filepath, std::ios::app);
                send_remote_frame(0x08, 0x00); // start byte address cmd
            }

            void stopMeasurements() {
                fileStream.close();

                send_remote_frame(0x09, 0x00); // stop byte address cmd
            }

            void asyncReadStart() {

                serial_.async_read_some(asio::buffer(read_buf_),
                    boost::bind(&TofSensor::handleRead, this,
                        asio::placeholders::error,
                        asio::placeholders::bytes_transferred));

                Sleep(10);

            }

    private:
            serial_port serial_;
            asio::steady_timer timer_;
            std::array<char, 256> read_buf_;
            std::string read_buffer_;
            std::ofstream fileStream;
            std::string filepath;

            // GRABS TIME OF MEASUREMENT CLOCK CYCLE
            // THIS IS WHERE WE NEED TO ADJUST
            std::chrono::_V2::system_clock::time_point measCycleClock_Start = std::chrono::high_resolution_clock::now();
            

            // sends command via Lawicel CAN/CAN232 Protocol
            // Protocol pdf: https://www.canusb.com/files/canusb_manual.pdf
            void send_command(const std::string& cmd) {
                asio::write(serial_, asio::buffer(cmd.c_str(), cmd.size()));

                std::cout << "sending " << cmd << std::endl;
            }

            // create RTR (remote frame) for Lawicel CAN/CAN232 protocol
            void send_remote_frame(uint32_t can_id, int data_length){
                std::ostringstream cmd;
                // converts can_id to hexadecimal with length of 3 and appends data length
                // data length will always be zero since the TOF does not read data
                cmd << "r" << std::hex << std::setw(3) << std::setfill('0') << can_id << data_length << "\r";
                send_command(cmd.str());
            }


            void handleRead(const boost::system::error_code& ec, size_t bytes) {
                if (!ec) {
                    read_buffer_.append(read_buf_.data(), bytes);
                    processBuffer();
                    asyncReadStart();
                }
            }

            void processBuffer() {
                size_t pos;
                while ((pos = read_buffer_.find('\r')) != std::string::npos) {
                    std::string frame = read_buffer_.substr(0, pos);
                    read_buffer_.erase(0, pos+1);

                    if (frame[0] == 't') { // SLCAN data frame
                        parseCanFrame(frame);
                        //std::cout << frame << std::endl;
                    }
                }
            }

            void parseCanFrame(const std::string& frame) {
                uint32_t id;
                std::istringstream(frame.substr(1, 3)) >> std::hex >> id;

                // We need to chop off the leading and end 8s from the frame
                std::string new_frame = frame.substr(5, frame.size() - 5);

                if (id == 0x01C) {
                    std::vector<uint16_t> data;


                    for (size_t i = 0; i < new_frame.size(); i += 2) {
                        uint16_t formatted_byte;
                        
                        std::string raw_byte = new_frame.substr(i, 2);

                        std::istringstream(raw_byte) >> std::hex >> formatted_byte;

                        data.push_back(formatted_byte);
                    }

                    if (data.size() >= 8) {

                        // for (auto i = data.begin(); i != data.end(); i++) {
                        //     std::cout << *i << " ";
                        // }

                        interpretMeasurements(data);


                    }
                }
            }

            void writeMeasurementsToFile(DataStruct data) {

                // std::cout << "FILE IS OPEN???? " << fileStream.is_open() << std::endl;

                if (fileStream.is_open()) {

                    fileStream << data.distance << "," << data.amplitude << "," << data.signalQuality << "," << data.status << "," << data.timestamp.count() << "\n";

                } else {

                    std::cout << "There is no open file!" << std::endl;

                }

            }

            void interpretMeasurements(const std::vector<uint16_t>& meas) {

                DataStruct measurementData; 

                // measured in mm
                double distance = (meas[0] << 16) | (meas[1] << 8) | (meas[2]);

                // convert from mm to m
                measurementData.distance = distance / 1000.0;

                // measured in LSB
                measurementData.amplitude = (meas[3] << 8) + (meas[4]);

                // In percentage (0-100%)
                measurementData.signalQuality = meas[5];

                // see https://broadcom.github.io/AFBR-S50-API/group__argus__status.html#gaaabdaf7ee58ca7269bd4bf24efcde092
                // for error codes
                measurementData.status = (meas[6] << 8 | meas[7]);

                // GRABS THE CURRENT TIME OF CLOCK CYCLE 
                auto measCycleClock_Point = std::chrono::high_resolution_clock::now();

                // SUPPOSED TO GET TIME DIFFERENCE BUT THIS METHOD WON'T WORK
                // USES LINE 83 AND LINE 194 TIMES
                //auto elapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(measCycleClock_Point - measCycleClock_Start).count();  //measCycleClock_Point - measCycleClock_Start;
                measurementData.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(measCycleClock_Point - measCycleClock_Start);

                // output values
                // std::cout << "Distance: " << measurementData.distance << " | Amplitude: " << measurementData.amplitude << " | Signal Quality: " << measurementData.signalQuality << " | Status: " << measurementData.status << std::endl;

                writeMeasurementsToFile(measurementData);

            }
};

time_t grabCurrentTime() {

    auto rawCurrentTime = std::chrono::system_clock::now();

    time_t currentTime = std::chrono::system_clock::to_time_t(rawCurrentTime);

    return currentTime;

}

int connectToMemsClient(char* output) {
    std::cout << "====C++====\r\n";
    win_string name = TEXT("\\\\.\\pipe\\rsx_mems_pipe");

    // Try to access pipe
    fileHandle = CreateFile(name, GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING, 0, NULL);

    // Keep trying to connect if failed.
    while (fileHandle == INVALID_HANDLE_VALUE)
    {
        // Open pipe.
        fileHandle = CreateFile(name, GENERIC_READ | GENERIC_WRITE, 
            0, NULL, OPEN_EXISTING, 0, NULL);
        Sleep(100);
    }
}

int readFromMems() {
    char* buffer = new char[BUFFER_SIZE];
    memset(buffer, 0, BUFFER_SIZE);

    // The ReadString will halt until a message is received.
    ReadString(buffer);
    std::cout << "recv<< " << buffer << "\r\n";

    // We are asked for the angle.
    if (strcmp(buffer, "Angle request.\0") == 0)
    {
        std::string str = std::string("Angle: ").append(std::to_string(angle)).append("\r\n");
        const char* msg = str.c_str();

        // DO SOME WORK HERE

        // Send data to C#
        WriteFile(fileHandle, msg, strlen(msg), nullptr, NULL);
        std::cout << "send>> " << msg << "\r\n";
        angle++;
    }
    else if (std::string(buffer).rfind("SERVER ERROR NOT", 0) == 0)
    {
        // We have an error.
        std::cout << "We have a server error.";
    }
    else if (std::string(buffer).rfind("Set to ", 0) == 0)
    {
        // The angle returned is the actual angle.
        std::string actualAngleString = std::string(buffer).substr(7);
        double actualAngle = std::stod(actualAngleString); // https://stackoverflow.com/a/30809670

        // Do something with actualAngle
        

    }
}


// Credit https://dev.to/gabbersepp/ipc-between-c-and-c-by-using-named-pipes-4em9
void ReadString(char* output)
{
    ULONG read = 0;
    int index = 0;
    do
    {
        // ReadFile halts until we have data from C# to read
        (void) ReadFile(fileHandle, output + index++, 1, &read, NULL);
    } while (read > 0 && *(output + index - 1) != 0);
}

int getCurrentMemsAngle() {
    
    
    return 0;

}

int main (int argc, char** argv) {

    try {

        // create IO context allowing for read/write operations
        asio::io_context io;

        // grab current time for file timestamp
        time_t now = grabCurrentTime();

        std::string filename = OUTPUT_FILE_PATH + std::to_string(now) + ".csv";

        // creates TOF sensor instance
        TofSensor sensor(io, filename);

        

        // begin measurements (send 0x08 to TOF)
        sensor.startMeasurements();

        // start reading from 0x1C address 
        sensor.asyncReadStart();

        // open seperate thread
        std::thread([&io]() { io.run(); }).detach();
        std::this_thread::sleep_for(std::chrono::seconds(20));

        // stop measurements (send 0x09 to TOF)
        sensor.stopMeasurements();
        
        // add slight delay to ensure proper stopping
        Sleep(200);

        io.stop();

    }

    catch (const std::exception& e) {

        std::cerr << "Error: " << e.what() << std::endl;

    }
    
    return 0;

}