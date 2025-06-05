#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <thread>

namespace asio = boost::asio;
using asio::serial_port;

class TofSensor {
    public:
        TofSensor(asio::io_context& io, const std::string& port = "COM6") // might need to change port, should be this since running off windows os?
            : serial_(io, port), timer_(io) {

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
                send_remote_frame(0x08, 0x00); // start byte address cmd
            }

            void stopMeasurements() {
                send_remote_frame(0x09, 0x00); // stop byte address cmd
            }

            void asyncReadStart() {

                serial_.async_read_some(asio::buffer(read_buf_),
                    boost::bind(&TofSensor::handleRead, this,
                        asio::placeholders::error,
                        asio::placeholders::bytes_transferred));

                Sleep(400);

            }

    private:
            serial_port serial_;
            asio::steady_timer timer_;
            std::array<char, 256> read_buf_;
            std::string read_buffer_;

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
                        std::cout << frame << std::endl;
                    }
                }
            }

            void parseCanFrame(const std::string& frame) {
                uint32_t id;
                std::istringstream(frame.substr(1, 3)) >> std::hex >> id;

                // We need to chop off the leading and end 8s from the frame
                std::string new_frame = frame.substr(5, frame.size() - 5);

                std::cout << new_frame << std::endl;

                if (id == 0x01C) {
                    std::vector<uint16_t> data;


                    for (size_t i = 0; i < new_frame.size(); i += 2) {
                        uint16_t formatted_byte;
                        
                        std::string raw_byte = new_frame.substr(i, 2);

                        std::istringstream(raw_byte) >> std::hex >> formatted_byte;

                        std::cout << raw_byte << " | " << formatted_byte << " | ";
                        data.push_back(formatted_byte);
                    }

                    if (data.size() >= 8) {

                        // for (auto i = data.begin(); i != data.end(); i++) {
                        //     std::cout << *i << " ";
                        // }

                        std::cout << "\n";

                        interpretMeasurements(data);
                    }
                }
            }

            void interpretMeasurements(const std::vector<uint16_t>& meas) {

                // measured in mm
                double distance = (meas[0] << 16) | (meas[1] << 8) | (meas[2]);

                // convert from mm to m
                distance = distance / 1000.0;

                // measured in LSB
                double amplitude = (meas[3] << 8) + (meas[4]);

                // In percentage (0-100%)
                uint8_t signal_quality = meas[5];

                // see https://broadcom.github.io/AFBR-S50-API/group__argus__status.html#gaaabdaf7ee58ca7269bd4bf24efcde092
                // for error codes
                int16_t status = (meas[6] << 8 | meas[7]);

                // output values
                std::cout << "Distance: " << distance << " | Amplitude: " << amplitude << " | Status: " << status << std::endl;

            }
};

int main (int argc, char** argv) {
    try {

        // create IO context allowing for read/write operations
        asio::io_context io;

        // creates TOF sensor instance
        TofSensor sensor(io);

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