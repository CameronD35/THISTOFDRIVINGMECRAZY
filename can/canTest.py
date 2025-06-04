import can
import time

# constant variables pertaining to the various CAN IDs accessible on the TOF sensor
START = 0x08
STOP = 0x09
QUERY = 0x1C

# Docs for the AFBR-S50 TOF and its CAN subsystem can be found here: https://broadcom.github.io/AFBR-S50-API/can_app.html

class TOF:
    
    # default bitrate will be 1 Mbps (1 million bits)
    # this is unlikely to change so you can make this a constant integer in C++
    # port will be a string such as "COM6"
    def __init__(self, port, br = 1000000):

        self.bitrate = br
        self.port = port

        # initialize the can bus interface for this object
        # slcan is the interface adopted by the RH-02 CAN-to-USB Analyzer
        self.interface = can.interface.Bus(interface = 'slcan', channel = port, 
                                           bitrate = br)

    # sends a signal to the START (0x08) address of the device via CAN
    def startMeasurements(self):
        
        # crafts remote (empty) frame for START address with no data
        message = can.Message(arbitration_id = START, is_remote_frame = True, dlc = 0, is_extended_id = False)
        
        try:

            # sends remote frame
            self.interface.send(message)
            print(f"Start message '{START}' sent on {self.port}")

        except can.CanError:
            
            # shuts down the CAN bus in the event of an error
            self.interface.shutdown()
            print("Start message NOT sent.")

    # Sends the STOP (0x09) byte to the device via CAN
    def stopMeasurements(self):
        
        # crafts remote (empty) frame for STOP address with no data
        # I assume the message datatype will be a struct in C++
        message = can.Message(arbitration_id = STOP, is_remote_frame = True, dlc = 0, is_extended_id = False)
        
        try:

            # sends remote frame
            self.interface.send(message)
            print(f"Stop message '{STOP}' sent on {self.port}")

        except can.CanError:
            
            # shuts down the CAN bus in the event of an error
            self.interface.shutdown()
            print("Stop message NOT sent.")

    # grabs any data found on the devices BUS connection (0x1C)
    def collectMeasurements(self):
        
        # Add UTC time to data output

        # blocks the program until a message is recieved (unless the time extends pasr the timeout of 2 seconds)
        # the repsonse is a byte array in python
        response = self.interface.recv(timeout = 2)
        print(response)

        # extracts data from message
        data = response.data
        
        while not len(data) >= 8:
            response = self.interface.recv(timeout = 2)
            data = response.data

        if response == None:
        
            print("Data not recieved.")

        else:
            
            return self.interpretMeasurements(data)

    # takes measurment data and translates to human-readable format
    # According to the AFBR-S50 API Reference Manual
    # - The range (distance) is contained within the first 3 bytes (24 bits)
    # - The amplitude (light intensity) is contained within the 4th and 5th byte (16 bits)
    # - The signal quality is contained in the 6th byte (8 bits)
    # - The measurement status is contained within the 7th and 8th bytes (16 bits) and is a signed integer
    def interpretMeasurements(self, meas):

        dataDictionary = {}
        print(meas)
        
        # The first byte is moved to the left 2 bytes to make room for the other bytes
        # ex: The sequence goes as follows for a distance represented by 0x23F46E:
        # - Input: b'\x23\xf4\x6e' -> Byte 1: 23, Byte 2: F4, Byte 3: 6E
        # - distance variable initialized with Byte 1: 0x23
        # - move Byte 1 over 2 bytes: 0x230000 (essentially appends 2 bytes)
        # - move Byte 2 (originally 0xF4) over 1 byte: 0xF400 (similar to above, but appends 1 byte)
        # - add Byte 2 to distance: 0x23F400
        # - add Byte 3 to distance: 0x23F46E
        distance = (meas[0] << 16) + (meas[1] << 8) + meas[2]

        # converts the distance from Q9.14
        # according to Wikipedia (https://en.wikipedia.org/wiki/Q_(number_format)):
        # The Q number format is Q{bits for integer}.{bits for fraction}
        # according to the TOF API reference, the raw distance uses 9 bits for the integer and 14 for the fraction
        distance = distance / 1000.0

        dataDictionary["distance"] = distance 

        amplitude = (meas[3] << 8) + meas[4]

        dataDictionary["amplitude"] = amplitude 

        signalQuality = meas[5]

        dataDictionary["signalQuality"] = signalQuality

        # similar to the distance bit shifts but we are only dealing with 2 bytes here (not 3) 
        status = (meas[6] << 8) + meas[7]
        dataDictionary["status"] = status
        
        # converts status from 16-bit unsigned int to 16-bit signed int
        if not status < 0x8000:

            status = status - 0x10000

        dataDictionary["distance"] = distance 

        return dataDictionary

sensor = TOF(port = 'COM6')

# if the user presses Ctrl+C while the program is running
# it is important to properly stop measurements and shutdown the CAN
# Otherwise, the TOF will get stuck
try:

    sensor.startMeasurements()
    
    for i in range(100):
        
        try: 

            data = sensor.collectMeasurements()

        except:
            
            sensor.interface.shutdown()

        # print(f"distance: {data.distance}, amplitude: {data.amplitude}, signal quality: {data.signalQuality}, status: {data.status}", end = "\r")

        print(data)

        # this delay may be changed (used for testing)
        time.sleep(0.2)

    print("")

    time.sleep(2)
    sensor.stopMeasurements()

    sensor.interface.shutdown()

# KeyboardInterrupt is the Ctrl+C sequence
except KeyboardInterrupt:

    sensor.stopMeasurements()
    sensor.interface.shutdown()

    # this gives the CAN the small amount of time it needs to shutdown properly
    time.sleep(0.1)