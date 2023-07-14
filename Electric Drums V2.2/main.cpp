#include <iostream>
#include <wiringPi.h>
#include <mcp3004.h>
#include <vector>
#include <algorithm>
#include <wiringSerial.h>
#include <lo/lo.h>
#include <lo/lo_cpp.h>

#define BASE 100
#define SPI_CHAN 0
#define SERIAL_DEVICE "/dev/ttyAMA0"

std::vector<int> thresholds = { 100, 100, 100, 100, 100 };
std::vector<int> maxThresholds = { 1023, 1023, 1023, 1023, 1023 };
std::vector<int> last_values(5, 0);
std::vector<bool> was_increasing(5, false);
std::vector<int> peak_values(5, 0);
const double peak_drop_percent = 0.4; // 40%

double envelope_alpha = 0.1;
double min_alpha = 0.1;
double max_alpha = 0.9;

int fd; // Global variable for the file descriptor

// Create an OSC target
const char* targetIP = "127.0.0.1";
int targetPort = 8000;
lo::Address target(targetIP, targetPort, LO_UDP);

void sendMidiMessage(int command, int channel, int data1, int data2) {
    int status = command | channel;
    serialPutchar(fd, status);
    serialPutchar(fd, data1);
    serialPutchar(fd, data2);
}

void sendNoteOn(int channel, int note, int velocity) {
    sendMidiMessage(0x90, channel, note, velocity);

    // Send OSC message
    lo::Message message("/noteOn");
    message.add(channel);
    message.add(note);
    message.add(velocity);
    target.send(message);
}

void sendNoteOff(int channel, int note) {
    sendMidiMessage(0x80, channel, note, 0);

    // Send OSC message
    lo::Message message("/noteOff");
    message.add(channel);
    message.add(note);
    target.send(message);
}

void setup() {
    if (wiringPiSetup() == -1)
        exit(1);
    mcp3004Setup(BASE, SPI_CHAN);

    // Open the serial port.
    fd = serialOpen(SERIAL_DEVICE, 115200);
    if (fd < 0) {
        std::cerr << "Failed to open serial port " << SERIAL_DEVICE << std::endl;
        exit(1);
    }
}

void loop() {
    for (int channel = 0; channel < 5; ++channel) {
        if (channel == 1 || channel == 2 || channel == 4 || channel == 5 || channel == 6) {
            continue;
        }

        int raw_value = analogRead(BASE + channel);
        int envelope = envelope_alpha * raw_value + (1.0 - envelope_alpha) * last_values[channel];
        double alpha = min_alpha + (max_alpha - min_alpha) * ((double)envelope / maxThresholds[channel]);
        double filtered_value = alpha * raw_value + (1.0 - alpha) * last_values[channel];
        int value = static_cast<int>(filtered_value);

        if (value > thresholds[channel] && value < maxThresholds[channel]) {
            if (value > last_values[channel] && value > peak_values[channel] * (1 - peak_drop_percent)) {
                peak_values[channel] = value;
                was_increasing[channel] = true;
            }
            else if (was_increasing[channel] && value < peak_values[channel] - peak_values[channel] * peak_drop_percent) {
                std::cout << "Peak for CH" << channel << ": " << peak_values[channel] << "\n";
                was_increasing[channel] = false;

                int note_number = (channel == 0) ? 38 : (channel == 3) ? 43 : 41;

                sendNoteOn(channel, note_number, 127);
                delay(500); // Some delay might be required between note-on and note-off
                sendNoteOff(channel, note_number);
            }
        }

        last_values[channel] = raw_value;
    }
}

int main() {
    setup();
    while (true) {
        loop();
    }
    return 0;
}
