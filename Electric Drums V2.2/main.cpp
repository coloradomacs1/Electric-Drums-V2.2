#include <iostream>
#include <wiringPi.h>
#include <mcp3004.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define BASE 100
#define SPI_CHAN 0

std::vector<int> thresholds = { 30, 30, 30, 30, 30 };
std::vector<int> maxThresholds = { 900, 900, 900, 900, 900 };
std::vector<int> last_values(5, 0);
std::vector<bool> was_increasing(5, false);
std::vector<int> peak_values(5, 0);
std::vector<float> hp_filter_buffer(5, 0);
std::vector<float> lp_filter_buffer(5, 0);
std::vector<std::chrono::steady_clock::time_point> last_increase_time(5);
const double peak_drop_percent = 0.6; // 40%
const double highpass_coeff = 0.99; // highpass filter coefficient
const double lowpass_coeff = 0.05; // lowpass filter coefficient
const std::chrono::milliseconds peak_hold_time(2);
double sensitivity = 1; // Adjust this to change sensitivity

double envelope_alpha = 0.1;
double min_alpha = 0.1;
double max_alpha = 0.9;

int fd;

void setup() {
    if (wiringPiSetup() == -1)
        exit(1);
    mcp3004Setup(BASE, SPI_CHAN);

    fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY); // Open serial port

    if (fd == -1) {
        std::cerr << "Unable to open /dev/ttyAMA0" << std::endl;
        exit(1);
    }

    struct termios options;
    tcgetattr(fd, &options); // Get the current options for the port
    cfsetispeed(&options, B9600); // Set the baud rates to 9600
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode
    options.c_cflag &= ~PARENB; // No parity bit
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE; // Mask data size
    options.c_cflag |= CS8; // 8 data bits
    tcsetattr(fd, TCSANOW, &options); // Set the new options for the port
}

void sendMidiMessage(int command, int channel, int data1, int data2) {
    uint8_t msg[3];
    msg[0] = command | channel;
    msg[1] = data1 & 0x7F;
    msg[2] = data2 & 0x7F;
    write(fd, msg, sizeof(msg));
}

void sendNoteOn(int channel, int note, int velocity) {
    sendMidiMessage(0x90, channel, note, velocity);
}

void sendNoteOff(int channel, int note) {
    sendMidiMessage(0x80, channel, note, 0);
}

void loop() {
    for (int channel = 0; channel < 5; ++channel) {
        int raw_value = analogRead(BASE + channel);

        // Apply high-pass filter on channel 0
        if (channel == 0) {
            raw_value -= hp_filter_buffer[channel] * highpass_coeff;
            hp_filter_buffer[channel] = raw_value;
        }

        int envelope = envelope_alpha * raw_value + (1.0 - envelope_alpha) * last_values[channel];
        double alpha = min_alpha + (max_alpha - min_alpha) * ((double)envelope / maxThresholds[channel]);
        double filtered_value = alpha * raw_value + (1.0 - alpha) * last_values[channel];
        int value = static_cast<int>(filtered_value);

        if (value > thresholds[channel] && value < maxThresholds[channel]) {
            if (value > last_values[channel] && value > peak_values[channel] * (1 - peak_drop_percent)) {
                peak_values[channel] = value;
                was_increasing[channel] = true;
                last_increase_time[channel] = std::chrono::steady_clock::now();
            }
            else if (was_increasing[channel] && value < peak_values[channel] - peak_values[channel] * peak_drop_percent) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_increase_time[channel]) > peak_hold_time) {
                    was_increasing[channel] = false;

                    int note_number = (channel == 0) ? 60 : 64;
                    int velocity = static_cast<int>(std::min(1.0, sensitivity * peak_values[channel] / maxThresholds[channel]) * 127);

                    sendNoteOn(channel, note_number, velocity);
                    std::this_thread::sleep_for(std::chrono::milliseconds(25));
                    sendNoteOff(channel, note_number);
                    peak_values[channel] = 0;
                }
            }
        }

        last_values[channel] = value;
    }
}

int main() {
    setup();
    while (true) {
        loop();
    }
    close(fd);  // Close the serial port when done
    return 0;
}
