#include <iostream>
#include <wiringPi.h>
#include <mcp3004.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <wiringSerial.h>

#define BASE 100
#define SPI_CHAN 0

std::vector<int> thresholds = { 50, 50, 50, 50, 50 };
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

void printMidiMessage(int command, int channel, int data1, int data2) {
    int status = command | channel;
    std::cout << "Status: " << status << ", Data1: " << data1 << ", Data2: " << data2 << std::endl;
}

void printNoteOn(int channel, int note, int velocity) {
    printMidiMessage(0x90, channel, note, velocity);
}

void printNoteOff(int channel, int note) {
    printMidiMessage(0x80, channel, note, 0);
}

void setup() {
    if (wiringPiSetup() == -1)
        exit(1);
    mcp3004Setup(BASE, SPI_CHAN);
}

void loop() {
    for (int channel = 0; channel < 5; ++channel) {
        // Rest of your existing code...

        if (value > thresholds[channel] && value < maxThresholds[channel]) {
            // Rest of your existing code...

        else if (was_increasing[channel] && value < peak_values[channel] - peak_values[channel] * peak_drop_percent) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_increase_time[channel]) > peak_hold_time) {
                was_increasing[channel] = false;

                int note_number = (channel == 0) ? 38 : (channel == 3) ? 43 : 41;
                int adjusted_velocity = std::min(127, std::max(0, (int)(peak_values[channel] * 127 * sensitivity / 1023)));

                printNoteOn(channel, note_number, adjusted_velocity);
                delay(25);
                printNoteOff(channel, note_number);
            }
        }
        }
    for (int channel = 0; channel < 5; ++channel) {
        if (channel == 1 || channel == 2 || channel == 4 || channel == 5 || channel == 6) {
            continue;
        }

        int raw_value = analogRead(BASE + channel);

        // Apply high-pass filter on channel 0
        if (channel == 0) {
            raw_value -= hp_filter_buffer[channel] * highpass_coeff;
            hp_filter_buffer[channel] = raw_value;
        }
        // Apply low-pass filter on other channels
        else if (channel == 3 || channel == 5) {
            raw_value = raw_value * lowpass_coeff + lp_filter_buffer[channel] * (1.0 - lowpass_coeff);
            lp_filter_buffer[channel] = raw_value;
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

                    int note_number = (channel == 0) ? 38 : (channel == 3) ? 43 : 41;
                    int adjusted_velocity = std::min(127, std::max(0, (int)(peak_values[channel] * 127 * sensitivity / 1023)));

                    printNoteOn(channel, note_number, adjusted_velocity);
                    delay(25);
                    printNoteOff(channel, note_number);
                }
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
