#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "rpi_ws281x/ws2811.h"

#include <execinfo.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

constexpr int LED_COUNT = 300;
constexpr size_t IMAGE_SIZE = LED_COUNT / 2;

constexpr size_t BORDER_SIZE = 3;
constexpr size_t FPS = 30;
constexpr std::chrono::milliseconds FRAME_TIME(1000 / FPS);
constexpr int DMA = 10;
constexpr int GPIO_PIN = 18;

static const cv::Size RESIZE_SIZE(IMAGE_SIZE + BORDER_SIZE * 2, IMAGE_SIZE + BORDER_SIZE * 2);
static const cv::Rect CROP_RECT(BORDER_SIZE, BORDER_SIZE, IMAGE_SIZE, IMAGE_SIZE);

// This appears to be the max brightness before the end of the 300-LED strand at
// full white starts to go off-white with a single 20A PSU connection at the
// beginning of the strand
static float BRIGHTNESS = 32;

static uint8_t GAMMA_E[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
    2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5,
    6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11,
    11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
    19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28,
    29, 29, 30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40,
    40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54,
    55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 88, 89,
    90, 91, 93, 94, 95, 96, 98, 99,100,102,103,104,106,107,109,110,
    111,113,114,116,117,119,120,121,123,124,126,128,129,131,132,134,
    135,137,138,140,142,143,145,146,148,150,151,153,155,157,158,160,
    162,163,165,167,169,170,172,174,176,178,179,181,183,185,187,189,
    191,193,194,196,198,200,202,204,206,208,210,212,214,216,218,220,
    222,224,227,229,231,233,235,237,239,241,244,246,248,250,252,255
};

static bool gRunning = true;
static ws2811_t gLEDs = []{
    ws2811_t leds{};
    leds.freq = WS2811_TARGET_FREQ;
    leds.dmanum = DMA;
    leds.channel[0].gpionum = GPIO_PIN;
    leds.channel[0].count = LED_COUNT;
    leds.channel[0].invert = 0;
    leds.channel[0].brightness = BRIGHTNESS;
    leds.channel[0].strip_type = WS2811_STRIP_GRB;
    leds.channel[1].gpionum = 0;
    leds.channel[1].count = 0;
    leds.channel[1].invert = 0;
    leds.channel[1].brightness = 0;
    return leds;
}();

static void log(const std::string& msg) {
    std::cout << msg << std::endl;
}

static void crashHandler(int sig) {
    gRunning = false;

    void *array[10];
    size_t size = backtrace(array, 10);
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

static void signalHandler(int sig) {
    gRunning = false;
}

static std::string pixelToString(const cv::Vec3b pixel) {
    auto r = pixel.val[0];
    auto g = pixel.val[1];
    auto b = pixel.val[2];
    return "<" + std::to_string(r) + ", " + std::to_string(g) + ", " + std::to_string(b) + ">";
}

static ws2811_led_t pixelToLEDColor(const cv::Vec3b pixel) {
    uint8_t red = GAMMA_E[pixel.val[0]];
    uint8_t green = GAMMA_E[pixel.val[1]];
    uint8_t blue = GAMMA_E[pixel.val[2]];
    return (uint32_t(red) << 16) | (uint32_t(green) << 8) | uint32_t(blue);
}

static void transform(cv::Vec3f& p, size_t y) {
    float h = p[0];
    float s = p[1];
    float v = p[2];
}

static void clearLEDs() {
    for (size_t i = 0; i < LED_COUNT; ++i) {
        gLEDs.channel[0].leds[i] = 0;
    }
}

int main(int, char**) {
    std::signal(SIGSEGV, crashHandler);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    log("Initializing LED driver. OpenCV version " + std::string(CV_VERSION));
    ws2811_return_t ret;
    if ((ret = ws2811_init(&gLEDs)) != WS2811_SUCCESS) {
        fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
        return ret;
    }

    log("Opening video capture device");
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        return -1;
    }

    log("Starting video capture");
    cv::Mat3b frame;
    cv::Mat3b downsampled;
    cv::Mat3b cropped;
    cv::Mat3f inputHSV;
    cv::Mat3f outputHSV = cv::Mat::zeros(1, LED_COUNT, CV_32FC3);
    cv::Mat3b outputRGB;

    while (gRunning) {
        auto frameEnd = std::chrono::steady_clock::now() + FRAME_TIME;

        cap >> frame;

        cv::resize(frame, downsampled, RESIZE_SIZE, 0, 0, cv::INTER_AREA);
        cropped = downsampled(CROP_RECT);

        cropped.convertTo(inputHSV, CV_32FC3, 1.0 / 255.0);
        cv::cvtColor(inputHSV, inputHSV, CV_BGR2HSV);

        // Read, transform, and write the left column
        for (size_t y = 0; y < IMAGE_SIZE; y++) {
            cv::Vec3f p = inputHSV.at<cv::Vec3f>(y, 0);
            transform(p, y);
            outputHSV.at<cv::Vec3f>(0, y) = p;
        }

        // Read, transform, and write the right column
        for (size_t y = 0; y < IMAGE_SIZE; y++) {
            cv::Vec3f p = inputHSV.at<cv::Vec3f>(y, IMAGE_SIZE - 2);
            transform(p, y);
            outputHSV.at<cv::Vec3f>(0, IMAGE_SIZE + y) = p;
        }

        // Convert HSV back to RGB
        cv::cvtColor(outputHSV, outputHSV, CV_HSV2RGB);
        outputHSV.convertTo(outputRGB, CV_8UC3, 255.0f);

        // Write the RGB colors out to the driver
        for (size_t i = 0; i < LED_COUNT; i++) {
            cv::Vec3b pixel = outputRGB.at<cv::Vec3b>(0, i);
            gLEDs.channel[0].leds[i] = pixelToLEDColor(pixel);
        }

        ws2811_render(&gLEDs);
        std::this_thread::sleep_until(frameEnd);
    }

    // Clear the LEDs and gracefully stop DMA writes
    clearLEDs();
    ws2811_render(&gLEDs);
    ws2811_fini(&gLEDs);

    return 0;
}
