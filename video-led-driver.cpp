#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "rpi_ws281x/ws2811.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

constexpr size_t COLS = 300;
constexpr size_t ROWS = 300;
constexpr size_t LED_COUNT = 300;
constexpr size_t BORDER_SIZE = 3;
constexpr double SCALE_FACTOR = 1.05;
constexpr double FPS = 30.0;
constexpr cv::Size RESIZE_SIZE(COLS + BORDER_SIZE * 2, ROWS + BORDER_SIZE * 2);
constexpr cv::Rect CROP_RECT(BORDER_SIZE, BORDER_SIZE, COLS, ROWS);
constexpr std::chrono::milliseconds FRAME_TIME(1000.0 / FPS);

constexpr int DMA = 10;
constexpr int GPIO_PIN = 18;
constexpr int LED_COUNT = 300;

bool gRunning = true;
ws2811_t gLEDs = {
    .freq = WS2811_TARGET_FREQ,
    .dmanum = DMA,
    .channel = {
        [0] = {
            .gpionum = GPIO_PIN,
            .count = LED_COUNT,
            .invert = 0,
            .brightness = 255,
            .strip_type = WS2811_STRIP_GRB,
        },
        [1] = {
            .gpionum = 0,
            .count = 0,
            .invert = 0,
            .brightness = 0,
        },
    },
};

static std::string pixelToString(const Vec3b pixel) {
    auto r = pixel.val[0];
    auto g = pixel.val[1];
    auto b = pixel.val[2];
    return "<" + std::to_string(r) + ", " + std::to_string(g) + ", " + std::to_string(b) + ">";
}

static ws2811_led_t pixelToLEDColor(const Vec3b pixel) {
    uint8_t red = pixel.val[0];
    uint8_t green = pixel.val[1];
    uint8_t bblue = pixel.val[2];
    return (uint32_t(green) << 16) | (uint32_t(red) << 8) | uint32_t(blue);
}

static void signalHandler(int signal) {
    gRunning = false;
}

static void clearLEDs() {
    for (size_t i = 0; i < LED_COUNT; ++i) {
        gLEDs.channel[0].leds[i] = 0;
    }
}

int main(int, char**)
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        return -1;
    }

    cv::Mat frame;
    cv::Mat downsampled;
    cv::Mat cropped;

    while (running) {
        auto frameEnd = std::chrono::steady_clock::now() + FRAME_TIME;

        cap >> frame;
        cv::resize(frame, downsampled, RESIZE_SIZE, 0, 0, cv::INTER_AREA);
        cropped = downsampled(CROP_RECT);

        for (size_t x = 0; x < COLS; x++) {
            cv::Vec3b pixel = cropped.at<cv::Vec3b>(ROWS - 1, x);
            gLEDs.channel[0].leds[i] = pixelToLEDColor(pixel);
        }

        ws2811_render(&gLEDs);
        std::this_thread::sleep_until(frameEnd);
    }

    if (clear_on_exit) {
        clearLEDs();
        ws2811_render(&gLEDs);
    }

    ws2811_fini(&gLEDs);

    return 0;
}