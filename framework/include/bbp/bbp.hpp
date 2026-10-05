// =============================================================================
//  Ball Balancing Platform - student library
// =============================================================================
//  Everything you can use in tracking.cpp and control.cpp. The same code runs in
//  the simulator and on the real platform.
//
//  You write two functions:
//
//    void track(const Image& image, Detection& ball);
//        Called for every camera frame. Find the ball in the image and report
//        its centre in PIXELS:  ball.found = true; ball.x = ...; ball.y = ...;
//
//    void control(const Ball& ball, const Target& target, Platform& platform);
//        Called after every track(). The library has already turned your pixel
//        position into millimetres on the plate and keeps its history. Set the
//        plate angles you want:  platform.angleX = ...; platform.angleY = ...;
//
//  Units: positions in millimetres (origin at the plate centre), time in
//  seconds, angles in degrees.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstdio>

namespace bbp {

// ---------------------------------------------------------------------------
//  Image (input to track)
// ---------------------------------------------------------------------------
// The camera looks UP at the ball through the transparent plate.
// Pixel (0, 0) is the top-left corner; x grows to the right, y grows down.
class Image {
public:
    int width() const { return width_; }
    int height() const { return height_; }

    // Brightness of a pixel, 0 (black) .. 255 (white).
    int gray(int x, int y) const;
    // Colour channels, each 0 .. 255.
    int red(int x, int y) const;
    int green(int x, int y) const;
    int blue(int x, int y) const;

    // (used by the library)
    Image(const std::uint8_t* data, int width, int height, int format)
        : data_(data), width_(width), height_(height), format_(format) {}

private:
    const std::uint8_t* data_;
    int width_, height_, format_;
};

// Output of track(): where you found the ball, in pixels.
struct Detection {
    bool found = false;
    float x = 0.0f;  // pixel column of the ball centre
    float y = 0.0f;  // pixel row of the ball centre
};

// ---------------------------------------------------------------------------
//  Ball (input to control)
// ---------------------------------------------------------------------------
struct BallSample {
    float x, y;  // position [mm]
    float t;     // time of the measurement [s]
};

struct Ball {
    bool found = false;  // false if track() did not find the ball this frame
    float x = 0, y = 0;  // latest position on the plate [mm]
    float vx = 0, vy = 0;  // velocity [mm/s], filtered (see Setup > Control)
    float dt = 0;        // time since the previous control() call [s]
    float time = 0;      // current time [s]

    // Past positions: history(0) is the newest, history(historySize() - 1) the oldest.
    int historySize() const { return count_; }
    BallSample history(int k) const;

    // (used by the library)
    static constexpr int kMaxHistory = 64;
    BallSample samples_[kMaxHistory] = {};
    int count_ = 0, head_ = 0;
};

// Where the ball should go [mm]. Set in the Setup tab or live in the Simulation tab.
struct Target {
    float x = 0, y = 0;
};

// ---------------------------------------------------------------------------
//  Platform (output of control)
// ---------------------------------------------------------------------------
// angleX: tilt of the plate along x [degrees]. Positive lifts the +x edge,
//         so the ball rolls towards -x.   Likewise angleY for y.
// The library limits the angles (Setup > Control > Max angle) and moves the servos.
struct Platform {
    float angleX = 0;
    float angleY = 0;
};

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------
// PID controller:  output = kp*error + ki*integral(error) + kd*d(error)/dt
class PID {
public:
    PID(float kp, float ki, float kd);
    float update(float error, float dt);  // call once per control() with ball.dt
    void reset();

    void setOutputLimit(float limit);        // clamp output to [-limit, limit] (0 = off)
    void setIntegralLimit(float limit);      // anti-windup clamp on the integral (0 = off)
    void setDerivativeFilter(float seconds); // low-pass time constant on the derivative (0 = off)

    float kp, ki, kd;
    float lastP = 0, lastI = 0, lastD = 0;   // the three terms of the last update

private:
    float integral_ = 0, prevError_ = 0, dFiltered_ = 0;
    float outLimit_ = 0, intLimit_ = 0, dTau_ = 0;
    bool first_ = true;
};

// Plot a value live in the Simulation tab, e.g.  log("error x", target.x - ball.x);
void log(const char* name, float value);

float clamp(float value, float lo, float hi);

}  // namespace bbp

// ---------------------------------------------------------------------------
//  The two functions you write
// ---------------------------------------------------------------------------
void track(const bbp::Image& image, bbp::Detection& ball);
void control(const bbp::Ball& ball, const bbp::Target& target, bbp::Platform& platform);
