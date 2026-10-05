// Control: tilt the plate so the ball goes to the target.
//
// ball.x, ball.y   position on the plate [mm]       ball.vx, ball.vy  velocity [mm/s]
// target.x/y       where the ball should be [mm]    ball.dt           time step [s]
// platform.angleX  plate tilt [deg]. Positive lifts the +x edge -> ball rolls to -x.
#include <bbp/bbp.hpp>
using namespace bbp;

// One PID per axis. Output: plate angle in degrees.
PID pidX(0.09f, 0.0f, 0.045f);
PID pidY(0.09f, 0.0f, 0.045f);

void control(const Ball& ball, const Target& target, Platform& platform) {
    if (!ball.found) {  // lost the ball: level the plate
        platform.angleX = 0;
        platform.angleY = 0;
        return;
    }
    pidX.setDerivativeFilter(0.03f);
    pidY.setDerivativeFilter(0.03f);

    const float errorX = target.x - ball.x;
    const float errorY = target.y - ball.y;

    // Ball too far in +x (error < 0) -> lift the +x edge (angle > 0): hence the minus sign.
    platform.angleX = -pidX.update(errorX, ball.dt);
    platform.angleY = -pidY.update(errorY, ball.dt);

    log("error x [mm]", errorX);
    log("error y [mm]", errorY);
}
