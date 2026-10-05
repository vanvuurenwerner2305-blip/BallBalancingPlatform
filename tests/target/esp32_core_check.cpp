// Compile-only check that the shared core builds for the ESP32-S3 in single precision without
// silent float->double promotion (double is software-emulated on the target). Built by
// tools/check_esp32_core.sh with the PlatformIO Xtensa toolchain.
#include "bbp/core/camera_geometry.hpp"
#include "bbp/core/kinematics.hpp"

using namespace bbp;

extern "C" bool bbp_core_check(float sx, float sy, float* theta, float u, float v, float* xy) {
    const Kinematics<float> kin(PlatformGeometry<float>::fromCad());
    float th[3];
    Pose<float> pose;
    if (!kin.inverse(Kinematics<float>::normalFromSlopes(sx, sy), kin.levelHeight(0.0f), th, &pose)) return false;
    for (int i = 0; i < 3; ++i) theta[i] = th[i];
    Pose<float> fk = pose;
    if (kin.forward(th, fk, 10, 1e-6f) < 0) return false;
    float J[6][3];
    if (!kin.jacobian(th, fk, J)) return false;
    const CameraGeometry<float> cam(CameraIntrinsics<float>::fromHfov(320, 240, 1.5707964f),
                                    Vec3<float>{0.0f, 0.0f, -0.0924f}, 0.0f, 0.005f, 1.49f);
    Vec3<float> p;
    if (!cam.backprojectToPlate(u, v, pose, 0.1f, 0.01f, p)) return false;
    xy[0] = p.x;
    xy[1] = p.y;
    return true;
}
