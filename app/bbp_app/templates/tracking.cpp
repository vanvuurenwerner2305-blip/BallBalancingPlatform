// Ball tracking: find the ball in the camera image.
//
// The camera looks up through the plate, so the ball appears as a dark disc on the
// bright ceiling. This starter version thresholds dark pixels and takes their centroid.
#include <bbp/bbp.hpp>
using namespace bbp;

void track(const Image& image, Detection& ball) {
    const int threshold = 80;        // pixels darker than this belong to the ball
    const float searchRadius = 90;   // ignore the dark joint bosses near the plate edge

    const float cx = image.width() / 2.0f;
    const float cy = image.height() / 2.0f;
    long sumX = 0, sumY = 0, count = 0;

    for (int y = 0; y < image.height(); y++) {
        for (int x = 0; x < image.width(); x++) {
            const float dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy > searchRadius * searchRadius) continue;
            if (image.gray(x, y) < threshold) {
                sumX += x;
                sumY += y;
                count++;
            }
        }
    }

    if (count < 20) {  // too few dark pixels: no ball
        ball.found = false;
        return;
    }
    ball.found = true;
    ball.x = float(sumX) / count + 0.5f;  // +0.5: centre of the pixel
    ball.y = float(sumY) / count + 0.5f;
}
