#include "Kernels.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


// Poly6 kernel: use for everything but pressure forces & viscosity
float Poly6::W(Vec2f r, float h) {
    float rd = norm(r);
    if (rd >= 0 && rd <= h) {
        return 315 / (64 * M_PI * pow(h, 9)) * pow(h * h - rd * rd, 3);
    }
    return 0;
}

Vec2f Poly6::dW(Vec2f r, float h) {
    float rd = norm(r);
    if (rd >= 0 && rd <= h) {
        return 315 / (64 * M_PI * pow(h, 9)) * -6 * pow(h * h - rd * rd, 2) * r;
    }
    return Vec2f(0,0);
}

float Poly6::sdW(Vec2f r, float h) {
    float rd = norm(r);
    if (rd >= 0 && rd <= h) {
        return 315 / (64 * M_PI * pow(h, 9)) * (24 * rd * rd * (h * h - rd * rd) - 6 * pow(h * h - rd * rd, 2));
    }
    return 0;
}

// Spiky kernel: use for pressure forces
float Spiky::W(Vec2f r, float h) {
    float rd = norm(r);
    if (rd >= 0 && rd <= h) {
        return 15 / (M_PI * pow(h, 6)) * pow(h - rd, 3);
    }
    return 0;
}

Vec2f Spiky::dW(Vec2f r, float h) {
    float rd = norm(r);
    if (rd > 0 && rd <= h) {
        return - 45 / (M_PI * pow(h, 6) * rd) * pow(h - rd, 2) * r;
    }
    return Vec2f(0,0);
};


float Viscos::W(Vec2f r, float h) {
    float rd = norm(r);
    if (rd >= 0 && rd <= h) {
        return 15 / (2 * M_PI * pow(h, 3)) * (-pow(rd, 3) / (2 * pow(h, 3)) + (rd * rd) / (h * h) + h / (2 * rd) - 1);
    }
    return 0;
}

float Viscos::sdW(Vec2f r, float h) {
    float rd = norm(r);
    if (rd >= 0 && rd <= h) {
        return 45 / (M_PI * pow(h, 6)) * (h - rd);
    }
    return 0;