#include <gfx/vec2.h>

static float hDist = 0.05f;

class Poly6 {
public:
    static float W(Vec2f r, float h = hDist);
    static Vec2f dW(Vec2f r, float h = hDist);
    static  float sdW(Vec2f r, float h = hDist);
};

class Spiky {
public:
    static float W(Vec2f r, float h = hDist);
    static Vec2f dW(Vec2f r, float h = hDist);
};

class Viscos {
public:
    static float W(Vec2f r, float h = hDist);
    static float sdW(Vec2f r, float h = hDist);
};
