#pragma once
#include <stdio.h>
#include <gfx/mat2.h>
#include "Particle.h"
#include "Force.h"
#include <map>

class Viscosity : public Force {
  public:
    Viscosity(vector<Particle*> particles);
    void setTarget(vector<Particle*> particles) override;
    void apply(System* s) override;
    map<int, map<int, float>> dx() override;
    MatrixXf dv() override;
    void draw() override;
    double W(Vec2f distance);
    double ddW(Vec2f distance);
  private:
    Vec2f g = Vec2f(0.0f, -9.8f);     // gravity
};
