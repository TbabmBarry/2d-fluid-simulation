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
    Vec2f ddW(pi->m_Position - pj->m_Position);
  private:
    Vec2f g = Vec2f(0.0f, -9.8f);     // gravity
};
