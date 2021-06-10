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
    float mu = 10;//Ns/m^2
};
