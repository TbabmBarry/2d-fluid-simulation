#pragma once
#include <stdio.h>
#include <gfx/mat2.h>
#include "Particle.h"
#include "Force.h"
#include <map>

class PressureForce : public Force {
  public:
    PressureForce(vector<Particle*> particles);
    void setTarget(vector<Particle*> particles) override;
    void apply(System* s) override;
    void Pressure(System* s);
    map<int, map<int, float>> dx() override;
    MatrixXf dv() override;
    void draw() override;
    double k=0.08; //(L*atm)/(L*mol)
    double restrho=1000; //kg/m^3 for water
    // double restrho=100; //kg/m^3 for air
};
