#include "PressureForce.h"
#include "Kernels.h"
#include "System.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>
#define PI 3.1415926535897932384626433832795

PressureForce::PressureForce(vector<Particle*> particles)
{
    this->setTarget(particles);
}

void PressureForce::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void PressureForce::apply(System* s)
{
  if (this->active)
  {
    float mu = 10;
    for (Particle *pi : particles) {
        if (pi->rigid || pi->cloth) continue;
        Vec2f viscosityForce = Vec2f(0, 0);

        vector<Particle*> targets = s->grid->query(pi);
        for (Particle *pj : targets) {
            viscosityForce += mu * pj->mass * (pj->m_Velocity - pi->m_Velocity) / pj->density
                             * Viscos::sdW(pi->m_Position - pj->m_Position);
        }
        pi->vForce = viscosityForce;
        pi->m_Force += viscosityForce;
    }
  }
}

map<int, map<int, float>> Viscosity::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf Viscosity::dv()
{
    return MatrixXf();
}

void Viscosity::draw()
{

}
