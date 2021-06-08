#include "Viscosity.h"
#include "Kernels.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

Viscosity::Viscosity(vector<Particle*> particles)
{
    this->setTarget(particles);
}

void Viscosity::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void Viscosity::apply(System* s)
{
  if (this->active)
  {
    float mu = 10;
    for (Particle *pi : particles) {
        if (pi->rigid || pi->cloth) continue;
        Vec2f viscosityForce = Vec2f(0, 0);

        vector<Particle*> targets = s->grid.query(pi);
        for (Particle *pj : targets) {
            viscosityForce += mu * pj->mass * (pj->m_Velocity - pi->m_Velocity) / pj->density
                             * Viscos::sdW(pi->m_Position - pj->m_Position);
        }
        pi->vForce = viscosityForce;
        pi->m_Force += viscosityForce;
    }
  }
}

void Viscosity::draw()
{

}
