#include "Viscosity.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

Viscosity::Viscosity(vector<Particle*> particles)
{
    this->setTarget(particles);
}

// double h=0.02;

void Viscosity::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void Viscosity::apply(System* s)
{
  if (this->active)
  {
    double mu=10;
    for (Particle *pi : particles) {
        if (pi->rigid || pi->cloth) continue;
        Vec2f viscosityForce = Vec2f(0, 0);

        vector<Particle*> targets = s->grid.query(pi);
        for (Particle *pj : targets) {
            viscosityForce += mu * pj->mass * (pj->m_Velocity - pi->m_Velocity) / pj->density
                             * Viscosity::ddW(pi->m_Position - pj->m_Position, h);
        }
        pi->Viscosity = viscosityForce;
        pi->m_Force += viscosityForce;
    }
  }
}

double Viscosity::W(Vec2f distance, h){

}

void Viscosity::draw()
{

}