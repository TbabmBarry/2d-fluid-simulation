#include "Viscosity.h"
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
{   // force = mg
  if (this->active)
  {
    for (Particle *pi : particles) {
        // if (pi->rigid || pi->cloth) continue;
        Vec2f viscosityForce = Vec2f(0, 0);

        vector<Particle*> targets = s->grid.query(pi->position);
        for (Particle *pj : targets) {
            viscosityForce = pj->mass * (pj->velocity - pi->velocity) / pj->density
                             * Viscosity::ddW(pi->position - pj->position);
        }
        pi->vForce = u * viscosityForce;
        pi->force += u * viscosityForce;
    }
  }
}

void Viscosity::draw()
{

}
