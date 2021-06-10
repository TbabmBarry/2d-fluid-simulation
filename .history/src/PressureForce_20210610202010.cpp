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
    float k = 10;
    for (Particle *pi : particles) {
        Vec2f pressureForce = Vec2f(0, 0);
        double rho;
        vector<Particle*> targets = s->grid->query(pi);
        for (Particle *pj : targets) {
            rho += pj->mass * Spiky::W(pi->m_Position - pj->m_Position);
        }
        pi->pressure += k * (rho - );
        for (Particle *pj : targets) {
            pressureForce += pj->mass * (pj->pressure - pi->pressure) / (2*pj->density)
                             * Spiky::dW(pi->m_Position - pj->m_Position);
        }
        pi->pForce = pressureForce;
        pi->m_Force += pressureForce;
    }
  }
}

map<int, map<int, float>> PressureForce::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf PressureForce::dv()
{
    return MatrixXf();
}

void PressureForce::draw()
{

}
