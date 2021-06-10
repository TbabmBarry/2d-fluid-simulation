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

void PressureForce::Pressure(System* s)
{
    for (Particle *pi : particles) {
        double rho;
        vector<Particle*> targets = s->grid->query(pi);
        for (Particle *pj : targets) {
            rho += pj->mass * Spiky::W(pi->m_Position - pj->m_Position);
        }
        pi->pressure += k * (rho - restrho);
    }
}

void PressureForce::apply(System* s)
{
  if (this->active)
  {
    for (Particle *pi : particles) {
        Vec2f pressureForce = Vec2f(0, 0);
        // double rho;
        vector<Particle*> targets = s->grid->query(pi);
        // for (Particle *pj : targets) {
        //     rho += pj->mass * Spiky::W(pi->m_Position - pj->m_Position);
        // }
        // pi->pressure += k * (rho - restrho);
        for (Particle *pj : targets) {
            if(pi->cloth && pj->cloth) continue;
            if(pi->rigid && pj->rigid) continue;
            if(pi->cloth){
                pressureForce += -0.2 * pj->mass * (pj->pressure - pi->pressure) / (2*pj->density)
                             * Spiky::dW(pi->m_Position - pj->m_Position);
            }
            if(pi->rigid){
                pressureForce += -0.002 * pj->mass * (pj->pressure - pi->pressure) / (2*pj->density)
                             * Spiky::dW(pi->m_Position - pj->m_Position);
            }
            else{
                pressureForce += -pj->mass * (pj->pressure - pi->pressure) / (2*pj->density)
                             * Spiky::dW(pi->m_Position - pj->m_Position);
            }
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
