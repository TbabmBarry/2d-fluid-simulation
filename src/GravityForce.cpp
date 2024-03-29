#include "SpringForce.h"
#include "GravityForce.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>

GravityForce::GravityForce(vector<Particle*> particles, Vec2f g) :
  g(g) {
    this->setTarget(particles);
  }

void GravityForce::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void GravityForce::apply(System* s)
{   // force = mg
  if (this->active)
  {
    for (Particle *p : particles){
      p->m_Force += p->mass * g;
    }
  }
}

map<int, map<int, float>> GravityForce::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf GravityForce::dv()
{
    return MatrixXf();
}

void GravityForce::draw()
{

}
