#include "AngularSpring.h"
#include <GL/glut.h>
#include <gfx/vec2.h>
#include <math.h>
#include "unistd.h"
#include <assert.h>

#define PI 3.14159265

AngularSpring::AngularSpring(Particle *p1, Particle * midpoint,Particle * p3, float m_angle, float m_ks, float m_kd) :
  AngularSpring({p1, midpoint, p3}, m_angle, m_ks, m_kd) {}

AngularSpring::AngularSpring(vector<Particle*> particles, float m_angle, float m_ks, float m_kd) : m_angle(m_angle * PI / 180.0), m_ks(m_ks), m_kd(m_kd)
{
    this->setTarget(particles);
}

void AngularSpring::setTarget(vector<Particle*> particles)
{
    assert(particles.size() == 3);
    this->particles = particles;
}

void AngularSpring::apply(System* s)
{
  if (this->active)
  {
    Vec2f midtoP1 = particles[0]->m_Position - particles[1]->m_Position; //l1=particle p1-particle midpoint
    Vec2f P3tomid = particles[1]->m_Position - particles[2]->m_Position; //l2=particle p1-particle midpoint
    float cos_angle = (midtoP1 * P3tomid)/(norm(midtoP1) * norm(P3tomid));
    if (cos_angle > 1.0) cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;
    Vec2f length = particles[0]->m_Position - particles[2]->m_Position;
    Vec2f velocity = particles[0]->m_Velocity - particles[2]->m_Velocity;

    // Compute spring force
    double b = norm(midtoP1);
    double c = norm(P3tomid);
    Vec2f result = -(m_ks * (norm(length) - sqrt(b * b + c * c - 2 * b * c * cos(m_angle))) + m_kd * ((length * velocity) / norm(length))) *
                   (length / norm(length));

    // usleep(300);
    particles[0]->m_Force += result;
    particles[2]->m_Force -= result;
  }
}

map<int, map<int, float>> AngularSpring::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf AngularSpring::dv()
{
    return MatrixXf();
}

void AngularSpring::draw()
{
}
