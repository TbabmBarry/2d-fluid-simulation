#include "RigidBody.h"
#include <GL/glut.h>
#include <iostream>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

RigidBody::RigidBody(Vec2f MassCenter, Vec2f dimension, Vec2f numParticles, float particleMass) :
        MassCenter(MassCenter), dimension(dimension) {
    initializeVariables();

    float density = particleMass / (dimension[0]*dimension[1]); //mass/(x*y)
    //generate particles with body dimensions
    //assume mass center (0,0) for convenience
    int index = 0;
    for (int x = 0; x < numParticles[0]; x++) {
        for (int y = 0; y < numParticles[1]; y++) {
            float xStart = -dimension[0] / 2 + dimension[0] * (float) x / (numParticles[0] - 1);
            float yStart = -dimension[1] / 2 + dimension[1] * (float) y / (numParticles[1] - 1);
            Particle *p = new Particle(Vec2f(xStart, yStart), particleMass, index, false, true);
            //A rigid body has constant density
            p->density = density;
            particles.push_back(p);
            index++;
        }
    }
    //Calculate total mass
    for (Particle *p : particles) {
        M += p->mass;
    }
    //Calculate Ibody
    // Ibody = M* (pow(dimension[0],2)+pow(dimension[0],2));
}

RigidBody::~RigidBody(void) {
}

void RigidBody::reset() {
    initializeVariables();
    for (Particle *p : particles) {
        p->reset();
    }
}

void RigidBody::initializeVariables() {
    x = MassCenter;
    R = Matrix2f::Identity();
    P = Vec2f(0, 0);//M*v(t)
    L = 0.0;//I*w(t)
    v = Vec2f(0, 0);
    omega = L/I;
    force = Vec2f(0, 0);
    torque = 0.0;
    angle = 10 * 180 / M_PI;
}

// Vec2f RigidBody::getBodyCoordinates(Vec2f world) {
//     return R.transpose() * (world - x);
// }

void RigidBody::updateForce() {
    force = Vec2f(0, 0);
    for (Particle *p : particles)
        force += p->m_Force;
}

void RigidBody::updateTorque() {
    torque = 0.0;
    for (Particle *p : particles)
        torque += (p->m_Position[0] - x[0])*p->m_Force[1] 
        - (p->m_Position[1] - x[1])*p->m_Force[0]; //(0,0,torque)=(0,0,x*Fy-y*Fx) in 2D
}

void RigidBody::setState(VectorXf newState) {
    x[0] = newState[0];
    x[1] = newState[1];
    angle=newState[2]; //for R
    P[0] = newState[3];
    P[1] = newState[4];
    L = newState[5];

    //Compute derived variables
    R = Matrix2f(cos(angle), -sin(angle), sin(angle), cos(angle)); //counter-clockwise
    v = P / M;
    I = M* (pow(dimension[0],2)+pow(dimension[0],2));
    omega = L/I;

    //update positions
    for (Particle * p : particles) {
        p->m_Position = R * p->m_Position + x;
    }
}

/*
 * pack x, R, P and L into a single vector
 */
VectorXf RigidBody::getState() {
    VectorXf y(13);
    y[0] = x[0];
    y[1] = x[1];

    y[2] = angle;

    y[7] = P[0];
    y[8] = P[1];

    y[10] = L[0];
    return y;
}

VectorXf RigidBody::getDerivativeState() {
    updateForce();
    updateTorque();
    VectorXf y(13);
    //xdot
    y[0] = v[0];
    y[1] = v[1];
    y[2] = v[2];

    //calculate product, convert to resulting matrix to quaternion
    Quaternionf omegaQuaternion(0, omega[0], omega[1], omega[2]);
    Quaternionf qdot(omegaQuaternion * q);
    y[3] = qdot.w() * 0.05f;
    y[4] = qdot.x() * 0.05f;
    y[5] = qdot.y() * 0.05f;
    y[6] = qdot.z() * 0.05f;

    //Pdot = F
    y[7] = force[0];
    y[8] = force[1];
    y[9] = force[2];

    //Ldot = torque
    y[10] = torque[0];
    y[11] = torque[1];
    y[12] = torque[2];
    return y;
}

void RigidBody::handleSweep(bool isStart, vector<RigidBody *> *activeRigidBodies,
                            vector<pair<RigidBody *, Particle *>> *range) {
    if (isStart) {
        (*activeRigidBodies).push_back(this);
    } else {
        //remove
        auto it = std::find((*activeRigidBodies).begin(), (*activeRigidBodies).end(), this);
        if (it != (*activeRigidBodies).end())
            (*activeRigidBodies).erase(it);
    }
}

float RigidBody::density() {
    return particles[0]->density * particles.size();
}

void RigidBody::draw(bool drawVelocity, bool drawForce) {
    Vec2f v1 = R * Vec2f(-dimension[0] / 2, -dimension[1] / 2, -dimension[2] / 2) + x;
    Vec2f v2 = R * Vec2f(dimension[0] / 2, -dimension[1] / 2, -dimension[2] / 2) + x;
    Vec2f v3 = R * Vec2f(-dimension[0] / 2, -dimension[1] / 2, dimension[2] / 2) + x;
    Vec2f v4 = R * Vec2f(dimension[0] / 2, -dimension[1] / 2, dimension[2] / 2) + x;
    Vec2f v5 = R * Vec2f(-dimension[0] / 2, dimension[1] / 2, -dimension[2] / 2) + x;
    Vec2f v6 = R * Vec2f(dimension[0] / 2, dimension[1] / 2, -dimension[2] / 2) + x;
    Vec2f v7 = R * Vec2f(-dimension[0] / 2, dimension[1] / 2, dimension[2] / 2) + x;
    Vec2f v8 = R * Vec2f(dimension[0] / 2, dimension[1] / 2, dimension[2] / 2) + x;
    glBegin(GL_LINES);
    glColor3f(1.f, 1.f, 1.f);
    glVertex3f(v1[0], v1[1], v1[2]);
    glVertex3f(v2[0], v2[1], v2[2]);
    glVertex3f(v1[0], v1[1], v1[2]);
    glVertex3f(v3[0], v3[1], v3[2]);
    glVertex3f(v2[0], v2[1], v2[2]);
    glVertex3f(v4[0], v4[1], v4[2]);
    glVertex3f(v3[0], v3[1], v3[2]);
    glVertex3f(v4[0], v4[1], v4[2]);

    glVertex3f(v5[0], v5[1], v5[2]);
    glVertex3f(v6[0], v6[1], v6[2]);
    glVertex3f(v5[0], v5[1], v5[2]);
    glVertex3f(v7[0], v7[1], v7[2]);
    glVertex3f(v6[0], v6[1], v6[2]);
    glVertex3f(v8[0], v8[1], v8[2]);
    glVertex3f(v7[0], v7[1], v7[2]);
    glVertex3f(v8[0], v8[1], v8[2]);

    glVertex3f(v1[0], v1[1], v1[2]);
    glVertex3f(v5[0], v5[1], v5[2]);
    glVertex3f(v2[0], v2[1], v2[2]);
    glVertex3f(v6[0], v6[1], v6[2]);
    glVertex3f(v3[0], v3[1], v3[2]);
    glVertex3f(v7[0], v7[1], v7[2]);
    glVertex3f(v4[0], v4[1], v4[2]);
    glVertex3f(v8[0], v8[1], v8[2]);
    glEnd();

}