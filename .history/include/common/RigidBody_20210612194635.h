#include "Eigen/Dense"
#include "Particle.h"
#include "Force.h"

// using namespace Eigen;

class RigidBody {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int STATE_SIZE = 18;
    RigidBody(Vec2f startPos, Vec2f dimensions, Vec2f numParticles, float particleMass);

    // virtual ~RigidBody(void);

    void reset();
    void draw(bool drawVelocity, bool drawForce);

    float density();
    //from object
    void handleSweep(bool start, vector<RigidBody *>* activeRigidBodies, vector<pair<RigidBody *, Particle *>> *range) ;

    VectorXf getBoundingBox();//minX, minY, maxX, maxY
    Vec2f getBodyCoordinates(Vec2f world);

    VectorXf getState();

    VectorXf getDerivativeState();

    void setState(VectorXf newState);
    void recomputeAuxiliaryVars();

    std::vector<Particle *> particles;
    Vec2f MassCenter;
    Vec2f dimension;  //lengths of the edges

    //Constants
    double M; //totalMass
    float Ibody;
    float I;

    //State variables
    Vec2f x; //position x(t)
    Quaternionf q; //quaternion representing R
    Vec2f P; //linear momentum P(t)
    Vec2f L; //angular momentum L(t)

    //Derived quantities
    Matrix3f R;              //rotation R(t)
    Matrix3f Iinv;           //I^-1(t)
    Vec2f v;                 //velocity v(t)
    Vec2f omega;             //angular velocity omega(t)

    //Computed quantities
    Vec2f force;
    Vec2f torque;

private:
    void updateForce();
    void updateTorque();
    void initializeVariables();
};