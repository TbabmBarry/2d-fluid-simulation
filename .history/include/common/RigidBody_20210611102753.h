#include "Eigen/Dense"
#include "Particle.h"
#include "Force.h"

// using namespace Eigen;

class RigidBody {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int STATE_SIZE = 18;
    RigidBody(Eigen::Vector3f startPos, Vector3f dimensions, Vector3f numParticles, float particleMass);

    float density();
    void reset();

    void draw(bool drawVelocity, bool drawForce);

    //from object
    void handleSweep(bool start, vector<RigidBody *>* activeRigidBodies, vector<pair<RigidBody *, Particle *>> *range) ;

    VectorXf getBoundingBox();          //minX, minY, minZ, maxX, maxY, maxZ
    Vector3f getBodyCoordinates(Vector3f world);

    VectorXf getState();

    VectorXf getDerivativeState();

    void setState(VectorXf newState);
    void recomputeAuxiliaryVars();

    std::vector<Particle *> particles;
    Vector3f startPos;
    Vector3f dimensions;  //lengths of the edges

    //Constants
    double M;                    //totalMass
    Matrix3f Ibody, IbodyInv;

    //State variables
    Vector3f x;                 //position x(t)
    Quaternionf q;              //quaternion representing R
    Vector3f P;                 //linear momentum P(t)
    Vector3f L;                 //angular momentum L(t)

    //Derived quantities
    Matrix3f R;                   //rotation R(t)
    Matrix3f Iinv;              //I^-1(t)
    Vector3f v;                 //velocity v(t)
    Vector3f omega;             //angular velocity omega(t)

    //Computed quantities
    Vector3f force;
    Vector3f torque;

private:
    void updateForce();
    void updateTorque();
    void initializeVariables();
};