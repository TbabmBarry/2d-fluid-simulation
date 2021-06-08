#include "Particle.h"
#include <vector>

using namespace std;

class Cell {
    public:
        Cell() : particles(vector<Particle*>()) {};
        void insert(Particle* p);
        void clear();
        vector<Particle*> particles;
};

class Grid {
    public:
        Grid(int x, int y, float delta, Vec2f offset = Vec2f(0,0));
        void insert(vector<Particle*> particles);
        void clear();
        vector<Particle*> query(Vec2f &pos);

    private:
        void insert(Particle *p);
        float delta;
        Vec2f offset;
        int xMax, yMax;
        vector< vector< Cell > > grid;     // 2d Grid with vectors
};