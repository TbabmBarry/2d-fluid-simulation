#include "Grid.h"
#include <math.h>

Grid::Grid(int x, int y, float delta, Vec2f offset): xMax(x), yMax(y), delta(delta), offset(offset),
    grid(
        vector<vector<Cell>>
            (x, vector<Cell>
                (y, Cell())
            )
    ) {}

void Grid::insert(const vector<Particle *> particles) {
    for (int i=0; i<particles.size(); ++i) {
        this->insert(particles[i]);
    }
}

void Grid::insert(Particle *p) {//insert a particle into grid by position projection
    int grid_x = floorf((p->m_Position[0] + offset[0]) / delta);//to be redefined: projection
    int grid_y = floorf((p->m_Position[1] + offset[1]) / delta);//to be redefined: projection

    if (grid_x >= 0 && grid_y >= 0 && grid_x < xMax && grid_y < yMax )
        grid[grid_x][grid_y].insert(p);
}

void appendVect(vector<Particle*> &target, vector<Particle*> &from) {
    //helper func. to save "from"(particles) into "target"(vector of particles)
    target.insert(target.end(), make_move_iterator(from.begin()),
                  make_move_iterator(from.end()));
}

vector<Particle *> Grid::query(Particle *p) {
    //according to a particle position get all particles(当前,上,下,左,右cells) in grid
    int grid_x = floorf((p->m_Position[0] + offset[0]) / delta);//to be redefined: projection
    int grid_y = floorf((p->m_Position[1] + offset[1]) / delta);//to be redefined: projection

    vector<Particle*> result;

    if (grid_x < 0 || grid_y < 0 || grid_x >= xMax || grid_y >= yMax )
        return result;

    unsigned long count = 0;
    for (int xneighbor = -1; xneighbor <= 1; xneighbor++) {
        if (grid_x + xneighbor < xMax && grid_x + xneighbor >= 0)
            for (int yneighbor = -1; yneighbor < 1; yneighbor++) {
                if (grid_y + yneighbor < yMax && grid_y + yneighbor >= 0)
                    count += grid[grid_x + xneighbor][grid_y + yneighbor].particles.size();
            }
    }
    result.reserve(count);//initialize a vector of particles: result
    for (int xneighbor = -1; xneighbor <= 1; xneighbor++) {
        if (grid_x + xneighbor < xMax && grid_x + xneighbor >= 0)
            for (int yneighbor = -1; yneighbor <= 1; yneighbor++) {
                if (grid_y + yneighbor < yMax && grid_y + yneighbor >= 0)
                    appendVect(result, grid[grid_x + xneighbor][grid_y + yneighbor].particles);
            }
    }
    return result;
}

void Grid::clear() {
    for (int i = 0; i < xMax; i++) {
        for (int j = 0; j < yMax; j++) {
            grid[i][j].clear();
        }
    }
}

void Cell::insert(Particle *p) {
    particles.push_back(p);
}

void Cell::clear() {
    particles.clear();
}