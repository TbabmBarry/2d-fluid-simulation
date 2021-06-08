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

void Grid::insert(Particle *p) {
    int xC = floorf((p->m_Position[0] + offset[0]) / delta);
    int yC = floorf((p->m_Position[1] + offset[1]) / delta);

    if (xC >= 0 && yC >= 0 && xC < xMax && yC < yMax )
        grid[xC][yC].insert(p);
}

void appendVect(vector<Particle*> &target, vector<Particle*> &from) {
    target.insert(target.end(), make_move_iterator(from.begin()),
                  make_move_iterator(from.end()));
}

vector<Particle *> Grid::query(Vec2f &pos) {
    int xC = floorf((pos[0] + offset[0]) / delta);
    int yC = floorf((pos[1] + offset[1]) / delta);

    vector<Particle*> result;

    if (xC < 0 || yC < 0 || xC >= xMax || yC >= yMax )
        return result;

    unsigned long count = 0;
    for (int x = -1; x < 1; x++) {
        if (xC + x < xMax && xC + x >= 0)
            for (int y = -1; y < 1; y++) {
                if (yC + y < yMax && yC + y >= 0)
                    count += grid[xC + x][yC + y].particles.size();
            }
    }
    result.reserve(count);
    for (int x = -1; x <= 1; x++) {
        if (xC + x < xMax && xC + x >= 0)
            for (int y = -1; y <= 1; y++) {
                if (yC + y < yMax && yC + y >= 0)
                    appendVect(result, grid[xC + x][yC + y].particles);
            }
    }
    return result;
}

void Grid::clear() {
    for (int x = 0; x < xMax; x++) {
        for (int y = 0; y < yMax; y++) {
            grid[x][y].clear();
        }
    }
}

void Cell::insert(Particle *p) {
    particles.push_back(p);
}

void Cell::clear() {
    particles.clear();
}