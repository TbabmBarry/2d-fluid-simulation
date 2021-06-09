#pragma once

#include <gfx/vec2.h>

class Particle
{
public:

	Particle(const Vec2f & ConstructPos, float mass, int index);
	virtual ~Particle(void);

	void reset();
	void draw();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_Force;
	Vec2f Viscosity;
	double mass;
	double pressure;
	double density;//????? density of particle?
	int index;
	bool rigid;
	bool cloth;
};
