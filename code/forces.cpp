#include "forces.h"

void ForceConstAcceleration::apply() {
    for (Particle* p : particles) {
        p->force += p->mass * acceleration;
    }
}

void ForceDrag::apply() {
    for (Particle* p : particles) {
        Vec3 v = p->vel;
        p->force -= klinear * v + kquadratic * v * v.norm();
    }
}

void ForceSpring::apply() {
    if (particles.size() < 2) return;
    Particle* p1 = getParticle1();
    Particle* p2 = getParticle2();

    Vec3 dp = p2->pos - p1->pos;
    double dist = dp.norm();
    if (dist < 1e-8) return;

    Vec3 dir = dp / dist;
    Vec3 dv = p2->vel - p1->vel;
    double relVel = dv.dot(dir);

    Vec3 F = ks * (dist - L) * dir + kd * relVel * dir;

    p1->force += F;
    p2->force -= F;
}

void ForceGravitation::apply() {
    if (!attractor) return;

    for (Particle* p : particles) {
        if (p == attractor) continue;

        Vec3 r_vec = attractor->pos - p->pos;
        double r = r_vec.norm();
        if (r < 1e-6) continue;

        Vec3 dir = r_vec / r;

        double smoothFactor = (2.0 / (1.0 + std::exp(-a * (r*r) / (b*b)))) - 1.0;

        Vec3 F = G * (p->mass * attractor->mass) * smoothFactor / (r*r) * dir;
        p->force += F;
    }
}
