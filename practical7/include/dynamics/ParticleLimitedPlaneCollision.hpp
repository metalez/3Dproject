#ifndef PARTICLE_LIMITEDPLANE_COLLISION_HPP
#define PARTICLE_LIMITEDPLANE_COLLISION_HPP

#include "Collision.hpp"
#include "Particle.hpp"
#include "LimitedPlane.hpp"

/**@brief Implement the resolution of a collision event between a plane and a particle.
 *
 * Implementation of the resolution of a collision event between a plane and a particle.
 */
class ParticleLimitedPlaneCollision : public Collision
{
public:
    /**@brief Build a new collision event between a plane and a particle.
     *
     * Build a collision event between a plane and a particle. This collision will
     * be resolved assuming the plane is fixed.
     * @param particle The particle colliding a plane.
     * @param plane The plane colliding a particle.
     * @param restitution Restitution factor of this collision.
     */
    ParticleLimitedPlaneCollision(ParticlePtr particle, LimitedPlanePtr plane, float restitution);

    virtual ~ParticleLimitedPlaneCollision();

private:
    /**@brief Solve the collision between the plane and the particle.
     *
     * Update the particle position and velocity after its collision with
     * the fixed plane.
     */
    void do_solveCollision();

    ParticlePtr m_particle;
    LimitedPlanePtr m_plane;
};

typedef std::shared_ptr<ParticleLimitedPlaneCollision> ParticleLimitedPlaneCollisionPtr;

bool testParticleLimitedPlane(const ParticlePtr& particle, const LimitedPlanePtr& plane);

#endif //PARTICLE_LIMITEDPLANE_COLLISION_HPP
