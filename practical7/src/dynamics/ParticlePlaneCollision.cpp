#include "./../../include/dynamics/ParticlePlaneCollision.hpp"

ParticlePlaneCollision::ParticlePlaneCollision(ParticlePtr particle, PlanePtr plane, float restitution) :
    Collision(restitution)
{
    m_particle = particle;
    m_plane = plane;
}

ParticlePlaneCollision::~ParticlePlaneCollision()
{}

void ParticlePlaneCollision::do_solveCollision()
{

    //Don't process fixed particles (ground plane is assumed fixed)
    if (m_particle->isFixed())
        return;

    if (!m_plane->isGround)
    {
      glm::vec3 q = m_particle->getPosition();
      glm::vec3 proj = m_plane->projectOnPlane(q);
      float particlePlaneDist = glm::distance(q, proj);
      float penetration =  m_particle->getRadius()  - particlePlaneDist;
      glm::vec3 k = glm::normalize(m_plane->normal());

      m_particle->setPosition(q - penetration*k);


      //Compute post-collision velocity
      glm::vec3 prev_v = m_particle->getVelocity();
      float proj_v = (1.0f + m_restitution)
          * glm::dot(k, prev_v)
          / (1.0 / m_particle->getMass());
      glm::vec3 new_v = prev_v - proj_v/m_particle->getMass()*k;
      m_particle->setVelocity(new_v);    
    }else{
      glm::vec3 q = m_particle->getPosition();
      glm::vec3 proj = m_plane->projectOnPlane(q);
      float particlePlaneDist = glm::distance(q, proj);
      float penetration =  m_particle->getRadius()  - particlePlaneDist;
      glm::vec3 k = glm::normalize(m_plane->normal());
      glm::vec3 newpos = q - penetration*k;
      newpos.z=0;

      m_particle->setPosition(newpos);
    }








    //TODO: Solve ParticlePlane collisions,
    // update particle position and velocity after collision
    //
    //Functions to use:
    //glm::dot(v1, v2): Return the dot product of two vector.
    //Plane::distanceToOrigin(): Return the distance to origin from the plane
    //Plane::normal(): Return the normal of the plane
    //Particle::getRadius(), Particle::getPosition(), Particle::getVelocity(), Particle::setPosition(), Particle::setVelocity()


}



bool testParticlePlane(const ParticlePtr &particle, const PlanePtr &plane)
{
    /* Equation of a plane passing through A and normal to n:
   * dot( p - A, n ) = dot( p, n ) - dot( A, n ) = 0
   * dot( A, n ) is stored in the "distanceToOrigin" of the plane.
   *
   * Equation of a particle of radius r centered in c:
   * dot( p - c, p - c ) = r²
   *
   * distance( plane, particle )
   *   = min( distance( plane, c ) - r, 0 )              //definition
   *   = min( abs( dot( c - A, n ) ) - r, 0 )
   *   = min( abs( dot( c, n ) - dot( A, n ) ) - r, 0 )
   *
   * So, there is intersection if distance( plane, particle ) = 0
   * <=> abs( dot( c, n ) - dot( A, n ) ) - r <= 0
   * <=> abs( dot( c, n ) - dot( A, n ) ) <= r
   */

    //TODO: Test collision between particle and plane
    //Functions to use:
    //glm::dot(v1, v2): Return the dot product of two vector.
    //Plane::distanceToOrigin(): Return the distance to origin from the plane
    //Plane::normal(): Return the normal of the plane
    //Particle::getRadius(), Particle::getPosition()

    float distance = plane->distanceToOrigin();
    glm::vec3 c = particle->getPosition();
    glm::vec3 n = plane->normal();
    float distancePartPlane = abs(dot(c,n)-distance);
    float radius = particle->getRadius();


    if (distancePartPlane  <= radius ){
        return true;
    }


    return false;
}
